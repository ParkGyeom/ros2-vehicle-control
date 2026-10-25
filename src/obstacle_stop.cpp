// 전방 장애물 정지 노드: [인지] YOLO 검출 → [판단] 단안 거리 추정·차선 필터 → [제어] P제어 감속 정지
//
// 거리 추정 (ground-plane 가정):
//   카메라가 높이 h에 수평으로 달려 있고 바닥이 평평하면, 바닥에 닿은 점이 이미지에서
//   v행에 보일 때 그 점까지의 전방 거리 Z = fy * h / (v - cy), 좌우 위치 X = (u - cx) * Z / fx.
//   → 박스 아랫변(= 물체가 바닥에 닿는 곳)으로 거리와 좌우 위치를 구한다.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace std::chrono_literals;

std::atomic<bool> g_stop_requested{false};
void signal_handler(int) { g_stop_requested = true; }

class ObstacleStop : public rclcpp::Node {
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Detections = vision_msgs::msg::Detection2DArray;

  static constexpr double kRobotMaxSpeed = 0.5;  // diff_drive.sdf의 max_linear_velocity
  static constexpr double kInf = std::numeric_limits<double>::infinity();

public:
  ObstacleStop() : Node("obstacle_stop") {
    auto classes = this->declare_parameter<std::vector<std::string>>(
      "target_classes", {"person", "bicycle", "car", "motorcycle", "bus", "truck"});
    target_classes_ = std::set<std::string>(classes.begin(), classes.end());
    min_score_       = this->declare_parameter("min_score", 0.4);
    camera_height_   = this->declare_parameter("camera_height", 0.7);    // 바닥~카메라 (m), 월드 파일과 일치
    lane_half_width_ = this->declare_parameter("lane_half_width", 1.2);  // 차폭 1.85m의 절반 + 여유
    stop_distance_   = this->declare_parameter("stop_distance", 3.0);    // 카메라(앞 범퍼)~장애물 목표 간격
    resume_margin_   = this->declare_parameter("resume_margin", 0.3);    // 정지 후 재출발 히스테리시스
    cruise_speed_    = this->declare_parameter("cruise_speed", 0.5);
    min_speed_       = this->declare_parameter("min_speed", 0.05);
    kp_              = this->declare_parameter("kp", 0.5);
    tolerance_       = this->declare_parameter("tolerance", 0.05);
    det_timeout_     = this->declare_parameter("detection_timeout", 1.5);  // 인식 결과가 끊기면 정지 (s)
    max_travel_      = this->declare_parameter("max_travel", 30.0);        // 월드 밖으로 나가지 않게
    clear_frames_    = static_cast<int>(this->declare_parameter("clear_frames", 3));  // 연속 N프레임 비어야 출발
    odom_prediction_ = this->declare_parameter("odom_prediction", true);   // false: 측정값 그대로 사용 (비교 실험용)

    if (cruise_speed_ > kRobotMaxSpeed) {
      RCLCPP_WARN(this->get_logger(), "cruise_speed %.2f -> clamped to %.2f m/s",
                  cruise_speed_, kRobotMaxSpeed);
      cruise_speed_ = kRobotMaxSpeed;
    }

    cmd_pub_ = this->create_publisher<Twist>("/model/vehicle_blue/cmd_vel", 10);
    odom_sub_ = this->create_subscription<Odometry>(
      "/model/vehicle_blue/odometry", 10,
      std::bind(&ObstacleStop::odom_callback, this, std::placeholders::_1));
    info_sub_ = this->create_subscription<CameraInfo>(
      "/front_camera/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleStop::info_callback, this, std::placeholders::_1));
    det_sub_ = this->create_subscription<Detections>(
      "/perception/detections", 10,
      std::bind(&ObstacleStop::detection_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(50ms, std::bind(&ObstacleStop::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "stop %.1f m | lane ±%.2f m | cam h %.2f m | cruise %.2f m/s | kp %.2f | odom prediction %s",
                stop_distance_, lane_half_width_, camera_height_, cruise_speed_, kp_,
                odom_prediction_ ? "on" : "off");
  }

  void stop() {
    timer_->cancel();
    Twist zero;
    for (int i = 0; i < 5; ++i) {
      cmd_pub_->publish(zero);
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "Stop command sent.");
  }

private:
  enum class State { WaitInput, Cruise, Approach, Hold, FailSafe, Done };

  struct Obstacle {
    std::string label;
    double score{0.0};
    double distance{kInf};          // 카메라 기준 전방 거리 (m)
    double lat_min{0.0}, lat_max{0.0};  // 박스 좌우 끝의 좌우 위치 (m, 오른쪽 +)
    bool truncated{false};          // 박스가 이미지 아래에 잘림 = 매우 가까움
  };

  // [인지 1] 카메라 내부 파라미터 (fx, fy, cx, cy)
  void info_callback(const CameraInfo::SharedPtr msg) {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) return;
    fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
    img_h_ = msg->height;
    if (!has_info_) {
      has_info_ = true;
      RCLCPP_INFO(this->get_logger(), "Camera: %ux%u fx %.1f fy %.1f cx %.1f cy %.1f",
                  msg->width, msg->height, fx_, fy_, cx_, cy_);
    }
  }

  // [인지 2] 오도메트리: 누적 주행거리 + (시각, 누적거리) 기록 → 인식 지연 보상에 사용
  void odom_callback(const Odometry::SharedPtr msg) {
    const double x = msg->pose.pose.position.x, y = msg->pose.pose.position.y;
    if (has_odom_) path_ += std::hypot(x - last_x_, y - last_y_);
    last_x_ = x; last_y_ = y; has_odom_ = true;
    odom_stamp_ = rclcpp::Time(msg->header.stamp).seconds();
    path_hist_.emplace_back(odom_stamp_, path_);
    while (path_hist_.size() > 200) path_hist_.pop_front();  // 50Hz × 4초
  }

  // 이미지 촬영 시각(sim time)에 차가 어디까지 왔었는지 → 그 뒤로 이동한 만큼 거리에서 뺀다
  double path_at(double stamp) const {
    if (path_hist_.empty()) return path_;
    for (auto it = path_hist_.rbegin(); it != path_hist_.rend(); ++it) {
      if (it->first <= stamp) return it->second;
    }
    return path_hist_.front().second;
  }

  // 검출 박스 하나 → 거리/좌우 위치 (ground-plane 역투영)
  Obstacle project(const vision_msgs::msg::Detection2D & det, const std::string & label,
                   double score) const {
    Obstacle ob;
    ob.label = label;
    ob.score = score;
    const double u_min = det.bbox.center.position.x - det.bbox.size_x / 2.0;
    const double u_max = det.bbox.center.position.x + det.bbox.size_x / 2.0;
    double v_bottom = det.bbox.center.position.y + det.bbox.size_y / 2.0;

    if (v_bottom >= img_h_ - 2.0) {   // 아랫변이 화면 밖 → 실제 거리는 추정보다 가까움
      ob.truncated = true;
      v_bottom = img_h_ - 1.0;
    }
    if (v_bottom <= cy_ + 1.0) return ob;  // 지평선 위: 바닥 접점이 안 보임 → 거리 무한대

    const double z = fy_ * camera_height_ / (v_bottom - cy_);
    ob.distance = ob.truncated ? 0.0 : z;  // 잘린 물체는 즉시 정지 대상으로
    ob.lat_min = (u_min - cx_) * z / fx_;
    ob.lat_max = (u_max - cx_) * z / fx_;
    return ob;
  }

  // [판단] 진행 경로(차선) 안의 가장 가까운 장애물
  void detection_callback(const Detections::SharedPtr msg) {
    last_det_time_ = this->now();
    has_det_ = true;
    if (!has_info_) return;

    Obstacle nearest;
    int ignored_out_of_lane = 0;
    for (const auto & det : msg->detections) {
      if (det.results.empty()) continue;
      const auto & hyp = det.results.front().hypothesis;
      if (hyp.score < min_score_ || !target_classes_.count(hyp.class_id)) continue;

      Obstacle ob = project(det, hyp.class_id, hyp.score);
      if (!std::isfinite(ob.distance)) continue;
      const bool in_lane = ob.lat_max > -lane_half_width_ && ob.lat_min < lane_half_width_;
      if (!in_lane) { ++ignored_out_of_lane; continue; }
      if (ob.distance < nearest.distance) nearest = ob;
    }

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    latency_ = has_odom_ ? std::max(0.0, odom_stamp_ - stamp) : 0.0;
    if (std::isfinite(nearest.distance)) {
      clear_count_ = 0;
      meas_distance_ = nearest.distance;
      meas_path_ = path_at(stamp);
    } else if (++clear_count_ >= clear_frames_) {
      meas_distance_ = kInf;  // 몇 프레임 연속으로 비어 있어야 "치워졌다"고 판단 (한 프레임 미검출 대비)
    }

    if (std::isfinite(nearest.distance)) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[in-lane] %s %.2f | dist %.2f m%s | lateral [%+.2f, %+.2f] m | "
        "latency %.2f s | out-of-lane ignored %d",
        nearest.label.c_str(), nearest.score, nearest.distance,
        nearest.truncated ? " (truncated)" : "", nearest.lat_min, nearest.lat_max,
        latency_, ignored_out_of_lane);
    } else if (ignored_out_of_lane > 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Lane clear | out-of-lane ignored %d", ignored_out_of_lane);
    }
  }

  // [제어] 20Hz: 지연 보상된 거리로 P제어 감속
  void control_loop() {
    Twist cmd;  // 기본값 0 = 정지

    if (state_ == State::Done) { cmd_pub_->publish(cmd); return; }
    if (!has_info_ || !has_odom_ || !has_det_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for input... camera_info %d | odometry %d | detections %d",
        has_info_, has_odom_, has_det_);
      set_state(State::WaitInput);
      cmd_pub_->publish(cmd);
      return;
    }
    if ((this->now() - last_det_time_).seconds() > det_timeout_) {
      // 인식이 멈추면 앞을 모르는 것 → 정지 (이미 HOLD로 서 있으면 그대로 유지)
      if (state_ != State::Hold) set_state(State::FailSafe);
      cmd_pub_->publish(cmd);
      return;
    }
    if (path_ >= max_travel_) {
      set_state(State::Done);
      cmd_pub_->publish(cmd);
      return;
    }

    // 이미지 촬영 이후 이동한 만큼 빼서 "지금" 거리를 예측 (정지 장애물 가정)
    // → YOLO 추론 지연 + 프레임 사이 이동을 오도메트리로 메운다
    const double distance = odom_prediction_ ? meas_distance_ - (path_ - meas_path_)
                                             : meas_distance_;
    if (!std::isfinite(distance)) {
      holding_ = false;
      set_state(State::Cruise);
      cmd.linear.x = cruise_speed_;
    } else {
      const double gap = distance - stop_distance_;
      // 한번 멈췄으면 gap이 resume_margin을 넘어야 다시 출발 (거리 추정 잡음으로 들썩이지 않게).
      // FAILSAFE를 거쳐 와도 유지되도록 상태와 별도로 기억한다.
      const bool resume = !holding_ || gap > resume_margin_;
      if (gap > tolerance_ && resume) {
        holding_ = false;
        set_state(State::Approach);
        cmd.linear.x = std::clamp(kp_ * gap, min_speed_, cruise_speed_);
      } else {
        holding_ = true;
        set_state(State::Hold);
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "distance %.2f m (measured %.2f) | gap %+.2f | cmd %.2f",
        distance, meas_distance_, gap, cmd.linear.x);
    }
    cmd_pub_->publish(cmd);
  }

  void set_state(State s) {
    if (s == state_) return;
    static const char * names[] = {"WAIT_INPUT", "CRUISE", "APPROACH", "HOLD", "FAILSAFE", "DONE"};
    const auto level = (s == State::FailSafe) ? RCUTILS_LOG_SEVERITY_WARN : RCUTILS_LOG_SEVERITY_INFO;
    if (level == RCUTILS_LOG_SEVERITY_WARN) {
      RCLCPP_WARN(this->get_logger(),
                  "State %s -> %s (no detections for > %.1f s; if YOLO FPS is low, "
                  "raise detection_timeout or run with gui:=false / imgsz:=320)",
                  names[static_cast<int>(state_)], names[static_cast<int>(s)], det_timeout_);
    } else {
      RCLCPP_INFO(this->get_logger(), "State %s -> %s | traveled %.2f m",
                  names[static_cast<int>(state_)], names[static_cast<int>(s)], path_);
    }
    state_ = s;
  }

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<Detections>::SharedPtr det_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::set<std::string> target_classes_;
  double min_score_, camera_height_, lane_half_width_, stop_distance_, resume_margin_;
  double cruise_speed_, min_speed_, kp_, tolerance_, det_timeout_, max_travel_;

  double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0}, img_h_{0.0};
  bool has_info_{false}, has_odom_{false}, has_det_{false};

  double last_x_{0.0}, last_y_{0.0}, path_{0.0}, odom_stamp_{0.0};
  std::deque<std::pair<double, double>> path_hist_;  // (sim time, 누적 주행거리)

  int clear_frames_{3}, clear_count_{0};
  bool odom_prediction_{true}, holding_{false};
  double meas_distance_{kInf}, meas_path_{0.0}, latency_{0.0};
  rclcpp::Time last_det_time_;
  State state_{State::WaitInput};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);

  auto node = std::make_shared<ObstacleStop>();

  while (rclcpp::ok() && !g_stop_requested) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  node->stop();
  rclcpp::shutdown();
  return 0;
}
