#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

std::atomic<bool> g_stop_requested{false};
void signal_handler(int) { g_stop_requested = true; }

class DriveDistance : public rclcpp::Node {
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;

  static constexpr double kRobotMaxSpeed = 0.5;  // diff_drive.sdf의 max_linear_velocity

public:
  DriveDistance() : Node("drive_distance") {
    target_    = this->declare_parameter("target_distance", 2.0);
    max_speed_ = this->declare_parameter("max_speed", 0.5);   // 최고 속도
    min_speed_ = this->declare_parameter("min_speed", 0.05);  // 목표 직전 최저 속도
    kp_        = this->declare_parameter("kp", 1.5);          // 비례 게인
    tolerance_ = this->declare_parameter("tolerance", 0.01);  // 허용 오차 (m)

    // 로봇 한계를 넘는 속도는 잘라내고 경고
    if (max_speed_ > kRobotMaxSpeed) {
      RCLCPP_WARN(this->get_logger(),
                  "max_speed %.2f exceeds robot limit %.2f m/s -> clamped",
                  max_speed_, kRobotMaxSpeed);
      max_speed_ = kRobotMaxSpeed;
    }

    cmd_pub_ = this->create_publisher<Twist>("/model/vehicle_blue/cmd_vel", 10);
    odom_sub_ = this->create_subscription<Odometry>(
      "/model/vehicle_blue/odometry", 10,
      std::bind(&DriveDistance::odom_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
      100ms, std::bind(&DriveDistance::control_loop, this));

    RCLCPP_INFO(this->get_logger(),
                "Target %.2f m | max %.2f, min %.2f m/s | kp %.2f | tol %.3f m",
                target_, max_speed_, min_speed_, kp_, tolerance_);
  }

  bool is_done() const { return state_ == State::Done; }

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
  enum class State { Driving, Settling, Done };

  // [인지] 위치와 현재 속도 저장
  void odom_callback(const Odometry::SharedPtr msg) {
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    v_ = msg->twist.twist.linear.x;
    if (!has_start_) {
      start_x_ = x_;
      start_y_ = y_;
      has_start_ = true;
      RCLCPP_INFO(this->get_logger(), "Start position: (%.2f, %.2f)", start_x_, start_y_);
    }
  }

  // [판단 + 제어] 0.1초마다 실행
  void control_loop() {
    if (!has_start_ || state_ == State::Done) return;

    double traveled = std::hypot(x_ - start_x_, y_ - start_y_);
    double remaining = target_ - traveled;
    Twist cmd;  // 기본값 0

    if (state_ == State::Driving) {
      if (remaining > tolerance_) {
        // P제어: 남은 거리에 비례한 속도, [min, max] 범위로 제한
        cmd.linear.x = std::clamp(kp_ * remaining, min_speed_, max_speed_);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Traveled %.3f | remaining %.3f | cmd %.3f | v %.3f",
                             traveled, remaining, cmd.linear.x, v_);
      } else {
        decided_at_ = traveled;
        state_ = State::Settling;
        RCLCPP_INFO(this->get_logger(), "Stop decided at %.4f m", decided_at_);
      }
    } else if (state_ == State::Settling) {
      // 0을 계속 보내며 실제로 멈출 때까지 대기
      still_count_ = (std::abs(v_) < 0.001) ? still_count_ + 1 : 0;
      ++settle_ticks_;
      if (still_count_ >= 5 || settle_ticks_ >= 30) {  // 0.5초 정지 유지 or 3초 타임아웃
        RCLCPP_INFO(this->get_logger(),
                    "Final %.4f m | decided %.4f m | slide %.4f m | error %+.4f m",
                    traveled, decided_at_, traveled - decided_at_, traveled - target_);
        state_ = State::Done;
      }
    }
    cmd_pub_->publish(cmd);
  }

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double target_, max_speed_, min_speed_, kp_, tolerance_;
  double x_{0.0}, y_{0.0}, v_{0.0}, start_x_{0.0}, start_y_{0.0};
  double decided_at_{0.0};
  bool has_start_{false};
  State state_{State::Driving};
  int still_count_{0}, settle_ticks_{0};
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);

  auto node = std::make_shared<DriveDistance>();

  while (rclcpp::ok() && !g_stop_requested && !node->is_done()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  node->stop();
  rclcpp::shutdown();
  return 0;
}
