#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

// Ctrl+C가 눌렸는지 기록하는 플래그
std::atomic<bool> g_stop_requested{false};

void signal_handler(int) {
  g_stop_requested = true;  // 여기서는 표시만 하고, 실제 정지는 main에서
}

class CircleDriver : public rclcpp::Node {
public:
  CircleDriver() : Node("circle_driver") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/model/vehicle_blue/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&CircleDriver::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Driving in a circle. Press Ctrl+C to stop.");
  }

  void stop() {
    timer_->cancel();                    // 주행 명령 중단
    geometry_msgs::msg::Twist zero;      // 모든 값이 0인 메시지 = 정지
    for (int i = 0; i < 5; ++i) {        // 유실 대비 5번 전송
      publisher_->publish(zero);
      std::this_thread::sleep_for(50ms);
    }
    RCLCPP_INFO(this->get_logger(), "Stop command sent.");
  }

private:
  void timer_callback() {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.0;
    msg.angular.z = 0.5;
    publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  // ROS2 기본 Ctrl+C 처리를 끄고 직접 처리
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, signal_handler);

  auto node = std::make_shared<CircleDriver>();

  // spin() 대신 직접 루프를 돌면서 정지 요청을 확인
  while (rclcpp::ok() && !g_stop_requested) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  node->stop();        // ROS2가 살아 있는 동안 정지 명령 전송
  rclcpp::shutdown();  // 그다음에 종료
  return 0;
}
