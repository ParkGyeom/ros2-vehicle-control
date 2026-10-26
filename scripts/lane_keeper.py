#!/usr/bin/env python3
"""차선 유지 노드: 카메라 → 차선 색 골라내기 → bird's-eye → 앞 7 m 지점의 차선 중앙 → 조향.

lane_mask.py(1~3단계)를 그대로 쓰고, 조향만 새로 붙였다.
"""
import math
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image

# ---- 2단계: bird's-eye 변환 (어제와 같은 값) ----
fx = fy = 554.38
cx, cy = 320.0, 240.0
h = 0.7
near, far, side, ppm = 5.0, 20.0, 3.0, 50
W, H = int(2 * side * ppm), int((far - near) * ppm)   # 300 x 750


def to_pixel(X, Z):
    return [cx + fx * X / Z, cy + fy * h / Z]


src = np.float32([to_pixel(-side, near), to_pixel(side, near),
                  to_pixel(side, far), to_pixel(-side, far)])
dst = np.float32([[0, H], [W, H], [W, 0], [0, 0]])
M = cv2.getPerspectiveTransform(src, dst)


def row_of(z):
    """앞으로 z m 떨어진 바닥이 bird's-eye 사진의 몇 번째 행인지."""
    return int((far - z) * ppm)


CAM_TO_AXLE = 0.5    # 카메라가 바퀴 축보다 0.5 m 앞에 있음 (월드 파일 값으로 계산)
LANE_WIDTH = 3.0


class LaneKeeper(Node):
    def __init__(self):
        super().__init__('lane_keeper')
        self.speed = self.declare_parameter('speed', 1.0).value          # m/s
        self.lookahead = self.declare_parameter('lookahead', 7.0).value  # 앞 몇 m를 볼지
        self.max_travel = self.declare_parameter('max_travel', 20.0).value

        self.create_subscription(Image, '/front_camera/image', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/model/vehicle_blue/odometry', self.on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.debug_pub = self.create_publisher(Image, '/lane/debug', 1)
        self.create_timer(0.2, self.watchdog)

        self.traveled = 0.0
        self.last_xy = None
        self.last_image = time.monotonic()
        self.done = False
        self.get_logger().info(
            f'speed {self.speed} m/s | lookahead {self.lookahead} m | stop after {self.max_travel} m')

    # ---- 주행 거리 (정해진 거리만 달리고 멈추기) ----
    def on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_xy is not None:
            self.traveled += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    # ---- 3단계 응용: 앞 z m 근처에서 차선 중앙의 좌우 위치 ----
    def lane_center(self, bev, z):
        band = bev[row_of(z + 1.0):row_of(z - 1.0)]       # z±1 m 가로 띠
        hist = band.sum(axis=0) / 255                      # 세로 줄마다 흰 픽셀 수
        mid = W // 2
        left_x = int(np.argmax(hist[:mid]))
        right_x = mid + int(np.argmax(hist[mid:]))
        left_ok, right_ok = hist[left_x] > 20, hist[right_x] > 20

        if left_ok and right_ok:
            center_x = (left_x + right_x) / 2
        elif left_ok:                                      # 한쪽만 보이면 차선 폭으로 추정
            center_x = left_x + LANE_WIDTH / 2 * ppm
        elif right_ok:
            center_x = right_x - LANE_WIDTH / 2 * ppm
        else:
            return None, left_x, right_x
        return (center_x - mid) / ppm, left_x, right_x    # +: 차선 중앙이 내 차보다 오른쪽

    def on_image(self, msg):
        self.last_image = time.monotonic()
        if self.done:
            return
        if self.traveled >= self.max_travel:
            self.stop(f'{self.traveled:.1f} m 주행 완료')
            self.done = True
            return

        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)         # 시뮬 카메라는 rgb8

        # 1단계: 차선 색만 남기기
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, (0, 0, 150), (179, 30, 255))
        yellow = cv2.inRange(hsv, (20, 80, 100), (35, 255, 255))
        mask = cv2.bitwise_or(white, yellow)
        # 2단계: bird's-eye
        bev = cv2.warpPerspective(mask, M, (W, H))
        # 3단계: 앞 lookahead m 지점의 차선 중앙
        lat, left_x, right_x = self.lane_center(bev, self.lookahead)

        cmd = Twist()
        if lat is None:
            self.get_logger().warn('차선이 안 보임 → 정지', throttle_duration_sec=1.0)
        else:
            # 4단계: Pure Pursuit 조향
            #   목표점 = 바퀴 축에서 앞으로 L, 왼쪽으로 -lat 떨어진 점
            #   그 점을 지나는 원을 따라가려면 곡률 = 2·sin(alpha) / 거리
            L = self.lookahead + CAM_TO_AXLE
            alpha = math.atan2(-lat, L)                    # 목표점 방향 (왼쪽 +)
            curvature = 2 * math.sin(alpha) / math.hypot(L, lat)
            cmd.linear.x = self.speed
            cmd.angular.z = max(-1.0, min(1.0, self.speed * curvature))  # 로봇 한계 ±1 rad/s
            self.get_logger().info(
                f'앞 {self.lookahead:.0f} m 차선 중앙 {lat:+.2f} m | '
                f'조향 {cmd.angular.z:+.3f} rad/s | 주행 {self.traveled:.1f} m',
                throttle_duration_sec=0.5)
        self.cmd_pub.publish(cmd)
        self.publish_debug(bev, left_x, right_x, lat, msg.header)

    def publish_debug(self, bev, left_x, right_x, lat, header):
        """rqt로 볼 화면: bird's-eye + 찾은 차선(초록) + 목표점(빨강)."""
        if self.debug_pub.get_subscription_count() == 0:
            return
        dbg = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR)
        y0, y1 = row_of(self.lookahead + 1.0), row_of(self.lookahead - 1.0)
        cv2.rectangle(dbg, (0, y0), (W - 1, y1), (255, 160, 0), 1)
        for x in (left_x, right_x):
            cv2.line(dbg, (x, y0), (x, y1), (0, 200, 0), 2)
        cv2.line(dbg, (W // 2, H - 1), (W // 2, H - 40), (200, 200, 200), 2)    # 내 차
        if lat is not None:
            tx = int(W // 2 + lat * ppm)
            cv2.circle(dbg, (tx, row_of(self.lookahead)), 6, (0, 0, 255), -1)
        out = Image()
        out.header = header
        out.height, out.width, out.encoding, out.step = H, W, 'bgr8', W * 3
        out.data = dbg.tobytes()
        self.debug_pub.publish(out)

    def watchdog(self):
        if not self.done and time.monotonic() - self.last_image > 1.0:
            self.stop('카메라 영상이 1초 넘게 안 옴')

    def stop(self, why):
        for _ in range(3):
            self.cmd_pub.publish(Twist())
        self.get_logger().info(f'정지: {why}')


def main():
    # Ctrl+C 때 ROS가 먼저 꺼지지 않게 해서, 정지 명령을 보낼 수 있게 함
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = LaneKeeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop('Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
