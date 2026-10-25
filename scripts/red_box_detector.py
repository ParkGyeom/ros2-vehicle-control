#!/usr/bin/env python3
"""검증용 가짜 인식 노드: 빨간 픽셀 덩어리를 'car'로 검출해 /perception/detections로 발행.

YOLO 대신 쓰면 박스가 픽셀 단위로 정확하므로, distance_test.sdf(정답 거리를 아는 빨간 상자)와 함께
"거리 추정 공식"과 "지연 보상"만 따로 검증할 수 있다.

  delay (s): 추론이 느린 상황을 흉내 내기 위해 프레임마다 일부러 기다린다.
"""
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class RedBoxDetector(Node):
    def __init__(self):
        super().__init__('red_box_detector')
        # delay:=1 처럼 정수로 줘도 받도록 dynamic_typing
        self.delay = float(self.declare_parameter(
            'delay', 0.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.create_subscription(Image, '/front_camera/image', self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.get_logger().info(f'red box detector | delay {self.delay:.2f} s')

    def on_image(self, msg: Image):
        if msg.encoding != 'rgb8':
            self.get_logger().error(f'rgb8만 지원: {msg.encoding}', throttle_duration_sec=5.0)
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
        img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        red = (img[:, :, 0] > 200) & (img[:, :, 1] < 60) & (img[:, :, 2] < 60)

        out = Detection2DArray()
        out.header = msg.header
        ys, xs = np.nonzero(red)
        if len(ys) > 20:
            x0, x1, y0, y1 = xs.min(), xs.max() + 1, ys.min(), ys.max() + 1
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x0 + x1) / 2.0
            det.bbox.center.position.y = (y0 + y1) / 2.0
            det.bbox.size_x = float(x1 - x0)
            det.bbox.size_y = float(y1 - y0)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = 'car'
            hyp.hypothesis.score = 1.0
            det.results.append(hyp)
            out.detections.append(det)

        if self.delay > 0.0:
            time.sleep(self.delay)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = RedBoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
