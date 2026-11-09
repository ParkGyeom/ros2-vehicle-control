#!/usr/bin/env python3
"""YOLO 객체 인식 노드.

카메라 이미지(sensor_msgs/Image)를 받아 YOLO로 객체를 검출하고
  - /perception/detections      : vision_msgs/Detection2DArray (다른 노드가 쓰는 결과)
  - /perception/image_annotated : 박스를 그린 이미지 (rqt_image_view로 확인용)
을 발행한다.

cv_bridge를 쓰지 않고 numpy로 직접 변환한다.
(pip으로 설치한 ultralytics/opencv와 ROS 시스템 패키지의 numpy 버전 충돌을 피하기 위함)
"""
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLO
except ImportError as e:  # 설치 안내를 바로 보여주기 위해
    raise SystemExit('ultralytics가 없습니다: pip3 install --user ultralytics "numpy<2"') from e


def image_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> OpenCV BGR 배열 (row padding(step)까지 고려)."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if msg.encoding not in channels:
        raise ValueError(f'지원하지 않는 encoding: {msg.encoding}')
    ch = channels[msg.encoding]
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = buf[:, :msg.width * ch].reshape(msg.height, msg.width, ch)

    if msg.encoding == 'rgb8':
        return img[:, :, ::-1]
    if msg.encoding == 'rgba8':
        return img[:, :, 2::-1]          # R,G,B,A -> B,G,R
    if msg.encoding == 'bgra8':
        return img[:, :, :3]
    if msg.encoding == 'mono8':
        return np.repeat(img, 3, axis=2)
    return img                            # bgr8


def bgr_to_image(img: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.step = msg.width * 3
    msg.data = np.ascontiguousarray(img).tobytes()
    return msg


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        model_path = self.declare_parameter('model', 'yolo26n.pt').value
        self.conf = self.declare_parameter('conf', 0.4).value          # 최소 신뢰도
        self.imgsz = self.declare_parameter('imgsz', 640).value        # 추론 해상도 (느리면 320)
        self.device = self.declare_parameter('device', 'cpu').value
        image_topic = self.declare_parameter('image_topic', '/front_camera/image').value
        self.publish_annotated = self.declare_parameter('publish_annotated', True).value
        # 비워 두면 COCO 80개 클래스 전부, 예: ['person', 'car']
        self.keep_classes = set(
            self.declare_parameter('classes', ['']).value) - {''}

        self.get_logger().info(f'YOLO 모델 로드: {model_path} (처음이면 다운로드)')
        self.model = YOLO(model_path)
        self.class_ids = None
        if self.keep_classes:
            name_to_id = {n: i for i, n in self.model.names.items()}
            unknown = self.keep_classes - name_to_id.keys()
            if unknown:
                self.get_logger().warn(f'모르는 클래스 무시: {sorted(unknown)}')
            self.class_ids = [name_to_id[n] for n in self.keep_classes if n in name_to_id]

        # 최신 프레임만 처리: 추론이 느려도 밀린 프레임을 쌓지 않는다
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, sensor_qos)
        self.det_pub = self.create_publisher(Detection2DArray, '/perception/detections', 10)
        self.img_pub = self.create_publisher(Image, '/perception/image_annotated', 1)

        self.frames = 0
        self.infer_ms_sum = 0.0
        self.last_report = time.monotonic()
        self.get_logger().info(f'구독: {image_topic} | conf {self.conf} | imgsz {self.imgsz}')

    def on_image(self, msg: Image):
        try:
            img = image_to_bgr(msg)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        t0 = time.monotonic()
        result = self.model.predict(img, conf=self.conf, imgsz=self.imgsz,
                                    device=self.device, classes=self.class_ids,
                                    verbose=False)[0]
        self.infer_ms_sum += (time.monotonic() - t0) * 1000.0

        out = Detection2DArray()
        out.header = msg.header  # 이미지 촬영 시각을 그대로 전달 (지연 계산에 사용 가능)
        boxes = result.boxes
        for (x1, y1, x2, y2), score, cls in zip(boxes.xyxy.tolist(),
                                                boxes.conf.tolist(),
                                                boxes.cls.tolist()):
            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.model.names[int(cls)]
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)
        # 검출이 0개여도 발행한다 → 받는 쪽이 "인식 노드가 살아 있음"을 알 수 있음
        self.det_pub.publish(out)

        if self.publish_annotated and self.img_pub.get_subscription_count() > 0:
            self.img_pub.publish(bgr_to_image(result.plot(), msg.header))

        self.frames += 1
        now = time.monotonic()
        if now - self.last_report >= 5.0:
            fps = self.frames / (now - self.last_report)
            self.get_logger().info(
                f'{fps:.1f} FPS | 추론 평균 {self.infer_ms_sum / self.frames:.0f} ms | '
                f'최근 검출 {len(out.detections)}개')
            self.frames, self.infer_ms_sum, self.last_report = 0, 0.0, now


def main():
    rclpy.init()
    node = YoloDetector()
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
