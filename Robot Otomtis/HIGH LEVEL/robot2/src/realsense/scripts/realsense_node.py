#!/usr/bin/env python3
import os
import json
import threading
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String, Float32
# pyrealsense2, numpy, cv2, ultralytics dimuat lazy supaya tidak ganggu rclpy init


class RealSenseNode(Node):
    def __init__(self, model_path: str, conf: float, imgsz: int):
        super().__init__('realsense_node')

        self._model_path = model_path
        self.conf        = conf
        self.imgsz       = imgsz
        self.model       = None
        self._running    = True

        self.color_pub  = self.create_publisher(Image, 'camera/color/image', 10)
        self.depth_pub  = self.create_publisher(Image, 'camera/depth/image', 10)
        self.detect_pub = self.create_publisher(String,  'camera/detections',   10)
        self.target_pub = self.create_publisher(String,  '/servo/target_xy',    10)
        self.depth_target_pub = self.create_publisher(Float32, '/camera/target_depth', 10)

        import pyrealsense2 as rs
        pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16,  30)
        pipeline.start(cfg)
        self.pipeline = pipeline
        self.align    = rs.align(rs.stream.color)

        # Capture + inference jalan di thread sendiri, tidak block executor ROS2
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        self.get_logger().info('RealSense D435i ready')

    # ------------------------------------------------------------------ #

    def _load_model(self):
        import numpy as np
        import cv2
        if not hasattr(np, 'bool'):    np.bool    = bool
        if not hasattr(np, 'int'):     np.int     = int
        if not hasattr(np, 'float'):   np.float   = float
        if not hasattr(np, 'complex'): np.complex = complex
        if not hasattr(np, 'object'):  np.object  = object
        if not hasattr(np, 'str'):     np.str     = str
        import torch
        import torchvision
        def _nms_pure(boxes, scores, iou_threshold):
            order = scores.argsort(descending=True)
            keep = []
            while order.numel() > 0:
                i = order[0].item(); keep.append(i)
                if order.numel() == 1: break
                xx1 = boxes[order[1:], 0].clamp(min=boxes[i, 0])
                yy1 = boxes[order[1:], 1].clamp(min=boxes[i, 1])
                xx2 = boxes[order[1:], 2].clamp(max=boxes[i, 2])
                yy2 = boxes[order[1:], 3].clamp(max=boxes[i, 3])
                inter = (xx2 - xx1).clamp(0) * (yy2 - yy1).clamp(0)
                area  = lambda b: (b[:, 2]-b[:, 0]) * (b[:, 3]-b[:, 1])
                iou   = inter / (area(boxes[[i]]) + area(boxes[order[1:]]) - inter)
                order = order[(iou <= iou_threshold).nonzero().squeeze(1) + 1]
            return torch.tensor(keep, dtype=torch.long)
        torchvision.ops.nms = _nms_pure
        from ultralytics import YOLO
        self._np   = np
        self._cv2  = cv2
        self.model = YOLO(self._model_path)
        self.get_logger().info(f'Model loaded: {self._model_path}')

    def _to_img_msg(self, arr, encoding: str) -> Image:
        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height   = arr.shape[0]
        msg.width    = arr.shape[1]
        msg.encoding = encoding
        msg.step     = arr.strides[0]
        msg.data     = arr.tobytes()
        return msg

    def _capture_loop(self):
        # Load model sekali di thread ini
        self._load_model()
        np  = self._np
        cv2 = self._cv2

        import os
        _DATASET_DIR = '/home/krai/Desktop/Huda/robot2/datasetBiru'
        os.makedirs(_DATASET_DIR, exist_ok=True)
        _frame_count  = 0
        _saved_count  = 0

        while self._running and rclpy.ok():
            frames  = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())   # BGR uint8
            depth_img = np.asanyarray(depth_frame.get_data())   # uint16 mm

            _frame_count += 1
            if _frame_count % 5 == 0:
                fname = os.path.join(_DATASET_DIR, f'frame_{_saved_count:06d}.jpg')
                cv2.imwrite(fname, color_img)
                _saved_count += 1

            results = self.model(
                color_img,
                verbose=False,
                conf=self.conf,
                device=0,
                half=True,
                imgsz=self.imgsz,
            )[0]

            detections = []
            annotated  = color_img.copy()

            for box in results.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls_id  = int(box.cls[0])
                conf    = float(box.conf[0])
                label   = self.model.names[cls_id]
                if label == 'rak':
                    continue
                cx, cy  = (x1 + x2) // 2, (y1 + y2) // 2
                # rata-rata depth dari 9 titik di sekitar center (3x3 grid, step 10px)
                _depths = []
                for _dy in (-10, 0, 10):
                    for _dx in (-10, 0, 10):
                        _d = depth_frame.get_distance(cx + _dx, cy + _dy)
                        if _d > 0.0:
                            _depths.append(_d)
                depth_m = sum(_depths) / len(_depths) if _depths else 0.0

                detections.append({
                    'label':   label,
                    'conf':    round(conf, 2),
                    'depth_m': round(depth_m, 3),
                    'bbox':    [x1, y1, x2, y2],
                    'center':  [cx, cy],
                })

                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # titik tengah objek
                cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)

                # label atas: nama + conf
                cv2.putText(
                    annotated,
                    f'{label} {conf:.2f}',
                    (x1, max(y1 - 8, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                )
                # label bawah: x, y, depth
                cv2.putText(
                    annotated,
                    f'x={cx} y={cy} d={depth_m:.2f}m',
                    (x1, y2 + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2,
                )

            # Publish Image hanya kalau ada subscriber (hemat bandwidth)
            if self.color_pub.get_subscription_count() > 0:
                self.color_pub.publish(self._to_img_msg(annotated, 'bgr8'))
            if self.depth_pub.get_subscription_count() > 0:
                depth_vis = np.clip(depth_img // 10, 0, 255).astype(np.uint8)
                self.depth_pub.publish(self._to_img_msg(depth_vis, 'mono8'))

            # crosshair tengah frame
            h, w = annotated.shape[:2]
            cx_f, cy_f = w // 2, h // 2
            cv2.line(annotated, (cx_f - 20, cy_f), (cx_f + 20, cy_f), (255, 255, 0), 1)
            cv2.line(annotated, (cx_f, cy_f - 20), (cx_f, cy_f + 20), (255, 255, 0), 1)

            cv2.imshow('Color + Detection', annotated)
            depth_vis = np.clip(depth_img // 10, 0, 255).astype(np.uint8)
            cv2.imshow('Depth', depth_vis)
            cv2.waitKey(1)

            if detections:
                self.get_logger().info(
                    '  '.join(f"{d['label']}({d['depth_m']:.2f}m)" for d in detections)
                )
                self.detect_pub.publish(String(data=json.dumps(detections)))

                # publish bottom-most object (y terbesar = paling bawah di frame)
                bottom = max(detections, key=lambda d: d['center'][1])
                cx_t, cy_t = bottom['center']
                self.target_pub.publish(String(
                    data=f"{bottom['label']}  x={cx_t}  y={cy_t}  depth={bottom['depth_m']:.3f}m"
                ))
                self.depth_target_pub.publish(Float32(data=float(bottom['depth_m'])))

                # highlight objek terpilih dengan bbox merah + lingkaran besar
                bx1, by1, bx2, by2 = bottom['bbox']
                cv2.rectangle(annotated, (bx1, by1), (bx2, by2), (0, 0, 255), 3)
                cv2.circle(annotated, (cx_t, cy_t), 10, (0, 0, 255), -1)
                cv2.putText(annotated, f'TARGET d={bottom["depth_m"]:.2f}m',
                    (bx1, by1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    def destroy_node(self):
        self._running = False
        if self.pipeline:
            self.pipeline.stop()
        if hasattr(self, '_cv2'):
            self._cv2.destroyAllWindows()
        super().destroy_node()


def prepare_engine(pt_path: str, imgsz: int) -> str:
    engine_path = pt_path.replace('.pt', '.engine')
    if os.path.exists(engine_path):
        print(f'[realsense] TensorRT engine found: {engine_path}')
        return engine_path
    print(f'[realsense] Exporting {pt_path} → TensorRT FP16, tunggu sebentar...')
    from ultralytics import YOLO
    YOLO(pt_path).export(
        format='engine',
        half=True,
        device=0,
        imgsz=imgsz,
        workspace=4,
    )
    print(f'[realsense] Export selesai: {engine_path}')
    return engine_path


def main(args=None):
    import sys
    pt_path = 'kfsnew.pt'
    conf    = 0.5
    imgsz   = 640
    for arg in (args or sys.argv[1:]):
        if arg.startswith('model:='):   pt_path = arg.split(':=')[1]
        if arg.startswith('conf:='):    conf    = float(arg.split(':=')[1])
        if arg.startswith('imgsz:='):   imgsz   = int(arg.split(':=')[1])

    model_path = prepare_engine(pt_path, imgsz)

    rclpy.init(args=args)
    node = RealSenseNode(model_path, conf, imgsz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
