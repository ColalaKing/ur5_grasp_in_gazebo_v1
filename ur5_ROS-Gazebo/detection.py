#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import base64
import json
import math
import os
import re
import select
import subprocess
import sys
import threading
import time

import cv2
import cv_bridge
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ur5_notebook.msg import Detection2D
from ur5_notebook.msg import Detection2DArray


class YoloDigitDetector(object):
    """Python 2 ROS wrapper for a Python 3 Ultralytics YOLOv8 worker."""

    def __init__(self):
        rospy.init_node("detection", anonymous=False)

        self.bridge = cv_bridge.CvBridge()
        package_dir = os.path.dirname(os.path.abspath(__file__))
        default_model = os.path.join(package_dir, "yolo", "weights", "best.pt")
        default_worker = os.path.join(package_dir, "yolo", "yolov8_worker.py")

        self.image_topic = rospy.get_param("~image_topic", "/ur5/wrist_camera/image_raw")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/ur5/wrist_camera/camera_info"
        )
        self.model_path = rospy.get_param("~model_path", default_model)
        self.yolo_python = rospy.get_param(
            "~yolo_python", os.environ.get("YOLO_PYTHON", "/opt/yolo_py/bin/python")
        )
        self.worker_script = rospy.get_param("~worker_script", default_worker)
        self.input_size = int(rospy.get_param("~input_size", 512))
        self.confidence_threshold = float(rospy.get_param("~confidence_threshold", 0.50))
        self.iou_threshold = float(rospy.get_param("~iou_threshold", 0.45))
        self.max_detections = int(rospy.get_param("~max_detections", 50))
        self.worker_timeout = float(rospy.get_param("~worker_timeout", 2.0))
        self.worker_startup_timeout = float(
            rospy.get_param("~worker_startup_timeout", 30.0)
        )
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.confirm_frames = max(1, int(rospy.get_param("~confirm_frames", 3)))
        self.confirm_distance_px = float(rospy.get_param("~confirm_distance_px", 35.0))
        self.show_window = self._as_bool(rospy.get_param("~show_window", False))
        self.publish_annotated = self._as_bool(rospy.get_param("~publish_annotated", True))
        self.annotated_topic = rospy.get_param(
            "~annotated_topic", "/detection/annotated_image"
        )
        self.window_name = rospy.get_param("~window_name", "yolov8_digit_detection")
        self.min_color_ratio = float(rospy.get_param("~min_color_ratio", 0.10))
        self.saturation_threshold = int(rospy.get_param("~saturation_threshold", 45))
        self.min_yaw_contour_area = float(rospy.get_param("~min_yaw_contour_area", 80.0))

        self.allowed_class_ids, self.allowed_class_names = self._parse_classes(
            rospy.get_param("~classes", "0,1,2,3,4,5,6,7,8,9")
        )

        self.camera_info = None
        self.tracks = {}
        self.frame_index = 0
        self.request_id = 0
        self.worker = None
        self.worker_lock = threading.Lock()
        self.window_ready = False
        self.window_failed = False

        self.start_worker()

        self.results_pub = rospy.Publisher("/detection/results", Detection2DArray, queue_size=1)
        self.annotated_pub = rospy.Publisher(self.annotated_topic, Image, queue_size=1)
        self.stats_pub = rospy.Publisher("/detection/stats", String, queue_size=1)
        self.info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1
        )
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )
        rospy.on_shutdown(self.shutdown_worker)

        rospy.loginfo(
            "YOLOv8 wrapper started: model=%s worker=%s python=%s image_topic=%s annotated_topic=%s",
            self.model_path,
            self.worker_script,
            self.yolo_python,
            self.image_topic,
            self.annotated_topic,
        )

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def _parse_classes(self, value):
        if isinstance(value, list):
            raw_items = value
        else:
            raw_items = str(value).replace(";", ",").split(",")
        ids = set()
        names = set()
        for item in raw_items:
            token = str(item).strip()
            if not token:
                continue
            names.add(token)
            try:
                ids.add(int(token))
            except ValueError:
                pass
        if not ids and not names:
            ids.update(range(10))
            names.update([str(digit) for digit in range(10)])
        return ids, names

    def start_worker(self):
        if not os.path.exists(self.yolo_python):
            raise RuntimeError("YOLO Python runtime is missing: %s" % self.yolo_python)
        if not os.path.exists(self.worker_script):
            raise RuntimeError("YOLO worker script is missing: %s" % self.worker_script)
        if not os.path.exists(self.model_path):
            raise RuntimeError("YOLOv8 model file is missing: %s" % self.model_path)

        env = os.environ.copy()
        env.setdefault("YOLO_CONFIG_DIR", "/tmp/Ultralytics")
        env.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
        env.pop("PYTHONPATH", None)
        env["PYTHONNOUSERSITE"] = "1"
        cmd = [
            self.yolo_python,
            self.worker_script,
            "--model",
            self.model_path,
        ]
        self.worker = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=1,
            env=env,
        )
        try:
            self.wait_for_worker_ready()
        except Exception:
            self.shutdown_worker()
            raise
        thread = threading.Thread(target=self.drain_worker_stderr)
        thread.daemon = True
        thread.start()

    def wait_for_worker_ready(self):
        deadline = time.time() + self.worker_startup_timeout
        while time.time() < deadline:
            if self.worker.poll() is not None:
                raise RuntimeError("YOLOv8 worker exited during startup")
            remaining = max(0.0, deadline - time.time())
            ready, _, _ = select.select([self.worker.stderr], [], [], min(0.5, remaining))
            if not ready:
                continue
            line = self.worker.stderr.readline()
            if not line:
                continue
            text = self.decode_text(line).strip()
            if text:
                rospy.loginfo("[yolov8_worker] %s", text)
            if "YOLOv8 worker ready" in text:
                return
        raise RuntimeError(
            "YOLOv8 worker did not become ready within %.1fs"
            % self.worker_startup_timeout
        )

    def restart_worker(self):
        self.shutdown_worker()
        self.start_worker()

    def shutdown_worker(self):
        worker = self.worker
        self.worker = None
        if worker is None:
            return
        try:
            if worker.poll() is None:
                worker.terminate()
                worker.wait()
        except Exception:
            try:
                worker.kill()
            except Exception:
                pass

    def drain_worker_stderr(self):
        worker = self.worker
        if worker is None or worker.stderr is None:
            return
        for line in iter(worker.stderr.readline, b""):
            text = self.decode_text(line).strip()
            if text:
                rospy.loginfo("[yolov8_worker] %s", text)

    def decode_text(self, value):
        if sys.version_info[0] >= 3:
            if hasattr(value, "decode"):
                value = value.decode("utf-8", "replace")
            return str(value).encode("ascii", "replace").decode("ascii")

        if isinstance(value, unicode):
            return value.encode("ascii", "replace")
        if hasattr(value, "decode"):
            return value.decode("utf-8", "replace").encode("ascii", "replace")
        return str(value)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, image_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except cv_bridge.CvBridgeError as exc:
            rospy.logerr("Failed to convert camera image: %s", exc)
            return

        started_at = time.time()
        try:
            detections = self.detect(image)
        except Exception as exc:
            import traceback
            rospy.logerr("YOLOv8 detection failed: %s\n%s" % (exc, traceback.format_exc()))
            detections = []
        self.update_stability(detections)
        self.publish_results(image_msg.header, detections)
        elapsed_ms = (time.time() - started_at) * 1000.0
        self.publish_stats(image_msg.header, elapsed_ms, detections)

        annotated = self.draw_detections(image.copy(), detections)
        if self.publish_annotated:
            self.publish_annotated_image(image_msg.header, annotated)
        self.show_image(annotated)

    def detect(self, image):
        raw_detections = self.request_worker(image)
        detections = []
        height, width = image.shape[:2]
        for raw in raw_detections:
            class_name = str(raw.get("class_name", raw.get("class_id", "")))
            class_id = self.class_id_from_name(class_name, int(raw.get("class_id", -1)))
            if not self.class_allowed(class_id, class_name):
                continue

            xmin = max(0, min(width - 1, int(round(float(raw.get("xmin", 0))))))
            ymin = max(0, min(height - 1, int(round(float(raw.get("ymin", 0))))))
            xmax = max(0, min(width - 1, int(round(float(raw.get("xmax", 0))))))
            ymax = max(0, min(height - 1, int(round(float(raw.get("ymax", 0))))))
            if xmax <= xmin or ymax <= ymin:
                continue
            if not self.looks_like_colored_block(image, xmin, ymin, xmax, ymax):
                continue

            center_x = int(round((xmin + xmax) * 0.5))
            center_y = int(round((ymin + ymax) * 0.5))
            detections.append(
                {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": float(raw.get("confidence", 0.0)),
                    "xmin": xmin,
                    "ymin": ymin,
                    "xmax": xmax,
                    "ymax": ymax,
                    "center_x": center_x,
                    "center_y": center_y,
                    "yaw_deg": self.estimate_yaw_deg(image, xmin, ymin, xmax, ymax),
                    "stable": False,
                }
            )
        return sorted(detections, key=lambda item: (item["class_id"], -item["confidence"]))

    def request_worker(self, image):
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("Failed to JPEG-encode camera image")
        payload = base64.b64encode(encoded.tostring())

        self.request_id += 1
        request = {
            "id": self.request_id,
            "image": payload,
            "imgsz": self.input_size,
            "conf": self.confidence_threshold,
            "iou": self.iou_threshold,
            "max_det": self.max_detections,
        }

        with self.worker_lock:
            if self.worker is None or self.worker.poll() is not None:
                self.restart_worker()
            try:
                self.worker.stdin.write(json.dumps(request) + "\n")
                self.worker.stdin.flush()
            except Exception:
                self.restart_worker()
                self.worker.stdin.write(json.dumps(request) + "\n")
                self.worker.stdin.flush()

            ready, _, _ = select.select([self.worker.stdout], [], [], self.worker_timeout)
            if not ready:
                self.restart_worker()
                raise RuntimeError("YOLOv8 worker timed out after %.2fs" % self.worker_timeout)

            line = self.worker.stdout.readline()
            if not line:
                self.restart_worker()
                raise RuntimeError("YOLOv8 worker exited without a response")
            response = json.loads(line)

        if int(response.get("id", -1)) != self.request_id:
            raise RuntimeError("YOLOv8 worker response id mismatch")
        if not response.get("ok", False):
            raise RuntimeError(response.get("error", "YOLOv8 worker error"))
        return response.get("detections", [])

    def class_id_from_name(self, class_name, class_index):
        match = re.search(r"\d+", str(class_name))
        if match:
            return int(match.group(0))
        return int(class_index)

    def class_allowed(self, class_id, class_name):
        return class_id in self.allowed_class_ids or str(class_name) in self.allowed_class_names

    def looks_like_colored_block(self, image, xmin, ymin, xmax, ymax):
        if self.min_color_ratio <= 0.0:
            return True
        roi = image[ymin : ymax + 1, xmin : xmax + 1]
        if roi.size == 0:
            return False
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        saturated = hsv[:, :, 1] >= self.saturation_threshold
        color_ratio = float(np.count_nonzero(saturated)) / float(saturated.size)
        return color_ratio >= self.min_color_ratio

    def estimate_yaw_deg(self, image, xmin, ymin, xmax, ymax):
        roi = image[ymin : ymax + 1, xmin : xmax + 1]
        if roi.size == 0:
            return 0.0
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, self.saturation_threshold, 35], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        if not contours:
            return 0.0
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < self.min_yaw_contour_area:
            return 0.0
        rect = cv2.minAreaRect(contour)
        points = cv2.boxPoints(rect)
        best_dx = 0.0
        best_dy = 0.0
        best_len = -1.0
        for index in range(4):
            p0 = points[index]
            p1 = points[(index + 1) % 4]
            dx = float(p1[0] - p0[0])
            dy = float(p1[1] - p0[1])
            length = dx * dx + dy * dy
            if length > best_len:
                best_dx = dx
                best_dy = dy
                best_len = length
        if best_len <= 0.0:
            return 0.0
        image_angle = math.degrees(math.atan2(best_dy, best_dx))
        return self.normalize_degrees(-image_angle)

    def normalize_degrees(self, angle):
        while angle >= 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return float(angle)

    def update_stability(self, detections):
        self.frame_index += 1
        new_tracks = {}
        for det in detections:
            key = det["class_id"]
            prev = self.tracks.get(key)
            if prev is None:
                count = 1
            else:
                dx = det["center_x"] - prev["center_x"]
                dy = det["center_y"] - prev["center_y"]
                if (dx * dx + dy * dy) ** 0.5 <= self.confirm_distance_px:
                    count = prev["count"] + 1
                else:
                    count = 1
            det["stable"] = count >= self.confirm_frames
            new_tracks[key] = {
                "center_x": det["center_x"],
                "center_y": det["center_y"],
                "count": count,
                "frame": self.frame_index,
            }
        self.tracks = new_tracks

    def publish_results(self, header, detections):
        array_msg = Detection2DArray()
        array_msg.header = header
        for det in detections:
            msg = Detection2D()
            msg.class_id = int(det["class_id"])
            msg.class_name = str(det["class_name"])
            msg.confidence = float(det["confidence"])
            msg.xmin = int(det["xmin"])
            msg.ymin = int(det["ymin"])
            msg.xmax = int(det["xmax"])
            msg.ymax = int(det["ymax"])
            msg.center_x = int(det["center_x"])
            msg.center_y = int(det["center_y"])
            msg.yaw_deg = float(det.get("yaw_deg", 0.0))
            msg.stable = bool(det["stable"])
            array_msg.detections.append(msg)
        self.results_pub.publish(array_msg)

    def publish_stats(self, header, elapsed_ms, detections):
        msg = String()
        stable_count = len([det for det in detections if det["stable"]])
        payload = {
            "backend": "yolo",
            "method": "yolov8-worker",
            "device": "cpu",
            "model_path": self.model_path,
            "stamp": header.stamp.to_sec() if header.stamp else 0.0,
            "frame_id": header.frame_id,
            "elapsed_ms": elapsed_ms,
            "detections": len(detections),
            "stable_detections": stable_count,
            "classes": [int(det["class_id"]) for det in detections],
            "cxy": [
                [int(det["center_x"]), int(det["center_y"])] for det in detections
            ],
            "yaw_deg": [float(det.get("yaw_deg", 0.0)) for det in detections],
        }
        msg.data = json.dumps(payload, sort_keys=True)
        self.stats_pub.publish(msg)

    def publish_annotated_image(self, header, annotated):
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = header
            self.annotated_pub.publish(annotated_msg)
        except cv_bridge.CvBridgeError as exc:
            rospy.logerr("Failed to publish annotated image: %s", exc)

    def draw_detections(self, image, detections):
        for det in detections:
            color = (0, 180, 0) if det["stable"] else (0, 180, 255)
            cv2.rectangle(
                image,
                (det["xmin"], det["ymin"]),
                (det["xmax"], det["ymax"]),
                color,
                2,
            )
            cv2.circle(image, (det["center_x"], det["center_y"]), 4, color, -1)
            label = "%s %.2f cxy=(%d,%d) yaw=%.1f" % (
                det["class_name"],
                det["confidence"],
                det["center_x"],
                det["center_y"],
                det.get("yaw_deg", 0.0),
            )
            (text_w, text_h), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
            )
            text_y = max(det["ymin"] - 6, text_h + 4)
            cv2.rectangle(
                image,
                (det["xmin"], text_y - text_h - baseline - 4),
                (det["xmin"] + text_w + 4, text_y + baseline),
                color,
                -1,
            )
            cv2.putText(
                image,
                label,
                (det["xmin"] + 2, text_y - 3),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                1,
                cv2.LINE_AA,
            )
        return image

    def show_image(self, image):
        if not self.show_window or self.window_failed:
            return
        try:
            if not self.window_ready:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self.window_ready = True
            cv2.imshow(self.window_name, image)
            cv2.waitKey(1)
        except Exception as exc:
            self.window_failed = True
            rospy.logwarn("OpenCV window disabled: %s", exc)


def main():
    try:
        YoloDigitDetector()
    except Exception as exc:
        rospy.logfatal("Cannot start YOLOv8 wrapper: %s", exc)
        return 1
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
