#!/usr/bin/env python3
"""JSON-lines YOLOv8 CPU inference worker for the ROS Kinetic Python 2 wrapper."""

import argparse
import base64
import contextlib
import json
import os
import sys

import cv2
import numpy as np


os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/Ultralytics")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")


def eprint(*args):
    print(*args, file=sys.stderr)


def load_model(model_path):
    with contextlib.redirect_stdout(sys.stderr):
        from ultralytics import YOLO

        model = YOLO(model_path)
        model.to("cpu")
    return model


def class_id_from_name(class_name, class_index):
    text = str(class_name)
    digits = "".join(ch for ch in text if ch.isdigit())
    return int(digits) if digits else int(class_index)


def decode_image(encoded):
    raw = base64.b64decode(encoded)
    data = np.frombuffer(raw, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("failed to decode JPEG image")
    return image


def run_inference(model, image, request):
    with contextlib.redirect_stdout(sys.stderr):
        results = model.predict(
            source=image,
            imgsz=int(request.get("imgsz", 512)),
            conf=float(request.get("conf", 0.5)),
            iou=float(request.get("iou", 0.45)),
            max_det=int(request.get("max_det", 50)),
            device="cpu",
            verbose=False,
        )
    if not results:
        return []

    names = getattr(model, "names", {}) or {}
    boxes = getattr(results[0], "boxes", None)
    if boxes is None:
        return []

    detections = []
    for box in boxes:
        xyxy = box.xyxy[0].detach().cpu().numpy().tolist()
        class_index = int(box.cls[0].detach().cpu().item())
        class_name = str(names.get(class_index, class_index))
        detections.append(
            {
                "class_id": class_id_from_name(class_name, class_index),
                "class_name": class_name,
                "confidence": float(box.conf[0].detach().cpu().item()),
                "xmin": float(xyxy[0]),
                "ymin": float(xyxy[1]),
                "xmax": float(xyxy[2]),
                "ymax": float(xyxy[3]),
                "center_x": float((xyxy[0] + xyxy[2]) * 0.5),
                "center_y": float((xyxy[1] + xyxy[3]) * 0.5),
            }
        )
    return detections


def serve(model_path):
    model = load_model(model_path)
    eprint("YOLOv8 worker ready: model=%s device=cpu" % model_path)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        request_id = None
        try:
            request = json.loads(line)
            request_id = request.get("id")
            image = decode_image(request["image"])
            detections = run_inference(model, image, request)
            response = {"id": request_id, "ok": True, "detections": detections}
        except Exception as exc:
            response = {"id": request_id, "ok": False, "error": str(exc)}
        sys.stdout.write(json.dumps(response, separators=(",", ":")) + "\n")
        sys.stdout.flush()


def find_default_image(model_path):
    root = os.path.abspath(os.path.join(os.path.dirname(model_path), "..", ".."))
    candidates = [
        os.path.join(root, "yolo", "data", "images", "frame_003200.jpg"),
        os.path.join(root, "yolo", "data", "images", "frame_000000.jpg"),
    ]
    for path in candidates:
        if os.path.exists(path):
            return path
    image_dir = os.path.join(root, "yolo", "data", "images")
    if os.path.isdir(image_dir):
        for name in sorted(os.listdir(image_dir)):
            if name.lower().endswith((".jpg", ".jpeg", ".png")):
                return os.path.join(image_dir, name)
    return None


def self_test(model_path, image_path):
    if image_path is None:
        image_path = find_default_image(model_path)
    if not image_path or not os.path.exists(image_path):
        raise SystemExit("self-test image not found")

    model = load_model(model_path)
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit("failed to read self-test image: %s" % image_path)
    request = {"imgsz": 512, "conf": 0.5, "iou": 0.45, "max_det": 50}
    detections = run_inference(model, image, request)
    classes = sorted({int(det["class_id"]) for det in detections})
    print(
        json.dumps(
            {
                "ok": True,
                "image": image_path,
                "detections": len(detections),
                "classes": classes,
            },
            sort_keys=True,
        )
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--image")
    args = parser.parse_args()

    if args.self_test:
        self_test(args.model, args.image)
    else:
        serve(args.model)


if __name__ == "__main__":
    main()
