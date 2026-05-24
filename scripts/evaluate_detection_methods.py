#!/usr/bin/env python3
"""Evaluate traditional and YOLO digit detectors on labeled Gazebo images."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
import types
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[1]
PACKAGE_DIR = ROOT / "ur5_ROS-Gazebo"
DEFAULT_VALID_LIST = PACKAGE_DIR / "yolo" / "data" / "valid_highres.txt"
DEFAULT_MODEL = PACKAGE_DIR / "yolo" / "weights" / "best.pt"
DEFAULT_OUTPUT_DIR = PACKAGE_DIR / "experiments" / "eval_100"


def install_ros_stubs() -> None:
    """Allow importing ROS Python2 node code from a Python3 offline evaluator."""

    rospy = types.ModuleType("rospy")
    for name in ("logwarn", "loginfo", "logerr", "logfatal"):
        setattr(rospy, name, lambda *args, **kwargs: None)
    rospy.get_param = lambda name, default=None: default
    rospy.init_node = lambda *args, **kwargs: None
    rospy.Publisher = lambda *args, **kwargs: None
    rospy.Subscriber = lambda *args, **kwargs: None
    rospy.on_shutdown = lambda *args, **kwargs: None
    sys.modules.setdefault("rospy", rospy)

    cv_bridge = types.ModuleType("cv_bridge")
    cv_bridge.CvBridge = object
    cv_bridge.CvBridgeError = Exception
    sys.modules.setdefault("cv_bridge", cv_bridge)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Image = type("Image", (), {})
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs_msg)

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = type("String", (), {})
    sys.modules.setdefault("std_msgs", std_msgs)
    sys.modules.setdefault("std_msgs.msg", std_msgs_msg)

    ur5_notebook = types.ModuleType("ur5_notebook")
    ur5_notebook_msg = types.ModuleType("ur5_notebook.msg")
    ur5_notebook_msg.Detection2D = type("Detection2D", (), {})
    ur5_notebook_msg.Detection2DArray = type("Detection2DArray", (), {})
    sys.modules.setdefault("ur5_notebook", ur5_notebook)
    sys.modules.setdefault("ur5_notebook.msg", ur5_notebook_msg)


def make_traditional_detector():
    install_ros_stubs()
    if str(PACKAGE_DIR) not in sys.path:
        sys.path.insert(0, str(PACKAGE_DIR))
    from traditional_detection import TraditionalDigitDetector

    detector = TraditionalDigitDetector.__new__(TraditionalDigitDetector)
    detector.method = "digit"
    detector.target_classes = list(range(10))
    detector.confidence_threshold = 0.40
    detector.min_area = 350.0
    detector.max_area_ratio = 0.35
    detector.min_saturation = 45
    detector.min_value = 35
    detector.dark_threshold = 70
    detector.color_distance_threshold = 0.36
    detector.template_threshold = 0.18
    detector.color_assist_classes = [6, 9]
    detector.color_assist_confidence = 0.45
    detector.use_detection_roi = True
    detector.roi_x_min_ratio = 0.50
    detector.roi_x_max_ratio = 1.00
    detector.roi_y_min_ratio = 0.00
    detector.roi_y_max_ratio = 1.00
    detector.min_extent = 0.35
    detector.max_candidate_width_ratio = 0.18
    detector.max_candidate_height_ratio = 0.24
    detector.view_bounds = (0.26, 0.83, -0.38, 0.36)
    detector.templates = detector.build_digit_templates(80)
    detector.color_vectors = detector.build_color_vectors()
    detector.tracks = {}
    detector.frame_index = 0
    return detector


def read_image_list(path: Path, limit: int) -> list[Path]:
    base = path.parent.parent
    images = []
    for raw in path.read_text().splitlines():
        item = raw.strip()
        if not item:
            continue
        image_path = Path(item)
        if not image_path.is_absolute():
            image_path = base / image_path
        images.append(image_path)
        if len(images) >= limit:
            break
    return images


def read_labels(label_path: Path, shape) -> list[dict]:
    height, width = shape[:2]
    labels = []
    for raw in label_path.read_text().splitlines():
        parts = raw.strip().split()
        if len(parts) != 5:
            continue
        class_id, cx, cy, bw, bh = parts
        cx = float(cx) * width
        cy = float(cy) * height
        bw = float(bw) * width
        bh = float(bh) * height
        labels.append(
            {
                "class_id": int(class_id),
                "center_x": cx,
                "center_y": cy,
                "xmin": cx - bw * 0.5,
                "ymin": cy - bh * 0.5,
                "xmax": cx + bw * 0.5,
                "ymax": cy + bh * 0.5,
            }
        )
    return labels


def normalize_yolo_detections(result) -> list[dict]:
    detections = []
    names = getattr(result, "names", {}) or {}
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return detections
    for box in boxes:
        xyxy = box.xyxy[0].detach().cpu().numpy().tolist()
        cls_index = int(box.cls[0].detach().cpu().item())
        class_name = str(names.get(cls_index, cls_index))
        digits = "".join(ch for ch in class_name if ch.isdigit())
        class_id = int(digits) if digits else cls_index
        detections.append(
            {
                "class_id": class_id,
                "class_name": class_name,
                "confidence": float(box.conf[0].detach().cpu().item()),
                "xmin": float(xyxy[0]),
                "ymin": float(xyxy[1]),
                "xmax": float(xyxy[2]),
                "ymax": float(xyxy[3]),
                "center_x": float((xyxy[0] + xyxy[2]) * 0.5),
                "center_y": float((xyxy[1] + xyxy[3]) * 0.5),
                "yaw_deg": 0.0,
            }
        )
    return detections


def match_predictions(labels, detections, shape):
    height, width = shape[:2]
    max_distance = max(80.0, 0.08 * min(width, height))
    used = set()
    matches = []
    wrong = []
    extra = []
    for det_index, det in enumerate(detections):
        best_index = None
        best_distance = None
        for label_index, label in enumerate(labels):
            if label_index in used:
                continue
            dx = float(det["center_x"]) - float(label["center_x"])
            dy = float(det["center_y"]) - float(label["center_y"])
            distance = (dx * dx + dy * dy) ** 0.5
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_index = label_index
        if best_index is None or best_distance > max_distance:
            extra.append(det_index)
            continue
        used.add(best_index)
        record = {
            "det_index": det_index,
            "label_index": best_index,
            "distance": float(best_distance),
            "pred": int(det["class_id"]),
            "truth": int(labels[best_index]["class_id"]),
        }
        if record["pred"] == record["truth"]:
            matches.append(record)
        else:
            wrong.append(record)
    missed = [idx for idx in range(len(labels)) if idx not in used]
    return matches, wrong, missed, extra


def update_stats(stats, labels, detections, matches, wrong, missed, extra, elapsed_ms):
    stats["frames"] += 1
    stats["objects"] += len(labels)
    stats["predictions"] += len(detections)
    stats["correct"] += len(matches)
    stats["wrong"] += len(wrong)
    stats["missed"] += len(missed)
    stats["extra"] += len(extra)
    stats["elapsed_ms"].append(elapsed_ms)
    if len(matches) == len(labels) and not wrong and not missed and not extra:
        stats["exact_frames"] += 1
    for label in labels:
        stats["per_class"][int(label["class_id"])]["truth"] += 1
    for match in matches:
        stats["per_class"][match["truth"]]["correct"] += 1
        stats["confusion"][match["truth"], match["pred"]] += 1
    for item in wrong:
        stats["per_class"][item["truth"]]["wrong"] += 1
        stats["confusion"][item["truth"], item["pred"]] += 1
    for idx in missed:
        stats["per_class"][int(labels[idx]["class_id"])]["missed"] += 1


def finalize_stats(stats):
    objects = max(1, stats["objects"])
    predictions = max(1, stats["predictions"])
    matched = stats["correct"] + stats["wrong"]
    stats["object_accuracy"] = stats["correct"] / float(objects)
    stats["recall"] = matched / float(objects)
    stats["precision"] = stats["correct"] / float(predictions)
    stats["classification_accuracy_matched"] = stats["correct"] / float(max(1, matched))
    stats["exact_frame_rate"] = stats["exact_frames"] / float(max(1, stats["frames"]))
    stats["avg_ms"] = float(np.mean(stats["elapsed_ms"])) if stats["elapsed_ms"] else 0.0
    stats["median_ms"] = float(np.median(stats["elapsed_ms"])) if stats["elapsed_ms"] else 0.0
    for class_id, item in stats["per_class"].items():
        item["recall"] = item["correct"] / float(max(1, item["truth"]))
    return stats


def new_stats():
    return {
        "frames": 0,
        "objects": 0,
        "predictions": 0,
        "correct": 0,
        "wrong": 0,
        "missed": 0,
        "extra": 0,
        "exact_frames": 0,
        "elapsed_ms": [],
        "per_class": {
            class_id: {"truth": 0, "correct": 0, "wrong": 0, "missed": 0}
            for class_id in range(10)
        },
        "confusion": np.zeros((10, 10), dtype=np.int32),
    }


def draw_detections(image, labels, detections, title):
    canvas = image.copy()
    for label in labels:
        cv2.rectangle(
            canvas,
            (int(label["xmin"]), int(label["ymin"])),
            (int(label["xmax"]), int(label["ymax"])),
            (160, 160, 160),
            1,
        )
    for det in detections:
        color = (0, 190, 0)
        cv2.rectangle(
            canvas,
            (int(det["xmin"]), int(det["ymin"])),
            (int(det["xmax"]), int(det["ymax"])),
            color,
            2,
        )
        label = "%d %.2f" % (int(det["class_id"]), float(det.get("confidence", 0.0)))
        if "yaw_deg" in det:
            label += " yaw %.0f" % float(det.get("yaw_deg", 0.0))
        cv2.putText(
            canvas,
            label,
            (int(det["xmin"]), max(18, int(det["ymin"]) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.circle(canvas, (int(det["center_x"]), int(det["center_y"])), 4, color, -1)
    cv2.putText(canvas, title, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (30, 30, 30), 3)
    return canvas


def make_summary_plot(results, output_path: Path) -> None:
    names = ["traditional", "yolo"]
    fig = plt.figure(figsize=(15, 10), dpi=140)
    grid = fig.add_gridspec(2, 2)

    ax = fig.add_subplot(grid[0, 0])
    metrics = ["object_accuracy", "precision", "recall", "exact_frame_rate"]
    x = np.arange(len(metrics))
    width = 0.35
    for offset, name in enumerate(names):
        values = [results[name][metric] * 100.0 for metric in metrics]
        ax.bar(x + (offset - 0.5) * width, values, width, label=name)
    ax.set_xticks(x)
    ax.set_xticklabels(["Object acc.", "Precision", "Recall", "Exact frames"], rotation=15)
    ax.set_ylim(0, 105)
    ax.set_ylabel("%")
    ax.set_title("100-frame Detection Accuracy")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()

    ax = fig.add_subplot(grid[0, 1])
    x = np.arange(10)
    for offset, name in enumerate(names):
        values = [results[name]["per_class"][str(i)]["recall"] * 100.0 for i in range(10)]
        ax.bar(x + (offset - 0.5) * width, values, width, label=name)
    ax.set_xticks(x)
    ax.set_xticklabels([str(i) for i in range(10)])
    ax.set_ylim(0, 105)
    ax.set_ylabel("Recall %")
    ax.set_title("Per-class Recall")
    ax.grid(axis="y", alpha=0.25)
    ax.legend()

    for col, name in enumerate(names):
        ax = fig.add_subplot(grid[1, col])
        confusion = np.array(results[name]["confusion"], dtype=np.int32)
        im = ax.imshow(confusion, cmap="Blues", vmin=0)
        ax.set_title("%s confusion (truth rows, pred cols)" % name)
        ax.set_xlabel("Predicted")
        ax.set_ylabel("Truth")
        ax.set_xticks(range(10))
        ax.set_yticks(range(10))
        for i in range(10):
            for j in range(10):
                value = int(confusion[i, j])
                if value:
                    ax.text(j, i, str(value), ha="center", va="center", fontsize=8)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def serialize_stats(stats):
    result = dict(stats)
    result["elapsed_ms"] = [float(v) for v in stats["elapsed_ms"]]
    result["per_class"] = {str(k): v for k, v in stats["per_class"].items()}
    result["confusion"] = stats["confusion"].astype(int).tolist()
    return result


def evaluate(args):
    image_paths = read_image_list(args.valid_list, args.count)
    if len(image_paths) < args.count:
        raise SystemExit("Only found %d images in %s" % (len(image_paths), args.valid_list))

    args.output_dir.mkdir(parents=True, exist_ok=True)
    traditional = make_traditional_detector()

    from ultralytics import YOLO

    yolo_model = YOLO(str(args.model))
    yolo_model.to("cpu")

    stats = {"traditional": new_stats(), "yolo": new_stats()}
    rows = []
    sample_canvases = []

    for frame_index, image_path in enumerate(image_paths, start=1):
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Cannot read image: %s" % image_path)
        labels = read_labels(image_path.with_suffix(".txt"), image.shape)

        started = time.perf_counter()
        traditional_dets = traditional.detect(image)
        traditional_dets = traditional.nms(traditional_dets, 0.30)
        traditional_dets = [
            det
            for det in traditional_dets
            if det["confidence"] >= traditional.confidence_threshold
        ]
        traditional_ms = (time.perf_counter() - started) * 1000.0

        started = time.perf_counter()
        yolo_result = yolo_model.predict(
            source=image,
            imgsz=args.imgsz,
            conf=args.conf,
            iou=args.iou,
            max_det=args.max_det,
            device="cpu",
            verbose=False,
        )[0]
        yolo_dets = normalize_yolo_detections(yolo_result)
        yolo_ms = (time.perf_counter() - started) * 1000.0

        for name, detections, elapsed in (
            ("traditional", traditional_dets, traditional_ms),
            ("yolo", yolo_dets, yolo_ms),
        ):
            matches, wrong, missed, extra = match_predictions(labels, detections, image.shape)
            update_stats(stats[name], labels, detections, matches, wrong, missed, extra, elapsed)
            rows.append(
                {
                    "frame": image_path.stem,
                    "method": name,
                    "labels": len(labels),
                    "predictions": len(detections),
                    "correct": len(matches),
                    "wrong": len(wrong),
                    "missed": len(missed),
                    "extra": len(extra),
                    "elapsed_ms": "%.3f" % elapsed,
                    "pred_classes": " ".join(str(int(det["class_id"])) for det in detections),
                }
            )

        if len(sample_canvases) < args.sample_count and frame_index in args.sample_frames:
            top = draw_detections(image, labels, traditional_dets, image_path.stem + " traditional")
            bottom = draw_detections(image, labels, yolo_dets, image_path.stem + " yolo")
            sample_canvases.append(np.vstack([top, bottom]))

        if args.verbose:
            print(
                "%s: traditional %d/%d, yolo %d/%d"
                % (
                    image_path.stem,
                    rows[-2]["correct"],
                    len(labels),
                    rows[-1]["correct"],
                    len(labels),
                )
            )

    finalized = {name: finalize_stats(item) for name, item in stats.items()}
    output = {name: serialize_stats(item) for name, item in finalized.items()}

    metrics_path = args.output_dir / "metrics.json"
    metrics_path.write_text(json.dumps(output, indent=2, sort_keys=True))

    csv_path = args.output_dir / "per_frame.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    plot_path = args.output_dir / "summary.png"
    make_summary_plot(output, plot_path)

    sample_path = args.output_dir / "sample_annotations.jpg"
    if sample_canvases:
        resized = [
            cv2.resize(canvas, (640, 960), interpolation=cv2.INTER_AREA)
            for canvas in sample_canvases
        ]
        cv2.imwrite(str(sample_path), np.hstack(resized))

    print(json.dumps(output, indent=2, sort_keys=True))
    print("metrics=%s" % metrics_path)
    print("per_frame=%s" % csv_path)
    print("summary=%s" % plot_path)
    if sample_canvases:
        print("samples=%s" % sample_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--valid-list", type=Path, default=DEFAULT_VALID_LIST)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--conf", type=float, default=0.50)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--max-det", type=int, default=50)
    parser.add_argument("--sample-count", type=int, default=4)
    parser.add_argument(
        "--sample-frames",
        type=int,
        nargs="*",
        default=[1, 25, 50, 100],
        help="1-based frame positions to include in the sample annotation panel.",
    )
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    evaluate(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
