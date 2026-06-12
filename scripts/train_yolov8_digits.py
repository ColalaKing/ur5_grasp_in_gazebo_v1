#!/usr/bin/env python3
"""Train the Gazebo digit-block detector with Ultralytics YOLOv8 on Apple MPS."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = ROOT / "ur5_ROS-Gazebo" / "yolo" / "data_yolov8.yaml"
DEFAULT_PROJECT = ROOT / "runs" / "detect"
RUNTIME_DATA_DIR = ROOT / "runs" / "tmp"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Train YOLOv8 for 0-9 Gazebo digit blocks on Apple Silicon MPS."
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=DEFAULT_DATA,
        help="Ultralytics data.yaml path.",
    )
    parser.add_argument(
        "--model",
        default="yolov8n.pt",
        help="YOLOv8 checkpoint, e.g. yolov8n.pt or yolov8s.pt.",
    )
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--name", default="yolov8_digits_mps")
    parser.add_argument("--seed", type=int, default=20260523)
    parser.add_argument(
        "--deploy-path",
        type=Path,
        default=ROOT / "ur5_ROS-Gazebo" / "yolo" / "weights" / "best.pt",
        help="Where to copy the trained best.pt for ROS inference.",
    )
    parser.add_argument(
        "--device",
        default="mps",
        help='Training device. Default is "mps"; use another value only intentionally.',
    )
    parser.add_argument(
        "--cache",
        action="store_true",
        help="Cache images in RAM. Faster, but can use more memory.",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Resume an interrupted Ultralytics run.",
    )
    parser.add_argument(
        "--no-extra-albumentations",
        action="store_true",
        help="Disable optional blur/brightness/contrast Albumentations transforms.",
    )
    return parser.parse_args()


def require_mps(device: str) -> None:
    if device != "mps":
        return

    import torch

    mps_backend = getattr(torch.backends, "mps", None)
    if mps_backend is None or not mps_backend.is_built() or not mps_backend.is_available():
        raise SystemExit(
            "MPS is not available in this Python/PyTorch environment. "
            "Install a recent PyTorch build for macOS Apple Silicon, or pass "
            "--device cpu only if you deliberately want CPU training."
        )

    # Force a tiny tensor allocation so failures are caught before training starts.
    _ = torch.ones(1, device="mps")
    print("Using Apple Silicon MPS acceleration: device=mps")


def optional_albumentations(disabled: bool):
    if disabled:
        return None
    try:
        import albumentations as A
    except ImportError:
        print(
            "albumentations is not installed; continuing with Ultralytics built-in "
            "HSV/geometric augmentation only."
        )
        return None

    return [
        A.Blur(blur_limit=(3, 5), p=0.08),
        A.RandomBrightnessContrast(
            brightness_limit=0.12,
            contrast_limit=0.12,
            p=0.25,
        ),
    ]


def write_absolute_image_list(source_list: Path, output_list: Path, dataset_root: Path) -> None:
    lines = []
    for raw_line in source_list.read_text().splitlines():
        item = raw_line.strip()
        if not item:
            continue
        path = Path(item)
        if not path.is_absolute():
            path = dataset_root / path
        lines.append(path.resolve().as_posix())
    if not lines:
        raise SystemExit(f"No image paths found in {source_list}")
    output_list.write_text("\n".join(lines) + "\n")


def prepare_data_yaml(data_path: Path) -> Path:
    data_path = data_path.resolve()
    if data_path != DEFAULT_DATA.resolve():
        try:
            import yaml
        except ImportError as exc:
            raise SystemExit(
                "PyYAML is required to prepare custom YOLOv8 data files: %s" % exc
            )

        config = yaml.safe_load(data_path.read_text()) or {}
        dataset_root = Path(config.get("path", data_path.parent))
        if not dataset_root.is_absolute():
            root_relative = (ROOT / dataset_root).resolve()
            yaml_relative = (data_path.parent / dataset_root).resolve()
            dataset_root = root_relative if root_relative.exists() else yaml_relative

        train_entry = config.get("train")
        val_entry = config.get("val")
        if not train_entry or not val_entry:
            raise SystemExit(f"Custom data.yaml must define train and val: {data_path}")
        if isinstance(train_entry, (list, tuple)) or isinstance(val_entry, (list, tuple)):
            raise SystemExit("This trainer expects train/val to be text-file lists.")

        train_list = Path(train_entry)
        val_list = Path(val_entry)
        if not train_list.is_absolute():
            train_list = dataset_root / train_list
        if not val_list.is_absolute():
            val_list = dataset_root / val_list
        if not train_list.exists() or not val_list.exists():
            raise SystemExit(
                "Expected split files are missing: "
                f"{train_list} and/or {val_list}"
            )

        RUNTIME_DATA_DIR.mkdir(parents=True, exist_ok=True)
        runtime_train = RUNTIME_DATA_DIR / f"{data_path.stem}_train_abs.txt"
        runtime_val = RUNTIME_DATA_DIR / f"{data_path.stem}_val_abs.txt"
        runtime_yaml = RUNTIME_DATA_DIR / f"{data_path.stem}_runtime.yaml"
        write_absolute_image_list(train_list, runtime_train, dataset_root)
        write_absolute_image_list(val_list, runtime_val, dataset_root)

        names = config.get("names") or {str(i): str(i) for i in range(10)}
        if isinstance(names, list):
            name_lines = [f'  {index}: "{name}"' for index, name in enumerate(names)]
        else:
            name_lines = [
                f'  {int(key)}: "{value}"'
                for key, value in sorted(
                    names.items(), key=lambda item: int(item[0])
                )
            ]
        runtime_yaml.write_text(
            "\n".join(
                [
                    f"path: {dataset_root.as_posix()}",
                    f"train: {runtime_train.as_posix()}",
                    f"val: {runtime_val.as_posix()}",
                    "test:",
                    "names:",
                ]
                + name_lines
                + [""]
            )
        )
        print(f"Using runtime data YAML: {runtime_yaml}")
        return runtime_yaml

    dataset_root = data_path.parent.resolve()
    train_list = dataset_root / "data" / "train_highres.txt"
    val_list = dataset_root / "data" / "valid_highres.txt"
    if not train_list.exists() or not val_list.exists():
        raise SystemExit(
            "Expected split files are missing: "
            f"{train_list} and/or {val_list}"
        )

    RUNTIME_DATA_DIR.mkdir(parents=True, exist_ok=True)
    runtime_train = RUNTIME_DATA_DIR / "train_highres_abs.txt"
    runtime_val = RUNTIME_DATA_DIR / "valid_highres_abs.txt"
    runtime_yaml = RUNTIME_DATA_DIR / "data_yolov8_runtime.yaml"
    write_absolute_image_list(train_list, runtime_train, dataset_root)
    write_absolute_image_list(val_list, runtime_val, dataset_root)
    runtime_yaml.write_text(
        "\n".join(
            [
                f"path: {dataset_root.as_posix()}",
                f"train: {runtime_train.as_posix()}",
                f"val: {runtime_val.as_posix()}",
                "test:",
                "names:",
                '  0: "0"',
                '  1: "1"',
                '  2: "2"',
                '  3: "3"',
                '  4: "4"',
                '  5: "5"',
                '  6: "6"',
                '  7: "7"',
                '  8: "8"',
                '  9: "9"',
                "",
            ]
        )
    )
    print(f"Using runtime data YAML: {runtime_yaml}")
    return runtime_yaml


def clean_checkpoint_args(args, device: str = "cpu") -> None:
    if args is None:
        return
    if isinstance(args, dict):
        args.pop("augmentations", None)
        args.pop("close_mosaic", None)
        args["device"] = device
        return
    for name in ("augmentations", "close_mosaic"):
        if hasattr(args, name):
            try:
                delattr(args, name)
            except Exception:
                setattr(args, name, None)
    if hasattr(args, "device"):
        setattr(args, "device", device)


def save_ros_inference_checkpoint(source_path: Path, deploy_path: Path) -> None:
    """Save a CPU inference checkpoint without training-only random state."""

    import torch

    checkpoint = torch.load(source_path, map_location="cpu", weights_only=False)
    model = checkpoint.get("ema") or checkpoint.get("model")
    if model is None:
        raise SystemExit(f"No model weights found in {source_path}")

    for candidate in (model, checkpoint.get("model"), checkpoint.get("ema")):
        if candidate is not None:
            clean_checkpoint_args(getattr(candidate, "args", None))

    checkpoint["model"] = model
    checkpoint["ema"] = None
    checkpoint["optimizer"] = None
    checkpoint["scaler"] = None
    checkpoint["train_results"] = {}
    checkpoint["train_metrics"] = {}
    clean_checkpoint_args(checkpoint.get("train_args"))

    deploy_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, deploy_path)


def train(args: argparse.Namespace) -> None:
    if not args.data.exists():
        raise SystemExit(f"data.yaml not found: {args.data}")

    require_mps(args.device)

    from ultralytics import YOLO

    data_yaml = prepare_data_yaml(args.data)
    model = YOLO(args.model)
    train_kwargs = {
        "data": str(data_yaml),
        "epochs": args.epochs,
        "imgsz": args.imgsz,
        "batch": args.batch,
        "device": args.device,
        "workers": args.workers,
        "project": str(args.project),
        "name": args.name,
        "seed": args.seed,
        "cache": args.cache,
        "resume": args.resume,
        "plots": True,
        "amp": False,
        # Mild color changes help with Gazebo lighting variance while preserving
        # block color cues that may be useful for distinguishing 6 and 9.
        "hsv_h": 0.005,
        "hsv_s": 0.35,
        "hsv_v": 0.30,
        # Keep geometry modest. The collected Gazebo data already randomizes yaw.
        "degrees": 5.0,
        "translate": 0.06,
        "scale": 0.15,
        "shear": 0.0,
        "perspective": 0.0005,
        # Do not flip digits; flips can create ambiguous or invalid glyphs.
        "fliplr": 0.0,
        "flipud": 0.0,
        "mosaic": 0.30,
        "close_mosaic": 10,
        "mixup": 0.0,
        "copy_paste": 0.0,
        "erasing": 0.0,
        "patience": 30,
    }

    extra_aug = optional_albumentations(args.no_extra_albumentations)
    if extra_aug is not None:
        train_kwargs["augmentations"] = extra_aug

    try:
        results = model.train(**train_kwargs)
    except (SyntaxError, TypeError, ValueError) as exc:
        message = str(exc).lower()
        if "augmentations" not in train_kwargs or "augmentation" not in message:
            raise
        print(
            "This Ultralytics version does not accept the custom 'augmentations' "
            "argument. Retrying with built-in augmentation only."
        )
        train_kwargs.pop("augmentations", None)
        results = model.train(**train_kwargs)

    save_dir = getattr(results, "save_dir", None)
    if save_dir:
        best_path = Path(save_dir) / "weights" / "best.pt"
        print(f"Training complete. Run directory: {save_dir}")
        print(f"Best weights: {best_path}")
        if best_path.exists() and args.deploy_path:
            training_full_path = args.deploy_path.with_name(
                f"{args.deploy_path.stem}_training_full{args.deploy_path.suffix}"
            )
            shutil.copy2(best_path, training_full_path)
            save_ros_inference_checkpoint(best_path, args.deploy_path)
            print(f"Saved ROS inference weights: {args.deploy_path}")
            print(f"Kept full training checkpoint: {training_full_path}")


def main() -> int:
    args = parse_args()
    train(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
