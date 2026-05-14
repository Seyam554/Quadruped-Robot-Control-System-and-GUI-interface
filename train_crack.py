"""
Crack Detection Training Script
Dataset  : Crack-Detection-10 (Roboflow, YOLOv8 OBB format)
Model    : YOLOv8s-OBB  (small, suits 4 GB VRAM)
GPU      : NVIDIA RTX 3050 Laptop 4 GB
"""

import subprocess, sys
from pathlib import Path

# ── install ultralytics if missing ────────────────────────────────────────────
try:
    import ultralytics
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "ultralytics"])
    import ultralytics

from ultralytics import YOLO
import torch

# ── paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent
DATA_YAML  = SCRIPT_DIR / "Crack-Detection-10" / "data.yaml"
PROJECT    = SCRIPT_DIR / "runs" / "crack"
RUN_NAME   = "yolov8s_obb"


def main():
    assert DATA_YAML.exists(), f"data.yaml not found at {DATA_YAML}"
    assert torch.cuda.is_available(), "CUDA not available — check PyTorch install"

    gpu_name = torch.cuda.get_device_name(0)
    vram_gb  = torch.cuda.get_device_properties(0).total_memory / 1024**3
    print(f"\nGPU  : {gpu_name}  ({vram_gb:.1f} GB VRAM)")
    print(f"Data : {DATA_YAML}")
    print(f"Out  : {PROJECT / RUN_NAME}\n")

    # yolov8s-obb.pt is pretrained on DOTAv1 (aerial oriented objects)
    model = YOLO("yolov8s-obb.pt")

    model.train(
        task          = "obb",
        data          = str(DATA_YAML),
        epochs        = 100,
        imgsz         = 640,
        batch         = 8,        # safe for 4 GB VRAM; raise to 16 if no OOM
        optimizer     = "AdamW",
        lr0           = 1e-3,
        lrf           = 0.01,
        weight_decay  = 5e-4,
        warmup_epochs = 3,
        patience      = 20,       # early-stop if no improvement for 20 epochs
        augment       = True,
        hsv_h         = 0.015,
        hsv_s         = 0.7,
        hsv_v         = 0.4,
        degrees       = 10.0,
        flipud        = 0.5,
        fliplr        = 0.5,
        project       = str(PROJECT),
        name          = RUN_NAME,
        exist_ok      = True,
        save_period   = 10,
        device        = 0,
        workers       = 4,
        verbose       = True,
    )

    # ── evaluate on test split ────────────────────────────────────────────────
    print("\n=== Evaluating on TEST set ===")
    best_weights = PROJECT / RUN_NAME / "weights" / "best.pt"
    model_best   = YOLO(str(best_weights))

    test_metrics = model_best.val(
        data   = str(DATA_YAML),
        task   = "obb",
        split  = "test",
        imgsz  = 640,
        device = 0,
    )

    print(f"\nmAP@0.50      : {test_metrics.box.map50:.4f}")
    print(f"mAP@0.50:0.95 : {test_metrics.box.map:.4f}")
    print(f"\nBest weights  : {best_weights}")
    print("Training complete.")


if __name__ == "__main__":
    main()
