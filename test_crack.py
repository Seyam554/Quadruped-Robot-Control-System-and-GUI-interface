"""
Crack Detection — Visual Test Script
Shows N random test images side-by-side:
  Left  : Ground truth OBB annotations
  Right : Model predictions with confidence %
Display uses matplotlib (works on Windows without GUI-OpenCV).
"""

import random
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import torch
from ultralytics import YOLO

# ── config ────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = Path(__file__).parent
WEIGHTS     = SCRIPT_DIR / "runs/crack/yolov8s_obb/weights/best.pt"
TEST_IMAGES = SCRIPT_DIR / "Crack-Detection-10/test/images"
TEST_LABELS = SCRIPT_DIR / "Crack-Detection-10/test/labels"
CONF_THRESH = 0.25   # minimum confidence to show a prediction box
N_SAMPLES   = 10     # number of random images to visualise

# BGR colours for OpenCV drawing
GT_COLOR   = (0, 255, 0)   # green  — ground truth
PRED_COLOR = (0, 0, 255)   # red    — prediction
TEXT_BG    = (0, 0, 0)


def obb_label_to_points(line: str, img_w: int, img_h: int) -> np.ndarray:
    """Parse one OBB label line (normalised) → pixel corner array (4,2)."""
    vals   = list(map(float, line.strip().split()))
    coords = vals[1:]   # class id removed; remaining: x1 y1 x2 y2 x3 y3 x4 y4
    pts = [(int(coords[i] * img_w), int(coords[i + 1] * img_h))
           for i in range(0, 8, 2)]
    return np.array(pts, dtype=np.int32)


def draw_obb(img, points, color, thickness=2):
    cv2.polylines(img, [points], isClosed=True, color=color, thickness=thickness)


def put_label(img, text, origin, color):
    (tw, th), bl = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
    x, y = origin
    cv2.rectangle(img, (x, y - th - bl), (x + tw + 2, y + bl), TEXT_BG, -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 1, cv2.LINE_AA)


def iou_obb(pts_a: np.ndarray, pts_b: np.ndarray) -> float:
    """IoU between two OBB quadrilaterals via convex polygon intersection."""
    pa = pts_a.reshape(-1, 1, 2).astype(np.float32)
    pb = pts_b.reshape(-1, 1, 2).astype(np.float32)
    # intersectConvexConvex returns (intersection_area_float, polygon_points)
    inter_area, _ = cv2.intersectConvexConvex(pa, pb)
    if inter_area == 0:
        return 0.0
    area_a = cv2.contourArea(pa)
    area_b = cv2.contourArea(pb)
    union  = area_a + area_b - inter_area
    return float(inter_area) / union if union > 0 else 0.0


def build_panel(img_path: Path, model: YOLO, iou_threshold: float = 0.5):
    """
    Returns (panel_rgb, tp, fp, fn).
    panel_rgb : HxWx3 uint8 RGB image — GT left, Prediction right.
    """
    img_bgr = cv2.imread(str(img_path))
    h, w    = img_bgr.shape[:2]
    gt_img  = img_bgr.copy()
    pr_img  = img_bgr.copy()

    # ── ground truth ──────────────────────────────────────────────────────────
    label_file = TEST_LABELS / (img_path.stem + ".txt")
    gt_boxes   = []
    if label_file.exists():
        for line in label_file.read_text().strip().splitlines():
            if line.strip():
                pts = obb_label_to_points(line, w, h)
                gt_boxes.append(pts)
                draw_obb(gt_img, pts, GT_COLOR, thickness=2)
        put_label(gt_img, f"GT: {len(gt_boxes)} crack(s)", (6, 22), GT_COLOR)
    else:
        put_label(gt_img, "GT: no label", (6, 22), GT_COLOR)

    cv2.putText(gt_img, "GROUND TRUTH", (6, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, GT_COLOR, 2, cv2.LINE_AA)

    # ── prediction ────────────────────────────────────────────────────────────
    results    = model.predict(str(img_path), task="obb", conf=CONF_THRESH,
                               device=0, verbose=False)
    result     = results[0]
    pred_boxes = []

    if result.obb is not None and len(result.obb) > 0:
        for box in result.obb:
            conf = float(box.conf[0])
            # xyxyxyxy returns PIXEL coordinates already — shape (1,4,2)
            pts_px = box.xyxyxyxy[0].cpu().numpy().astype(np.int32)  # (4,2)
            pred_boxes.append((pts_px, conf))
            draw_obb(pr_img, pts_px, PRED_COLOR, thickness=2)
            cx = int(pts_px[:, 0].min())
            cy = int(pts_px[:, 1].min()) - 4
            put_label(pr_img, f"{conf * 100:.1f}%", (max(cx, 0), max(cy, 14)),
                      PRED_COLOR)

    # ── IoU-based TP / FP / FN ───────────────────────────────────────────────
    matched_gt   = set()
    matched_pred = set()
    for pi, (pp, _) in enumerate(pred_boxes):
        for gi, gp in enumerate(gt_boxes):
            if gi in matched_gt:
                continue
            if iou_obb(pp, gp) >= iou_threshold:
                matched_gt.add(gi)
                matched_pred.add(pi)
                break

    tp = len(matched_pred)
    fp = len(pred_boxes) - tp
    fn = len(gt_boxes)   - tp
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0

    summary = (f"Det:{len(pred_boxes)}  TP:{tp} FP:{fp} FN:{fn}"
               f"  P:{precision*100:.0f}% R:{recall*100:.0f}% F1:{f1*100:.0f}%")
    put_label(pr_img, summary, (6, 22), PRED_COLOR)
    cv2.putText(pr_img, "PREDICTION", (6, h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, PRED_COLOR, 2, cv2.LINE_AA)

    # ── assemble side-by-side panel (BGR → RGB for matplotlib) ───────────────
    target_h = 480
    scale    = target_h / h
    new_w    = int(w * scale)
    gt_r  = cv2.cvtColor(cv2.resize(gt_img, (new_w, target_h)), cv2.COLOR_BGR2RGB)
    pr_r  = cv2.cvtColor(cv2.resize(pr_img, (new_w, target_h)), cv2.COLOR_BGR2RGB)
    div   = np.full((target_h, 4, 3), 200, dtype=np.uint8)
    panel = np.hstack([gt_r, div, pr_r])

    return panel, tp, fp, fn


def main():
    assert WEIGHTS.exists(), f"Weights not found: {WEIGHTS}"
    assert torch.cuda.is_available(), "CUDA not available"

    model    = YOLO(str(WEIGHTS))
    all_imgs = sorted(TEST_IMAGES.glob("*.jpg")) + sorted(TEST_IMAGES.glob("*.png"))
    assert all_imgs, f"No images in {TEST_IMAGES}"

    samples = random.sample(all_imgs, min(N_SAMPLES, len(all_imgs)))

    total_tp = total_fp = total_fn = 0
    panels   = []

    print(f"\nLoaded  : {WEIGHTS.name}")
    print(f"Testing : {len(samples)} random images from {len(all_imgs)} total\n")
    print(f"{'Image':<55} {'Det':>4}  TP  FP  FN   P%   R%  F1%")
    print("-" * 85)

    for img_path in samples:
        panel, tp, fp, fn = build_panel(img_path, model)
        panels.append((panel, img_path.name))
        total_tp += tp
        total_fp += fp
        total_fn += fn
        det = tp + fp
        p   = tp / (tp + fp) * 100 if (tp + fp) else 0
        r   = tp / (tp + fn) * 100 if (tp + fn) else 0
        f1  = 2 * p * r / (p + r)  if (p + r)   else 0
        print(f"{img_path.name:<55} {det:>4}  {tp:>2}  {fp:>2}  {fn:>2}"
              f"  {p:>4.0f}  {r:>4.0f}  {f1:>4.0f}")

    P  = total_tp / (total_tp + total_fp) * 100 if (total_tp + total_fp) else 0
    R  = total_tp / (total_tp + total_fn) * 100 if (total_tp + total_fn) else 0
    F1 = 2 * P * R / (P + R) if (P + R) else 0
    print("-" * 85)
    print(f"{'OVERALL':<55} {'':>4}  {total_tp:>2}  {total_fp:>2}  {total_fn:>2}"
          f"  {P:>4.0f}  {R:>4.0f}  {F1:>4.0f}")
    print(f"\nOverall Precision: {P:.1f}%  Recall: {R:.1f}%  F1: {F1:.1f}%\n")

    # ── display with matplotlib (Windows-safe) ────────────────────────────────
    gt_patch   = mpatches.Patch(color="lime",  label="Ground Truth")
    pred_patch = mpatches.Patch(color="red",   label="Prediction")

    for i, (panel, name) in enumerate(panels):
        fig, ax = plt.subplots(figsize=(14, 5))
        ax.imshow(panel)
        ax.axis("off")
        ax.set_title(f"[{i+1}/{len(panels)}]  {name}", fontsize=9, pad=4)
        fig.legend(handles=[gt_patch, pred_patch], loc="lower center",
                   ncol=2, fontsize=9, framealpha=0.7)
        plt.tight_layout()
        plt.show()   # blocks until window is closed

    print("Done.")


if __name__ == "__main__":
    main()
