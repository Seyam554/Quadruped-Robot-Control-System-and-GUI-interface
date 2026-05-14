"""
Crack detection inference worker.
Runs as a spawned subprocess so CUDA DLLs never share a process with Qt.
Communication: multiprocessing.Queue objects passed from CrackDetectorThread.
"""
import os
import sys
from pathlib import Path

# ── DLL search fix ────────────────────────────────────────────────────────────
# Root cause: the parent process PATH begins with CUDA v13.2\bin which provides
# cudart64_13.dll, but torch was built for CUDA 12.6 (requires cudart64_12.dll).
# torch\lib ships the correct cudart64_12.dll, so we must put it *first* in PATH
# and evict all system CUDA bin directories to prevent version mismatch.
#
# Also applies to any PyQt5 Qt DLL directories that may appear in PATH on some
# setups, which can conflict with torch's MSVC runtime dependencies.
if sys.platform == "win32":
    _TORCH_LIB = Path(sys.executable).parent.parent / "Lib" / "site-packages" / "torch" / "lib"
    if _TORCH_LIB.is_dir():
        try:
            os.add_dll_directory(str(_TORCH_LIB))
        except (OSError, AttributeError):
            pass

    _clean = []
    for _part in os.environ.get("PATH", "").split(os.pathsep):
        _low = _part.lower()
        # Strip system CUDA bin dirs (wrong cudart version) and Qt dirs
        if ("cuda" in _low and "nvidia gpu computing toolkit" in _low) or \
           "pyqt5" in _low or "\\qt\\" in _low or "/qt/" in _low:
            continue
        _clean.append(_part)
    # Put torch/lib at the very front so cudart64_12.dll resolves first
    if _TORCH_LIB.is_dir():
        _clean.insert(0, str(_TORCH_LIB))
    os.environ["PATH"] = os.pathsep.join(_clean)

WEIGHTS = Path(__file__).parent / "runs/crack/yolov8s_obb/weights/best.pt"

def _best_device():
    """Return 0 (GPU) if CUDA is available, otherwise 'cpu'."""
    try:
        import torch
        return 0 if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def inference_worker(in_queue, out_queue):
    """
    in_queue  : receives numpy BGR arrays (H,W,3) or None (sentinel)
    out_queue : sends ("ready",None) | ("result", dict) | ("error", str)
                dict = {"cracks": [(pts, conf), ...], "objects": [{"bbox":[x1,y1,x2,y2], "class":str, "conf":float}, ...]}
    """
    try:
        from ultralytics import YOLO
        import numpy as np
        import glob
        import os

        # Auto-detect latest segmentation model; fallback to global WEIGHTS OBB if none exists
        is_seg = False
        crack_pth = str(WEIGHTS)
        seg_runs = sorted(glob.glob("runs/segment/train*/weights/best.pt"), key=os.path.getmtime)
        if seg_runs:
            crack_pth = seg_runs[-1]
            is_seg = True

        model_crack = YOLO(crack_pth)
        obj_weights = Path(__file__).parent / "yolo26n.pt"
        if not obj_weights.exists():
            out_queue.put(("error", f"Missing object model: {obj_weights}"))
            return
        model_obj = YOLO(str(obj_weights))
        out_queue.put(("ready", None))
    except Exception as exc:
        out_queue.put(("error", str(exc)))
        return

    device = _best_device()
    while True:
        data = in_queue.get()
        if data is None:
            break
        item, conf = data
        try:
            # Run object detection first
            res_obj = model_obj.predict(item, conf=0.3, device=device, verbose=False)[0]
            obj_boxes = []
            if res_obj.boxes is not None:
                for box in res_obj.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype("int32").tolist()
                    cls_id = int(box.cls[0].item())
                    cls_name = res_obj.names[cls_id]
                    obj_conf = float(box.conf[0].item())
                    obj_boxes.append({"bbox": [x1, y1, x2, y2], "class": cls_name, "conf": obj_conf})

            # Run crack detection safely
            crack_boxes = []
            if is_seg:
                res_crack = model_crack.predict(item, task="segment", conf=conf, device=device, verbose=False)[0]
                if res_crack.masks is not None:
                    for i, seg in enumerate(res_crack.masks.xy):
                        pts = seg.astype("int32").tolist()
                        if len(pts) < 3: 
                            continue
                        c_conf = float(res_crack.boxes.conf[i] if res_crack.boxes is not None else 0.9)
                        cx = sum(p[0] for p in pts) / len(pts)
                        cy = sum(p[1] for p in pts) / len(pts)
                        
                        is_fp = False
                        for obj in obj_boxes:
                            ox1, oy1, ox2, oy2 = obj["bbox"]
                            if ox1 <= cx <= ox2 and oy1 <= cy <= oy2:
                                is_fp = True
                                break
                                
                        if not is_fp:
                            crack_boxes.append((pts, c_conf))
            else:
                res_crack = model_crack.predict(item, task="obb", conf=conf, device=device, verbose=False)[0]
                if res_crack.obb is not None:
                    for box in res_crack.obb:
                        pts  = box.xyxyxyxy[0].cpu().numpy().astype("int32").tolist()
                        c_conf = float(box.conf[0])
                        cx = sum(p[0] for p in pts) / 4.0
                        cy = sum(p[1] for p in pts) / 4.0
                        
                        is_fp = False
                        for obj in obj_boxes:
                            ox1, oy1, ox2, oy2 = obj["bbox"]
                            if ox1 <= cx <= ox2 and oy1 <= cy <= oy2:
                                is_fp = True
                                break
                        
                        if not is_fp:
                            crack_boxes.append((pts, c_conf))

            out_queue.put(("result", {"cracks": crack_boxes, "objects": obj_boxes}))
        except Exception as exc:
            out_queue.put(("error", str(exc)))
