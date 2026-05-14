"""
Live crack detection — tkinter display, no PyQt5, no cv2.imshow.
Run:  python crack_live.py [camera_index]
      python crack_live.py 0     (default)
      python crack_live.py 1
"""
import sys
import tkinter as tk
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageTk
from ultralytics import YOLO

WEIGHTS   = Path(__file__).parent / "runs/crack/yolov8s_obb/weights/best.pt"
CONF      = 0.75
BOX_COLOR = (0, 0, 255)   # BGR red
TXT_BG    = (0, 0, 0)
TARGET_W  = 960
TARGET_H  = 540


def draw_box(frame, pts_list, conf):
    pts = np.array(pts_list, dtype=np.int32)
    cv2.polylines(frame, [pts], isClosed=True, color=BOX_COLOR, thickness=2)
    label = f"{conf*100:.1f}%"
    x0, y0 = int(pts[:, 0].min()), int(pts[:, 1].min())
    (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    y0 = max(y0, th + bl + 2)
    cv2.rectangle(frame, (x0, y0 - th - bl), (x0 + tw + 2, y0 + bl), TXT_BG, -1)
    cv2.putText(frame, label, (x0, y0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, BOX_COLOR, 1, cv2.LINE_AA)


def main():
    cam_idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    assert WEIGHTS.exists(), f"Weights not found: {WEIGHTS}"

    print(f"Loading model …")
    model = YOLO(str(WEIGHTS))
    print("Model loaded.")

    cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"ERROR: Cannot open camera index {cam_idx}")
        sys.exit(1)
    print(f"Camera {cam_idx} opened. Close the window to quit.")

    root = tk.Tk()
    root.title(f"Crack Detection — Camera {cam_idx}")
    root.resizable(False, False)
    label = tk.Label(root)
    label.pack()

    running = True

    def on_close():
        nonlocal running
        running = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    def update():
        if not running:
            return
        ok, frame = cap.read()
        if not ok:
            root.after(50, update)
            return

        results = model.predict(frame, task="obb", conf=CONF, device=0, verbose=False)
        result  = results[0]

        if result.obb is not None:
            for box in result.obb:
                pts  = box.xyxyxyxy[0].cpu().numpy().astype("int32").tolist()
                conf = float(box.conf[0])
                draw_box(frame, pts, conf)

        n = len(result.obb) if result.obb is not None else 0
        cv2.putText(frame, f"Cracks: {n}", (8, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, BOX_COLOR, 2, cv2.LINE_AA)

        # Resize for display then convert BGR→RGB→PIL→ImageTk
        disp = cv2.resize(frame, (TARGET_W, TARGET_H))
        img  = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(disp, cv2.COLOR_BGR2RGB)))
        label.configure(image=img)
        label.image = img   # keep reference

        root.after(1, update)  # schedule next frame immediately

    root.after(0, update)
    root.mainloop()

    cap.release()
    print("Done.")


if __name__ == "__main__":
    main()
