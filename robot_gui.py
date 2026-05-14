#!/usr/bin/env python3
"""
Quadruped Robot Teleop GUI  —  TACTICAL EDITION  v2
====================================================
Pure PyQt5 desktop application. No HTML. No browser. No Chromium.

New in v2:
  - Appearance settings bar (font size +/−, dark/light mode toggle)
  - Laptop camera (OpenCV index 0) alongside ESP32-S3-CAM
  - Camera source toggle (switch between laptop / ESP32 feeds)
"""

import sys
import re
import ctypes
import socket
import subprocess
import time
from datetime import datetime
from pathlib import Path
from queue import Queue, Empty

import multiprocessing as mp
if mp.current_process().name != "MainProcess":
    # ── SUBPROCESS DLL FIX ──
    # Windows `spawn` unpickles the main module (robot_gui.py) in the child process.
    # If PyQt5 is imported first, it locks an older C++ runtime into process memory
    # which later causes torch's c10.dll to crash with [WinError 1114].
    # By forcing torch to load FIRST in the child, we ensure the newer DLLs are used.
    try:
        import torch
    except ImportError:
        pass


from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QFrame, QGroupBox,
    QSplitter, QSizePolicy, QButtonGroup, QSlider, QProgressBar
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QSize, QObject, QEvent
from PyQt5.QtGui import QFont, QColor, QImage, QPixmap, QPainter, QPen, QBrush

# ── optional deps ──────────────────────────────────────────────────────────────
try:
    import inputs as _inputs
    INPUTS_OK = True
except ImportError:
    INPUTS_OK = False

try:
    import requests
    import numpy as np
    import cv2
    CAM_OK = True
except ImportError:
    CAM_OK = False

# ══════════════════════════════════════════════════════════════════════════════
# CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════
ROBOT_PORT          = 5555
TX_HZ               = 50
HEARTBEAT_SEC       = 0.5
TOGGLE_DEBOUNCE_SEC = 0.30
STICK_MAX           = 32768.0
STICK_DEADZONE      = 0.20
TRIGGER_MAX         = 255.0
TRIGGER_DEADZONE    = 0.10
ARM_PACKETS_PER_SEC = 12

DEFAULT_ROBOT_IP = "192.168.181.2"
DEFAULT_CAM_URL  = "http://192.168.181.5:81/stream"
DEFAULT_AQM_IP   = "192.168.181.3"
AQM_PORT         = 5555

# ── colour palettes ────────────────────────────────────────────────────────────
_DARK = dict(
    bg="#060809",      panel="#0a0e0b",   panel2="#0e1410",
    border="#1c3020",  border2="#2c4830", border3="#3a6040",
    text="#9ecfaa",    dim="#3a5a40",     subdim="#253525",
    green="#39ff14",   amber="#ffb000",   red="#ff2200",
    blue="#00bfff",    yellow="#ffe500",  teal="#00e5cc",
    log_bg="#030508",  cam_bg="#040608",  input_bg="#070d08",
    grp_title="#00e5cc",
)
_LIGHT = dict(
    bg="#edf2eb",      panel="#dce8da",   panel2="#cfe0cc",
    border="#7aaa80",  border2="#5a9460", border3="#3a7045",
    text="#1a3a20",    dim="#608a66",     subdim="#b0cfb4",
    green="#0a7a1a",   amber="#b06000",   red="#cc1500",
    blue="#0050aa",    yellow="#7a6000",  teal="#006a58",
    log_bg="#d5e4d2",  cam_bg="#c8d8c5",  input_bg="#e5ede3",
    grp_title="#006a58",
)

# ══════════════════════════════════════════════════════════════════════════════
# HELPERS
# ══════════════════════════════════════════════════════════════════════════════
def apply_deadzone(v, dz):
    if abs(v) < dz:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - dz) / (1.0 - dz)


def walk_cmd(lx, ly):
    lx = apply_deadzone(lx, STICK_DEADZONE)
    ly = apply_deadzone(ly, STICK_DEADZONE)
    if abs(lx) < 0.01 and abs(ly) < 0.01:
        return "IDLE"
    if abs(ly) >= abs(lx):
        return "S" if ly > 0 else "W"
    return "D" if lx > 0 else "A"


def _xinput_any_connected() -> bool:
    """Return True if any XInput gamepad slot (0-3) is connected."""
    buf = (ctypes.c_byte * 16)()
    for lib in ("xinput1_4", "xinput1_3", "xinput9_1_0"):
        try:
            xi = getattr(ctypes.windll, lib)
            return any(xi.XInputGetState(i, buf) == 0 for i in range(4))
        except OSError:
            continue
    return False


# ══════════════════════════════════════════════════════════════════════════════
# BACKGROUND THREADS
# ══════════════════════════════════════════════════════════════════════════════
class GamepadThread(QThread):
    state_updated = pyqtSignal(dict)
    log_msg       = pyqtSignal(str, str)
    pad_found     = pyqtSignal(str)

    _ZERO = {
        "lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0,
        "lt": 0.0, "rt": 0.0, "lb": False, "rb": False,
        "a": False, "b": False, "x": False, "y": False,
        "dpad_x": 0, "dpad_y": 0, "back": False, "start": False,
        "l3": False, "r3": False,
    }

    def __init__(self):
        super().__init__()
        self._run  = True
        self.state = dict(self._ZERO)

    def stop(self): self._run = False

    def run(self):
        if not INPUTS_OK:
            self.log_msg.emit("ERROR", "inputs missing  →  pip install inputs"); return

        pad_announced = False
        while self._run:
            # Keep scanning until a gamepad appears (supports hot-plug)
            if not pad_announced:
                if not _xinput_any_connected():
                    self.log_msg.emit("WARN", "No gamepad — plug in F710 (X mode), retrying...")
                    time.sleep(2.0); continue
                # Gamepad detected — refresh inputs lib device list
                try:
                    _inputs.devices._find_devices()
                except Exception:
                    pass
                try:
                    pads = _inputs.devices.gamepads
                    name = str(pads[0]) if pads else "Logitech F710"
                except Exception:
                    name = "Logitech F710"
                # F710 in XInput mode enumerates as "Xbox 360" — show the real name
                if "x-box" in name.lower() or "xbox" in name.lower():
                    name = "Logitech F710"
                self.pad_found.emit(name)
                self.log_msg.emit("INFO", f"Gamepad: {name}")
                pad_announced = True

            try: events = _inputs.get_gamepad()
            except _inputs.UnpluggedError:
                self.log_msg.emit("WARN", "Gamepad unplugged — waiting for reconnect...")
                self.state = dict(self._ZERO); self.state_updated.emit(dict(self.state))
                pad_announced = False
                time.sleep(1.0); continue
            except Exception: time.sleep(0.05); continue

            changed = False
            for ev in events:
                c, s = ev.code, ev.state
                if   c == "ABS_X":      self.state["lx"]    = s / STICK_MAX;   changed = True
                elif c == "ABS_Y":      self.state["ly"]    = s / STICK_MAX;   changed = True
                elif c == "ABS_RX":     self.state["rx"]    = s / STICK_MAX;   changed = True
                elif c == "ABS_RY":     self.state["ry"]    = s / STICK_MAX;   changed = True
                elif c == "ABS_Z":      self.state["lt"]    = s / TRIGGER_MAX; changed = True
                elif c == "ABS_RZ":     self.state["rt"]    = s / TRIGGER_MAX; changed = True
                elif c == "ABS_HAT0X":  self.state["dpad_x"]= s;              changed = True
                elif c == "ABS_HAT0Y":  self.state["dpad_y"]= -s;             changed = True
                elif c == "BTN_SOUTH":  self.state["a"]     = bool(s);        changed = True
                elif c == "BTN_EAST":   self.state["b"]     = bool(s);        changed = True
                elif c == "BTN_WEST":   self.state["x"]     = bool(s);        changed = True
                elif c == "BTN_NORTH":  self.state["y"]     = bool(s);        changed = True
                elif c == "BTN_TL":     self.state["lb"]    = bool(s);        changed = True
                elif c == "BTN_TR":     self.state["rb"]    = bool(s);        changed = True
                elif c == "BTN_SELECT": self.state["back"]  = bool(s);        changed = True
                elif c == "BTN_START":  self.state["start"] = bool(s);        changed = True
                elif c == "BTN_THUMBL": self.state["l3"]    = bool(s);        changed = True
                elif c == "BTN_THUMBR": self.state["r3"]    = bool(s);        changed = True
            if changed: self.state_updated.emit(dict(self.state))


class ESP32CamThread(QThread):
    log_msg      = pyqtSignal(str, str)
    disconnected = pyqtSignal()

    def __init__(self, url):
        super().__init__()
        self.url          = url
        self._run         = True
        self.latest_frame = None   # written by thread, read by main-thread timer

    def stop(self): self._run = False

    def run(self):
        if not CAM_OK:
            self.log_msg.emit("ERROR", "requests/cv2/numpy missing"); self.disconnected.emit(); return

        while self._run:
            self.log_msg.emit("INFO", f"ESP32-CAM connecting  →  {self.url}")
            try:
                # timeout=(connect_s, read_s) — read timeout applies to every iter_content chunk
                resp = requests.get(self.url, stream=True, timeout=(5, 3))
                buf  = b""
                for chunk in resp.iter_content(chunk_size=32768):  # 32 KB — ~1 JPEG per read
                    if not self._run: break
                    buf += chunk
                    # Drain every complete JPEG present in the buffer before fetching more data
                    while True:
                        a = buf.find(b'\xff\xd8')
                        if a == -1:
                            buf = b""       # no SOI marker — discard garbage bytes
                            break
                        b_pos = buf.find(b'\xff\xd9', a + 2)  # search EOI only after SOI
                        if b_pos == -1:
                            buf = buf[a:]   # incomplete frame — keep from SOI and wait
                            break
                        jpg = buf[a: b_pos + 2]
                        buf = buf[b_pos + 2:]
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is not None:
                            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            h, w, ch = rgb.shape
                            if w > 640:
                                new_h = int(h * 640 / w)
                                rgb = cv2.resize(rgb, (640, new_h), interpolation=cv2.INTER_LINEAR)
                                h, w, ch = rgb.shape
                            qi = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
                            self.latest_frame = qi.copy()   # overwrite; timer picks latest
            except Exception as e:
                self.log_msg.emit("WARN", f"ESP32-CAM: {e}")

            if not self._run:
                break
            self.log_msg.emit("INFO", "ESP32-CAM stream ended — retrying in 3 s …")
            time.sleep(3)

        self.disconnected.emit()


class LaptopCamThread(QThread):
    log_msg      = pyqtSignal(str, str)
    disconnected = pyqtSignal()

    def __init__(self, idx=0):
        super().__init__()
        self.idx          = idx
        self._run         = True
        self.latest_frame = None   # written by thread, read by main-thread timer

    def stop(self): self._run = False

    def run(self):
        if not CAM_OK:
            self.log_msg.emit("ERROR", "cv2/numpy missing"); self.disconnected.emit(); return
        cap = cv2.VideoCapture(self.idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(self.idx)   # fallback without DSHOW
        if not cap.isOpened():
            self.log_msg.emit("ERROR", f"Laptop camera index {self.idx} not accessible.")
            self.disconnected.emit(); return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.log_msg.emit("INFO", f"Laptop cam #{self.idx} opened  ({int(cap.get(3))}×{int(cap.get(4))})")

        _DISP_W = 640   # downscale target width; keeps main-thread rendering cheap
        while self._run:
            ret, frame = cap.read()
            if not ret:
                self.log_msg.emit("WARN", "Laptop cam: no frame"); time.sleep(0.1); continue
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            if w > _DISP_W:
                new_h = int(h * _DISP_W / w)
                rgb = cv2.resize(rgb, (_DISP_W, new_h), interpolation=cv2.INTER_LINEAR)
                h, w, ch = rgb.shape
            qi = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
            self.latest_frame = qi.copy()   # overwrite; timer picks latest
            time.sleep(0.018)  # cap at ~55fps; gives GC breathing room between frames

        cap.release()
        self.log_msg.emit("INFO", "Laptop camera released.")
        self.disconnected.emit()


class AQMThread(QThread):
    """TCP client for the ESP32-C6 Air Quality Monitor (JSON lines on port 5555).

    Commands (e.g. LED_ON / LED_OFF) are queued via send_cmd() and written
    from inside the thread so only one thread ever touches the socket.
    """
    data_ready     = pyqtSignal(dict)
    log_msg        = pyqtSignal(str, str)
    status_changed = pyqtSignal(bool)   # True = connected

    def __init__(self, ip: str, port: int = AQM_PORT):
        super().__init__()
        self.ip     = ip
        self.port   = port
        self._run   = True
        self._cmd_q = Queue()   # main-thread → AQM-thread commands

    def stop(self): self._run = False

    def send_cmd(self, msg: str):
        """Thread-safe: queue a newline-terminated command for the AQM."""
        self._cmd_q.put(msg)

    def run(self):
        import json as _json
        while self._run:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                sock.connect((self.ip, self.port))
                sock.settimeout(0.2)   # short recv timeout so cmd_q is drained quickly
                self.status_changed.emit(True)
                self.log_msg.emit("INFO", f"AQM connected  →  {self.ip}:{self.port}")
                buf = ""
                while self._run:
                    # Send any queued commands before the next recv
                    while True:
                        try:
                            cmd = self._cmd_q.get_nowait()
                            sock.sendall((cmd + "\n").encode())
                            self.log_msg.emit("INFO", f"AQM ← {cmd}")
                        except Empty:
                            break
                        except Exception as send_err:
                            self.log_msg.emit("WARN", f"AQM send: {send_err}")

                    # Receive data; socket.timeout is normal when ESP32 is quiet
                    try:
                        chunk = sock.recv(2048).decode("utf-8", errors="ignore")
                        if not chunk:
                            break   # server closed connection
                        buf += chunk
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue
                            try:
                                self.data_ready.emit(_json.loads(line))
                            except _json.JSONDecodeError:
                                pass
                    except socket.timeout:
                        pass   # no data this 200 ms window — normal
                    except Exception:
                        break   # real error — reconnect
            except Exception as e:
                self.log_msg.emit("WARN", f"AQM: {e}")
            finally:
                if sock:
                    try: sock.close()
                    except: pass
            self.status_changed.emit(False)
            if self._run:
                time.sleep(3)


# ══════════════════════════════════════════════════════════════════════════════
# NETWORK MONITOR — periodic ping + WiFi signal, runs every 3 s
# ══════════════════════════════════════════════════════════════════════════════
class NetworkMonitorThread(QThread):
    stats_ready = pyqtSignal(float, int)   # (ping_ms, wifi_pct)  –1 = unavailable

    def __init__(self, ip: str):
        super().__init__()
        self.ip    = ip
        self._run  = True

    def stop(self): self._run = False

    def run(self):
        while self._run:
            ping_ms  = self._ping()
            wifi_pct = self._wifi()
            self.stats_ready.emit(ping_ms, wifi_pct)
            for _ in range(30):          # sleep 3 s in 100 ms slices for fast stop
                if not self._run: break
                time.sleep(0.1)

    def _ping(self) -> float:
        try:
            r = subprocess.run(
                ["ping", "-n", "1", "-w", "1000", self.ip],
                capture_output=True, text=True, timeout=3
            )
            m = re.search(r'[Tt]ime[=<](\d+)', r.stdout)
            if m:
                return float(m.group(1))
        except Exception:
            pass
        return -1.0

    def _wifi(self) -> int:
        try:
            r = subprocess.run(
                ["netsh", "wlan", "show", "interfaces"],
                capture_output=True, text=True, timeout=3
            )
            m = re.search(r'Signal\s*:\s*(\d+)%', r.stdout)
            if m:
                return int(m.group(1))
        except Exception:
            pass
        return -1


# ══════════════════════════════════════════════════════════════════════════════
# KEYBOARD EVENT FILTER
# Installed on QApplication so key presses are captured regardless of which
# widget has focus. Text input widgets (QLineEdit, QTextEdit) are excluded.
# ══════════════════════════════════════════════════════════════════════════════
class _KbdFilter(QObject):
    def __init__(self, callback):
        super().__init__()
        self._cb = callback

    def eventFilter(self, obj, event):
        t = event.type()
        if t in (QEvent.KeyPress, QEvent.KeyRelease):
            from PyQt5.QtWidgets import QLineEdit, QTextEdit
            fw = QApplication.focusWidget()
            if isinstance(fw, (QLineEdit, QTextEdit)):
                return False
            if not event.isAutoRepeat():
                self._cb(event.key(), t == QEvent.KeyPress)
        return False


# ══════════════════════════════════════════════════════════════════════════════
# CRACK STABILIZATION TRACKER
# Prevents crack bounding boxes from flickering by sustaining highly-confident
# detections across frames if the model briefly mis-detects them.
# ══════════════════════════════════════════════════════════════════════════════
class CrackTracker:
    def __init__(self):
        self.tracks = []  # [{"pts": list, "conf": float, "ttl": int}]

    def update(self, payload):
        new_cracks = payload.get("cracks", [])
        updated = []
        
        # 1. Register all fresh cracks from this frame directly.
        for pts, conf in new_cracks:
            # If the network is >= 80% confident, sustain the box for up to 15 frames 
            # (approx 1 - 1.5 seconds) if it briefly vanishes.
            ttl = 15 if conf >= 0.80 else 3
            updated.append({"pts": pts, "conf": conf, "ttl": ttl})
            
        # 2. Re-inject tracks that were lost this frame but still have a TTL reserve.
        for t in self.tracks:
            # Spatial matching check using basic Euclidean center bounds
            t_pts = t["pts"]
            tcx = sum(p[0] for p in t_pts) / 4.0
            tcy = sum(p[1] for p in t_pts) / 4.0
            
            matched = False
            for n_pts, n_conf in new_cracks:
                ncx = sum(p[0] for p in n_pts) / 4.0
                ncy = sum(p[1] for p in n_pts) / 4.0
                if ((tcx - ncx)**2 + (tcy - ncy)**2) < 2500:  # ~50 pixel radius
                    matched = True
                    break
                    
            if not matched and t["ttl"] > 1:
                updated.append({"pts": t["pts"], "conf": t["conf"], "ttl": t["ttl"] - 1})
                
        self.tracks = updated
        # Return a synthesized payload maintaining the background generic objects
        return {
            "cracks": [(t["pts"], t["conf"]) for t in self.tracks],
            "objects": payload.get("objects", [])
        }


# ══════════════════════════════════════════════════════════════════════════════
# CRACK DETECTION THREAD
# Inference runs in a *spawned subprocess* (crack_worker.py) so Qt DLLs and
# CUDA DLLs never share the same process address space on Windows.
# This thread only converts frames and draws boxes — no torch imported here.
# ══════════════════════════════════════════════════════════════════════════════
class CrackDetectorThread(QThread):
    frame_ready = pyqtSignal(QImage)
    log_msg     = pyqtSignal(str, str)

    _WEIGHTS = Path(__file__).parent / "runs/crack/yolov8s_obb/weights/best.pt"

    def __init__(self):
        super().__init__()
        self._q    = Queue(maxsize=2)
        self._run  = True
        self._proc = None
        self._in_q = None
        self._out_q = None
        self._conf = 0.25

    def set_conf(self, conf: float):
        self._conf = conf

    def push_frame(self, img: QImage):
        if self._q.full():
            try: self._q.get_nowait()
            except: pass
        self._q.put(img)

    def stop(self):
        self._run = False
        self._q.put(None)

    def run(self):
        import multiprocessing as mp

        if not self._WEIGHTS.exists():
            self.log_msg.emit("ERROR", f"Crack model not found: {self._WEIGHTS}")
            return

        try:
            from crack_worker import inference_worker
        except ImportError:
            self.log_msg.emit("ERROR", "crack_worker.py not found next to robot_gui.py")
            return

        # 'spawn' creates a fresh Python process that inherits the parent's
        # os.environ at the moment Process.start() is called.
        # Problem: system PATH begins with CUDA v13.2\bin (cudart64_13.dll) but
        # torch was built for CUDA 12.6 (needs cudart64_12.dll in torch\lib).
        # Fix: temporarily clean PATH in the parent so the child inherits it right.
        import os as _os
        _torch_lib = (Path(sys.executable).parent.parent /
                      "Lib" / "site-packages" / "torch" / "lib")
        _orig_path = _os.environ.get("PATH", "")
        _clean_parts = []
        for _p in _orig_path.split(_os.pathsep):
            _pl = _p.lower()
            if ("cuda" in _pl and "nvidia gpu computing toolkit" in _pl) or \
               "pyqt5" in _pl or "\\qt\\" in _pl:
                continue
            _clean_parts.append(_p)
        if _torch_lib.is_dir():
            _clean_parts.insert(0, str(_torch_lib))
        _os.environ["PATH"] = _os.pathsep.join(_clean_parts)

        ctx = mp.get_context("spawn")
        self._in_q  = ctx.Queue(maxsize=2)
        self._out_q = ctx.Queue()
        self._proc  = ctx.Process(
            target=inference_worker,
            args=(self._in_q, self._out_q),
            daemon=True,
        )
        self._proc.start()
        _os.environ["PATH"] = _orig_path   # restore parent PATH immediately

        # Wait up to 30 s for the model to load inside the subprocess
        try:
            kind, payload = self._out_q.get(timeout=30)
        except Exception as exc:
            self.log_msg.emit("ERROR", f"Crack model startup timeout: {exc}")
            self._proc.terminate()
            return
        if kind == "error":
            self.log_msg.emit("ERROR", f"Crack model: {payload}")
            self._proc.terminate()
            return
        self.log_msg.emit("INFO", "Crack detection model loaded (subprocess).")

        last_result   = {"cracks": [], "objects": []}   # most recent inference result
        tracker       = CrackTracker()
        infer_pending = False

        while self._run:
            try:
                img = self._q.get(timeout=0.1)
            except Empty:
                # drain any result that arrived while we were waiting
                try:
                    kind, payload = self._out_q.get_nowait()
                    if kind == "result":
                        last_result   = tracker.update(payload)
                        infer_pending = False
                    elif kind == "error":
                        self.log_msg.emit("ERROR", f"Crack inference: {payload}")
                        infer_pending = False
                except Empty:
                    pass
                continue

            if img is None:
                break

            try:
                img_rgb = img.convertToFormat(QImage.Format_RGB888)
                w, h    = img_rgb.width(), img_rgb.height()
                ptr     = img_rgb.bits(); ptr.setsize(h * w * 3)
                frame   = np.frombuffer(ptr, dtype=np.uint8).reshape((h, w, 3)).copy()
                bgr     = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Draw generic object bounding boxes (Neon Yellow)
                for obj in last_result.get("objects", []):
                    x1, y1, x2, y2 = obj["bbox"]
                    cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 255), 2)
                    lbl = f"{obj['class']} {obj['conf']*100:.1f}%"
                    (tw, th), bl = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(bgr, (x1, y1 - th - bl), (x1 + tw, y1), (0, 255, 255), -1)
                    cv2.putText(bgr, lbl, (x1, y1 - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

                # Draw last known crack boxes (Bright Magenta Segments)
                crack_boxes = last_result.get("cracks", [])
                
                if crack_boxes:
                    overlay = bgr.copy()
                    # 1. Fill the polygon masks onto a shadow buffer
                    for pts_list, conf in crack_boxes:
                        pts = np.array(pts_list, dtype=np.int32)
                        cv2.fillPoly(overlay, [pts], (255, 0, 255))
                        cv2.polylines(overlay, [pts], True, (255, 0, 255), 2)
                    
                    # 2. Composite with 40% transparency onto the active frame
                    cv2.addWeighted(overlay, 0.4, bgr, 0.6, 0, bgr)
                    
                    # 3. Draw text confidence overlays purely opaque
                    for pts_list, conf in crack_boxes:
                        pts = np.array(pts_list, dtype=np.int32)
                        cx  = max(int(pts[:, 0].min()), 0)
                        cy  = max(int(pts[:, 1].min()) - 4, 14)
                        lbl = f"{conf * 100:.1f}%"
                        (tw, th), bl = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                        cv2.rectangle(bgr, (cx, cy - th - bl), (cx + tw, cy + bl), (0, 0, 0), -1)
                        cv2.putText(bgr, lbl, (cx, cy),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 1, cv2.LINE_AA)

                n_det = len(crack_boxes)
                n_obj = len(last_result.get("objects", []))
                badge_col = (0, 220, 0) if n_det == 0 else (0, 80, 255)
                cv2.putText(bgr, f"CRACK DETECT: ON  |  {n_det} cracks, {n_obj} objs",
                            (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, badge_col, 2, cv2.LINE_AA)

                out = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                qi  = QImage(out.data, w, h, 3 * w, QImage.Format_RGB888)
                self.frame_ready.emit(qi.copy())

                # Send frame to subprocess only when it is free
                if not infer_pending:
                    try:
                        self._in_q.put_nowait((bgr.copy(), self._conf))
                        infer_pending = True
                    except Exception:
                        pass

                # Non-blocking check for new result
                try:
                    kind, payload = self._out_q.get_nowait()
                    if kind == "result":
                        last_result   = tracker.update(payload)
                        infer_pending = False
                    elif kind == "error":
                        self.log_msg.emit("ERROR", f"Crack inference: {payload}")
                        infer_pending = False
                except Empty:
                    pass

            except Exception as exc:
                self.log_msg.emit("ERROR", f"Crack frame: {exc}")
                self.frame_ready.emit(img)

        # ── clean up subprocess ────────────────────────────────────────────
        try:
            self._in_q.put(None)
        except Exception:
            pass
        if self._proc and self._proc.is_alive():
            self._proc.join(timeout=3)
            if self._proc.is_alive():
                self._proc.terminate()


# ══════════════════════════════════════════════════════════════════════════════
# CAMERA VIEW — fixed-size-hint label that rescales its frame on resize
# ══════════════════════════════════════════════════════════════════════════════
class _CamLabel(QLabel):
    def __init__(self):
        super().__init__()
        self._raw_pix = None
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(1, 1)

    def set_frame(self, img: QImage):
        self._raw_pix = QPixmap.fromImage(img)
        self._refresh()

    def _refresh(self):
        if self._raw_pix is None:
            return
        scaled = self._raw_pix.scaled(self.size(), Qt.KeepAspectRatio, Qt.FastTransformation)
        # bypass QLabel.setPixmap so sizeHint stays at our fixed value
        super().setPixmap(scaled)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh()

    def sizeHint(self):
        return QSize(640, 480)


# ══════════════════════════════════════════════════════════════════════════════
# CUSTOM WIDGETS
# ══════════════════════════════════════════════════════════════════════════════
class TacticalLED(QLabel):
    def __init__(self, label, color_on, fs_ref):
        """fs_ref: callable that returns current font size"""
        super().__init__()
        self._label    = label
        self._color_on = color_on
        self._active   = False
        self._fs_ref   = fs_ref
        self._bg_off     = "#0a0f0a"
        self._fg_off     = "#2e4f35"
        self._border_off = "#1c3020"
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.setMinimumWidth(52)
        self.setFixedHeight(30)
        self._paint()

    def set_active(self, v):
        if v != self._active:
            self._active = v; self._paint()

    def retheme(self, c, fs):
        self._bg_off     = c.get('panel',  '#0a0f0a')
        self._fg_off     = c.get('dim',    '#2e4f35')
        self._border_off = c.get('border', '#1c3020')
        self._paint()

    def _paint(self):
        fs = max(9, self._fs_ref() - 3)
        ff = '"Times New Roman", Times, serif'
        if self._active:
            self.setStyleSheet(f"""
                QLabel {{ background:{self._color_on}; color:#000; border:2px solid {self._color_on};
                          border-radius:4px; font-family:{ff};
                          font-size:{fs}px; font-weight:bold; letter-spacing:1px; }}""")
        else:
            self.setStyleSheet(f"""
                QLabel {{ background:{self._bg_off}; color:{self._fg_off}; border:2px solid {self._border_off};
                          border-radius:4px; font-family:{ff};
                          font-size:{fs}px; font-weight:bold; letter-spacing:1px; }}""")
        self.setText(self._label)


class StickViz(QWidget):
    def __init__(self, label):
        super().__init__()
        self._label = label
        self._x = 0.0; self._y = 0.0
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(60, 60)
        self.setMaximumSize(160, 160)
        self._green = "#39ff14"; self._dim = "#2e4f35"; self._border = "#2c4830"

    def set_values(self, x, y):
        self._x = x; self._y = y; self.update()

    def retheme(self, c, fs):
        self._green  = c["green"]
        self._dim    = c["dim"]
        self._border = c["border2"]
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h   = self.width(), self.height()
        cx, cy = w // 2, h // 2
        r      = min(cx, cy) - 8

        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor("#080c08")))
        p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)

        p.setPen(QPen(QColor(self._border), 2)); p.setBrush(Qt.NoBrush)
        p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)

        p.setPen(QPen(QColor(self._dim), 1))
        p.drawLine(cx - r + 4, cy, cx + r - 4, cy)
        p.drawLine(cx, cy - r + 4, cx, cy + r - 4)

        half = int(r * 0.5)
        p.setPen(QPen(QColor(self._dim), 1, Qt.DotLine))
        p.drawEllipse(cx - half, cy - half, 2 * half, 2 * half)

        dx = int(apply_deadzone(self._x, STICK_DEADZONE) * (r - 10))
        dy = int(apply_deadzone(self._y, STICK_DEADZONE) * (r - 10))
        dot = 7
        p.setPen(Qt.NoPen); p.setBrush(QBrush(QColor(self._green)))
        p.drawEllipse(cx + dx - dot, cy + dy - dot, dot * 2, dot * 2)

        p.setPen(QColor(self._dim))
        p.setFont(QFont("Times New Roman", 10, QFont.Bold))
        p.drawText(0, h - 18, w, 18, Qt.AlignCenter, self._label)


class ToggleSwitch(QPushButton):
    """Compact sliding toggle switch — drop-in replacement for CONNECT/DISCONNECT buttons."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setFixedSize(52, 24)
        self._col_on  = "#00e5cc"
        self._col_off = "#1c3020"

    def retheme(self, col_on, col_off):
        self._col_on = col_on; self._col_off = col_off; self.update()

    def sizeHint(self):
        return QSize(52, 24)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = h / 2
        p.setBrush(QBrush(QColor(self._col_on if self.isChecked() else self._col_off)))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(0, 0, w, h, r, r)
        m = 3
        tx = w - h + m if self.isChecked() else m
        p.setBrush(QBrush(QColor("#d0ead0")))
        p.drawEllipse(tx, m, h - 2 * m, h - 2 * m)


def _sep(c):
    f = QFrame(); f.setFrameShape(QFrame.HLine)
    f.setStyleSheet(f"background:{c['border']};max-height:1px;border:none;")
    return f


# ══════════════════════════════════════════════════════════════════════════════
# MAIN WINDOW
# ══════════════════════════════════════════════════════════════════════════════
class MainWindow(QMainWindow):

    _LOG_COLORS_DARK  = {"INFO":"#39ff14","TX":"#00bfff","WARN":"#ffb000",
                         "ERROR":"#ff2200","CONN":"#00ff88","DISC":"#ff4444",
                         "DEBUG":"#2e4f35","RX":"#00e5cc"}
    _LOG_COLORS_LIGHT = {"INFO":"#0a7a1a","TX":"#0050aa","WARN":"#b06000",
                         "ERROR":"#cc1500","CONN":"#0a6a1a","DISC":"#cc2200",
                         "DEBUG":"#608a66","RX":"#006a58"}

    def __init__(self):
        super().__init__()
        self.setWindowTitle("◈  QUADRUPED TELEOP — TACTICAL CONTROL SYSTEM  v2")
        self.setMinimumSize(1440, 900)

        # ── appearance state ──────────────────────────────────────────────
        self._fs        = 12       # base font size
        self._dark_mode = True
        self._colors    = dict(_DARK)

        # ── runtime state ─────────────────────────────────────────────────
        self._robot_ok   = False
        self._cam_source = "none"   # "laptop" | "esp32" | "none"
        self._sock       = None
        self._robot_ip   = ""       # cached at connect time — avoids widget read on every TX
        self._net_mon    : NetworkMonitorThread | None = None
        self._esp32_cam  : ESP32CamThread  | None = None
        self._laptop_cam : LaptopCamThread | None = None
        self._gpad_thread: GamepadThread   | None = None

        self._last_walk   = None;  self._last_walk_t = 0.0
        self._tog_t       = {};    self._arm_acc = {"lat":0.0,"sh":0.0,"elbow":0.0,"claw":0.0}
        self._gpad        = {};    self._last_tick_t = time.time()
        self._verbose     = False
        self._crack_thread: CrackDetectorThread | None = None
        self._crack_on    = False

        self._kbd_enabled = False
        self._kbd_pressed: set = set()
        self._kbd_filter  = _KbdFilter(self._on_kbd_key)
        self._z_active    = False
        self._p_active    = False

        self._aqm_thread: AQMThread | None = None
        self._aqm_rx     = 0
        self._aqm_led_on = False

        # collection of (widget, style_fn) for theme refreshing
        self._theme_targets = []   # list of callables with no args

        self._build_ui()
        self._apply_theme()
        self._start_gamepad()
        QApplication.instance().installEventFilter(self._kbd_filter)

        self._tx_timer = QTimer(self)
        self._tx_timer.timeout.connect(self._tx_tick)
        self._tx_timer.start(int(1000 / TX_HZ))

        self._cam_poll_timer = QTimer(self)
        self._cam_poll_timer.timeout.connect(self._poll_camera_frame)
        # started/stopped by _switch_to_* and _on_cam_disc

    # ── colour / font helpers ─────────────────────────────────────────────────
    def _c(self, key): return self._colors[key]
    def _fs_(self, delta=0): return max(10, self._fs + delta)

    def _log_color(self, level):
        m = self._LOG_COLORS_DARK if self._dark_mode else self._LOG_COLORS_LIGHT
        return m.get(level, self._c("text"))

    # ══════════════════════════════════════════════════════════════════════════
    # UI BUILD
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        root_w = QWidget(); self.setCentralWidget(root_w)
        root = QVBoxLayout(root_w)
        root.setContentsMargins(12, 10, 12, 8)
        root.setSpacing(6)

        root.addWidget(self._make_header())

        self._sep_header = QFrame(); self._sep_header.setFrameShape(QFrame.HLine)
        root.addWidget(self._sep_header)

        root.addWidget(self._make_settings_bar())

        self._sep_settings = QFrame(); self._sep_settings.setFrameShape(QFrame.HLine)
        root.addWidget(self._sep_settings)

        self._splitter = QSplitter(Qt.Horizontal)
        self._splitter.setHandleWidth(3)
        root.addWidget(self._splitter, 1)

        # LEFT
        left = QWidget(); lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 8, 0); lv.setSpacing(10)
        lv.addWidget(self._make_conn_panel())
        lv.addWidget(self._make_gamepad_panel(), 1)
        self._splitter.addWidget(left)

        # RIGHT  —  camera on top; AQM + Log side by side on bottom
        self._right_split = QSplitter(Qt.Vertical)
        self._right_split.setHandleWidth(6)
        self._right_split.addWidget(self._make_camera_panel())

        self._bottom_split = QSplitter(Qt.Horizontal)
        self._bottom_split.setHandleWidth(4)
        self._bottom_split.addWidget(self._make_aqm_panel())
        self._bottom_split.addWidget(self._make_log_panel())
        self._bottom_split.setSizes([720, 480])

        self._right_split.addWidget(self._bottom_split)
        self._right_split.setSizes([560, 300])
        self._splitter.addWidget(self._right_split)

        self._splitter.setSizes([380, 1100])

        # status bar
        self._sb_robot = QLabel("ROBOT: OFFLINE")
        self._sb_cam   = QLabel("CAM: OFFLINE")
        self._sb_tx    = QLabel("LAST TX: —")
        self._sb_ping  = QLabel("PING: —")
        self._sb_wifi  = QLabel("WiFi: —")
        self._sb_clock = QLabel("")
        self._sb_sep1  = QLabel("  |  ")
        self._sb_sep2  = QLabel("  |  ")
        self._sb_sep3  = QLabel("  |  ")
        self._sb_sep4  = QLabel("  |  ")
        self._sb_sep5  = QLabel("  |  ")
        for w, sep in [
            (self._sb_robot, self._sb_sep1),
            (self._sb_cam,   self._sb_sep2),
            (self._sb_tx,    self._sb_sep3),
            (self._sb_ping,  self._sb_sep4),
            (self._sb_wifi,  self._sb_sep5),
        ]:
            self.statusBar().addPermanentWidget(w)
            self.statusBar().addPermanentWidget(sep)
        self.statusBar().addPermanentWidget(self._sb_clock)
        self.statusBar().showMessage(
            "  QUADRUPED TACTICAL CONTROL SYSTEM  v2.0  |  F710 XINPUT"
        )
        clk = QTimer(self); clk.timeout.connect(self._tick_clock); clk.start(1000)
        self._tick_clock()

    def _tick_clock(self):
        self._sb_clock.setText(f"  {datetime.now().strftime('%H:%M:%S')}")

    # ── header ────────────────────────────────────────────────────────────────
    def _make_header(self):
        w = QWidget(); h = QHBoxLayout(w); h.setContentsMargins(0,0,0,0)
        self._title_lbl = QLabel("◈  QUADRUPED TACTICAL TELEOP  ◈")
        h.addWidget(self._title_lbl)
        h.addStretch()
        self._sub_lbl = QLabel("XIAO ESP32-S3  |  F710 XINPUT  |  UDP:5555")
        h.addWidget(self._sub_lbl)
        return w

    # ── appearance settings bar ───────────────────────────────────────────────
    def _make_settings_bar(self):
        self._settings_bar = QWidget()
        h = QHBoxLayout(self._settings_bar)
        h.setContentsMargins(4, 4, 4, 4)
        h.setSpacing(10)

        self._set_lbl = QLabel("⚙  APPEARANCE")
        h.addWidget(self._set_lbl)

        self._sep_v1 = QFrame(); self._sep_v1.setFrameShape(QFrame.VLine)
        h.addWidget(self._sep_v1)

        self._font_lbl = QLabel("FONT SIZE:")
        h.addWidget(self._font_lbl)

        self._btn_font_dec = QPushButton("  A−  ")
        self._btn_font_dec.setFixedWidth(70)
        self._btn_font_dec.clicked.connect(self._font_dec)
        h.addWidget(self._btn_font_dec)

        self._font_size_lbl = QLabel(f"  {self._fs}px  ")
        self._font_size_lbl.setAlignment(Qt.AlignCenter)
        self._font_size_lbl.setFixedWidth(60)
        h.addWidget(self._font_size_lbl)

        self._btn_font_inc = QPushButton("  A+  ")
        self._btn_font_inc.setFixedWidth(70)
        self._btn_font_inc.clicked.connect(self._font_inc)
        h.addWidget(self._btn_font_inc)

        self._sep_v2 = QFrame(); self._sep_v2.setFrameShape(QFrame.VLine)
        h.addWidget(self._sep_v2)

        self._mode_lbl = QLabel("THEME:")
        h.addWidget(self._mode_lbl)

        self._btn_mode = QPushButton("◑  DARK MODE")
        self._btn_mode.setCheckable(True)
        self._btn_mode.setChecked(False)
        self._btn_mode.clicked.connect(self._toggle_mode)
        h.addWidget(self._btn_mode)

        h.addStretch()
        return self._settings_bar

    def _font_inc(self):
        if self._fs < 24:
            self._fs += 1
            self._font_size_lbl.setText(f"  {self._fs}px  ")
            self._apply_theme()

    def _font_dec(self):
        if self._fs > 10:
            self._fs -= 1
            self._font_size_lbl.setText(f"  {self._fs}px  ")
            self._apply_theme()

    def _toggle_mode(self, checked):
        self._dark_mode = not checked   # checked = light mode (pressed = light)
        self._colors = dict(_DARK if self._dark_mode else _LIGHT)
        self._btn_mode.setText("☀  LIGHT MODE" if not self._dark_mode else "◑  DARK MODE")
        self._apply_theme()

    # ── connection panel ──────────────────────────────────────────────────────
    def _make_conn_panel(self):
        self._conn_grp = QGroupBox("[ CONNECTION MANAGER ]")
        v = QVBoxLayout(self._conn_grp)
        v.setSpacing(3); v.setContentsMargins(8, 6, 8, 6)

        def _ip_row(label_text, default_ip, port_str):
            h = QHBoxLayout(); h.setSpacing(4)
            sec = QLabel(label_text)
            sec.setFixedWidth(60)
            sec.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            h.addWidget(sec)
            ip = QLineEdit(default_ip)
            ip.setPlaceholderText("192.x.x.x")
            ip.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            h.addWidget(ip, 1)
            port = QLabel(port_str)
            port.setFixedWidth(46)
            h.addWidget(port)
            sw = ToggleSwitch()
            h.addWidget(sw)
            return h, sec, ip, port, sw

        # ── ROBOT ─────────────────────────────────────────────────────────
        rob_row, self._robot_sec_lbl, self._f_robot_ip, self._port_lbl, self._sw_robot = \
            _ip_row("ROBOT", DEFAULT_ROBOT_IP, f":{ROBOT_PORT}")
        self._sw_robot.toggled.connect(self._on_robot_toggle)
        v.addLayout(rob_row)

        self._lbl_robot_status = QLabel("●  OFFLINE")
        self._lbl_robot_status.setWordWrap(True)
        v.addWidget(self._lbl_robot_status)

        v.addSpacing(4)

        # ── CAMERA ────────────────────────────────────────────────────────
        cam_row = QHBoxLayout(); cam_row.setSpacing(4)
        self._cam_sec_lbl = QLabel("CAMERA")
        self._cam_sec_lbl.setFixedWidth(60)
        self._cam_sec_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        cam_row.addWidget(self._cam_sec_lbl)

        self._btn_src_laptop = QPushButton("💻 LAPTOP")
        self._btn_src_laptop.setCheckable(True)
        self._btn_src_laptop.setFixedHeight(24)
        self._btn_src_laptop.clicked.connect(self._on_src_laptop)
        self._btn_src_esp32 = QPushButton("📡 ESP32")
        self._btn_src_esp32.setCheckable(True)
        self._btn_src_esp32.setFixedHeight(24)
        self._btn_src_esp32.clicked.connect(self._on_src_esp32)
        cam_row.addWidget(self._btn_src_laptop)
        cam_row.addWidget(self._btn_src_esp32)
        cam_row.addStretch(1)
        self._sw_cam = ToggleSwitch()
        self._sw_cam.toggled.connect(self._on_cam_toggle)
        cam_row.addWidget(self._sw_cam)
        v.addLayout(cam_row)

        # camera sub-row: index + URL
        csub = QHBoxLayout(); csub.setSpacing(4)
        csub.addSpacing(64)
        self._cam_idx_lbl = QLabel("idx:")
        self._f_cam_idx = QLineEdit("0")
        self._f_cam_idx.setFixedWidth(34)
        self._f_cam_url = QLineEdit(DEFAULT_CAM_URL)
        self._f_cam_url.setPlaceholderText("http://x.x.x.x:81/stream")
        self._f_cam_url.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        csub.addWidget(self._cam_idx_lbl)
        csub.addWidget(self._f_cam_idx)
        csub.addWidget(self._f_cam_url, 1)
        v.addLayout(csub)

        self._lbl_cam_status = QLabel("●  OFFLINE")
        self._lbl_cam_status.setWordWrap(True)
        v.addWidget(self._lbl_cam_status)

        v.addSpacing(4)

        # ── AQM ───────────────────────────────────────────────────────────
        aqm_row, self._aqm_sec_lbl, self._f_aqm_ip, self._aqm_port_lbl, self._sw_aqm = \
            _ip_row("AQM", DEFAULT_AQM_IP, f":{AQM_PORT}")
        self._sw_aqm.toggled.connect(self._on_aqm_toggle)
        v.addLayout(aqm_row)

        self._lbl_aqm_status = QLabel("●  OFFLINE")
        self._lbl_aqm_status.setWordWrap(True)
        v.addWidget(self._lbl_aqm_status)

        v.addSpacing(4)

        # ── DISCONNECT ALL (compact) ───────────────────────────────────────
        disc_row = QHBoxLayout()
        disc_row.addStretch()
        self._btn_disc = QPushButton("■  DISC ALL")
        self._btn_disc.setFixedHeight(24)
        self._btn_disc.clicked.connect(self._disconnect_all)
        disc_row.addWidget(self._btn_disc)
        v.addLayout(disc_row)

        return self._conn_grp

    # ── gamepad panel ─────────────────────────────────────────────────────────
    def _make_gamepad_panel(self):
        self._gpad_grp = QGroupBox("[ GAMEPAD STATE — LOGITECH F710 ]")
        v = QVBoxLayout(self._gpad_grp); v.setSpacing(4); v.setContentsMargins(6, 4, 6, 4)

        self._lbl_pad = QLabel("⚠  NO GAMEPAD DETECTED")
        v.addWidget(self._lbl_pad)
        self._gsep1 = QFrame(); self._gsep1.setFrameShape(QFrame.HLine)
        v.addWidget(self._gsep1)

        # shoulder row
        sh = QHBoxLayout()
        self._led_lt = TacticalLED("LT", "#00bfff", lambda: self._fs)
        self._led_lb = TacticalLED("LB", "#00bfff", lambda: self._fs)
        self._led_y  = TacticalLED("Y",  "#ffe500", lambda: self._fs)
        self._led_rb = TacticalLED("RB", "#00bfff", lambda: self._fs)
        self._led_rt = TacticalLED("RT", "#00bfff", lambda: self._fs)
        for led in (self._led_lt, self._led_lb):       sh.addWidget(led)
        sh.addStretch(); sh.addWidget(self._led_y); sh.addStretch()
        for led in (self._led_rb, self._led_rt):        sh.addWidget(led)
        v.addLayout(sh)

        # face row
        fb = QHBoxLayout()
        self._led_back  = TacticalLED("BACK",  "#3a5a40", lambda: self._fs)
        self._led_x     = TacticalLED("X",     "#00bfff", lambda: self._fs)
        self._led_b     = TacticalLED("B",     "#ff2200", lambda: self._fs)
        self._led_a     = TacticalLED("A",     "#39ff14", lambda: self._fs)
        self._led_p     = TacticalLED("L3/P",  "#ff44aa", lambda: self._fs)
        self._led_start = TacticalLED("START", "#3a5a40", lambda: self._fs)
        fb.addWidget(self._led_back); fb.addStretch()
        fb.addWidget(self._led_x); fb.addWidget(self._led_b); fb.addWidget(self._led_a)
        fb.addStretch(); fb.addWidget(self._led_p); fb.addWidget(self._led_start)
        v.addLayout(fb)

        # d-pad + trigger %
        dp = QHBoxLayout()
        self._dpad_section_lbl = QLabel("D-PAD:")
        dp.addWidget(self._dpad_section_lbl)
        self._lbl_dpad = QLabel("○")
        dp.addWidget(self._lbl_dpad); dp.addStretch()
        self._lbl_lt_pct = QLabel("LT:   0%")
        self._lbl_rt_pct = QLabel("RT:   0%")
        dp.addWidget(self._lbl_lt_pct); dp.addSpacing(14); dp.addWidget(self._lbl_rt_pct)
        v.addLayout(dp)

        # sticks
        stk = QHBoxLayout()
        self._ls = StickViz("L-STICK"); self._rs = StickViz("R-STICK")
        stk.addStretch(1); stk.addWidget(self._ls); stk.addSpacing(16)
        stk.addWidget(self._rs); stk.addStretch(1)
        v.addLayout(stk)

        self._lbl_stk = QLabel("LS: (+0.00, +0.00)      RS: (+0.00, +0.00)")
        self._lbl_stk.setAlignment(Qt.AlignCenter)
        v.addWidget(self._lbl_stk)

        # ── axis invert ───────────────────────────────────────────────────
        inv_row = QHBoxLayout()
        self._inv_lbl = QLabel("INVERT:")
        inv_row.addWidget(self._inv_lbl)
        inv_row.addSpacing(6)
        self._btn_inv_lx = QPushButton("LX"); self._btn_inv_lx.setCheckable(True); self._btn_inv_lx.setFixedWidth(60)
        self._btn_inv_ly = QPushButton("LY"); self._btn_inv_ly.setCheckable(True); self._btn_inv_ly.setFixedWidth(60)
        self._btn_inv_rx = QPushButton("RX"); self._btn_inv_rx.setCheckable(True); self._btn_inv_rx.setFixedWidth(60)
        self._btn_inv_ry = QPushButton("RY"); self._btn_inv_ry.setCheckable(True); self._btn_inv_ry.setFixedWidth(60)
        for b in (self._btn_inv_lx, self._btn_inv_ly, self._btn_inv_rx, self._btn_inv_ry):
            inv_row.addWidget(b)
        inv_row.addStretch()
        v.addLayout(inv_row)

        self._gsep2 = QFrame(); self._gsep2.setFrameShape(QFrame.HLine)
        v.addWidget(self._gsep2)

        # active command
        cmd_row = QHBoxLayout()
        self._cmd_section_lbl = QLabel("ACTIVE CMD ▸")
        cmd_row.addWidget(self._cmd_section_lbl)
        self._lbl_cmd = QLabel("IDLE")
        cmd_row.addWidget(self._lbl_cmd); cmd_row.addStretch()
        v.addLayout(cmd_row)

        self._gsep3 = QFrame(); self._gsep3.setFrameShape(QFrame.HLine)
        v.addWidget(self._gsep3)

        # ── Z toggle row ──────────────────────────────────────────────────
        z_row = QHBoxLayout()
        self._z_sec_lbl = QLabel("B-BTN / Z-KEY ▸")
        self._btn_z = QPushButton("⚡  Z: OFF")
        self._btn_z.setCheckable(True)
        self._btn_z.clicked.connect(self._on_z_btn)
        z_row.addWidget(self._z_sec_lbl)
        z_row.addStretch()
        z_row.addWidget(self._btn_z)
        v.addLayout(z_row)

        # ── P (pause) toggle row ──────────────────────────────────────────
        p_row = QHBoxLayout()
        self._p_sec_lbl = QLabel("L3 / P-KEY ▸")
        self._btn_p = QPushButton("⏸  P: OFF")
        self._btn_p.setCheckable(True)
        self._btn_p.clicked.connect(self._on_p_btn)
        p_row.addWidget(self._p_sec_lbl)
        p_row.addStretch()
        p_row.addWidget(self._btn_p)
        v.addLayout(p_row)

        self._gsep4 = QFrame(); self._gsep4.setFrameShape(QFrame.HLine)
        v.addWidget(self._gsep4)

        # ── keyboard input section ────────────────────────────────────────
        kbd_hdr = QHBoxLayout()
        self._kbd_sec_lbl = QLabel("⌨  KEYBOARD INPUT")
        self._btn_kbd = QPushButton("⌨  KEYBOARD: OFF")
        self._btn_kbd.setCheckable(True)
        self._btn_kbd.clicked.connect(self._toggle_kbd)
        kbd_hdr.addWidget(self._kbd_sec_lbl)
        kbd_hdr.addStretch()
        kbd_hdr.addWidget(self._btn_kbd)
        v.addLayout(kbd_hdr)

        self._lbl_kbd_keys = QLabel("W  A  S  D")
        self._lbl_kbd_keys.setAlignment(Qt.AlignCenter)
        v.addWidget(self._lbl_kbd_keys)

        self._lbl_kbd_hint = QLabel()
        self._lbl_kbd_hint.setAlignment(Qt.AlignCenter)
        v.addWidget(self._lbl_kbd_hint)

        return self._gpad_grp

    # ── camera panel ──────────────────────────────────────────────────────────
    def _make_camera_panel(self):
        self._cam_grp = QGroupBox("[ CAMERA FEED ]")
        v = QVBoxLayout(self._cam_grp)
        self._cam_view = _CamLabel()
        self._cam_view.setText("NO FEED\n\nSelect a camera source and connect")
        v.addWidget(self._cam_view)

        crack_row = QHBoxLayout()
        self._btn_crack = QPushButton("🔍  CRACK DETECT: OFF")
        self._btn_crack.setCheckable(True)
        self._btn_crack.clicked.connect(self._toggle_crack_detection)
        crack_row.addWidget(self._btn_crack)
        
        crack_row.addSpacing(20)
        self._lbl_conf_title = QLabel("CONF:")
        self._slider_conf = QSlider(Qt.Horizontal)
        self._slider_conf.setRange(5, 95)
        self._slider_conf.setValue(25)  # 25%
        self._slider_conf.setFixedWidth(120)
        self._slider_conf.valueChanged.connect(self._on_conf_changed)
        self._lbl_conf_val = QLabel("25%")
        
        crack_row.addWidget(self._lbl_conf_title)
        crack_row.addWidget(self._slider_conf)
        crack_row.addWidget(self._lbl_conf_val)
        crack_row.addStretch()
        
        v.addLayout(crack_row)
        return self._cam_grp
        
    def _on_conf_changed(self, val):
        self._lbl_conf_val.setText(f"{val}%")
        if self._crack_on and self._crack_thread:
            self._crack_thread.set_conf(val / 100.0)

    # ── AQM panel ─────────────────────────────────────────────────────────────
    def _make_aqm_panel(self):
        self._aqm_grp = QGroupBox("[ AIR QUALITY MONITOR — ESP32-C6 ]")
        v = QVBoxLayout(self._aqm_grp)
        v.setSpacing(4); v.setContentsMargins(8, 6, 8, 6)

        self._lbl_aqm_conn = QLabel("⚠  NOT CONNECTED  —  set IP and press CONNECT AQM")
        v.addWidget(self._lbl_aqm_conn)

        def _make_bar():
            b = QProgressBar()
            b.setRange(0, 1000); b.setValue(0)
            b.setTextVisible(False); b.setFixedHeight(14)
            return b

        # ── AIR QUALITY grid: name | bar | value | ref ───────────────────
        self._lbl_aqm_air_hdr = QLabel("▸  AIR QUALITY")
        v.addWidget(self._lbl_aqm_air_hdr)

        air_grid = QGridLayout(); air_grid.setSpacing(3)
        air_grid.setColumnStretch(1, 1)

        def _gas_row(row, name, ref_text, grid):
            lbl_sym = QLabel(name);      lbl_sym.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            bar     = _make_bar()
            lbl_val = QLabel("—");       lbl_val.setAlignment(Qt.AlignLeft  | Qt.AlignVCenter)
            lbl_ref = QLabel(ref_text);  lbl_ref.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lbl_sym, row, 0)
            grid.addWidget(bar,     row, 1)
            grid.addWidget(lbl_val, row, 2)
            grid.addWidget(lbl_ref, row, 3)
            return lbl_sym, bar, lbl_val, lbl_ref

        self._aqm_o2_sym,  self._bar_aqm_o2,  self._lbl_aqm_o2,  self._aqm_o2_ref  = \
            _gas_row(0, "O2   Oxygen",      "norm 20.9%",    air_grid)
        self._aqm_co_sym,  self._bar_aqm_co,  self._lbl_aqm_co,  self._aqm_co_ref  = \
            _gas_row(1, "CO   Carbon Mon",  "lim 4.5 ppm",   air_grid)
        self._aqm_h2s_sym, self._bar_aqm_h2s, self._lbl_aqm_h2s, self._aqm_h2s_ref = \
            _gas_row(2, "H2S  Hydrogen S",  "lim 0.012 ppm", air_grid)
        self._aqm_lel_sym, self._bar_aqm_lel, self._lbl_aqm_lel, self._aqm_lel_ref = \
            _gas_row(3, "LEL  Combustible", "lim 1.2%",      air_grid)

        # raw ADC row
        self._aqm_raw_sym = QLabel("RAW  ADC")
        self._aqm_raw_sym.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self._bar_aqm_raw = _make_bar()
        self._lbl_aqm_raw = QLabel("—")
        self._lbl_aqm_raw.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._aqm_raw_ref = QLabel("12-bit 4095")
        self._aqm_raw_ref.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        air_grid.addWidget(self._aqm_raw_sym, 4, 0)
        air_grid.addWidget(self._bar_aqm_raw, 4, 1)
        air_grid.addWidget(self._lbl_aqm_raw, 4, 2)
        air_grid.addWidget(self._aqm_raw_ref, 4, 3)
        v.addLayout(air_grid)

        sep = QFrame(); sep.setFrameShape(QFrame.HLine); v.addWidget(sep)

        # ── CLIMATE grid ─────────────────────────────────────────────────
        self._lbl_aqm_clim_hdr = QLabel("▸  CLIMATE")
        v.addWidget(self._lbl_aqm_clim_hdr)

        clim_grid = QGridLayout(); clim_grid.setSpacing(3)
        clim_grid.setColumnStretch(1, 1)

        self._aqm_temp_sym, self._bar_aqm_temp, self._lbl_aqm_temp, self._aqm_temp_ref = \
            _gas_row(0, "Temp", "0 – 50°C",    clim_grid)
        self._aqm_hum_sym,  self._bar_aqm_hum,  self._lbl_aqm_hum,  self._aqm_hum_ref  = \
            _gas_row(1, "Hum%", "ideal 40-70%", clim_grid)
        v.addLayout(clim_grid)

        self._lbl_aqm_footer = QLabel("packets: 0  |  uptime: —")
        v.addWidget(self._lbl_aqm_footer)

        led_row = QHBoxLayout()
        self._btn_aqm_led = QPushButton("💡  LED: OFF")
        self._btn_aqm_led.setCheckable(True)
        self._btn_aqm_led.setFixedHeight(26)
        self._btn_aqm_led.clicked.connect(lambda _: self._toggle_aqm_led())
        led_row.addWidget(self._btn_aqm_led)
        led_row.addStretch()
        self._lbl_led_hint = QLabel("R3 / L-key")
        led_row.addWidget(self._lbl_led_hint)
        v.addLayout(led_row)

        return self._aqm_grp

    # ── log panel ─────────────────────────────────────────────────────────────
    def _make_log_panel(self):
        self._log_grp = QGroupBox("[ SYSTEM LOG  —  CLI DEBUG TERMINAL ]")
        v = QVBoxLayout(self._log_grp); v.setSpacing(6)

        self._log_box = QTextEdit()
        self._log_box.setReadOnly(True)
        self._log_box.document().setMaximumBlockCount(500)
        v.addWidget(self._log_box)

        bar = QHBoxLayout()
        self._btn_verbose = QPushButton("VERBOSE: OFF")
        self._btn_verbose.setCheckable(True)
        self._btn_verbose.toggled.connect(self._toggle_verbose)
        bar.addWidget(self._btn_verbose); bar.addStretch()
        self._btn_clr = QPushButton("CLR")
        self._btn_clr.setFixedWidth(80)
        self._btn_clr.clicked.connect(self._log_box.clear)
        bar.addWidget(self._btn_clr)
        v.addLayout(bar)
        return self._log_grp

    # ══════════════════════════════════════════════════════════════════════════
    # THEMING ENGINE
    # ══════════════════════════════════════════════════════════════════════════
    def _apply_theme(self):
        c  = self._colors
        fs = self._fs

        mono = '"Times New Roman", Times, serif'

        # ── global window QSS ────────────────────────────────────────────
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background: {c['bg']};
                color: {c['text']};
                font-family: {mono};
            }}
            QSplitter::handle {{
                background: {c['border2']};
            }}
            QSplitter::handle:horizontal {{
                width: 4px;
                background: {c['border2']};
            }}
            QSplitter::handle:vertical {{
                height: 8px;
                background: qlineargradient(
                    x1:0, y1:0, x2:0, y2:1,
                    stop:0 {c['bg']},
                    stop:0.4 {c['border2']},
                    stop:0.6 {c['border2']},
                    stop:1 {c['bg']}
                );
            }}
            QScrollBar:vertical {{
                background: {c['panel']}; width: 8px; border-radius: 4px; margin: 0;
            }}
            QScrollBar::handle:vertical {{
                background: {c['border2']}; border-radius: 4px; min-height: 20px;
            }}
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
            QStatusBar {{
                background: {c['panel']};
                border-top: 2px solid {c['border2']};
                color: {c['dim']};
                font-family: {mono};
                font-size: {max(10,fs-3)}px;
            }}
        """)

        # ── settings bar ────────────────────────────────────────────────
        sb_style = f"""
            QWidget {{ background:{c['panel']}; border-bottom:1px solid {c['border']}; }}
        """
        self._settings_bar.setStyleSheet(sb_style)

        sep_style = f"background:{c['border2']};max-height:1px;border:none;"
        self._sep_header.setStyleSheet(sep_style)
        self._sep_settings.setStyleSheet(sep_style)

        set_lbl_style = f"color:{c['teal']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:3px;"
        self._set_lbl.setStyleSheet(set_lbl_style)
        for lbl in (self._font_lbl, self._mode_lbl):
            lbl.setStyleSheet(f"color:{c['dim']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:1px;")

        self._font_size_lbl.setStyleSheet(
            f"color:{c['green']};font-family:{mono};font-size:{fs}px;font-weight:bold;"
        )

        def _settings_btn(btn):
            btn.setStyleSheet(f"""
                QPushButton {{
                    background:{c['panel2']}; border:2px solid {c['border2']};
                    border-radius:4px; color:{c['text']};
                    font-family:{mono}; font-size:{fs}px; font-weight:bold;
                    letter-spacing:1px; padding:4px 8px;
                }}
                QPushButton:hover  {{ background:{c['panel']}; border-color:{c['green']}; color:{c['green']}; }}
                QPushButton:pressed {{ background:{c['green']}; color:#000; }}
                QPushButton:checked {{ background:{c['green']}; color:#000; border-color:{c['green']}; }}
            """)
        for b in (self._btn_font_dec, self._btn_font_inc, self._btn_mode):
            _settings_btn(b)

        for f in (self._sep_v1, self._sep_v2):
            f.setStyleSheet(f"background:{c['border']};max-width:1px;border:none;")

        # ── header ──────────────────────────────────────────────────────
        self._title_lbl.setStyleSheet(f"""
            color:{c['green']}; font-family:{mono};
            font-size:{fs+7}px; font-weight:bold; letter-spacing:6px;
        """)
        self._sub_lbl.setStyleSheet(f"""
            color:{c['dim']}; font-family:{mono};
            font-size:{max(10,fs-3)}px; letter-spacing:3px;
        """)

        # ── group box style ──────────────────────────────────────────────
        grp_qss = f"""
            QGroupBox {{
                border:2px solid {c['border2']}; border-radius:6px; margin-top:26px;
                padding:10px 8px 6px 8px;
                font-family:{mono}; font-size:{max(13,fs+4)}px; font-weight:bold;
                color:{c['grp_title']}; letter-spacing:6px;
                background:{c['panel']};
            }}
            QGroupBox::title {{
                subcontrol-origin:margin; subcontrol-position:top left;
                left:12px; top:2px;
                background:{c['panel']}; padding:0 8px;
            }}
        """
        for grp in (self._conn_grp, self._gpad_grp, self._cam_grp, self._aqm_grp, self._log_grp):
            grp.setStyleSheet(grp_qss)

        # ── input fields ────────────────────────────────────────────────
        inp_qss = f"""
            QLineEdit {{
                background:{c['input_bg']}; border:2px solid {c['border']};
                border-radius:4px; color:{c['green']};
                font-family:{mono}; font-size:{fs}px; font-weight:bold;
                padding:7px 10px;
            }}
            QLineEdit:focus {{ border-color:{c['teal']}; }}
        """
        for inp in (self._f_robot_ip, self._f_cam_url, self._f_cam_idx, self._f_aqm_ip):
            inp.setStyleSheet(inp_qss)

        for lbl in (self._port_lbl, self._aqm_port_lbl):
            lbl.setStyleSheet(f"color:{c['dim']};font-family:{mono};font-size:{fs}px;")

        # ── all main action buttons ──────────────────────────────────────
        def _action_btn(btn, border_col=None, text_col=None):
            bc = border_col or c['border2']
            tc = text_col   or c['text']
            btn.setStyleSheet(f"""
                QPushButton {{
                    background:{c['panel2']}; border:2px solid {bc};
                    border-radius:4px; color:{tc};
                    font-family:{mono}; font-size:{fs}px; font-weight:bold;
                    letter-spacing:1px; padding:4px 10px;
                }}
                QPushButton:hover  {{ background:{c['panel']}; border-color:{c['green']}; color:{c['green']}; }}
                QPushButton:pressed {{ background:{c['green']}; color:#000; }}
                QPushButton:checked {{ background:{c['teal']}; color:#000; border-color:{c['teal']}; }}
            """)

        _action_btn(self._btn_disc, c['red'], c['red'])
        _action_btn(self._btn_src_laptop)
        _action_btn(self._btn_src_esp32)
        for sw in (self._sw_robot, self._sw_cam, self._sw_aqm):
            sw.retheme(c['teal'], c['border'])
        _action_btn(self._btn_verbose, c['dim'],  c['dim'])
        _action_btn(self._btn_clr,     c['dim'],  c['dim'])
        _action_btn(self._btn_crack,   c['teal'], c['teal'])

        # ── section/helper labels ────────────────────────────────────────
        sec_lbl_qss = f"color:{c['teal']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:1px;"
        for lbl in (self._robot_sec_lbl, self._cam_sec_lbl, self._aqm_sec_lbl):
            lbl.setStyleSheet(sec_lbl_qss)
        for lbl in (self._port_lbl, self._aqm_port_lbl):
            lbl.setStyleSheet(f"color:{c['dim']};font-family:{mono};font-size:{fs}px;")

        dim_lbl_qss = f"color:{c['dim']};font-family:{mono};font-size:{max(10,fs-2)}px;font-weight:bold;"
        for lbl in (self._dpad_section_lbl, self._cmd_section_lbl, self._cam_idx_lbl, self._lbl_conf_title):
            lbl.setStyleSheet(dim_lbl_qss)

        # Z toggle section
        self._z_sec_lbl.setStyleSheet(
            f"color:{c['dim']};font-family:{mono};font-size:{max(10,fs-2)}px;font-weight:bold;"
        )
        _action_btn(self._btn_z, c['teal'], c['teal'])

        # P toggle section
        self._p_sec_lbl.setStyleSheet(
            f"color:{c['dim']};font-family:{mono};font-size:{max(10,fs-2)}px;font-weight:bold;"
        )
        _action_btn(self._btn_p, c['teal'], c['teal'])

        # Invert axis buttons
        self._inv_lbl.setStyleSheet(dim_lbl_qss)
        for b in (self._btn_inv_lx, self._btn_inv_ly, self._btn_inv_rx, self._btn_inv_ry):
            _action_btn(b, c['border2'], c['dim'])

        # keyboard section
        self._kbd_sec_lbl.setStyleSheet(
            f"color:{c['amber']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:3px;"
        )
        _action_btn(self._btn_kbd, c['amber'], c['amber'])
        hint_fs  = max(10, fs - 2)
        hint_html = (
            f'<table cellpadding="3" cellspacing="0" align="center" '
            f'style="font-family:\'Times New Roman\',Times,serif;font-size:{hint_fs}px;">'
            f'<tr>'
            f'<td align="right" style="color:{c["teal"]};font-weight:bold;padding-right:6px;">WASD / ↑↓←→</td>'
            f'<td style="color:{c["dim"]};">Walk</td><td width="20"></td>'
            f'<td align="right" style="color:{c["teal"]};font-weight:bold;padding-right:6px;">PgUp / PgDn</td>'
            f'<td style="color:{c["dim"]};">Speed</td>'
            f'</tr><tr>'
            f'<td align="right" style="color:{c["teal"]};font-weight:bold;padding-right:6px;">Z</td>'
            f'<td style="color:{c["dim"]};">Toggle</td><td></td>'
            f'<td align="right" style="color:{c["teal"]};font-weight:bold;padding-right:6px;">P</td>'
            f'<td style="color:{c["dim"]};">Pause</td>'
            f'</tr><tr>'
            f'<td align="right" style="color:{c["teal"]};font-weight:bold;padding-right:6px;">L</td>'
            f'<td style="color:{c["dim"]};">AQM LED</td><td></td>'
            f'<td align="right" style="color:{c["teal"]};font-weight:bold;padding-right:6px;">1 – 8</td>'
            f'<td style="color:{c["dim"]};">Arm joints</td>'
            f'</tr></table>'
        )
        self._lbl_kbd_hint.setText(hint_html)
        self._lbl_kbd_hint.setStyleSheet(f"font-family:{mono};")
        self._lbl_kbd_keys.setStyleSheet(
            f"font-family:{mono};font-size:{fs+4}px;letter-spacing:6px;padding:4px;"
        )
        self._update_kbd_display()

        # AQM panel
        aqm_mono  = f"font-family:{mono};font-size:{fs}px;font-weight:bold;"
        dim_sm    = f"color:{c['dim']};font-family:{mono};font-size:{max(9,fs-1)}px;font-weight:bold;"
        hdr_style = f"color:{c['teal']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:1px;"
        bar_base  = (f"QProgressBar{{background:{c['panel2']};border:1px solid {c['border']};"
                     f"border-radius:2px;}}"
                     f"QProgressBar::chunk{{background:{c['dim']};border-radius:2px;}}")
        self._lbl_aqm_conn.setStyleSheet(f"color:{c['amber']};{aqm_mono}letter-spacing:1px;")
        self._lbl_aqm_footer.setStyleSheet(f"color:{c['dim']};{aqm_mono}")
        self._lbl_aqm_status.setStyleSheet(f"color:{c['red']};{aqm_mono}")
        self._lbl_aqm_air_hdr.setStyleSheet(hdr_style)
        self._lbl_aqm_clim_hdr.setStyleSheet(hdr_style)
        for lbl in (self._aqm_o2_sym,  self._aqm_co_sym,  self._aqm_h2s_sym, self._aqm_lel_sym,
                    self._aqm_raw_sym, self._aqm_temp_sym, self._aqm_hum_sym,
                    self._aqm_o2_ref,  self._aqm_co_ref,  self._aqm_h2s_ref, self._aqm_lel_ref,
                    self._aqm_raw_ref, self._aqm_temp_ref, self._aqm_hum_ref):
            lbl.setStyleSheet(dim_sm)
        for lbl in (self._lbl_aqm_o2, self._lbl_aqm_co, self._lbl_aqm_h2s, self._lbl_aqm_lel,
                    self._lbl_aqm_raw, self._lbl_aqm_temp, self._lbl_aqm_hum):
            lbl.setStyleSheet(f"color:{c['dim']};{aqm_mono}")
        for bar in (self._bar_aqm_o2, self._bar_aqm_co, self._bar_aqm_h2s, self._bar_aqm_lel,
                    self._bar_aqm_raw, self._bar_aqm_temp, self._bar_aqm_hum):
            bar.setStyleSheet(bar_base)
        _action_btn(self._btn_aqm_led, c['yellow'], c['yellow'])
        self._lbl_led_hint.setStyleSheet(dim_sm)

        self._lbl_conf_val.setStyleSheet(f"color:{c['teal']};font-family:{mono};font-size:{fs}px;font-weight:bold;")
        
        # ── slider ──────────────────────────────────────────────────────
        self._slider_conf.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                border: 1px solid {c['border2']};
                height: 8px;
                background: {c['input_bg']};
                margin: 2px 0;
                border-radius: 4px;
            }}
            QSlider::handle:horizontal {{
                background: {c['teal']};
                border: 1px solid {c['teal']};
                width: 14px;
                margin: -4px 0;
                border-radius: 7px;
            }}
            QSlider::handle:horizontal:hover {{
                background: {c['green']};
            }}
            QSlider::sub-page:horizontal {{
                background: {c['border3']};
                border-radius: 4px;
            }}
        """)

        self._lbl_pad.setStyleSheet(f"color:{c['amber']};font-family:{mono};font-size:{max(10,fs-2)}px;")
        self._lbl_stk.setStyleSheet(f"color:{c['dim']};font-family:{mono};font-size:{max(10,fs-2)}px;")
        self._lbl_lt_pct.setStyleSheet(f"color:{c['blue']};font-family:{mono};font-size:{max(10,fs-2)}px;")
        self._lbl_rt_pct.setStyleSheet(f"color:{c['blue']};font-family:{mono};font-size:{max(10,fs-2)}px;")
        self._lbl_dpad.setStyleSheet(f"color:{c['amber']};font-family:{mono};font-size:{fs+6}px;font-weight:bold;padding-left:6px;")

        # gamepad panel horizontal separators
        gsep_qss = f"background:{c['border2']};max-height:1px;border:none;"
        for gs in (self._gsep1, self._gsep2, self._gsep3, self._gsep4):
            gs.setStyleSheet(gsep_qss)

        # CMD display
        cmd_col = c['green'] if self._lbl_cmd.text() == "IDLE" else c['amber']
        self._lbl_cmd.setStyleSheet(f"""
            color:{cmd_col}; font-family:{mono};
            font-size:{fs+4}px; font-weight:bold; letter-spacing:6px; padding-left:12px;
        """)

        # camera view
        self._cam_view.setStyleSheet(f"""
            background:{c['cam_bg']}; border:2px solid {c['border']};
            border-radius:4px; color:{c['dim']};
            font-family:{mono}; font-size:{fs}px; letter-spacing:3px;
        """)

        # log box
        self._log_box.setStyleSheet(f"""
            QTextEdit {{
                background:{c['log_bg']}; border:2px solid {c['border']};
                border-radius:4px; color:{c['green']};
                font-family:{mono}; font-size:{max(10,fs-1)}px;
                selection-background-color:{c['panel2']}; padding:4px;
            }}
        """)

        # status labels
        for lbl in (self._sb_robot, self._sb_cam, self._sb_tx,
                    self._sb_ping, self._sb_wifi, self._sb_clock):
            lbl.setStyleSheet(f"color:{c['dim']};font-family:{mono};font-size:{max(10,fs-3)}px;")
        sep_sb_qss = f"color:{c['border2']};font-family:{mono};font-size:{max(10,fs-3)}px;"
        for lbl in (self._sb_sep1, self._sb_sep2, self._sb_sep3, self._sb_sep4, self._sb_sep5):
            lbl.setStyleSheet(sep_sb_qss)

        # status-specific state colours (preserve connection state)
        if self._robot_ok:
            self._lbl_robot_status.setStyleSheet(f"color:{c['green']};font-family:{mono};font-size:{fs}px;font-weight:bold;")
        else:
            self._lbl_robot_status.setStyleSheet(f"color:{c['red']};font-family:{mono};font-size:{fs}px;font-weight:bold;")

        cam_active = self._cam_source != "none"
        if cam_active:
            self._lbl_cam_status.setStyleSheet(f"color:{c['green']};font-family:{mono};font-size:{fs}px;font-weight:bold;")
        else:
            self._lbl_cam_status.setStyleSheet(f"color:{c['red']};font-family:{mono};font-size:{fs}px;font-weight:bold;")

        # retheme custom widgets
        for led in (self._led_lt, self._led_lb, self._led_y, self._led_rb,
                    self._led_rt, self._led_back, self._led_x, self._led_b,
                    self._led_a, self._led_p, self._led_start):
            led.retheme(c, fs)

        self._ls.retheme(c, fs)
        self._rs.retheme(c, fs)

        # font_size label
        self._font_size_lbl.setText(f"  {self._fs}px  ")

    # ══════════════════════════════════════════════════════════════════════════
    # LOGGING
    # ══════════════════════════════════════════════════════════════════════════
    def _log(self, level, msg):
        if level == "DEBUG" and not self._verbose: return
        ts    = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = self._log_color(level)
        tc    = self._c("text")
        dim   = self._c("dim")
        html  = (f'<span style="color:{dim};">[{ts}]</span>'
                 f'<span style="color:{color};font-weight:bold;"> [{level:<5}] </span>'
                 f'<span style="color:{tc};">{msg}</span>')
        self._log_box.append(html)
        sb = self._log_box.verticalScrollBar(); sb.setValue(sb.maximum())

    def _toggle_verbose(self, on):
        self._verbose = on
        self._btn_verbose.setText(f"VERBOSE: {'ON' if on else 'OFF'}")
        self._log("INFO", f"Verbose {'ENABLED' if on else 'DISABLED'}.")

    # ══════════════════════════════════════════════════════════════════════════
    # CAMERA CONTROL
    # ══════════════════════════════════════════════════════════════════════════
    def _stop_all_cams(self):
        self._cam_poll_timer.stop()
        if self._esp32_cam and self._esp32_cam.isRunning():
            self._esp32_cam.stop(); self._esp32_cam.wait(2000)
        if self._laptop_cam and self._laptop_cam.isRunning():
            self._laptop_cam.stop(); self._laptop_cam.wait(2000)
        self._esp32_cam  = None
        self._laptop_cam = None

    def _switch_to_laptop(self):
        self._btn_src_laptop.setChecked(True)
        self._btn_src_esp32.setChecked(False)
        self._stop_all_cams()
        if not CAM_OK:
            self._log("ERROR", "cv2/numpy missing for laptop cam.")
            return
        try:
            cam_idx = int(self._f_cam_idx.text().strip())
        except ValueError:
            cam_idx = 0
            self._f_cam_idx.setText("0")
        self._laptop_cam = LaptopCamThread(cam_idx)
        self._laptop_cam.log_msg.connect(self._log)
        self._laptop_cam.disconnected.connect(self._on_cam_disc)
        self._laptop_cam.start()
        self._cam_poll_timer.start(33)   # 30 fps poll
        self._cam_source = "laptop"
        self._cam_grp.setTitle("[ CAMERA FEED  —  💻 LAPTOP ]")
        self._lbl_cam_status.setText("●  LAPTOP ACTIVE")
        self._sw_cam.blockSignals(True); self._sw_cam.setChecked(True); self._sw_cam.blockSignals(False)
        self._lbl_cam_status.setStyleSheet(
            f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;"
            f"font-size:{self._fs}px;font-weight:bold;"
        )
        self._sb_cam.setText("CAM: LAPTOP"); self._sb_cam.setStyleSheet(
            f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;font-size:{max(10,self._fs-3)}px;"
        )
        self._log("CONN", "Laptop camera #0 started.")

    def _switch_to_esp32(self):
        self._btn_src_esp32.setChecked(True)
        self._btn_src_laptop.setChecked(False)
        self._stop_all_cams()
        url = self._f_cam_url.text().strip()
        if not url:
            self._log("ERROR", "ESP32 Camera URL is empty."); return
        self._esp32_cam = ESP32CamThread(url)
        self._esp32_cam.log_msg.connect(self._log)
        self._esp32_cam.disconnected.connect(self._on_cam_disc)
        self._esp32_cam.start()
        self._cam_poll_timer.start(33)   # 30 fps poll
        self._cam_source = "esp32"
        self._cam_grp.setTitle("[ CAMERA FEED  —  📡 ESP32-CAM ]")
        self._lbl_cam_status.setText("●  ESP32 STREAMING")
        self._sw_cam.blockSignals(True); self._sw_cam.setChecked(True); self._sw_cam.blockSignals(False)
        self._lbl_cam_status.setStyleSheet(
            f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;"
            f"font-size:{self._fs}px;font-weight:bold;"
        )
        self._sb_cam.setText("CAM: ESP32"); self._sb_cam.setStyleSheet(
            f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;font-size:{max(10,self._fs-3)}px;"
        )
        self._log("CONN", f"ESP32 camera → {url}")

    def _on_cam_disc(self):
        self._cam_poll_timer.stop()
        self._cam_source = "none"
        self._btn_src_laptop.setChecked(False)
        self._btn_src_esp32.setChecked(False)
        self._sw_cam.blockSignals(True); self._sw_cam.setChecked(False); self._sw_cam.blockSignals(False)
        self._cam_grp.setTitle("[ CAMERA FEED ]")
        self._lbl_cam_status.setText("●  OFFLINE")
        self._lbl_cam_status.setStyleSheet(
            f"color:{self._c('red')};font-family:'Times New Roman',Times,serif;"
            f"font-size:{self._fs}px;font-weight:bold;"
        )
        self._sb_cam.setText("CAM: OFFLINE"); self._sb_cam.setStyleSheet(
            f"color:{self._c('red')};font-family:'Times New Roman',Times,serif;font-size:{max(10,self._fs-3)}px;"
        )
        self._cam_view.clear()
        self._cam_view.setText("FEED LOST\n\nSelect a camera source to reconnect")
        self._log("DISC", "Camera feed disconnected.")

    def _poll_camera_frame(self):
        """Called by _cam_poll_timer every 33 ms. Reads the latest frame from
        whichever camera thread is active and routes it for display.
        Only the most recent frame is ever in flight — no queue buildup."""
        img = None
        if self._cam_source == "laptop" and self._laptop_cam:
            img = self._laptop_cam.latest_frame
            self._laptop_cam.latest_frame = None
        elif self._cam_source == "esp32" and self._esp32_cam:
            img = self._esp32_cam.latest_frame
            self._esp32_cam.latest_frame = None
        if img is not None:
            self._on_frame(img)

    def _on_frame(self, img: QImage):
        if self._crack_on and self._crack_thread:
            # Route through crack detector — it draws boxes and emits frame_ready
            # (frame_ready is connected to cam_view.set_frame in _toggle_crack_detection)
            self._crack_thread.push_frame(img)
        else:
            self._cam_view.set_frame(img)      # pass raw frame directly

    def _toggle_crack_detection(self, checked: bool):
        if checked:
            if not CAM_OK:
                self._log("ERROR", "cv2/numpy required for crack detection.")
                self._btn_crack.setChecked(False)
                return
            if self._cam_source == "none":
                self._log("WARN", "No camera active — start a camera feed first, then enable crack detection.")
                self._btn_crack.setChecked(False)
                return
            self._crack_thread = CrackDetectorThread()
            self._crack_thread.frame_ready.connect(self._cam_view.set_frame)
            self._crack_thread.log_msg.connect(self._log)
            self._crack_thread.finished.connect(self._on_crack_thread_done)
            self._crack_thread.start()
            self._crack_on = True
            self._btn_crack.setText("🔍  CRACK DETECT: ON  ●")
            self._log("INFO", "Crack detection ENABLED — loading model in subprocess...")
            self._log("INFO", "  GPU will be used if CUDA is available, otherwise CPU.")
        else:
            if self._crack_thread:
                self._crack_thread.frame_ready.disconnect()
                self._crack_thread.stop()
                self._crack_thread.wait(2000)
                self._crack_thread = None
            self._crack_on = False
            self._btn_crack.setText("🔍  CRACK DETECT: OFF")
            self._log("INFO", "Crack detection disabled.")

    def _on_crack_thread_done(self):
        if self._crack_on:
            self._crack_on = False
            self._crack_thread = None
            self._btn_crack.setChecked(False)
            self._btn_crack.setText("🔍  CRACK DETECT: OFF")
            self._log("WARN", "Crack detection stopped — model failed to load.")

    # ══════════════════════════════════════════════════════════════════════════
    # KEYBOARD CONTROL
    # ══════════════════════════════════════════════════════════════════════════
    _KBD_DIRECT = {
        Qt.Key_1: "1", Qt.Key_2: "2", Qt.Key_3: "3", Qt.Key_4: "4",
        Qt.Key_5: "5", Qt.Key_6: "6", Qt.Key_7: "7", Qt.Key_8: "8",
        Qt.Key_PageUp: "SPD_UP", Qt.Key_PageDown: "SPD_DN",
        Qt.Key_Z: "Z", Qt.Key_P: "P",
    }
    _KBD_MOVE = {Qt.Key_W, Qt.Key_Up, Qt.Key_S, Qt.Key_Down,
                 Qt.Key_A, Qt.Key_Left, Qt.Key_D, Qt.Key_Right}

    def _toggle_kbd(self, checked: bool):
        self._kbd_enabled = checked
        self._btn_kbd.setText("⌨  KEYBOARD: ON  ●" if checked else "⌨  KEYBOARD: OFF")
        if not checked:
            self._kbd_pressed.clear()
            self._update_kbd_display()
        self._log("INFO", f"Keyboard control {'ENABLED — WASD/↑↓←→=walk, Z=toggle, 1-8=arm, PgUp/Dn=speed' if checked else 'DISABLED'}.")

    def _on_z_btn(self, checked: bool):
        self._z_active = checked
        self._btn_z.setText("⚡  Z: ON  ●" if checked else "⚡  Z: OFF")
        self._try_toggle("Z_BTN", "Z")

    def _sync_z_toggle(self):
        self._z_active = not self._z_active
        self._btn_z.setChecked(self._z_active)
        self._btn_z.setText("⚡  Z: ON  ●" if self._z_active else "⚡  Z: OFF")

    def _on_p_btn(self, checked: bool):
        self._p_active = checked
        self._btn_p.setText("⏸  P: ON  ●" if checked else "⏸  P: OFF")
        self._try_toggle("P_BTN", "P")

    def _sync_p_toggle(self):
        self._p_active = not self._p_active
        self._btn_p.setChecked(self._p_active)
        self._btn_p.setText("⏸  P: ON  ●" if self._p_active else "⏸  P: OFF")

    def _toggle_aqm_led(self):
        now = time.time()
        if now - self._tog_t.get("AQM_LED", 0.0) < TOGGLE_DEBOUNCE_SEC:
            # Revert the Qt auto-toggle so button stays in sync
            self._btn_aqm_led.blockSignals(True)
            self._btn_aqm_led.setChecked(self._aqm_led_on)
            self._btn_aqm_led.blockSignals(False)
            return
        self._tog_t["AQM_LED"] = now
        if self._aqm_thread and self._aqm_thread.isRunning():
            self._aqm_thread.send_cmd("L")   # firmware toggles on 'L'/'l'
            # Button state is synced from "led" field in the next JSON packet
        else:
            self._log("WARN", "AQM not connected — LED command not sent.")
            # Revert Qt auto-toggle — nothing actually happened
            self._btn_aqm_led.blockSignals(True)
            self._btn_aqm_led.setChecked(self._aqm_led_on)
            self._btn_aqm_led.blockSignals(False)
            self._btn_aqm_led.setText("💡  LED: ON  ●" if self._aqm_led_on else "💡  LED: OFF")

    def _on_kbd_key(self, key: int, pressed: bool):
        # L key toggles AQM LED regardless of keyboard mode
        if pressed and key == Qt.Key_L:
            self._toggle_aqm_led()
            return
        if not self._kbd_enabled:
            return
        if pressed:
            self._kbd_pressed.add(key)
            if key in self._KBD_DIRECT:
                self._send_reliable(self._KBD_DIRECT[key])
                if key == Qt.Key_Z:
                    self._sync_z_toggle()
                if key == Qt.Key_P:
                    self._sync_p_toggle()
        else:
            self._kbd_pressed.discard(key)
        self._update_kbd_display()

    def _update_kbd_display(self):
        keys = self._kbd_pressed
        gc = self._c("green"); dc = self._c("dim")

        def col(k1, k2=None):
            return gc if (k1 in keys or (k2 is not None and k2 in keys)) else dc

        wc = col(Qt.Key_W, Qt.Key_Up)
        ac = col(Qt.Key_A, Qt.Key_Left)
        sc = col(Qt.Key_S, Qt.Key_Down)
        dc_ = col(Qt.Key_D, Qt.Key_Right)
        html = (f'<span style="color:{wc};font-weight:bold;font-family:Times New Roman;">W</span>'
                f'&nbsp;&nbsp;'
                f'<span style="color:{ac};font-weight:bold;font-family:Times New Roman;">A</span>'
                f'&nbsp;&nbsp;'
                f'<span style="color:{sc};font-weight:bold;font-family:Times New Roman;">S</span>'
                f'&nbsp;&nbsp;'
                f'<span style="color:{dc_};font-weight:bold;font-family:Times New Roman;">D</span>')
        self._lbl_kbd_keys.setText(html)

    # ══════════════════════════════════════════════════════════════════════════
    # TOGGLE SWITCH HANDLERS
    # ══════════════════════════════════════════════════════════════════════════
    def _on_robot_toggle(self, checked: bool):
        if checked:
            self._connect_robot()
            if not self._robot_ok:  # connect failed — revert switch
                self._sw_robot.blockSignals(True)
                self._sw_robot.setChecked(False)
                self._sw_robot.blockSignals(False)
        else:
            self._disconnect_robot()

    def _disconnect_robot(self):
        if self._net_mon and self._net_mon.isRunning():
            self._net_mon.stop(); self._net_mon.wait(500)
        self._net_mon = None
        self._sb_ping.setText("PING: —"); self._sb_wifi.setText("WiFi: —")
        if self._sock:
            try: self._send_raw("IDLE")
            except: pass
            try: self._sock.close()
            except: pass
            self._sock = None
        self._robot_ok = False
        self._robot_ip = ""
        mono = "'Times New Roman',Times,serif"
        self._lbl_robot_status.setText("●  OFFLINE")
        self._lbl_robot_status.setStyleSheet(
            f"color:{self._c('red')};font-family:{mono};font-size:{self._fs}px;font-weight:bold;"
        )
        self._sb_robot.setText("ROBOT: OFFLINE")
        self._sb_robot.setStyleSheet(
            f"color:{self._c('red')};font-family:{mono};font-size:{max(10,self._fs-3)}px;"
        )

    def _on_src_laptop(self):
        self._switch_to_laptop()
        self._sw_cam.blockSignals(True)
        self._sw_cam.setChecked(True)
        self._sw_cam.blockSignals(False)

    def _on_src_esp32(self):
        self._switch_to_esp32()
        self._sw_cam.blockSignals(True)
        self._sw_cam.setChecked(True)
        self._sw_cam.blockSignals(False)

    def _on_cam_toggle(self, checked: bool):
        if checked:
            src = self._cam_source
            if src == "laptop":
                self._switch_to_laptop()
            elif src == "esp32":
                self._switch_to_esp32()
            else:
                self._log("WARN", "Select 💻 or 📡 camera source first.")
                self._sw_cam.blockSignals(True)
                self._sw_cam.setChecked(False)
                self._sw_cam.blockSignals(False)
        else:
            self._stop_all_cams()
            self._on_cam_disc()

    def _on_aqm_toggle(self, checked: bool):
        if checked:
            self._connect_aqm()
        else:
            self._disconnect_aqm()

    # ══════════════════════════════════════════════════════════════════════════
    # ROBOT CONNECTION
    # ══════════════════════════════════════════════════════════════════════════
    def _connect_robot(self):
        ip = self._f_robot_ip.text().strip()
        if not ip: self._log("ERROR", "Robot IP empty."); return
        try:
            if self._sock:
                try: self._sock.close()
                except: pass
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(1.0)
            self._sock.sendto(b"IDLE", (ip, ROBOT_PORT))
            self._robot_ok  = True
            self._robot_ip  = ip          # cache — used by _send_raw on every TX
            self._lbl_robot_status.setText("●  ONLINE")
            self._lbl_robot_status.setStyleSheet(
                f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;"
                f"font-size:{self._fs}px;font-weight:bold;"
            )
            self._sw_robot.blockSignals(True)
            self._sw_robot.setChecked(True)
            self._sw_robot.blockSignals(False)
            self._sb_robot.setText("ROBOT: ONLINE")
            self._sb_robot.setStyleSheet(
                f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;"
                f"font-size:{max(10,self._fs-3)}px;"
            )
            self._log("CONN", f"Robot ONLINE  →  {ip}:{ROBOT_PORT}")
            # Start network monitor (ping + WiFi signal every 3 s)
            if self._net_mon and self._net_mon.isRunning():
                self._net_mon.stop(); self._net_mon.wait(1000)
            self._net_mon = NetworkMonitorThread(ip)
            self._net_mon.stats_ready.connect(self._on_net_stats)
            self._net_mon.start()
        except Exception as e:
            self._log("ERROR", f"Robot connect failed: {e}")

    def _connect_both(self):
        self._connect_robot()
        self._switch_to_esp32()

    def _disconnect_all(self):
        self._stop_all_cams()
        self._on_cam_disc()
        if self._net_mon and self._net_mon.isRunning():
            self._net_mon.stop(); self._net_mon.wait(500)
        self._net_mon = None
        self._sb_ping.setText("PING: —"); self._sb_wifi.setText("WiFi: —")
        if self._sock:
            try: self._send_raw("IDLE")
            except: pass
            try: self._sock.close()
            except: pass
            self._sock = None
        self._robot_ok = False
        self._robot_ip = ""
        mono = "'Times New Roman',Times,serif"
        self._lbl_robot_status.setText("●  OFFLINE")
        self._lbl_robot_status.setStyleSheet(
            f"color:{self._c('red')};font-family:{mono};font-size:{self._fs}px;font-weight:bold;"
        )
        self._sb_robot.setText("ROBOT: OFFLINE")
        self._sb_robot.setStyleSheet(
            f"color:{self._c('red')};font-family:{mono};font-size:{max(10,self._fs-3)}px;"
        )
        for sw in (self._sw_robot, self._sw_cam, self._sw_aqm):
            sw.blockSignals(True); sw.setChecked(False); sw.blockSignals(False)
        self._disconnect_aqm()
        self._log("DISC", "All connections closed.")

    # ══════════════════════════════════════════════════════════════════════════
    # GAMEPAD
    # ══════════════════════════════════════════════════════════════════════════
    def _start_gamepad(self):
        if not INPUTS_OK:
            self._log("WARN", "inputs missing — pip install inputs"); return
        self._gpad_thread = GamepadThread()
        self._gpad_thread.state_updated.connect(self._on_gpad)
        self._gpad_thread.log_msg.connect(self._log)
        self._gpad_thread.pad_found.connect(lambda name: (
            self._lbl_pad.setText(f"✓  {name[:42]}"),
            self._lbl_pad.setStyleSheet(
                f"color:{self._c('green')};font-family:'Times New Roman',Times,serif;"
                f"font-size:{max(10,self._fs-2)}px;"
            ),
        ))
        self._gpad_thread.start()

    def _on_gpad(self, s):
        prev_r3 = self._gpad.get("r3", False)
        self._gpad = s
        self._led_lt.set_active(s.get("lt",0.0) > TRIGGER_DEADZONE)
        self._led_lb.set_active(s.get("lb", False))
        self._led_y.set_active(s.get("y",  False))
        self._led_rb.set_active(s.get("rb", False))
        self._led_rt.set_active(s.get("rt",0.0) > TRIGGER_DEADZONE)
        self._led_back.set_active(s.get("back",  False))
        self._led_x.set_active(s.get("x",    False))
        self._led_b.set_active(s.get("b",    False))
        self._led_a.set_active(s.get("a",    False))
        self._led_p.set_active(s.get("l3",   False))
        self._led_start.set_active(s.get("start", False))

        # R3 rising-edge → toggle AQM LED (independent of robot connection)
        if s.get("r3", False) and not prev_r3:
            self._toggle_aqm_led()

        lx = s.get("lx", 0.0) * (-1 if self._btn_inv_lx.isChecked() else 1)
        ly = s.get("ly", 0.0) * (-1 if self._btn_inv_ly.isChecked() else 1)
        rx = s.get("rx", 0.0) * (-1 if self._btn_inv_rx.isChecked() else 1)
        ry = s.get("ry", 0.0) * (-1 if self._btn_inv_ry.isChecked() else 1)
        self._ls.set_values(lx, ly)
        self._rs.set_values(rx, ry)
        self._lbl_stk.setText(f"LS: ({lx:+.2f}, {ly:+.2f})      RS: ({rx:+.2f}, {ry:+.2f})")

        _dp = {(0,0):"○",(0,1):"↑",(0,-1):"↓",(-1,0):"←",(1,0):"→"}
        self._lbl_dpad.setText(_dp.get((s.get("dpad_x",0), s.get("dpad_y",0)), "●"))
        self._lbl_lt_pct.setText(f"LT: {int(s.get('lt',0)*100):3d}%")
        self._lbl_rt_pct.setText(f"RT: {int(s.get('rt',0)*100):3d}%")

        # Fix A: event-driven walk — send immediately when direction changes,
        # no 40ms timer wait. Skipped when keyboard is actively providing movement.
        if self._robot_ok and not (self._kbd_enabled and (self._kbd_pressed & self._KBD_MOVE)):
            lx = s.get("lx", 0.0) * (-1 if self._btn_inv_lx.isChecked() else 1)
            ly = s.get("ly", 0.0) * (-1 if self._btn_inv_ly.isChecked() else 1)
            cmd = walk_cmd(lx, ly)
            if cmd != self._last_walk:
                self._send_raw(cmd)
                self._last_walk = cmd
                self._last_walk_t = time.time()

    # ══════════════════════════════════════════════════════════════════════════
    # SEND / TX TICK
    # ══════════════════════════════════════════════════════════════════════════
    def _send_raw(self, msg, silent: bool = False):
        if not self._robot_ok or not self._sock: return
        try:
            self._sock.sendto(msg.encode(), (self._robot_ip, ROBOT_PORT))
            self._sb_tx.setText(f"LAST TX: {msg}")
            if not silent:
                self._log("TX", msg)
        except Exception as e:
            self._log("ERROR", f"TX failed: {e}")
            self._robot_ok = False
            mono = "'Times New Roman',Times,serif"
            self._lbl_robot_status.setText("●  LOST")
            self._lbl_robot_status.setStyleSheet(
                f"color:{self._c('red')};font-family:{mono};font-size:{self._fs}px;font-weight:bold;"
            )
            self._sw_robot.blockSignals(True)
            self._sw_robot.setChecked(False)
            self._sw_robot.blockSignals(False)

    def _send_reliable(self, msg, repeat=3, gap_ms=20):
        for i in range(repeat):
            QTimer.singleShot(i * gap_ms, lambda m=msg: self._send_raw(m))

    def _try_toggle(self, tag, msg):
        now = time.time()
        if now - self._tog_t.get(tag, 0.0) >= TOGGLE_DEBOUNCE_SEC:
            self._tog_t[tag] = now
            self._send_reliable(msg)

    def _tx_tick(self):
        if not self._gpad or not self._robot_ok: return
        s   = self._gpad
        now = time.time()
        dt  = max(0.001, now - self._last_tick_t); self._last_tick_t = now

        rx_raw = s.get("rx", 0.0) * (-1 if self._btn_inv_rx.isChecked() else 1)
        ry_raw = s.get("ry", 0.0) * (-1 if self._btn_inv_ry.isChecked() else 1)

        if self._kbd_enabled and (self._kbd_pressed & self._KBD_MOVE):
            keys = self._kbd_pressed
            if Qt.Key_W in keys or Qt.Key_Up    in keys: cmd = "W"
            elif Qt.Key_S in keys or Qt.Key_Down  in keys: cmd = "S"
            elif Qt.Key_A in keys or Qt.Key_Left  in keys: cmd = "A"
            elif Qt.Key_D in keys or Qt.Key_Right in keys: cmd = "D"
            else: cmd = "IDLE"
            # Keyboard walk: still timer-driven; heartbeat resends are silent
            if cmd != self._last_walk or (now - self._last_walk_t) > HEARTBEAT_SEC:
                self._send_raw(cmd, silent=(cmd == self._last_walk))
                self._last_walk = cmd; self._last_walk_t = now
        else:
            # Gamepad walk already sent event-driven in _on_gpad; only heartbeat here
            cmd = self._last_walk if self._last_walk else "IDLE"
            if (now - self._last_walk_t) > HEARTBEAT_SEC:
                self._send_raw(cmd, silent=True)
                self._last_walk_t = now
        color = self._c("green") if cmd == "IDLE" else self._c("amber")
        self._lbl_cmd.setText(cmd)
        self._lbl_cmd.setStyleSheet(f"""
            color:{color}; font-family:"Times New Roman",Times,serif;
            font-size:{self._fs+4}px; font-weight:bold;
            letter-spacing:6px; padding-left:12px;
        """)

        rx = apply_deadzone(rx_raw, STICK_DEADZONE)
        ry = apply_deadzone(ry_raw, STICK_DEADZONE)
        lt = s.get("lt",0.0) if s.get("lt",0.0) > TRIGGER_DEADZONE else 0.0
        rt = s.get("rt",0.0) if s.get("rt",0.0) > TRIGGER_DEADZONE else 0.0

        rate = ARM_PACKETS_PER_SEC * dt
        self._arm_acc["lat"]   += rx * rate
        self._arm_acc["sh"]    += ry * rate
        self._arm_acc["elbow"] += (1.0 if s.get("rb") else -1.0 if s.get("lb") else 0.0) * rate
        self._arm_acc["claw"]  += (rt - lt) * rate

        for key, pc, nc in [("lat","2","1"),("sh","4","3"),("elbow","6","5"),("claw","8","7")]:
            while self._arm_acc[key] >= 1.0:
                self._send_raw(pc, silent=True); self._arm_acc[key] -= 1.0
            while self._arm_acc[key] <= -1.0:
                self._send_raw(nc, silent=True); self._arm_acc[key] += 1.0

        if s.get("a"):           self._try_toggle("A",  "0")
        if s.get("b"):
            prev = self._tog_t.get("B", 0.0)
            self._try_toggle("B", "Z")
            if self._tog_t.get("B", 0.0) != prev:
                self._sync_z_toggle()
        if s.get("l3"):
            prev = self._tog_t.get("L3", 0.0)
            self._try_toggle("L3", "P")
            if self._tog_t.get("L3", 0.0) != prev:
                self._sync_p_toggle()
        if s.get("x"):           self._try_toggle("X",  "X")
        if s.get("y"):           self._try_toggle("Y",  "M")
        if s.get("dpad_y",0)>0:  self._try_toggle("DU", "SPD_UP")
        if s.get("dpad_y",0)<0:  self._try_toggle("DD", "SPD_DN")

    # ══════════════════════════════════════════════════════════════════════════
    # NETWORK STATS
    # ══════════════════════════════════════════════════════════════════════════
    def _on_net_stats(self, ping_ms: float, wifi_pct: int):
        mono = f"font-family:'Times New Roman',Times,serif;font-size:{max(10,self._fs-3)}px;"
        if ping_ms >= 0:
            color = self._c("green") if ping_ms < 30 else self._c("amber") if ping_ms < 80 else self._c("red")
            self._sb_ping.setText(f"PING: {ping_ms:.0f} ms")
            self._sb_ping.setStyleSheet(f"color:{color};{mono}")
        else:
            self._sb_ping.setText("PING: —")
            self._sb_ping.setStyleSheet(f"color:{self._c('dim')};{mono}")
        if wifi_pct >= 0:
            bars = "▰▰▰▰" if wifi_pct >= 75 else "▰▰▰▱" if wifi_pct >= 50 else "▰▰▱▱" if wifi_pct >= 25 else "▰▱▱▱"
            color = self._c("green") if wifi_pct >= 60 else self._c("amber") if wifi_pct >= 35 else self._c("red")
            self._sb_wifi.setText(f"WiFi: {bars} {wifi_pct}%")
            self._sb_wifi.setStyleSheet(f"color:{color};{mono}")
        else:
            self._sb_wifi.setText("WiFi: —")
            self._sb_wifi.setStyleSheet(f"color:{self._c('dim')};{mono}")

    # ══════════════════════════════════════════════════════════════════════════
    # AIR QUALITY MONITOR
    # ══════════════════════════════════════════════════════════════════════════
    def _connect_aqm(self):
        ip = self._f_aqm_ip.text().strip()
        if not ip:
            self._log("ERROR", "AQM IP empty.")
            self._sw_aqm.blockSignals(True); self._sw_aqm.setChecked(False); self._sw_aqm.blockSignals(False)
            return
        if self._aqm_thread and self._aqm_thread.isRunning():
            self._aqm_thread.stop(); self._aqm_thread.wait(2000)
        self._aqm_rx = 0
        self._aqm_thread = AQMThread(ip)
        self._aqm_thread.data_ready.connect(self._on_aqm_data)
        self._aqm_thread.log_msg.connect(self._log)
        self._aqm_thread.status_changed.connect(self._on_aqm_status)
        self._aqm_thread.start()

    def _disconnect_aqm(self):
        if self._aqm_thread:
            self._aqm_thread.stop(); self._aqm_thread.wait(2000)
            self._aqm_thread = None
        self._on_aqm_status(False)
        self._log("INFO", "AQM disconnected.")

    def _on_aqm_status(self, connected: bool):
        c = self._colors; mono = "'Times New Roman',Times,serif"
        fs = self._fs
        self._sw_aqm.blockSignals(True)
        self._sw_aqm.setChecked(connected)
        self._sw_aqm.blockSignals(False)
        if connected:
            self._lbl_aqm_status.setText("●  ONLINE")
            self._lbl_aqm_status.setStyleSheet(
                f"color:{c['green']};font-family:{mono};font-size:{fs}px;font-weight:bold;")
            self._lbl_aqm_conn.setText(f"✓  CONNECTED  —  {self._f_aqm_ip.text().strip()}")
            self._lbl_aqm_conn.setStyleSheet(
                f"color:{c['green']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:1px;")
        else:
            self._lbl_aqm_status.setText("●  OFFLINE")
            self._lbl_aqm_status.setStyleSheet(
                f"color:{c['red']};font-family:{mono};font-size:{fs}px;font-weight:bold;")
            self._lbl_aqm_conn.setText("⚠  NOT CONNECTED  —  set IP and press CONNECT AQM")
            self._lbl_aqm_conn.setStyleSheet(
                f"color:{c['amber']};font-family:{mono};font-size:{fs}px;font-weight:bold;letter-spacing:1px;")

    def _on_aqm_data(self, data: dict):
        self._aqm_rx += 1
        c    = self._colors
        mono = "'Times New Roman',Times,serif"
        fs   = self._fs

        def _style(col):
            return f"color:{col};font-family:{mono};font-size:{fs}px;font-weight:bold;"

        def _set_bar(bar, value, min_v, max_v, color):
            pct = max(0.0, min(1.0, (value - min_v) / (max_v - min_v)))
            bar.setValue(int(pct * 1000))
            track = c["panel2"]
            bar.setStyleSheet(
                f"QProgressBar{{background:{track};border:1px solid {c['border']};"
                f"border-radius:2px;}}"
                f"QProgressBar::chunk{{background:{color};border-radius:2px;}}"
            )

        g = data.get("gas", {})
        d = data.get("dht", {})
        uptime = data.get("uptime", 0)
        ups = f"{uptime//3600:02d}:{(uptime%3600)//60:02d}:{uptime%60:02d}"

        # O2  (normal ~20.9%, bar shows 19.5–21.5)
        o2 = g.get("o2", 20.9)
        col_o2 = c["green"] if o2 >= 20.5 else c["red"]
        _set_bar(self._bar_aqm_o2, o2, 19.5, 21.5, col_o2)
        self._lbl_aqm_o2.setText(f"{o2:.1f} %"); self._lbl_aqm_o2.setStyleSheet(_style(col_o2))

        # CO  (limit 4.5 ppm)
        co = g.get("co", 0.0)
        col_co = c["green"] if co < 1.0 else (c["amber"] if co < 3.0 else c["red"])
        _set_bar(self._bar_aqm_co, co, 0, 4.5, col_co)
        self._lbl_aqm_co.setText(f"{co:.3f} ppm"); self._lbl_aqm_co.setStyleSheet(_style(col_co))

        # H2S  (limit 0.012 ppm)
        h2s = g.get("h2s", 0.0)
        col_h2s = c["green"] if h2s < 0.005 else (c["amber"] if h2s < 0.008 else c["red"])
        _set_bar(self._bar_aqm_h2s, h2s, 0, 0.012, col_h2s)
        self._lbl_aqm_h2s.setText(f"{h2s:.4f} ppm"); self._lbl_aqm_h2s.setStyleSheet(_style(col_h2s))

        # LEL  (limit 1.2%)
        lel = g.get("lel", 0.0)
        col_lel = c["green"] if lel < 0.5 else (c["amber"] if lel < 0.9 else c["red"])
        _set_bar(self._bar_aqm_lel, lel, 0, 1.2, col_lel)
        self._lbl_aqm_lel.setText(f"{lel:.3f} %"); self._lbl_aqm_lel.setStyleSheet(_style(col_lel))

        # RAW ADC
        raw = g.get("raw", 0)
        _set_bar(self._bar_aqm_raw, raw, 0, 4095, c["dim"])
        self._lbl_aqm_raw.setText(f"{raw}/4095"); self._lbl_aqm_raw.setStyleSheet(_style(c["dim"]))

        # Temperature / Humidity
        if d.get("ok"):
            tc  = d.get("temp_c", 0.0); tf = d.get("temp_f", 0.0)
            col_t = c["red"] if tc > 30 else (c["blue"] if tc < 18 else c["green"])
            lbl_t = "HOT" if tc > 30 else ("COLD" if tc < 18 else "OK")
            _set_bar(self._bar_aqm_temp, tc, 0, 50, col_t)
            self._lbl_aqm_temp.setText(f"{tc:.1f}°C  {tf:.1f}°F  {lbl_t}")
            self._lbl_aqm_temp.setStyleSheet(_style(col_t))

            hum = d.get("humidity", 0.0)
            col_h = c["teal"] if 40 <= hum <= 70 else c["amber"]
            _set_bar(self._bar_aqm_hum, hum, 0, 100, col_h)
            self._lbl_aqm_hum.setText(f"{hum:.1f}%"); self._lbl_aqm_hum.setStyleSheet(_style(col_h))
        else:
            for bar in (self._bar_aqm_temp, self._bar_aqm_hum):
                bar.setValue(0)
            self._lbl_aqm_temp.setText("sensor err"); self._lbl_aqm_hum.setText("sensor err")
            for lbl in (self._lbl_aqm_temp, self._lbl_aqm_hum):
                lbl.setStyleSheet(_style(c["red"]))

        self._lbl_aqm_footer.setText(f"packets: {self._aqm_rx}  |  uptime: {ups}")

        # Sync LED button from firmware's authoritative "led" field
        led = data.get("led", None)
        if led is not None and led != self._aqm_led_on:
            self._aqm_led_on = led
            self._btn_aqm_led.blockSignals(True)
            self._btn_aqm_led.setChecked(led)
            self._btn_aqm_led.blockSignals(False)
            self._btn_aqm_led.setText("💡  LED: ON  ●" if led else "💡  LED: OFF")

    # ══════════════════════════════════════════════════════════════════════════
    # CLEANUP
    # ══════════════════════════════════════════════════════════════════════════
    def closeEvent(self, event):
        self._tx_timer.stop()
        QApplication.instance().removeEventFilter(self._kbd_filter)
        if self._gpad_thread:
            self._gpad_thread.stop(); self._gpad_thread.wait(2000)
        if self._crack_thread:
            self._crack_thread.stop(); self._crack_thread.wait(2000)
        if self._aqm_thread:
            self._aqm_thread.stop(); self._aqm_thread.wait(2000)
        self._stop_all_cams()
        if self._sock:
            try: self._send_raw("IDLE")
            except: pass
            try: self._sock.close()
            except: pass
        event.accept()


# ══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setFont(QFont("Times New Roman", 15))

    win = MainWindow()
    win.show()

    win._log("INFO",  "TACTICAL CONTROL SYSTEM  v2  —  ONLINE")
    win._log("INFO",  f"inputs lib  : {'OK' if INPUTS_OK else 'MISSING — pip install inputs'}")
    win._log("INFO",  f"Camera libs : {'OK' if CAM_OK    else 'MISSING — pip install requests opencv-python numpy'}")
    win._log("INFO",  "Set ROBOT IP → press CONNECT ROBOT.")
    win._log("INFO",  "Camera: press  💻 LAPTOP CAM  or  📡 ESP32 CAM.")
    win._log("INFO",  "F710 gamepad  →  back switch = X (XInput mode).")

    sys.exit(app.exec_())
