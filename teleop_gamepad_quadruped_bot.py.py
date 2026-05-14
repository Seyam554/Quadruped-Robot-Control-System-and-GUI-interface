"""
UDP Gamepad Teleop: Logitech F710 -> Quadruped + Direct Arm
============================================================
Requires: pip install inputs

Set the F710 back switch to X (XInput mode).

Control mapping:
  Left Stick        : Walk (Forward/Back/Left/Right)
  Right Stick X     : Arm Lateral  (1 / 2)
  Right Stick Y     : Arm Shoulder (3 / 4)
  LB / RB           : Arm Elbow    (5 / 6)
  LT / RT           : Arm Claw     (7 / 8)
  A                 : Arm Home (0)
  B                 : Calibration toggle (Z)
  X                 : Demo toggle (X)
  Y                 : Gait mode toggle (M)
  D-Pad Up/Down     : Speed +/-
  Back/Select       : Quit
"""

import socket
import time
import threading
import sys

try:
    import inputs
except ImportError:
    print("Install with: pip install inputs")
    sys.exit(1)

# =====================
# CONFIG
# =====================
IP = "10.196.72.146"
PORT = 5555
TX_HZ = 25
HEARTBEAT_SEC = 0.5
TOGGLE_DEBOUNCE_SEC = 0.30

STICK_MAX = 32768.0
STICK_DEADZONE = 0.20
TRIGGER_MAX = 255.0
TRIGGER_DEADZONE = 0.10
ARM_PACKETS_PER_SEC = 12

# =====================
# UDP
# =====================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

# =====================
# GAMEPAD STATE (updated by reader thread)
# =====================
state = {
    "lx": 0.0, "ly": 0.0,
    "rx": 0.0, "ry": 0.0,
    "lt": 0.0, "rt": 0.0,
    "lb": False, "rb": False,
    "a": False, "b": False,
    "x": False, "y": False,
    "dpad_x": 0, "dpad_y": 0,
    "back": False, "start": False,
}
state_lock = threading.Lock()
running = True

# =====================
# HELPERS
# =====================
last_sent_walk = None
last_walk_send_time = 0.0
last_toggle_time = {}
arm_accum = {"lat": 0.0, "sh": 0.0, "elbow": 0.0, "claw": 0.0}


def send(msg: str) -> None:
    global last_sent_walk, last_walk_send_time
    sock.sendto(msg.encode("utf-8"), (IP, PORT))
    if msg in ("W", "A", "S", "D", "IDLE"):
        last_sent_walk = msg
        last_walk_send_time = time.time()
    print(f"[TX] {msg}")


def send_reliable(msg: str, repeat: int = 3, gap: float = 0.02) -> None:
    for _ in range(repeat):
        send(msg)
        time.sleep(gap)


def try_toggle(tag: str, msg: str) -> None:
    now = time.time()
    if now - last_toggle_time.get(tag, 0.0) >= TOGGLE_DEBOUNCE_SEC:
        last_toggle_time[tag] = now
        send_reliable(msg)


def apply_deadzone(value: float, dz: float) -> float:
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)


def walk_cmd_from_stick(lx: float, ly: float) -> str:
    lx = apply_deadzone(lx, STICK_DEADZONE)
    ly = apply_deadzone(ly, STICK_DEADZONE)
    if abs(lx) < 0.01 and abs(ly) < 0.01:
        return "IDLE"
    if abs(ly) >= abs(lx):
        return "S" if ly > 0 else "W"
    else:
        return "D" if lx > 0 else "A"


def accumulate_arm(dt, rx, ry, lb, rb, lt, rt):
    rate = ARM_PACKETS_PER_SEC * dt

    rx = apply_deadzone(rx, STICK_DEADZONE)
    ry = apply_deadzone(ry, STICK_DEADZONE)
    lt = lt if lt > TRIGGER_DEADZONE else 0.0
    rt = rt if rt > TRIGGER_DEADZONE else 0.0

    arm_accum["lat"] += rx * rate
    arm_accum["sh"]  += ry * rate

    if rb:
        arm_accum["elbow"] += rate
    if lb:
        arm_accum["elbow"] -= rate

    arm_accum["claw"] += rt * rate
    arm_accum["claw"] -= lt * rate

    for key, pos_cmd, neg_cmd in [("lat", "2", "1"), ("sh", "4", "3"),
                                   ("elbow", "6", "5"), ("claw", "8", "7")]:
        while arm_accum[key] >= 1.0:
            send(pos_cmd)
            arm_accum[key] -= 1.0
        while arm_accum[key] <= -1.0:
            send(neg_cmd)
            arm_accum[key] += 1.0


# =====================
# GAMEPAD READER THREAD
# =====================
def gamepad_reader():
    global running

    while running:
        try:
            events = inputs.get_gamepad()
        except inputs.UnpluggedError:
            print("[Gamepad] Disconnected! Waiting...")
            time.sleep(1.0)
            continue
        except Exception as e:
            print(f"[Gamepad] Error: {e}")
            time.sleep(0.1)
            continue

        with state_lock:
            for ev in events:
                # ---- Sticks ----
                if ev.code == "ABS_X":
                    state["lx"] = ev.state / STICK_MAX
                elif ev.code == "ABS_Y":
                    state["ly"] = ev.state / STICK_MAX
                elif ev.code == "ABS_RX":
                    state["rx"] = ev.state / STICK_MAX
                elif ev.code == "ABS_RY":
                    state["ry"] = ev.state / STICK_MAX

                # ---- Triggers ----
                elif ev.code == "ABS_Z":
                    state["lt"] = ev.state / TRIGGER_MAX
                elif ev.code == "ABS_RZ":
                    state["rt"] = ev.state / TRIGGER_MAX

                # ---- D-Pad ----
                elif ev.code == "ABS_HAT0X":
                    state["dpad_x"] = ev.state
                elif ev.code == "ABS_HAT0Y":
                    state["dpad_y"] = -ev.state   # invert: up = +1

                # ---- Buttons (XInput names) ----
                elif ev.code == "BTN_SOUTH":
                    state["a"] = bool(ev.state)
                elif ev.code == "BTN_EAST":
                    state["b"] = bool(ev.state)
                elif ev.code == "BTN_WEST":
                    state["x"] = bool(ev.state)
                elif ev.code == "BTN_NORTH":
                    state["y"] = bool(ev.state)
                elif ev.code == "BTN_TL":
                    state["lb"] = bool(ev.state)
                elif ev.code == "BTN_TR":
                    state["rb"] = bool(ev.state)
                elif ev.code == "BTN_SELECT":
                    state["back"] = bool(ev.state)
                elif ev.code == "BTN_START":
                    state["start"] = bool(ev.state)


# =====================
# MAIN
# =====================
def main():
    global running

    pads = inputs.devices.gamepads
    if not pads:
        print("No gamepad detected!")
        print("Plug in the F710 and set back switch to X (XInput).")
        sys.exit(1)

    print(f"\nGamepad found: {pads[0]}")
    print()
    print("=============================================")
    print("  UDP Gamepad Teleop: Quadruped + Direct Arm")
    print(f"  Target : {IP}:{PORT}")
    print("  Left Stick   : Walk")
    print("  Right Stick X: Arm Lateral")
    print("  Right Stick Y: Arm Shoulder")
    print("  LB / RB      : Arm Elbow -/+")
    print("  LT / RT      : Arm Claw  -/+")
    print("  A             : Arm Home (0)")
    print("  B             : Calibration (Z)")
    print("  X             : Demo (X)")
    print("  Y             : Gait toggle (M)")
    print("  D-Pad Up/Down : Speed +/-")
    print("  Back          : Quit")
    print("=============================================")
    print()

    reader = threading.Thread(target=gamepad_reader, daemon=True)
    reader.start()

    send("IDLE")

    period = 1.0 / TX_HZ
    last_time = time.time()

    while running:
        now = time.time()
        dt = now - last_time
        last_time = now

        with state_lock:
            s = dict(state)

        if s["back"]:
            running = False
            break

        # Walk
        walk = walk_cmd_from_stick(s["lx"], s["ly"])
        if walk != last_sent_walk or (now - last_walk_send_time) > HEARTBEAT_SEC:
            send(walk)

        # Arm
        accumulate_arm(dt, s["rx"], s["ry"], s["lb"], s["rb"], s["lt"], s["rt"])

        # Toggles
        if s["a"]:
            try_toggle("A", "0")
        if s["b"]:
            try_toggle("B", "Z")
        if s["x"]:
            try_toggle("X", "X")
        if s["y"]:
            try_toggle("Y", "M")
        if s["dpad_y"] > 0:
            try_toggle("DU", "SPD_UP")
        if s["dpad_y"] < 0:
            try_toggle("DD", "SPD_DN")

        time.sleep(period)

    try:
        send("IDLE")
    except Exception:
        pass
    print("Exiting...")


if __name__ == "__main__":
    main()
