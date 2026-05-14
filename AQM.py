import socket
import json
import time
import os
import sys
from datetime import datetime

ESP32_IP   = "192.168.181.3"
ESP32_PORT = 5555

# ─── ANSI CODES ───────────────────────────────────────────────────────────────
RESET   = "\033[0m"
BOLD    = "\033[1m"
DIM     = "\033[2m"

BLACK   = "\033[30m"
RED     = "\033[31m"
GREEN   = "\033[32m"
YELLOW  = "\033[33m"
BLUE    = "\033[34m"
MAGENTA = "\033[35m"
CYAN    = "\033[36m"
WHITE   = "\033[37m"

BG_BLACK = "\033[40m"
BG_RED   = "\033[41m"
BG_GREEN = "\033[42m"

CLEAR      = "\033[2J\033[H"
CURSOR_OFF = "\033[?25l"
CURSOR_ON  = "\033[?25h"
HOME       = "\033[H"

def move(row, col):
    return f"\033[{row};{col}H"

# ─── BOX CHARS ────────────────────────────────────────────────────────────────
TL = "╔"; TR = "╗"; BL = "╚"; BR = "╝"
H  = "═"; V  = "║"
ML = "╠"; MR = "╣"; MT = "╦"; MB = "╩"; CR = "╬"
SH = "─"; SV = "│"
STL = "┌"; STR = "┐"; SBL = "└"; SBR = "┘"
SML = "├"; SMR = "┤"

WIDTH = 72

# ─── HELPERS ──────────────────────────────────────────────────────────────────
def bar(value, min_val, max_val, width=20, fill="█", empty="░"):
    ratio   = max(0.0, min(1.0, (value - min_val) / (max_val - min_val)))
    filled  = int(ratio * width)
    color   = GREEN if ratio < 0.5 else (YELLOW if ratio < 0.8 else RED)
    return color + fill * filled + DIM + empty * (width - filled) + RESET

def status_badge(ok, true_text="  OK  ", false_text=" FAIL "):
    if ok:
        return f"{BOLD}{BG_GREEN}{BLACK} {true_text} {RESET}"
    else:
        return f"{BOLD}{BG_RED}{WHITE} {false_text} {RESET}"

def wifi_badge(rssi):
    if rssi >= -60:
        return f"{BOLD}{GREEN}▂▄▆█ EXCELLENT{RESET}"
    elif rssi >= -70:
        return f"{BOLD}{GREEN}▂▄▆░ GOOD     {RESET}"
    elif rssi >= -80:
        return f"{BOLD}{YELLOW}▂▄░░ FAIR     {RESET}"
    else:
        return f"{BOLD}{RED}▂░░░ WEAK     {RESET}"

def hline(left, mid, right, width=WIDTH):
    return left + H * width + right

def row(content, width=WIDTH):
    # content should be exactly `width` visible chars wide
    return V + content + V

def pad(text, width, align="left"):
    # strips ANSI for length calc
    import re
    visible = re.sub(r'\033\[[0-9;]*m', '', text)
    pad_len = width - len(visible)
    if pad_len < 0: pad_len = 0
    if align == "right":
        return " " * pad_len + text
    elif align == "center":
        l = pad_len // 2
        r = pad_len - l
        return " " * l + text + " " * r
    else:
        return text + " " * pad_len

# ─── DRAW DASHBOARD ───────────────────────────────────────────────────────────
def draw(data, status, last_update, rx_count):
    g = data.get("gas", {})
    d = data.get("dht", {})
    w = data.get("wifi", {})
    uptime = data.get("uptime", 0)

    gas_raw  = g.get("raw",  0)
    gas_o2   = g.get("o2",   20.9)
    gas_co   = g.get("co",   0.0)
    gas_h2s  = g.get("h2s",  0.0)
    gas_lel  = g.get("lel",  0.0)

    dht_ok    = d.get("ok",      False)
    dht_tc    = d.get("temp_c",  0.0)
    dht_tf    = d.get("temp_f",  0.0)
    dht_hum   = d.get("humidity",0.0)

    wifi_ssid = w.get("ssid", "---")
    wifi_ip   = w.get("ip",   "---")
    wifi_rssi = w.get("rssi", -99)
    wifi_up   = status == "connected"

    ups = f"{uptime//3600:02d}:{(uptime%3600)//60:02d}:{uptime%60:02d}"

    out = []
    out.append(HOME)

    # ╔══ HEADER ══╗
    out.append(BOLD + CYAN + hline(TL, H, TR) + RESET + "\n")
    title = f"{BOLD}{CYAN}  ESP32-C6 · SENSOR DASHBOARD  {RESET}{DIM}│{RESET}{CYAN}  Uptime: {BOLD}{WHITE}{ups}{RESET}{CYAN}  │  Packets: {BOLD}{WHITE}{rx_count}{RESET}{CYAN}  │  {last_update}{RESET}"
    out.append(CYAN + V + RESET + pad(title, WIDTH) + CYAN + V + RESET + "\n")
    out.append(BOLD + CYAN + hline(ML, CR, MR) + RESET + "\n")

    # ╠══ AIR QUALITY ══╣
    section = f"{BOLD}{YELLOW}  ▸ AIR QUALITY{RESET}"
    out.append(CYAN + V + RESET + pad(section, WIDTH) + CYAN + V + RESET + "\n")
    out.append(CYAN + ML + DIM + H * WIDTH + MR + RESET + "\n")

    # O2
    o2_bar = bar(gas_o2, 19.5, 21.0, 24)
    o2_val = f"{BOLD}{GREEN if gas_o2 > 20.5 else RED}{gas_o2:.1f}%{RESET}"
    line   = f"  {CYAN}O2  (Oxygen)     {RESET} {o2_bar}  {pad(o2_val, 14)}  {DIM}Normal: 20.9%{RESET}"
    out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")

    # CO
    co_col = GREEN if gas_co < 1.0 else (YELLOW if gas_co < 3.0 else RED)
    co_bar = bar(gas_co, 0, 4.5, 24)
    co_val = f"{BOLD}{co_col}{gas_co:.3f} ppm{RESET}"
    line   = f"  {CYAN}CO  (Carbon Mon) {RESET} {co_bar}  {pad(co_val, 14)}  {DIM}Limit: 4.5ppm {RESET}"
    out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")

    # H2S
    h2s_col = GREEN if gas_h2s < 0.005 else (YELLOW if gas_h2s < 0.008 else RED)
    h2s_bar = bar(gas_h2s, 0, 0.012, 24)
    h2s_val = f"{BOLD}{h2s_col}{gas_h2s:.4f} ppm{RESET}"
    line    = f"  {CYAN}H2S (Hydrogen S) {RESET} {h2s_bar}  {pad(h2s_val, 14)}  {DIM}Limit: 0.012 {RESET}"
    out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")

    # LEL
    lel_col = GREEN if gas_lel < 0.5 else (YELLOW if gas_lel < 0.9 else RED)
    lel_bar = bar(gas_lel, 0, 1.2, 24)
    lel_val = f"{BOLD}{lel_col}{gas_lel:.3f}%{RESET}"
    line    = f"  {CYAN}LEL (Combustible){RESET} {lel_bar}  {pad(lel_val, 14)}  {DIM}Limit: 1.2%  {RESET}"
    out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")

    # Raw ADC
    raw_bar = bar(gas_raw, 0, 4095, 24, "▪", "·")
    raw_val = f"{BOLD}{WHITE}{gas_raw}{RESET}{DIM}/4095{RESET}"
    line    = f"  {DIM}RAW ADC          {RESET} {raw_bar}  {pad(raw_val, 14)}  {DIM}12-bit ADC   {RESET}"
    out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")

    # ╠══ DHT11 ══╣
    out.append(BOLD + CYAN + hline(ML, CR, MR) + RESET + "\n")
    dht_status = status_badge(dht_ok, " DHT OK ", " DHT ERR")
    section    = f"{BOLD}{YELLOW}  ▸ CLIMATE  {RESET}{dht_status}"
    out.append(CYAN + V + RESET + pad(section, WIDTH) + CYAN + V + RESET + "\n")
    out.append(CYAN + ML + DIM + H * WIDTH + MR + RESET + "\n")

    if dht_ok:
        tc_col  = RED if dht_tc > 30 else (BLUE if dht_tc < 18 else GREEN)
        tc_lbl  = "HOT 🔥" if dht_tc > 30 else ("COLD ❄" if dht_tc < 18 else "COMFY ✓")
        temp_bar= bar(dht_tc, 0, 50, 24)
        temp_val= f"{BOLD}{tc_col}{dht_tc:.1f}°C  {dht_tf:.1f}°F{RESET}"
        line    = f"  {CYAN}Temperature      {RESET} {temp_bar}  {pad(temp_val, 18)}  {BOLD}{tc_col}{tc_lbl}{RESET}"
        out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")

        hum_col  = CYAN if dht_hum > 40 and dht_hum < 70 else YELLOW
        hum_bar  = bar(dht_hum, 0, 100, 24)
        hum_val  = f"{BOLD}{hum_col}{dht_hum:.1f}%{RESET}"
        line     = f"  {CYAN}Humidity         {RESET} {hum_bar}  {pad(hum_val, 14)}  {DIM}Ideal: 40-70%{RESET}"
        out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")
    else:
        line = f"  {RED}  ✖  Sensor read failed — check wiring on pin D2{RESET}"
        out.append(CYAN + V + RESET + pad(line, WIDTH) + CYAN + V + RESET + "\n")
        out.append(CYAN + V + RESET + pad("", WIDTH) + CYAN + V + RESET + "\n")

    # ╠══ WIFI ══╣
    out.append(BOLD + CYAN + hline(ML, CR, MR) + RESET + "\n")
    wifi_st = status_badge(wifi_up, " CONNECTED ", "DISCONNECTED")
    section = f"{BOLD}{YELLOW}  ▸ NETWORK  {RESET}{wifi_st}"
    out.append(CYAN + V + RESET + pad(section, WIDTH) + CYAN + V + RESET + "\n")
    out.append(CYAN + ML + DIM + H * WIDTH + MR + RESET + "\n")

    ssid_line = f"  {CYAN}SSID   {RESET}  {BOLD}{WHITE}{wifi_ssid:<24}{RESET}   {CYAN}IP  {RESET}  {BOLD}{WHITE}{wifi_ip:<18}{RESET}"
    out.append(CYAN + V + RESET + pad(ssid_line, WIDTH) + CYAN + V + RESET + "\n")

    rssi_line = f"  {CYAN}RSSI   {RESET}  {BOLD}{WHITE}{wifi_rssi:>4} dBm{RESET}  {wifi_badge(wifi_rssi)}"
    out.append(CYAN + V + RESET + pad(rssi_line, WIDTH) + CYAN + V + RESET + "\n")

    # ╚══ FOOTER ══╝
    out.append(BOLD + CYAN + hline(BL, H, BR) + RESET + "\n")
    footer = f"{DIM}  TCP 5555  │  Gas@5Hz  │  DHT@0.5Hz  │  Press Ctrl+C to exit  {RESET}"
    out.append(pad(footer, WIDTH + 2) + "\n")

    sys.stdout.write("".join(out))
    sys.stdout.flush()

# ─── CONNECT ──────────────────────────────────────────────────────────────────
def connect():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((ESP32_IP, ESP32_PORT))
    return s

# ─── MAIN ─────────────────────────────────────────────────────────────────────
def main():
    sys.stdout.write(CLEAR + CURSOR_OFF)
    sys.stdout.flush()

    last_data   = {}
    rx_count    = 0
    status      = "disconnected"
    last_update = "--:--:--"

    try:
        while True:
            try:
                sys.stdout.write(HOME)
                print(f"{YELLOW}  Connecting to {ESP32_IP}:{ESP32_PORT} ...{RESET}")

                sock = connect()
                status = "connected"
                buf  = ""

                while True:
                    chunk = sock.recv(2048).decode("utf-8", errors="ignore")
                    if not chunk:
                        status = "disconnected"
                        break

                    buf += chunk
                    while "\n" in buf:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            last_data   = json.loads(line)
                            rx_count   += 1
                            last_update = datetime.now().strftime("%H:%M:%S")
                            draw(last_data, status, last_update, rx_count)
                        except json.JSONDecodeError:
                            pass

            except (ConnectionRefusedError, socket.timeout, OSError):
                status = "disconnected"
                if last_data:
                    draw(last_data, status, last_update, rx_count)
                else:
                    sys.stdout.write(HOME)
                    print(f"\n  {RED}✖ Cannot reach {ESP32_IP}:{ESP32_PORT}{RESET}")
                    print(f"  {DIM}Retrying in 3 seconds...{RESET}\n")
                    sys.stdout.flush()
                time.sleep(3)

    except KeyboardInterrupt:
        sys.stdout.write(CURSOR_ON + "\n")
        print(f"\n{CYAN}  Dashboard closed.{RESET}\n")

if __name__ == "__main__":
    main()