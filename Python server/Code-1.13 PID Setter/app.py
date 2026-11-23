import json
import os
import queue
import re
import socket
import threading
import time
from flask import Flask, Response, jsonify, make_response, render_template, request

app = Flask(__name__)

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

DEFAULT_CONFIG = {
    "Kp": 0.068,
    "Ki": 0.0008,
    "Kd": 0.0018,
    "speedStraight": 190,
    "speedGentle": 90,
    "speedMedium": 80,
    "speedSharp": 70,
    "speedMinBase": 65,
    "probeSpeed": 130,
    "alignSpeed": 120,
    "pivotSpeed": 160,
    "searchSpeed": 155,
    "trackMode": 0,
    "junctionPolicy": 1,
    "stopAtFinish": 0,
    "autoStart": 1,
    "uturnDir": -1,
    "leftInvert": 0,
    "rightInvert": 0,
    "encLeftInvert": 0,
    "encRightInvert": 0,
}

FLOAT_KEYS = {"Kp", "Ki", "Kd"}
CONFIG_ORDER = list(DEFAULT_CONFIG.keys())

# Calibration report regexes
CAL_ROW_RE = re.compile(
    r"S([0-7])\b[^|]*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(OK|WARN:\s*LOW)"
)
LEFT_AVG_RE = re.compile(r"Left Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")
RIGHT_AVG_RE = re.compile(r"Right Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")
ACT_WORDS = {"STOP", "STRAIGHT", "LEFT", "RIGHT", "BACK", "UTURN", "FORWARD", "REVERSE"}

clients: list[queue.Queue] = []
clients_lock = threading.Lock()

active_conn: socket.socket | None = None
conn_lock = threading.Lock()
config_lock = threading.Lock()

pos_state = {"count": 0, "hz_time": time.monotonic(), "hz": 0}


def load_config() -> dict:
    with config_lock:
        cfg = dict(DEFAULT_CONFIG)
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                    saved = json.load(f)
                for k in CONFIG_ORDER:
                    if k in saved:
                        cfg[k] = float(saved[k]) if k in FLOAT_KEYS else int(saved[k])
            except (OSError, ValueError, TypeError):
                pass
        return cfg


def save_config(data: dict) -> dict:
    cfg = load_config()
    for k in CONFIG_ORDER:
        if k in data:
            cfg[k] = float(data[k]) if k in FLOAT_KEYS else int(data[k])
    with config_lock:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
    return cfg


def fmt_num(key: str, val) -> str:
    if key in FLOAT_KEYS:
        s = f"{float(val):.6f}".rstrip("0").rstrip(".")
        return s if s else "0"
    return str(int(val))


def build_cfg_packet(cfg: dict) -> str:
    values = ",".join(fmt_num(k, cfg[k]) for k in CONFIG_ORDER)
    return f"$CFG,{values}\n"


def auto_send_saved_config(target_conn: socket.socket):
    """Send the saved config 2 seconds after ESP connects, if still connected."""
    packet = build_cfg_packet(load_config())
    with conn_lock:
        if active_conn is not target_conn:
            return
        try:
            target_conn.sendall(packet.encode("utf-8"))
        except OSError as exc:
            print(f"[TCP] Auto-send config failed: {exc}")
            return
    print(f"[TCP] Auto-sent config: {packet.strip()}")
    broadcast({"type": "sys", "raw": f"[AUTO-TX] {packet.strip()}"})


def norm_ir(val: str) -> str:
    v = val.strip().upper()
    if v in ("0", "W", "WHITE"):
        return "WHITE"
    if v in ("1", "B", "BLACK"):
        return "BLACK"
    return "-"


def broadcast(payload: dict):
    msg = json.dumps(payload, separators=(",", ":"))
    with clients_lock:
        for q in list(clients):
            if q.full():
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass


def process_line(raw_line: str):
    line = re.sub(r"[^\x20-\x7E]", "", raw_line).strip()
    if not line:
        return

    # 1. $POS,3131,1
    if line.startswith("$POS,"):
        parts = line.split(",")
        if len(parts) >= 3:
            try:
                val = int(parts[1])
                if not (0 <= val <= 7000):
                    return
                vis = parts[2] == "1"
                now = time.monotonic()
                pos_state["count"] += 1
                if now - pos_state["hz_time"] >= 1.0:
                    pos_state["hz"] = pos_state["count"]
                    pos_state["count"] = 0
                    pos_state["hz_time"] = now
                broadcast({"type": "pos", "pos": val, "vis": vis, "hz": pos_state["hz"], "raw": line})
                return
            except ValueError:
                pass

    # 2. $DAT,encL,encR,irL,irR,us,state[,act]
    if line.startswith("$DAT,"):
        parts = line.split(",")
        if len(parts) >= 7:
            try:
                broadcast({
                    "type": "dat",
                    "encL": int(parts[1]),
                    "encR": int(parts[2]),
                    "irL": norm_ir(parts[3]),
                    "irR": norm_ir(parts[4]),
                    "us": int(parts[5]),
                    "st": parts[6].upper(),
                    "act": parts[7].upper() if len(parts) >= 8 else None,
                    "raw": line,
                })
                return
            except ValueError:
                pass

    # 2b. Standalone or prefixed action
    upper = line.upper()
    if upper in ACT_WORDS:
        broadcast({"type": "act", "act": upper, "raw": line})
        return
    if upper.startswith("$ACT,") or upper.startswith("[ACT]"):
        act_val = upper.split(",", 1)[1].strip() if "," in upper else upper[5:].strip()
        if act_val:
            broadcast({"type": "act", "act": act_val, "raw": line})
            return

    # 3. Calibration report rows & averages
    m = CAL_ROW_RE.search(line)
    if m:
        broadcast({
            "type": "cal_row",
            "idx": int(m.group(1)),
            "min": int(m.group(2)),
            "max": int(m.group(3)),
            "span": int(m.group(4)),
            "ok": m.group(5) == "OK",
            "raw": raw_line.rstrip("\r"),
        })
        return

    ml = LEFT_AVG_RE.search(line)
    if ml:
        broadcast({"type": "cal_avg", "side": "L", "val": int(ml.group(1)), "ok": ml.group(2) == "OK", "raw": raw_line.rstrip("\r")})
        return
    mr = RIGHT_AVG_RE.search(line)
    if mr:
        broadcast({"type": "cal_avg", "side": "R", "val": int(mr.group(1)), "ok": mr.group(2) == "OK", "raw": raw_line.rstrip("\r")})
        return

    # 4. Status / mode announcements
    if "[TRACK]" in line:
        mode = "WHITE ON BLACK" if "WHITE Line" in line else ("BLACK ON WHITE" if "BLACK Line" in line else "UNCERTAIN")
        broadcast({"type": "track", "mode": mode, "raw": line})
        return
    if "[CAL RESULT]" in line:
        broadcast({"type": "cal_result", "ok": "SUCCESS" in line, "raw": line})
        return
    if "[CAL]" in line:
        broadcast({"type": "status", "status": "CALIBRATING", "raw": line})
        return
    if "[ERROR]" in line:
        broadcast({"type": "status", "status": "STOPPED", "raw": line})
        return
    if "[AUTO]" in line or "[USER]" in line or "[WIFI]" in line:
        broadcast({"type": "status", "status": "WAITING", "raw": line})
        return

    broadcast({"type": "log", "raw": raw_line.rstrip("\r")})


def start_tcp_listener(host="0.0.0.0", port=65432):
    def server_loop():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen(1)
            print(f"[TCP] Listening for ESP on {host}:{port}")
            while True:
                conn, addr = s.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                threading.Thread(target=handle_esp, args=(conn, addr), daemon=True).start()

    def handle_esp(conn, addr):
        global active_conn
        with conn_lock:
            active_conn = conn
        print(f"[TCP] ESP connected: {addr}")
        broadcast({"type": "sys", "connected": True, "raw": f"[SYS] ESP connected ({addr[0]})"})

        # Automatically push saved config 2 seconds after ESP connects
        timer = threading.Timer(2.0, auto_send_saved_config, args=(conn,))
        timer.daemon = True
        timer.start()

        buf = ""
        try:
            while True:
                chunk = conn.recv(2048)
                if not chunk:
                    break
                buf += chunk.decode("utf-8", errors="ignore")

                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    if line.strip():
                        process_line(line)

                if len(buf) > 4096:
                    buf = buf[-1024:]
        finally:
            timer.cancel()
            with conn_lock:
                if active_conn is conn:
                    active_conn = None
            conn.close()
            print(f"[TCP] ESP disconnected: {addr}")
            broadcast({"type": "sys", "connected": False, "raw": "[SYS] ESP disconnected"})

    threading.Thread(target=server_loop, daemon=True).start()


def no_cache(template_name: str):
    resp = make_response(render_template(template_name))
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    return resp


@app.route("/")
def index():
    return no_cache("index.html")


@app.route("/config")
def config_page():
    return no_cache("config.html")


@app.route("/api/status")
def api_status():
    with conn_lock:
        esp_connected = active_conn is not None
    return jsonify({"esp_connected": esp_connected})


@app.route("/api/config", methods=["GET", "POST"])
def api_config():
    if request.method == "GET":
        cfg = load_config()
        return jsonify({"config": cfg, "packet": build_cfg_packet(cfg).strip()})

    data = request.get_json(silent=True) or {}
    try:
        cfg = save_config(data)
        return jsonify({"ok": True, "config": cfg, "packet": build_cfg_packet(cfg).strip()})
    except (ValueError, TypeError) as exc:
        return jsonify({"ok": False, "error": f"Invalid value: {exc}"}), 400


@app.route("/api/config/reset", methods=["POST"])
def api_config_reset():
    cfg = save_config(DEFAULT_CONFIG)
    return jsonify({"ok": True, "config": cfg, "packet": build_cfg_packet(cfg).strip()})


@app.route("/api/config/send", methods=["POST"])
def api_config_send():
    data = request.get_json(silent=True) or {}
    try:
        cfg = save_config(data)
    except (ValueError, TypeError) as exc:
        return jsonify({"ok": False, "error": f"Invalid value: {exc}"}), 400

    packet = build_cfg_packet(cfg)
    with conn_lock:
        conn = active_conn
        if conn is None:
            return jsonify({
                "ok": False,
                "saved": True,
                "packet": packet.strip(),
                "error": "Saved on server, but ESP is not connected.",
            }), 503
        try:
            conn.sendall(packet.encode("utf-8"))
        except OSError as exc:
            return jsonify({
                "ok": False,
                "saved": True,
                "packet": packet.strip(),
                "error": f"Saved on server, but TCP send failed: {exc}",
            }), 500

    broadcast({"type": "sys", "raw": f"[TX] {packet.strip()}"})
    return jsonify({"ok": True, "saved": True, "packet": packet.strip()})


@app.route("/stream")
def stream():
    q = queue.Queue(maxsize=100)
    with clients_lock:
        clients.append(q)

    def event_stream():
        try:
            while True:
                try:
                    data = q.get(timeout=15)
                    yield f"data: {data}\n\n"
                except queue.Empty:
                    yield ": keep-alive\n\n"
        except GeneratorExit:
            pass
        finally:
            with clients_lock:
                if q in clients:
                    clients.remove(q)

    return Response(
        event_stream(),
        mimetype="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


if __name__ == "__main__":
    start_tcp_listener(port=65432)
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)