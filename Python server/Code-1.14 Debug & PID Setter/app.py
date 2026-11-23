import json
import os
import queue
import re
import shutil
import socket
import subprocess
import tempfile
import threading
import time
from flask import Flask, Response, jsonify, make_response, render_template, request, send_file

app = Flask(__name__)

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")

DEFAULT_CONFIG = {
    "Kp": 0.068,
    "Ki": 0.0008,
    "Kd": 0.0018,
    "speedScale": 1.0,
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

FLOAT_KEYS = {"Kp", "Ki", "Kd", "speedScale"}
CONFIG_ORDER = list(DEFAULT_CONFIG.keys())

# Telemetry regexes
SYNC_RE = re.compile(r"\[SYNC\]\s*LED_ON(?:(?:,|\s+at\s+ms:\s*)(\d+))?", re.IGNORECASE)
POS_RE = re.compile(r"\[POS\]\s*(?:Pos:\s*)?(-?\d+)(?:\s*\|\s*Vis:\s*([01]))?", re.IGNORECASE)
DAT_TEXT_RE = re.compile(
    r"\[DAT\]\s*"
    r"ENC:\s*L=(-?\d+)\s+R=(-?\d+)\s*\|\s*"
    r"IR:\s*L=([A-Za-z0-9-]+)\s+R=([A-Za-z0-9-]+)\s*\|\s*"
    r"US:\s*([^|]+?)\s*\|\s*"
    r"MOT:\s*L=\[([A-Za-z-]+):(-?\d+)\]\s+R=\[([A-Za-z-]+):(-?\d+)\]\s*\|\s*"
    r"PID:\s*\[Err:\s*(-?\d+(?:\.\d+)?)\s+P:\s*(-?\d+(?:\.\d+)?)\s+I:\s*(-?\d+(?:\.\d+)?)\s+D:\s*(-?\d+(?:\.\d+)?)\s+Out:\s*(-?\d+(?:\.\d+)?)\]\s*\|\s*"
    r"ST:\s*(\S+)",
    re.IGNORECASE,
)
ACTION_TRANS_RE = re.compile(r"\[ACTION\]\s*State:\s*(\S+)\s*->\s*(\S+)", re.IGNORECASE)
US_NUM_RE = re.compile(r"(-?\d+)")

# Calibration report regexes
CAL_ROW_RE = re.compile(
    r"S([0-7])\b[^|]*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(OK|WARN:\s*LOW)"
)
LEFT_AVG_RE = re.compile(r"Left Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")
RIGHT_AVG_RE = re.compile(r"Right Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")

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


def norm_dir(val: str) -> str:
    v = val.strip().upper()
    if v in ("1", "F", "FWD", "FORWARD"):
        return "FWD"
    if v in ("-1", "R", "REV", "REVERSE", "BACK"):
        return "REV"
    if v in ("0", "B", "BRK", "BRAKE", "STOP"):
        return "BRK"
    return v or "-"


def update_pos_hz() -> int:
    now = time.monotonic()
    pos_state["count"] += 1
    if now - pos_state["hz_time"] >= 1.0:
        pos_state["hz"] = pos_state["count"]
        pos_state["count"] = 0
        pos_state["hz_time"] = now
    return pos_state["hz"]


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

    # 1. [SYNC] LED_ON,<ms>  OR  [SYNC] LED_ON at ms: <ms>
    m = SYNC_RE.search(line)
    if m:
        mcu_ms = int(m.group(1)) if m.group(1) else None
        broadcast({"type": "sync", "mcu_ms": mcu_ms, "raw": line})
        return

    # 2. [POS] Pos: 3512 | Vis: 1  OR  $POS,3512,1
    m = POS_RE.search(line)
    if m:
        val = int(m.group(1))
        if 0 <= val <= 7000:
            vis = None if m.group(2) is None else (m.group(2) == "1")
            broadcast({"type": "pos", "pos": val, "vis": vis, "hz": update_pos_hz(), "raw": line})
        return

    if line.startswith("$POS,"):
        parts = line.split(",")
        if len(parts) >= 3:
            try:
                val = int(parts[1])
                if 0 <= val <= 7000:
                    broadcast({"type": "pos", "pos": val, "vis": parts[2] == "1", "hz": update_pos_hz(), "raw": line})
                return
            except ValueError:
                pass

    # 3. [DAT] ENC: L=... R=... | IR: L=... R=... | US: ... | MOT: L=[FWD:171] R=[FWD:171] | PID: [Err: ... P: ... I: ... D: ... Out: ...] | ST: FOLLOW
    m = DAT_TEXT_RE.search(line)
    if m:
        us_match = US_NUM_RE.search(m.group(5))
        us_dist = int(us_match.group(1)) if us_match else -1
        broadcast({
            "type": "dat",
            "encL": int(m.group(1)),
            "encR": int(m.group(2)),
            "irL": norm_ir(m.group(3)),
            "irR": norm_ir(m.group(4)),
            "us": us_dist,
            "dirL": norm_dir(m.group(6)),
            "pwmL": int(m.group(7)),
            "dirR": norm_dir(m.group(8)),
            "pwmR": int(m.group(9)),
            "err": round(float(m.group(10)), 2),
            "pTerm": round(float(m.group(11)), 2),
            "iTerm": round(float(m.group(12)), 2),
            "dTerm": round(float(m.group(13)), 2),
            "corr": round(float(m.group(14)), 2),
            "st": m.group(15).upper(),
            "raw": line,
        })
        return

    # 3b. Legacy CSV fallback: $DAT,<encL>,<encR>,<irL>,<irR>,<usDist>,<dirL>,<pwmL>,<dirR>,<pwmR>,<err>,<pTerm>,<iTerm>,<dTerm>,<corr>,<state>
    if line.startswith("$DAT,"):
        parts = line.split(",")
        if len(parts) >= 16:
            try:
                broadcast({
                    "type": "dat",
                    "encL": int(parts[1]),
                    "encR": int(parts[2]),
                    "irL": norm_ir(parts[3]),
                    "irR": norm_ir(parts[4]),
                    "us": int(parts[5]),
                    "dirL": norm_dir(parts[6]),
                    "pwmL": int(parts[7]),
                    "dirR": norm_dir(parts[8]),
                    "pwmR": int(parts[9]),
                    "err": round(float(parts[10]), 2),
                    "pTerm": round(float(parts[11]), 2),
                    "iTerm": round(float(parts[12]), 2),
                    "dTerm": round(float(parts[13]), 2),
                    "corr": round(float(parts[14]), 2),
                    "st": parts[15].upper(),
                    "raw": line,
                })
                return
            except ValueError:
                pass

    # 4. [ACTION] State: FOLLOW -> JPROBE
    m = ACTION_TRANS_RE.search(line)
    if m:
        broadcast({
            "type": "action",
            "from": m.group(1).upper(),
            "to": m.group(2).upper(),
            "raw": line,
        })
        return

    # 5. Calibration report rows & averages
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

    # 6. Status / system announcements
    if "[SYSTEM]" in line:
        st = "RACING" if "racing started" in line.lower() else None
        broadcast({"type": "system", "status": st, "raw": line})
        return
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

        # Auto-push saved config 2 seconds after ESP connects
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
    return jsonify({"esp_connected": esp_connected, "has_ffmpeg": shutil.which("ffmpeg") is not None})


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


@app.route("/api/convert_mp4", methods=["POST"])
def api_convert_mp4():
    """Fallback converter if the browser records WebM instead of native MP4."""
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        return jsonify({"ok": False, "error": "ffmpeg not found on server PATH"}), 501

    raw_bytes = request.get_data()
    if not raw_bytes:
        return jsonify({"ok": False, "error": "Empty video payload"}), 400

    tmp_in = tempfile.NamedTemporaryFile(delete=False, suffix=".webm")
    tmp_out = tempfile.NamedTemporaryFile(delete=False, suffix=".mp4")
    tmp_in_path, tmp_out_path = tmp_in.name, tmp_out.name
    tmp_in.close()
    tmp_out.close()

    try:
        with open(tmp_in_path, "wb") as f:
            f.write(raw_bytes)
        cmd = [
            ffmpeg, "-y", "-i", tmp_in_path,
            "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
            "-pix_fmt", "yuv420p", "-movflags", "+faststart",
            tmp_out_path,
        ]
        subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return send_file(tmp_out_path, mimetype="video/mp4", as_attachment=True, download_name="telemetry_recording.mp4")
    except subprocess.CalledProcessError:
        return jsonify({"ok": False, "error": "ffmpeg conversion failed"}), 500
    finally:
        if os.path.exists(tmp_in_path):
            try:
                os.remove(tmp_in_path)
            except OSError:
                pass


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