import io
import json
import os
import queue
import re
import shutil
import socket
import threading
import time
import uuid
import zipfile
from flask import Flask, Response, jsonify, make_response, render_template, request, send_file
from video_exporter import run_triple_export

app = Flask(__name__)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, "config.json")
SESSION_FILE = os.path.join(BASE_DIR, "session_data.jsonl")
CAM_DIR = os.path.join(BASE_DIR, "cam_frames")
CAM_INDEX_FILE = os.path.join(CAM_DIR, "index.jsonl")
MAX_SESSION_EVENTS = 60000

os.makedirs(CAM_DIR, exist_ok=True)

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
SYNC_RE = re.compile(r"(?:\[SYNC\]|\$SYNC)", re.IGNORECASE)
POS_RE = re.compile(r"\[POS\]\s*(?:Pos:\s*)?(-?\d+)(?:\s*\|\s*Vis:\s*([01]))?", re.IGNORECASE)
DAT_TEXT_RE = re.compile(
    r"\[DAT\]\s*"
    r"ENC:\s*L=(-?\d+)\s+R=(-?\d+)\s*\|\s*"
    r"IR:\s*L=([A-Za-z0-9-]+)\s+R=([A-Za-z0-9-]+)\s*\|\s*"
    r"US:\s*([^|]+?)\s*\|\s*"
    r"MOT:\s*L=\[([A-Za-z-]+):(-?\d+)\]\s+R=\[([A-Za-z-]+):(-?\d+)\]\s*\|\s*"
    r"PID:\s*\[Err:\s*([^\s\]]+)\s+P:\s*([^\s\]]+)\s+I:\s*([^\s\]]+)\s+D:\s*([^\s\]]+)\s+Out:\s*([^\s\]]+)\]\s*\|\s*"
    r"ST:\s*(\S+)",
    re.IGNORECASE,
)
ACTION_TRANS_RE = re.compile(r"\[ACTION\]\s*State:\s*(\S+)\s*->\s*(\S+)", re.IGNORECASE)
US_NUM_RE = re.compile(r"(-?\d+)")

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
session_lock = threading.Lock()

pos_state = {"count": 0, "hz_time": time.monotonic(), "hz": 0}

session_events: list[dict] = []
session_seq = 0
session_start_mono: float | None = None

# Camera state (live max-FPS stream + event-triggered frame capture)
cam_lock = threading.Lock()
cam_cond = threading.Condition(cam_lock)
latest_cam_jpeg: bytes | None = None
latest_cam_seq = 0
latest_cam_mono = 0.0
cam_fps_count = 0
cam_fps_time = time.monotonic()
cam_fps = 0

cam_records: list[dict] = []
cam_frame_seq = 0
last_saved_cam_seq = -1
last_saved_cam_file: str | None = None

# Export Jobs Store
export_jobs: dict[str, dict] = {}
export_lock = threading.Lock()


def load_session_from_disk():
    global session_events, session_seq, session_start_mono, cam_records, cam_frame_seq
    with session_lock:
        session_events = []
        session_seq = 0
        session_start_mono = None
        if os.path.exists(SESSION_FILE):
            try:
                with open(SESSION_FILE, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            ev = json.loads(line)
                            if "t" in ev and "d" in ev:
                                session_seq += 1
                                ev["seq"] = session_seq
                                session_events.append(ev)
                        except json.JSONDecodeError:
                            continue
                if len(session_events) > MAX_SESSION_EVENTS:
                    session_events = session_events[-MAX_SESSION_EVENTS:]
                if session_events:
                    session_start_mono = time.monotonic() - (float(session_events[-1]["t"]) / 1000.0)
            except OSError:
                pass

    with cam_lock:
        cam_records = []
        cam_frame_seq = 0
        if os.path.exists(CAM_INDEX_FILE):
            try:
                with open(CAM_INDEX_FILE, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            rec = json.loads(line)
                            full_path = os.path.join(CAM_DIR, rec["file"])
                            if os.path.exists(full_path):
                                cam_records.append({
                                    "seq": int(rec.get("seq", 0)),
                                    "t": float(rec["t"]),
                                    "file": rec["file"],
                                    "path": full_path,
                                })
                                cam_frame_seq += 1
                        except (json.JSONDecodeError, KeyError, ValueError):
                            continue
            except OSError:
                pass


def clear_cam_frames_disk():
    global cam_records, cam_frame_seq, last_saved_cam_seq, last_saved_cam_file
    with cam_lock:
        cam_records = []
        cam_frame_seq = 0
        last_saved_cam_seq = -1
        last_saved_cam_file = None
        for name in os.listdir(CAM_DIR):
            try:
                os.remove(os.path.join(CAM_DIR, name))
            except OSError:
                pass


def capture_cam_for_data_frame(ev_seq: int, t_ms: float) -> str | None:
    """Capture the latest phone camera frame synchronized to an incoming event."""
    global cam_frame_seq, last_saved_cam_seq, last_saved_cam_file
    now = time.monotonic()
    with cam_lock:
        if latest_cam_jpeg is None or (now - latest_cam_mono) > 3.0:
            return None

        if latest_cam_seq == last_saved_cam_seq and last_saved_cam_file is not None:
            fname = last_saved_cam_file
            fpath = os.path.join(CAM_DIR, fname)
        else:
            cam_frame_seq += 1
            fname = f"f_{cam_frame_seq:06d}.jpg"
            fpath = os.path.join(CAM_DIR, fname)
            try:
                with open(fpath, "wb") as f:
                    f.write(latest_cam_jpeg)
                last_saved_cam_seq = latest_cam_seq
                last_saved_cam_file = fname
            except OSError:
                return None

        rec = {"seq": ev_seq, "t": t_ms, "file": fname, "path": fpath}
        cam_records.append(rec)
        try:
            with open(CAM_INDEX_FILE, "a", encoding="utf-8") as f:
                f.write(json.dumps({"seq": ev_seq, "t": t_ms, "file": fname}, separators=(",", ":")) + "\n")
        except OSError:
            pass
        return fname


def rewrite_session_file_locked():
    try:
        with open(SESSION_FILE, "w", encoding="utf-8") as f:
            for ev in session_events:
                f.write(json.dumps(ev, separators=(",", ":")) + "\n")
    except OSError:
        pass


def push_sse_raw(payload: dict):
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


def record_and_broadcast(payload: dict):
    global session_seq, session_start_mono
    with session_lock:
        now = time.monotonic()
        if session_start_mono is None:
            session_start_mono = now
        t_ms = round((now - session_start_mono) * 1000.0, 1)
        session_seq += 1
        seq = session_seq

    # Record a camera frame on every incoming event (including pre-sync setup & calibration)
    cam_file = capture_cam_for_data_frame(seq, t_ms)

    ev = {"seq": seq, "t": t_ms, "d": payload}
    if cam_file:
        ev["cam"] = cam_file

    with session_lock:
        session_events.append(ev)
        if len(session_events) > MAX_SESSION_EVENTS:
            session_events.pop(0)
        try:
            with open(SESSION_FILE, "a", encoding="utf-8") as f:
                f.write(json.dumps(ev, separators=(",", ":")) + "\n")
        except OSError:
            pass

    push_sse_raw({"event": "record", "record": ev})


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
        return str(int(round(float(val) * 10000)))
    return str(int(val))


def build_cfg_packet(cfg: dict) -> str:
    values = ",".join(fmt_num(k, cfg[k]) for k in CONFIG_ORDER)
    return f"$CFG,{values}\n"


def auto_send_saved_config(target_conn: socket.socket):
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
    record_and_broadcast({"type": "sys", "raw": f"[AUTO-TX] {packet.strip()}"})


def safe_float(val: str, divisor: float = 1.0, decimals: int = 2):
    v = val.strip()
    try:
        return round(float(v) / divisor, decimals)
    except ValueError:
        return v if v else "-"


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
    if v in ("0", "B", "BRK", "BRAKE", "STOP", "STP"):
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


def process_line(raw_line: str):
    line = re.sub(r"[^\x20-\x7E]", "", raw_line).strip()
    if not line:
        return

    if SYNC_RE.search(line):
        num_m = re.search(r"(\d+)", line)
        mcu_ms = int(num_m.group(1)) if num_m else None
        record_and_broadcast({"type": "sync", "mcu_ms": mcu_ms, "raw": line})
        return

    m = POS_RE.search(line)
    if m:
        val = int(m.group(1))
        if 0 <= val <= 7000:
            vis = None if m.group(2) is None else (m.group(2) == "1")
            record_and_broadcast({"type": "pos", "pos": val, "vis": vis, "hz": update_pos_hz(), "raw": line})
        return

    if line.startswith("$POS,"):
        parts = line.split(",")
        if len(parts) >= 3:
            try:
                val = int(parts[1])
                if 0 <= val <= 7000:
                    record_and_broadcast({"type": "pos", "pos": val, "vis": parts[2] == "1", "hz": update_pos_hz(), "raw": line})
                return
            except ValueError:
                pass

    m = DAT_TEXT_RE.search(line)
    if m:
        us_match = US_NUM_RE.search(m.group(5))
        us_dist = int(us_match.group(1)) if us_match else -1
        record_and_broadcast({
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
            "err": safe_float(m.group(10)),
            "pTerm": safe_float(m.group(11), 10000.0),
            "iTerm": safe_float(m.group(12), 10000.0),
            "dTerm": safe_float(m.group(13), 10000.0),
            "corr": safe_float(m.group(14)),
            "st": m.group(15).upper(),
            "raw": line,
        })
        return

    if line.startswith("$DAT,"):
        parts = line.split(",")
        if len(parts) >= 16:
            try:
                record_and_broadcast({
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
                    "err": safe_float(parts[10]),
                    "pTerm": safe_float(parts[11], 10000.0),
                    "iTerm": safe_float(parts[12], 10000.0),
                    "dTerm": safe_float(parts[13], 10000.0),
                    "corr": safe_float(parts[14]),
                    "st": parts[15].strip().upper(),
                    "raw": line,
                })
                return
            except ValueError:
                pass

    m = ACTION_TRANS_RE.search(line)
    if m:
        record_and_broadcast({"type": "action", "from": m.group(1).upper(), "to": m.group(2).upper(), "raw": line})
        return

    m = CAL_ROW_RE.search(line)
    if m:
        record_and_broadcast({
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
        record_and_broadcast({"type": "cal_avg", "side": "L", "val": int(ml.group(1)), "ok": ml.group(2) == "OK", "raw": raw_line.rstrip("\r")})
        return
    mr = RIGHT_AVG_RE.search(line)
    if mr:
        record_and_broadcast({"type": "cal_avg", "side": "R", "val": int(mr.group(1)), "ok": mr.group(2) == "OK", "raw": raw_line.rstrip("\r")})
        return

    if "[SYSTEM]" in line:
        st = "RACING" if "racing started" in line.lower() else None
        record_and_broadcast({"type": "system", "status": st, "raw": line})
        return
    if "[TRACK]" in line:
        mode = "WHITE ON BLACK" if "WHITE Line" in line else ("BLACK ON WHITE" if "BLACK Line" in line else "UNCERTAIN")
        record_and_broadcast({"type": "track", "mode": mode, "raw": line})
        return
    if "[CAL RESULT]" in line:
        record_and_broadcast({"type": "cal_result", "ok": "SUCCESS" in line, "raw": line})
        return
    if "[CAL]" in line:
        record_and_broadcast({"type": "status", "status": "CALIBRATING", "raw": line})
        return
    if "[ERROR]" in line:
        record_and_broadcast({"type": "status", "status": "STOPPED", "raw": line})
        return
    if "[AUTO]" in line or "[USER]" in line or "[WIFI]" in line:
        record_and_broadcast({"type": "status", "status": "WAITING", "raw": line})
        return

    record_and_broadcast({"type": "log", "raw": raw_line.rstrip("\r")})


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
        record_and_broadcast({"type": "sys", "connected": True, "raw": f"[SYS] ESP connected ({addr[0]})"})

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
            record_and_broadcast({"type": "sys", "connected": False, "raw": "[SYS] ESP disconnected"})

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


@app.route("/phone")
def phone_cam_page():
    return no_cache("phone_cam.html")


@app.route("/camera")
def cam_view_page():
    return no_cache("cam_view.html")


@app.route("/replay")
def cam_replay_page():
    return no_cache("cam_replay.html")


@app.route("/api/status")
def api_status():
    with conn_lock:
        esp_connected = active_conn is not None
    return jsonify({"esp_connected": esp_connected, "has_ffmpeg": shutil.which("ffmpeg") is not None})


# ==================== PHONE CAMERA MAX-FPS STREAM & FRAME ACCESS ====================
@app.route("/api/cam/frame", methods=["POST"])
def api_cam_frame():
    global latest_cam_jpeg, latest_cam_seq, latest_cam_mono
    global cam_fps_count, cam_fps_time, cam_fps

    jpeg_bytes = request.get_data()
    if not jpeg_bytes:
        return jsonify({"ok": False}), 400

    now = time.monotonic()
    with cam_cond:
        latest_cam_jpeg = jpeg_bytes
        latest_cam_seq += 1
        latest_cam_mono = now
        cam_fps_count += 1
        if now - cam_fps_time >= 1.0:
            cam_fps = cam_fps_count
            cam_fps_count = 0
            cam_fps_time = now
        saved_count = len(cam_records)
        fps_now = cam_fps
        cam_cond.notify_all()

    with conn_lock:
        bot_online = active_conn is not None

    return jsonify({"ok": True, "recording": bot_online, "frames": saved_count, "fps": fps_now})


@app.route("/api/cam/status", methods=["GET"])
def api_cam_status():
    now = time.monotonic()
    with cam_lock:
        live = (latest_cam_jpeg is not None) and ((now - latest_cam_mono) < 3.0)
        fps = cam_fps if live else 0
        count = len(cam_records)
    with conn_lock:
        bot_online = active_conn is not None
    return jsonify({"live": live, "fps": fps, "recorded_frames": count, "bot_online": bot_online})


@app.route("/api/cam/list", methods=["GET"])
def api_cam_list():
    with cam_lock:
        items = [{"seq": r["seq"], "t": r["t"], "file": r["file"]} for r in cam_records]
    return jsonify({"frames": items})


@app.route("/api/cam/file/<filename>", methods=["GET"])
def api_cam_file(filename: str):
    safe_name = os.path.basename(filename)
    fpath = os.path.join(CAM_DIR, safe_name)
    if not os.path.exists(fpath):
        return "", 404
    resp = make_response(send_file(fpath, mimetype="image/jpeg"))
    resp.headers["Cache-Control"] = "public, max-age=3600"
    return resp


@app.route("/api/cam/mjpeg")
def api_cam_mjpeg():
    def generate():
        last_sent = -1
        while True:
            with cam_cond:
                if latest_cam_seq == last_sent:
                    cam_cond.wait(timeout=2.0)
                if latest_cam_jpeg is None or latest_cam_seq == last_sent:
                    continue
                frame = latest_cam_jpeg
                last_sent = latest_cam_seq
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-cache, no-store"},
    )


# ==================== SESSION PERSISTENCE & IMPORT/EXPORT ====================
@app.route("/api/session", methods=["GET"])
def api_session_get():
    with session_lock:
        return jsonify({"events": list(session_events)})


@app.route("/api/session/clear", methods=["POST"])
def api_session_clear():
    global session_events, session_seq, session_start_mono
    with session_lock:
        session_events = []
        session_seq = 0
        session_start_mono = None
        rewrite_session_file_locked()
    clear_cam_frames_disk()
    push_sse_raw({"event": "session_cleared"})
    return jsonify({"ok": True})


@app.route("/api/session/export", methods=["GET"])
def api_session_export():
    with session_lock:
        data = json.dumps({"version": 1, "events": session_events}, separators=(",", ":"))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return send_file(
        io.BytesIO(data.encode("utf-8")),
        mimetype="application/json",
        as_attachment=True,
        download_name=f"error404_session_{stamp}.json",
    )


@app.route("/api/session/import", methods=["POST"])
def api_session_import():
    global session_events, session_seq, session_start_mono
    payload = request.get_json(silent=True)
    if payload is None:
        return jsonify({"ok": False, "error": "Invalid JSON file"}), 400

    raw_events = payload.get("events") if isinstance(payload, dict) else payload
    if not isinstance(raw_events, list):
        return jsonify({"ok": False, "error": "JSON must contain an 'events' list"}), 400

    cleaned = []
    seq = 0
    for item in raw_events[-MAX_SESSION_EVENTS:]:
        if isinstance(item, dict) and "t" in item and "d" in item and isinstance(item["d"], dict):
            seq += 1
            ev = {"seq": seq, "t": float(item["t"]), "d": item["d"]}
            if "cam" in item:
                ev["cam"] = item["cam"]
            cleaned.append(ev)

    with session_lock:
        session_events = cleaned
        session_seq = seq
        session_start_mono = (time.monotonic() - (session_events[-1]["t"] / 1000.0)) if session_events else None
        rewrite_session_file_locked()

    push_sse_raw({"event": "session_reloaded"})
    return jsonify({"ok": True, "count": len(cleaned)})


# ==================== FULL DATA + CONFIG + FRAMES ZIP (NO MP4) ====================
@app.route("/api/bundle/export", methods=["GET"])
def api_bundle_export():
    """Export a ZIP containing config.json, session_data.json, and all recorded cam_frames (no MP4)."""
    cfg = load_config()
    with session_lock:
        events_copy = list(session_events)
    with cam_lock:
        cam_files = [
            (r["file"], r["path"])
            for r in cam_records
            if os.path.exists(r["path"])
        ]
        index_exists = os.path.exists(CAM_INDEX_FILE)

    buf = io.BytesIO()
    seen_files = set()
    with zipfile.ZipFile(buf, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("config.json", json.dumps(cfg, indent=2))
        zf.writestr("session_data.json", json.dumps({"version": 1, "events": events_copy}, separators=(",", ":")))
        if index_exists:
            zf.write(CAM_INDEX_FILE, "cam_frames/index.jsonl")
        for fname, fpath in cam_files:
            if fname not in seen_files:
                seen_files.add(fname)
                zf.write(fpath, f"cam_frames/{fname}")

    buf.seek(0)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return send_file(
        buf,
        mimetype="application/zip",
        as_attachment=True,
        download_name=f"error404_data_bundle_{stamp}.zip",
    )


@app.route("/api/bundle/import", methods=["POST"])
def api_bundle_import():
    """Import a full ZIP (config.json + session_data.json + cam_frames/) and restore server state."""
    global session_events, session_seq, session_start_mono
    global cam_records, cam_frame_seq, last_saved_cam_seq, last_saved_cam_file

    raw_bytes = request.get_data()
    if not raw_bytes:
        return jsonify({"ok": False, "error": "Empty ZIP payload"}), 400

    try:
        zf = zipfile.ZipFile(io.BytesIO(raw_bytes), "r")
    except zipfile.BadZipFile:
        return jsonify({"ok": False, "error": "Invalid ZIP file"}), 400

    with zf:
        names = set(zf.namelist())

        # 1. Restore config.json if present
        if "config.json" in names:
            try:
                cfg_data = json.loads(zf.read("config.json").decode("utf-8"))
                if isinstance(cfg_data, dict):
                    save_config(cfg_data)
            except (ValueError, TypeError, KeyError):
                pass

        # 2. Restore cam_frames/
        clear_cam_frames_disk()
        restored_files = set()
        for name in names:
            if name.startswith("cam_frames/") and not name.endswith("/"):
                base = os.path.basename(name)
                if not base:
                    continue
                target = os.path.join(CAM_DIR, base)
                with open(target, "wb") as out_f:
                    out_f.write(zf.read(name))
                restored_files.add(base)

        # 3. Restore session_data.json
        cleaned = []
        seq = 0
        if "session_data.json" in names:
            try:
                payload = json.loads(zf.read("session_data.json").decode("utf-8"))
                raw_events = payload.get("events") if isinstance(payload, dict) else payload
                if isinstance(raw_events, list):
                    for item in raw_events[-MAX_SESSION_EVENTS:]:
                        if isinstance(item, dict) and "t" in item and "d" in item and isinstance(item["d"], dict):
                            seq += 1
                            ev = {"seq": seq, "t": float(item["t"]), "d": item["d"]}
                            if "cam" in item:
                                ev["cam"] = os.path.basename(str(item["cam"]))
                            cleaned.append(ev)
            except (ValueError, TypeError):
                return jsonify({"ok": False, "error": "Corrupt session_data.json in ZIP"}), 400

        with session_lock:
            session_events = cleaned
            session_seq = seq
            session_start_mono = (time.monotonic() - (session_events[-1]["t"] / 1000.0)) if session_events else None
            rewrite_session_file_locked()

        # 4. Rebuild cam_records in memory from index.jsonl or session events
        with cam_lock:
            cam_records = []
            cam_frame_seq = 0
            last_saved_cam_seq = -1
            last_saved_cam_file = None
            if os.path.exists(CAM_INDEX_FILE):
                with open(CAM_INDEX_FILE, "r", encoding="utf-8") as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            rec = json.loads(line)
                            fname = os.path.basename(rec["file"])
                            fpath = os.path.join(CAM_DIR, fname)
                            if os.path.exists(fpath):
                                cam_records.append({
                                    "seq": int(rec.get("seq", 0)),
                                    "t": float(rec["t"]),
                                    "file": fname,
                                    "path": fpath,
                                })
                                cam_frame_seq += 1
                        except (json.JSONDecodeError, KeyError, ValueError):
                            continue
            else:
                for ev in cleaned:
                    fname = ev.get("cam")
                    if fname and fname in restored_files:
                        fpath = os.path.join(CAM_DIR, fname)
                        cam_records.append({"seq": ev["seq"], "t": ev["t"], "file": fname, "path": fpath})
                        cam_frame_seq += 1

    push_sse_raw({"event": "session_reloaded"})
    return jsonify({"ok": True, "events": len(cleaned), "frames": len(cam_records)})


# ==================== SERVER-SIDE RTX NVENC TRIPLE-MP4 EXPORT ====================
@app.route("/api/export/start", methods=["POST"])
def api_export_start():
    body = request.get_json(silent=True) or {}
    from_sync = bool(body.get("from_sync", True))
    show_sensor = bool(body.get("show_sensor", False))
    map_zoom = float(body.get("map_zoom", 6.0))

    with session_lock:
        events_copy = list(session_events)
    with cam_lock:
        cam_copy = list(cam_records)
    config_copy = load_config()

    if len(events_copy) < 2:
        return jsonify({"ok": False, "error": "Not enough data recorded to export."}), 400

    job_id = uuid.uuid4().hex[:10]
    with export_lock:
        export_jobs[job_id] = {"status": "running", "percent": 0, "stage": "Starting", "file": None, "error": None}

    def worker():
        def on_progress(pct, stage):
            with export_lock:
                export_jobs[job_id]["percent"] = pct
                export_jobs[job_id]["stage"] = stage

        try:
            zip_path = run_triple_export(events_copy, cam_copy, config_copy, from_sync, show_sensor, map_zoom, on_progress)
            with export_lock:
                export_jobs[job_id]["status"] = "done"
                export_jobs[job_id]["percent"] = 100
                export_jobs[job_id]["file"] = zip_path
        except Exception as exc:
            with export_lock:
                export_jobs[job_id]["status"] = "error"
                export_jobs[job_id]["error"] = str(exc)

    threading.Thread(target=worker, daemon=True).start()
    return jsonify({"ok": True, "job_id": job_id})


@app.route("/api/export/status/<job_id>", methods=["GET"])
def api_export_status(job_id: str):
    with export_lock:
        job = export_jobs.get(job_id)
        if not job:
            return jsonify({"ok": False, "error": "Job not found"}), 404
        return jsonify({
            "ok": True,
            "status": job["status"],
            "percent": job["percent"],
            "stage": job["stage"],
            "error": job["error"],
        })


@app.route("/api/export/download/<job_id>", methods=["GET"])
def api_export_download(job_id: str):
    with export_lock:
        job = export_jobs.get(job_id)
        if not job or job["status"] != "done" or not job["file"] or not os.path.exists(job["file"]):
            return jsonify({"ok": False, "error": "Export file not ready"}), 404
        zip_path = job["file"]

    stamp = time.strftime("%Y%m%d_%H%M%S")
    return send_file(
        zip_path,
        mimetype="application/zip",
        as_attachment=True,
        download_name=f"error404_mp4_export_{stamp}.zip",
    )


# ==================== CONFIG ENDPOINTS ====================
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

    record_and_broadcast({"type": "sys", "raw": f"[TX] {packet.strip()}"})
    return jsonify({"ok": True, "saved": True, "packet": packet.strip()})


@app.route("/stream")
def stream():
    q = queue.Queue(maxsize=200)
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
    load_session_from_disk()
    start_tcp_listener(port=65432)
    app.run(host="0.0.0.0", port=5000, threaded=True, debug=False)