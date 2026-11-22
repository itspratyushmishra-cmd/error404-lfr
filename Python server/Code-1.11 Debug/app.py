import json
import queue
import re
import socket
import threading
import time
from flask import Flask, Response, make_response, render_template

app = Flask(__name__)

# Calibration report regexes
CAL_ROW_RE = re.compile(
    r"S([0-7])\b[^|]*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(OK|WARN:\s*LOW)"
)
LEFT_AVG_RE = re.compile(r"Left Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")
RIGHT_AVG_RE = re.compile(r"Right Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")

ACT_WORDS = {"STOP", "STRAIGHT", "LEFT", "RIGHT", "BACK", "UTURN", "FORWARD", "REVERSE"}

clients: list[queue.Queue] = []
clients_lock = threading.Lock()

pos_state = {"count": 0, "hz_time": time.monotonic(), "hz": 0}


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

    # 2. $DAT,encL,encR,irL,irR,us,state[,act]  (e.g. $DAT,0,-1,0,0,90,PIVOT)
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

    # 2b. Standalone action prints ("STOP", "STRAIGHT", "LEFT", "RIGHT", etc.)
    #     or prefixed "$ACT,STRAIGHT" / "[ACT] STRAIGHT"
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
        print(f"[TCP] ESP connected: {addr}")
        broadcast({"type": "sys", "connected": True, "raw": f"[SYS] ESP connected ({addr[0]})"})
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
            conn.close()
            print(f"[TCP] ESP disconnected: {addr}")
            broadcast({"type": "sys", "connected": False, "raw": "[SYS] ESP disconnected"})

    threading.Thread(target=server_loop, daemon=True).start()


@app.route("/")
def index():
    resp = make_response(render_template("index.html"))
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    return resp


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