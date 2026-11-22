import json
import queue
import re
import socket
import threading
import time
from flask import Flask, Response, render_template

app = Flask(__name__)

# Matches calibration report rows: " S0 (Far L)|   120   |   890   |   770   |   OK"
CAL_ROW_RE = re.compile(
    r"^\s*S([0-7])\b[^|]*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*(OK|WARN:\s*LOW)"
)
LEFT_AVG_RE = re.compile(r"Left Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")
RIGHT_AVG_RE = re.compile(r"Right Side Avg Span.*?:\s*(\d+)\s*\((OK|LOW)\)")
POS_COMPLETE_RE = re.compile(r"\[POS\]\s*(\d+)(?=\s*\[POS\]|\s*[\r\n])")

clients: list[queue.Queue] = []
clients_lock = threading.Lock()

# Rate-limiting state for 500 Hz [POS] stream
pos_state = {"last_emit": 0.0, "count": 0, "hz_time": time.monotonic(), "hz": 0}


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


def emit_pos(val: int):
    if not (0 <= val <= 7000):
        return
    now = time.monotonic()
    pos_state["count"] += 1
    if now - pos_state["hz_time"] >= 1.0:
        pos_state["hz"] = pos_state["count"]
        pos_state["count"] = 0
        pos_state["hz_time"] = now

    # Cap browser pushes to ~50 Hz (20 ms) -> always latest value, zero lag buildup
    if now - pos_state["last_emit"] >= 0.02:
        pos_state["last_emit"] = now
        broadcast({"type": "pos", "pos": val, "hz": pos_state["hz"], "raw": f"[POS] {val}"})


def process_line(line: str):
    # Handle [POS] if println() is used
    if line.startswith("[POS]"):
        nums = re.findall(r"\[POS\]\s*(\d+)", line)
        if nums:
            emit_pos(int(nums[-1]))
            return

    # Parse calibration table rows
    m = CAL_ROW_RE.match(line)
    if m:
        broadcast({
            "type": "cal_row",
            "idx": int(m.group(1)),
            "min": int(m.group(2)),
            "max": int(m.group(3)),
            "span": int(m.group(4)),
            "ok": m.group(5) == "OK",
            "raw": line,
        })
        return

    # Parse left/right average spans
    ml = LEFT_AVG_RE.search(line)
    if ml:
        broadcast({"type": "cal_avg", "side": "L", "val": int(ml.group(1)), "ok": ml.group(2) == "OK", "raw": line})
        return
    mr = RIGHT_AVG_RE.search(line)
    if mr:
        broadcast({"type": "cal_avg", "side": "R", "val": int(mr.group(1)), "ok": mr.group(2) == "OK", "raw": line})
        return

    # Parse status / mode announcements
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
    if "[AUTO]" in line or "[USER]" in line:
        broadcast({"type": "status", "status": "WAITING", "raw": line})
        return

    broadcast({"type": "log", "raw": line})


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

                # 1. Process all complete newline-terminated lines first
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.rstrip("\r")
                    if line:
                        process_line(line)

                # 2. Extract complete glued "[POS] 3500" tokens when Mega uses print() without '\n'
                last_end = 0
                for m in POS_COMPLETE_RE.finditer(buf):
                    emit_pos(int(m.group(1)))
                    last_end = m.end()
                if last_end:
                    buf = buf[last_end:]

                # Safety guard against unbounded buffer on malformed stream
                if len(buf) > 4096:
                    buf = buf[-1024:]
        finally:
            conn.close()
            print(f"[TCP] ESP disconnected: {addr}")
            broadcast({"type": "sys", "connected": False, "raw": "[SYS] ESP disconnected"})

    threading.Thread(target=server_loop, daemon=True).start()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream")
def stream():
    q = queue.Queue(maxsize=60)
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