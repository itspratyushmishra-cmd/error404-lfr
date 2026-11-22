import json
import queue
import re
import socket
import threading
from flask import Flask, Response, render_template

app = Flask(__name__)

# Matches: [TEST] QTR:[1018,1019,...] | IR_L:0 IR_R:1 | Dist:207.7cm | Enc_L:0 Enc_R:0
LINE_RE = re.compile(
    r"QTR:\[([0-9,\s]+)\]\s*\|\s*"
    r"IR_L:(\d+)\s+IR_R:(\d+)\s*\|\s*"
    r"Dist:([0-9.]+)cm\s*\|\s*"
    r"Enc_L:(-?\d+)\s+Enc_R:(-?\d+)"
)

clients: list[queue.Queue] = []
clients_lock = threading.Lock()


def broadcast(payload: dict):
    msg = json.dumps(payload)
    with clients_lock:
        for q in list(clients):
            # Drop stale frame if browser hasn't consumed it yet -> zero lag buildup
            if q.full():
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass


def parse_line(line: str) -> dict:
    m = LINE_RE.search(line)
    if not m:
        return {"raw": line}
    return {
        "qtr": [int(x) for x in m.group(1).split(",") if x.strip()],
        "ir_l": int(m.group(2)),
        "ir_r": int(m.group(3)),
        "dist": float(m.group(4)),
        "enc_l": int(m.group(5)),
        "enc_r": int(m.group(6)),
    }


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
        buf = ""
        try:
            while True:
                chunk = conn.recv(1024)
                if not chunk:
                    break
                buf += chunk.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        broadcast(parse_line(line))
        finally:
            conn.close()
            print(f"[TCP] ESP disconnected: {addr}")

    threading.Thread(target=server_loop, daemon=True).start()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/stream")
def stream():
    q = queue.Queue(maxsize=1)
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