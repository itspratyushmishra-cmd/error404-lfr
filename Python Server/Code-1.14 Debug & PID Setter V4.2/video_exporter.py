import collections
import concurrent.futures
import json
import math
import os
import shutil
import subprocess
import tempfile
import zipfile
from PIL import Image, ImageDraw, ImageFont, ImageOps

# Physical Robot Geometry (matches real bot)
CM_PER_TICK = 16.0 / 115.0
WHEELBASE_CM = 17.8
SENSOR_OFFSET_CM = 19.0
SENSOR_WIDTH_CM = 6.5

LABELS = ["S0 (Far L)", "S1", "S2", "S3", "S4", "S5", "S6", "S7 (Far R)"]
CONSOLE_COLORS = {
    "r-log": "#e6edf3",
    "r-pos": "#6e7681",
    "r-dat": "#79c0ff",
    "r-act": "#79c0ff",
    "r-sync": "#bc8cff",
    "r-sys": "#d29922",
    "r-err": "#f85149",
    "r-ok": "#3fb950",
}

_FONTS = {}
_DATA_BG = None
_NO_CAM_BYTES = None


def _load_font(size: int, bold: bool = False):
    candidates = [
        "C:/Windows/Fonts/consolab.ttf" if bold else "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/lucon.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
        if bold
        else "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                pass
    return ImageFont.load_default()


def _init_worker():
    global _FONTS, _DATA_BG, _NO_CAM_BYTES
    _FONTS = {
        "12": _load_font(12, True),
        "13": _load_font(13, True),
        "14": _load_font(14, False),
        "14b": _load_font(14, True),
        "15": _load_font(15, False),
        "15b": _load_font(15, True),
        "18b": _load_font(18, True),
        "20b": _load_font(20, True),
        "24b": _load_font(24, True),
        "26b": _load_font(26, True),
        "42b": _load_font(42, True),
    }
    _DATA_BG = _build_static_data_bg()

    blank = Image.new("RGB", (1280, 720), "#090c10")
    bd = ImageDraw.Draw(blank)
    txt = "NO PHONE CAMERA FRAME AT THIS TIMESTAMP"
    tw = bd.textbbox((0, 0), txt, font=_FONTS["24b"])[2]
    bd.text(((1280 - tw) // 2, 345), txt, fill="#8b949e", font=_FONTS["24b"])
    _NO_CAM_BYTES = blank.tobytes()


def _draw_card(draw: ImageDraw.ImageDraw, x, y, w, h, title, right_title=""):
    draw.rounded_rectangle([x, y, x + w, y + h], radius=8, fill="#161b22", outline="#30363d", width=2)
    draw.text((x + 14, y + 8), title, fill="#8b949e", font=_FONTS["14b"])
    if right_title:
        rw = draw.textbbox((0, 0), right_title, font=_FONTS["14b"])[2]
        draw.text((x + w - 14 - rw, y + 8), right_title, fill="#8b949e", font=_FONTS["14b"])


def _draw_pill(draw: ImageDraw.ImageDraw, x, y, text, fg, bg, border):
    tw = draw.textbbox((0, 0), text, font=_FONTS["15b"])[2]
    w = tw + 20
    draw.rounded_rectangle([x, y, x + w, y + 28], radius=5, fill=bg, outline=border, width=2)
    draw.text((x + 10, y + 6), text, fill=fg, font=_FONTS["15b"])
    return w


def _build_static_data_bg() -> Image.Image:
    W, H = 1920, 1080
    img = Image.new("RGB", (W, H), "#0d1117")
    d = ImageDraw.Draw(img)

    d.rounded_rectangle([14, 14, W - 14, 68], radius=8, fill="#161b22", outline="#30363d", width=2)
    _draw_pill(d, 28, 27, "ERROR 404 MONITOR", "#39c5cf", "#0f3238", "#39c5cf")

    L_X, L_W = 14, 1130
    R_X, R_W = L_X + L_W + 14, W - (L_X + L_W + 14) - 14
    BODY_Y, BODY_H = 80, H - 94

    pY, pH = BODY_Y, 265
    _draw_card(d, L_X, pY, L_W, pH, "LIVE QTR POSITION ([POS])", "0 = Far Left | 3500 = Center | 7000 = Far Right")
    bX, bY, bW, bH = L_X + 16, pY + 84, L_W - 32, 20
    d.rounded_rectangle([bX, bY, bX + bW, bY + bH], radius=10, fill="#0d1117", outline="#30363d")
    d.line([(bX + bW // 2, bY), (bX + bW // 2, bY + bH)], fill="#484f58", width=2)
    tX, tY, tW, tH = L_X + 16, pY + 114, L_W - 32, 136
    d.rectangle([tX, tY, tX + tW, tY + tH], fill="#090c10", outline="#262c36")
    d.line([(tX, tY + tH // 2), (tX + tW, tY + tH // 2)], fill="#262c36", width=1)

    r2Y, r2H = pY + pH + 12, 120
    wEnc, wIr, wUs = 270, 240, 275
    wAct = L_W - wEnc - wIr - wUs - 36

    _draw_card(d, L_X, r2Y, wEnc, r2H, "ENCODERS")
    for ex, lbl in [(L_X + 12, "LEFT"), (L_X + 140, "RIGHT")]:
        d.rounded_rectangle([ex, r2Y + 34, ex + 118, r2Y + 106], radius=6, fill="#0d1117", outline="#262c36")
        d.text((ex + 40, r2Y + 42), lbl, fill="#8b949e", font=_FONTS["12"])

    irX = L_X + wEnc + 12
    _draw_card(d, irX, r2Y, wIr, r2H, "BACKGROUND IR")

    usX = irX + wIr + 12
    _draw_card(d, usX, r2Y, wUs, r2H, "ULTRASONIC", "0-400 cm")
    d.rectangle([usX + 14, r2Y + 86, usX + wUs - 14, r2Y + 100], fill="#0d1117")

    actX = usX + wUs + 12
    _draw_card(d, actX, r2Y, wAct, r2H, "STATE / ACTION")

    r3Y, r3H = r2Y + r2H + 12, 125
    wMot, wPid = 480, L_W - 492
    _draw_card(d, L_X, r3Y, wMot, r3H, "MOTORS (DIR & PWM)", "0-255")
    for mx_box in (L_X + 12, L_X + 246):
        d.rounded_rectangle([mx_box, r3Y + 34, mx_box + 222, r3Y + 110], radius=6, fill="#0d1117", outline="#262c36")
        d.rectangle([mx_box + 12, r3Y + 78, mx_box + 210, r3Y + 94], fill="#161b22")

    pidX = L_X + wMot + 12
    _draw_card(d, pidX, r3Y, wPid, r3H, "PID TELEMETRY", "Live Terms")
    bw_pid = (wPid - 24 - 32) // 5
    for idx, lbl in enumerate(["ERR", "P", "I", "D", "OUT"]):
        bx = pidX + 12 + idx * (bw_pid + 8)
        d.rounded_rectangle([bx, r3Y + 34, bx + bw_pid, r3Y + 110], radius=6, fill="#0d1117", outline="#262c36")
        d.text((bx + 12, r3Y + 42), lbl, fill="#8b949e", font=_FONTS["13"])

    r4Y = r3Y + r3H + 12
    r4H = BODY_Y + BODY_H - r4Y
    _draw_card(d, L_X, r4Y, L_W, r4H, "QTR SENSOR CALIBRATION REPORT", "Min Required Span: 120")
    cols = [L_X + 16, L_X + 200, L_X + 340, L_X + 480, L_X + 600, L_X + L_W - 80]
    for cx_t, hdr in zip(cols, ["Sensor", "Min", "Max", "Span", "Bar", "Status"]):
        d.text((cx_t, r4Y + 38), hdr, fill="#8b949e", font=_FONTS["14b"])
    d.line([(L_X + 14, r4Y + 60), (L_X + L_W - 14, r4Y + 60)], fill="#21262d")
    for i in range(8):
        ry = r4Y + 72 + i * 38
        d.text((cols[0], ry), LABELS[i], fill="#e6edf3", font=_FONTS["15"])
        d.rectangle([cols[4], ry + 3, cols[4] + 280, ry + 15], fill="#0d1117")
        d.line([(L_X + 14, ry + 26), (L_X + L_W - 14, ry + 26)], fill="#21262d")

    _draw_card(d, R_X, BODY_Y, R_W, BODY_H, "LOGS, TRANSITIONS & REPORTS")
    cX, cY, cW, cH = R_X + 12, BODY_Y + 34, R_W - 24, BODY_H - 46
    d.rounded_rectangle([cX, cY, cX + cW, cY + cH], radius=6, fill="#090c10", outline="#262c36")

    return img


def fmt_time(ms: float) -> str:
    neg = ms < 0
    s = abs(ms) / 1000.0
    m = int(s // 60)
    sec = s % 60.0
    return f"{'-' if neg else ''}{m:02d}:{sec:04.1f}"


def has_nvenc(ffmpeg_path: str) -> bool:
    try:
        res = subprocess.run(
            [ffmpeg_path, "-hide_banner", "-encoders"],
            capture_output=True, text=True, timeout=5,
        )
        return "h264_nvenc" in res.stdout
    except Exception:
        return False


def open_ffmpeg_pipe(ffmpeg_path: str, width: int, height: int, fps: int, out_path: str, use_nvenc: bool):
    cmd = [
        ffmpeg_path, "-y",
        "-f", "rawvideo",
        "-vcodec", "rawvideo",
        "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}",
        "-r", str(fps),
        "-i", "-",
    ]
    if use_nvenc:
        cmd += [
            "-c:v", "h264_nvenc",
            "-preset", "p4",
            "-tune", "hq",
            "-rc", "vbr",
            "-cq", "22",
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
        ]
    else:
        cmd += [
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-crf", "20",
            "-pix_fmt", "yuv420p",
            "-movflags", "+faststart",
        ]
    cmd.append(out_path)
    return subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class ReplayState:
    def __init__(self, show_sensor: bool = False):
        self.show_sensor = show_sensor
        self.pos = 3500
        self.vis = None
        self.hz = 0
        self.status = "IDLE"
        self.cal_txt = "CAL: -"
        self.cal_ok = None
        self.track_txt = "TRACK: -"
        self.track_err = False
        self.synced = False
        self.sync_mcu_ms = None
        self.sync_t = None
        self.encL = "-"
        self.encR = "-"
        self.irL = None
        self.irR = None
        self.us_dist = None
        self.act = "-"
        self.trans = ""
        self.dirL = "-"
        self.pwmL = 0
        self.dirR = "-"
        self.pwmR = 0
        self.pidErr = "-"
        self.pidP = "-"
        self.pidI = "-"
        self.pidD = "-"
        self.pidCorr = "-"
        self.avgL = None
        self.avgR = None
        self.rows = [None] * 8
        self.pos_list = []
        self.raw_list = []

        self.x = 0.0
        self.y = 0.0
        self.theta = -math.pi / 2.0
        self.dist_cm = 0.0
        self.speed_cms = 0.0
        self.prev_encL = None
        self.prev_encR = None
        self.prev_dat_t = None
        self.was_vis = False
        self.bot_path = [(0.0, 0.0)]
        self.line_pts = []

    def _raw_meta(self, d: dict):
        t = d.get("type")
        if t == "pos": return (self.show_sensor, "r-pos")
        if t == "dat": return (self.show_sensor, "r-dat")
        if t == "action": return (True, "r-act")
        if t == "sync": return (True, "r-sync")
        if t == "system": return (True, "r-ok")
        if t == "cal_row": return (True, "r-log" if d.get("ok") else "r-err")
        if t == "cal_avg": return (True, "r-log")
        if t == "cal_result": return (True, "r-ok" if d.get("ok") else "r-err")
        if t == "track": return (True, "r-err" if d.get("mode") == "UNCERTAIN" else "r-ok")
        if t == "status": return (True, "r-err" if d.get("status") == "STOPPED" else "r-sys")
        if t == "sys": return (True, "r-sys")
        return (True, "r-log")

    def apply(self, rec: dict):
        d = rec["d"]
        t_ms = float(rec["t"])
        vis_raw, cls = self._raw_meta(d)
        if vis_raw and d.get("raw"):
            self.raw_list.append((d["raw"], cls))
            if len(self.raw_list) > 80:
                self.raw_list.pop(0)

        t = d.get("type")
        if t == "sync":
            self.synced = True
            self.sync_mcu_ms = d.get("mcu_ms")
            self.sync_t = t_ms
        elif t == "pos":
            self.pos = int(d["pos"])
            self.vis = d.get("vis")
            self.hz = int(d.get("hz", 0))
            self.pos_list.append(self.pos)
            if len(self.pos_list) > 200:
                self.pos_list.pop(0)
            if not self.vis:
                self.was_vis = False
        elif t == "dat":
            self.encL = d["encL"]
            self.encR = d["encR"]
            self.irL = d["irL"]
            self.irR = d["irR"]
            self.us_dist = d["us"]
            self.dirL = d["dirL"]
            self.pwmL = d["pwmL"]
            self.dirR = d["dirR"]
            self.pwmR = d["pwmR"]
            self.pidErr = d["err"]
            self.pidP = d["pTerm"]
            self.pidI = d["iTerm"]
            self.pidD = d["dTerm"]
            self.pidCorr = d["corr"]
            self.status = d["st"]
            self.act = d["st"]

            eL, eR = float(d["encL"]), float(d["encR"])
            if self.prev_encL is None:
                self.prev_encL, self.prev_encR, self.prev_dat_t = eL, eR, t_ms
            else:
                dL = (eL - self.prev_encL) * CM_PER_TICK
                dR = (eR - self.prev_encR) * CM_PER_TICK
                self.prev_encL, self.prev_encR = eL, eR
                dS = (dL + dR) / 2.0
                dTheta = (dL - dR) / WHEELBASE_CM
                mid = self.theta + dTheta / 2.0
                self.x += dS * math.cos(mid)
                self.y += dS * math.sin(mid)
                self.theta += dTheta
                self.dist_cm += abs(dS)

                dt_sec = (t_ms - self.prev_dat_t) / 1000.0
                if 0.005 < dt_sec < 2.0:
                    inst = dS / dt_sec
                    self.speed_cms = 0.0 if abs(dS) < 1e-4 else (0.55 * inst + 0.45 * self.speed_cms)
                elif dt_sec >= 2.0:
                    self.speed_cms = 0.0
                self.prev_dat_t = t_ms

                lx0, ly0 = self.bot_path[-1]
                if math.hypot(self.x - lx0, self.y - ly0) >= 0.25:
                    self.bot_path.append((self.x, self.y))
                    if len(self.bot_path) > 4000:
                        self.bot_path.pop(0)

                if self.vis:
                    lat = ((self.pos - 3500.0) / 3500.0) * (SENSOR_WIDTH_CM / 2.0)
                    lx = self.x + math.cos(self.theta) * SENSOR_OFFSET_CM - math.sin(self.theta) * lat
                    ly = self.y + math.sin(self.theta) * SENSOR_OFFSET_CM + math.cos(self.theta) * lat
                    if not self.line_pts or not self.was_vis or math.hypot(lx - self.line_pts[-1][0], ly - self.line_pts[-1][1]) >= 0.25:
                        self.line_pts.append((lx, ly, not self.was_vis))
                        if len(self.line_pts) > 5000:
                            self.line_pts.pop(0)
                    self.was_vis = True
                else:
                    self.was_vis = False
        elif t == "action":
            self.status = d["to"]
            self.act = d["to"]
            self.trans = f"{d['from']} -> {d['to']}"
        elif t == "system" and d.get("status"):
            self.status = d["status"]
        elif t == "cal_row":
            self.rows[d["idx"]] = d
        elif t == "cal_avg":
            if d["side"] == "L":
                self.avgL = d
            else:
                self.avgR = d
        elif t == "cal_result":
            self.cal_txt = "CAL: OK" if d["ok"] else "CAL: LOW"
            self.cal_ok = d["ok"]
        elif t == "track":
            self.track_txt = f"TRACK: {d['mode']}"
            self.track_err = d["mode"] == "UNCERTAIN"
        elif t == "status":
            self.status = d["status"]

    def snapshot(self) -> dict:
        return {
            "pos": self.pos, "vis": self.vis, "hz": self.hz, "status": self.status,
            "cal_txt": self.cal_txt, "cal_ok": self.cal_ok,
            "track_txt": self.track_txt, "track_err": self.track_err,
            "synced": self.synced, "sync_mcu_ms": self.sync_mcu_ms, "sync_t": self.sync_t,
            "encL": self.encL, "encR": self.encR, "irL": self.irL, "irR": self.irR,
            "us_dist": self.us_dist, "act": self.act, "trans": self.trans,
            "dirL": self.dirL, "pwmL": self.pwmL, "dirR": self.dirR, "pwmR": self.pwmR,
            "pidErr": self.pidErr, "pidP": self.pidP, "pidI": self.pidI,
            "pidD": self.pidD, "pidCorr": self.pidCorr,
            "avgL": self.avgL, "avgR": self.avgR,
            "rows": list(self.rows),
            "pos_list": list(self.pos_list),
            "raw_list": list(self.raw_list),
            "x": self.x, "y": self.y, "theta": self.theta,
            "dist_cm": self.dist_cm, "speed_cms": self.speed_cms,
            "bot_path": list(self.bot_path),
            "line_pts": list(self.line_pts),
        }


def _rot_pt(lx: float, ly: float, cx: float, cy: float, cos_a: float, sin_a: float, S: float):
    return (cx + (lx * cos_a - ly * sin_a) * S, cy + (lx * sin_a + ly * cos_a) * S)


def _draw_rot_rect(draw, x1, y1, x2, y2, cx, cy, ca, sa, S, fill, outline=None, width=1):
    pts = [
        _rot_pt(x1, y1, cx, cy, ca, sa, S),
        _rot_pt(x2, y1, cx, cy, ca, sa, S),
        _rot_pt(x2, y2, cx, cy, ca, sa, S),
        _rot_pt(x1, y2, cx, cy, ca, sa, S),
    ]
    draw.polygon(pts, fill=fill, outline=outline)
    if outline and width > 1:
        draw.line(pts + [pts[0]], fill=outline, width=width)


def _draw_real_bot(draw: ImageDraw.ImageDraw, s: dict, cx: float, cy: float, S: float):
    ang = s["theta"] + math.pi / 2.0
    ca, sa = math.cos(ang), math.sin(ang)

    _draw_rot_rect(draw, -9.75, -2.8, -7.5, 2.8, cx, cy, ca, sa, S, fill="#11161d", outline="#8b949e", width=2)
    _draw_rot_rect(draw, 7.5, -2.8, 9.75, 2.8, cx, cy, ca, sa, S, fill="#11161d", outline="#8b949e", width=2)
    _draw_rot_rect(draw, -7.5, -10.5, 7.5, 2.5, cx, cy, ca, sa, S, fill="#9a6725", outline="#d49b4b", width=2)
    _draw_rot_rect(draw, -5.0, -19.5, 5.0, -10.5, cx, cy, ca, sa, S, fill="#a8732d", outline="#d49b4b", width=2)

    irL_col = "#e6edf3" if s["irL"] == "WHITE" else "#1f6feb"
    irR_col = "#e6edf3" if s["irR"] == "WHITE" else "#1f6feb"
    _draw_rot_rect(draw, -7.2, -13.2, -5.0, -10.8, cx, cy, ca, sa, S, fill="#1f6feb", outline=irL_col, width=2)
    _draw_rot_rect(draw, 5.0, -13.2, 7.2, -10.8, cx, cy, ca, sa, S, fill="#1f6feb", outline=irR_col, width=2)

    _draw_rot_rect(draw, -3.25, -19.7, 3.25, -18.3, cx, cy, ca, sa, S, fill="#b91c1c", outline="#f85149", width=1)
    active_idx = round((s["pos"] / 7000.0) * 7) if s["vis"] else -1
    for i in range(8):
        sx = -SENSOR_WIDTH_CM / 2.0 + (i / 7.0) * SENSOR_WIDTH_CM
        px, py = _rot_pt(sx, -19.0, cx, cy, ca, sa, S)
        is_hit = bool(s["vis"]) and abs(i - active_idx) <= 1
        r = max(2, int(0.35 * S)) if is_hit else max(1, int(0.22 * S))
        draw.ellipse([px - r, py - r, px + r, py + r], fill="#39c5cf" if is_hit else "#3d1518")

    _draw_rot_rect(draw, -2.4, -20.6, 2.4, -19.0, cx, cy, ca, sa, S, fill="#1f4e79", outline="#58a6ff", width=1)
    for ux in (-1.3, 1.3):
        px, py = _rot_pt(ux, -20.2, cx, cy, ca, sa, S)
        r = max(2, int(0.65 * S))
        draw.ellipse([px - r, py - r, px + r, py + r], fill="#c9d1d9", outline="#30363d")

    _draw_rot_rect(draw, -4.4, -16.8, 4.4, -10.8, cx, cy, ca, sa, S, fill="#161b22", outline="#58a6ff", width=2)
    _draw_rot_rect(draw, -1.6, -9.8, 1.6, -7.4, cx, cy, ca, sa, S, fill="#b91c1c", outline="#f85149", width=1)
    _draw_rot_rect(draw, -6.4, -6.2, -4.0, -1.5, cx, cy, ca, sa, S, fill="#1d4ed8", outline="#58a6ff", width=1)
    _draw_rot_rect(draw, 4.0, -6.2, 6.4, -1.5, cx, cy, ca, sa, S, fill="#1d4ed8", outline="#58a6ff", width=1)

    _draw_rot_rect(draw, -3.5, -6.5, 3.5, 1.5, cx, cy, ca, sa, S, fill="#0d1117", outline="#30363d", width=1)
    for bx in (-2.2, 0.0, 2.2):
        _draw_rot_rect(draw, bx - 0.9, -6.0, bx + 0.9, 1.0, cx, cy, ca, sa, S, fill="#a855f7", outline="#d8b4fe", width=1)

    px, py = _rot_pt(0.0, -13.8, cx, cy, ca, sa, S)
    r = max(3, int(0.45 * S))
    draw.ellipse([px - r, py - r, px + r, py + r], fill="#bc8cff" if s["synced"] else "#3fb950")


def _render_triple(args) -> tuple[bytes, bytes, bytes]:
    s, cur_ms, end_ms, map_zoom, cam_path = args
    return (
        _render_data_frame(s, cur_ms, end_ms),
        _render_map_frame(s, cur_ms, end_ms, map_zoom),
        _render_cam_frame(cam_path, cur_ms, end_ms, s["synced"], s["sync_t"]),
    )


def _render_cam_frame(cam_path: str | None, cur_ms: float, end_ms: float, synced: bool, sync_t) -> bytes:
    if not cam_path or not os.path.exists(cam_path):
        return _NO_CAM_BYTES
    try:
        with Image.open(cam_path) as raw:
            img = ImageOps.pad(raw.convert("RGB"), (1280, 720), color="#090c10")
        d = ImageDraw.Draw(img)
        flash = synced and sync_t is not None and 0 <= (cur_ms - sync_t) <= 350
        d.rounded_rectangle([14, 14, 340, 50], radius=6, fill="#161b22", outline="#30363d", width=2)
        d.ellipse([26, 25, 40, 39], fill="#ffffff" if flash else ("#bc8cff" if synced else "#30363d"))
        d.text((50, 23), f"CAM  |  {fmt_time(cur_ms)} / {fmt_time(end_ms)}", fill="#e6edf3", font=_FONTS["15b"])
        return img.tobytes()
    except OSError:
        return _NO_CAM_BYTES


def _render_data_frame(s: dict, cur_ms: float, end_ms: float) -> bytes:
    W, H = 1920, 1080
    img = _DATA_BG.copy()
    d = ImageDraw.Draw(img)

    px = 28 + d.textbbox((0, 0), "ERROR 404 MONITOR", font=_FONTS["15b"])[2] + 30
    st = s["status"]
    st_col = ("#3fb950", "#14361d", "#3fb950") if st in ("FOLLOW", "RACING") else (
        ("#f85149", "#3d1518", "#f85149") if st in ("LOST", "STOP", "STOPPED") else ("#d29922", "#3b2a0a", "#d29922")
    )
    px += _draw_pill(d, px, 27, st, *st_col) + 10

    cal_col = ("#e6edf3", "#21262d", "#30363d") if s["cal_ok"] is None else (
        ("#3fb950", "#14361d", "#3fb950") if s["cal_ok"] else ("#f85149", "#3d1518", "#f85149")
    )
    px += _draw_pill(d, px, 27, s["cal_txt"], *cal_col) + 10

    flash = s["synced"] and s["sync_t"] is not None and 0 <= (cur_ms - s["sync_t"]) <= 350
    sync_txt = (
        f"LED ON ({s['sync_mcu_ms'] or 0}ms) T+{fmt_time(max(0.0, cur_ms - (s['sync_t'] or cur_ms)))}"
        if s["synced"] else "SYNC: WAIT"
    )
    _draw_pill(
        d, px, 27, sync_txt,
        "#0d1117" if flash else ("#ffffff" if s["synced"] else "#8b949e"),
        "#ffffff" if flash else ("#3b1f5e" if s["synced"] else "#21262d"),
        "#bc8cff" if s["synced"] else "#30363d",
    )

    rx = W - 28
    for txt, fg, bg, bd in [
        (f"{fmt_time(cur_ms)} / {fmt_time(end_ms)}", "#e6edf3", "#21262d", "#30363d"),
        (f"{s['hz']} Hz", "#e6edf3", "#21262d", "#30363d"),
        (f"{s['speed_cms']:.1f} cm/s", "#39c5cf", "#0f3238", "#39c5cf"),
        (s["track_txt"],
         "#e6edf3" if s["track_txt"] == "TRACK: -" else ("#f85149" if s["track_err"] else "#3fb950"),
         "#21262d" if s["track_txt"] == "TRACK: -" else ("#3d1518" if s["track_err"] else "#14361d"),
         "#30363d" if s["track_txt"] == "TRACK: -" else ("#f85149" if s["track_err"] else "#3fb950")),
    ]:
        tw = d.textbbox((0, 0), txt, font=_FONTS["15b"])[2] + 20
        rx -= tw
        _draw_pill(d, rx, 27, txt, fg, bg, bd)
        rx -= 10

    L_X, L_W = 14, 1130
    R_X, R_W = L_X + L_W + 14, W - (L_X + L_W + 14) - 14
    BODY_Y, BODY_H = 80, H - 94

    pY, pH = BODY_Y, 265
    d.text((L_X + 16, pY + 32), str(s["pos"]), fill="#39c5cf", font=_FONTS["42b"])
    err = s["pos"] - 3500
    err_str = f"Error: {'+' if err > 0 else ''}{err}"
    ew = d.textbbox((0, 0), err_str, font=_FONTS["20b"])[2]
    d.text((L_X + L_W - 16 - ew, pY + 44), err_str, fill="#8b949e", font=_FONTS["20b"])
    vis_col = ("#e6edf3", "#21262d", "#30363d") if s["vis"] is None else (
        ("#3fb950", "#14361d", "#3fb950") if s["vis"] else ("#f85149", "#3d1518", "#f85149")
    )
    _draw_pill(d, L_X + L_W - 290, pY + 40, f"VIS: {'-' if s['vis'] is None else ('1' if s['vis'] else '0')}", *vis_col)

    bX, bY, bW = L_X + 16, pY + 84, L_W - 32
    mx = bX + max(0.0, min(1.0, s["pos"] / 7000.0)) * bW
    d.ellipse([mx - 8, bY + 2, mx + 8, bY + 18], fill="#39c5cf")

    tX, tY, tW, tH = L_X + 16, pY + 114, L_W - 32, 136
    pts_vals = [3500] * (200 - len(s["pos_list"])) + s["pos_list"]
    poly = [(tX + (i / 199.0) * tW, tY + tH - (v / 7000.0) * tH) for i, v in enumerate(pts_vals)]
    if len(poly) > 1:
        d.line(poly, fill="#39c5cf", width=3)

    r2Y, r2H = pY + pH + 12, 120
    wEnc, wIr, wUs = 270, 240, 275
    wAct = L_W - wEnc - wIr - wUs - 36

    d.text((L_X + 34, r2Y + 64), str(s["encL"]), fill="#58a6ff", font=_FONTS["24b"])
    d.text((L_X + 162, r2Y + 64), str(s["encR"]), fill="#58a6ff", font=_FONTS["24b"])

    irX = L_X + wEnc + 12
    for ix, lbl, val in [(irX + 12, "L", s["irL"]), (irX + 126, "R", s["irR"])]:
        is_w = val == "WHITE"
        d.rounded_rectangle([ix, r2Y + 38, ix + 102, r2Y + 102], radius=6,
                            fill="#e6edf3" if is_w else "#090c10", outline="#ffffff" if is_w else "#6e7681")
        d.text((ix + 12, r2Y + 60), f"{lbl}: {val or '-'}", fill="#0d1117" if is_w else "#c9d1d9", font=_FONTS["15b"])

    usX = irX + wIr + 12
    us = s["us_dist"]
    us_txt = "-" if us is None else ("Out of range" if us < 0 else f"{us} cm")
    us_col = "#8b949e" if (us is None or us < 0) else ("#f85149" if us <= 15 else ("#d29922" if us <= 35 else "#3fb950"))
    d.text((usX + 14, r2Y + 44), us_txt, fill=us_col, font=_FONTS["26b"])
    if us is not None and us > 0:
        bw = min(wUs - 28, int((us / 400.0) * (wUs - 28)))
        d.rectangle([usX + 14, r2Y + 86, usX + 14 + bw, r2Y + 100], fill=us_col)

    actX = usX + wUs + 12
    act = s["act"]
    act_col = ("#f85149", "#3d1518", "#f85149") if act in ("STOP", "STOPPED", "LOST", "BACK") else (
        ("#3fb950", "#14361d", "#3fb950") if act in ("STRAIGHT", "FORWARD", "FOLLOW") else (
            ("#8b949e", "#0d1117", "#30363d") if act == "-" else ("#d29922", "#3b2a0a", "#d29922")
        )
    )
    d.rounded_rectangle([actX + 12, r2Y + 34, actX + wAct - 12, r2Y + 106], radius=6, fill=act_col[1], outline=act_col[2], width=2)
    d.text((actX + 26, r2Y + 46), act, fill=act_col[0], font=_FONTS["24b"])
    if s["trans"]:
        d.text((actX + 26, r2Y + 80), s["trans"], fill=act_col[0], font=_FONTS["14b"])

    r3Y, r3H = r2Y + r2H + 12, 125
    wMot, wPid = 480, L_W - 492
    for mx_box, side, dr, pwm in [(L_X + 12, "L", s["dirL"], s["pwmL"]), (L_X + 246, "R", s["dirR"], s["pwmR"])]:
        dc = "#3fb950" if dr == "FWD" else ("#d29922" if dr == "REV" else ("#f85149" if dr == "BRK" else "#8b949e"))
        d.text((mx_box + 12, r3Y + 44), f"{side}  [{dr}]", fill=dc, font=_FONTS["15b"])
        d.text((mx_box + 160, r3Y + 42), str(pwm), fill="#e6edf3", font=_FONTS["18b"])
        pw = int((min(255, abs(int(pwm or 0))) / 255.0) * 198)
        d.rectangle([mx_box + 12, r3Y + 78, mx_box + 12 + pw, r3Y + 94], fill=dc)

    pidX = L_X + wMot + 12
    bw_pid = (wPid - 24 - 32) // 5
    for idx, (val, col) in enumerate([
        (s["pidErr"], "#39c5cf"), (s["pidP"], "#39c5cf"), (s["pidI"], "#39c5cf"),
        (s["pidD"], "#39c5cf"), (s["pidCorr"], "#d29922"),
    ]):
        bx = pidX + 12 + idx * (bw_pid + 8)
        d.text((bx + 12, r3Y + 68), str(val), fill=col, font=_FONTS["20b"])

    r4Y = r3Y + r3H + 12
    r4H = BODY_Y + BODY_H - r4Y
    cols = [L_X + 16, L_X + 200, L_X + 340, L_X + 480, L_X + 600, L_X + L_W - 80]
    for i in range(8):
        ry = r4Y + 72 + i * 38
        row = s["rows"][i]
        if row:
            d.text((cols[1], ry), str(row["min"]), fill="#e6edf3", font=_FONTS["15"])
            d.text((cols[2], ry), str(row["max"]), fill="#e6edf3", font=_FONTS["15"])
            d.text((cols[3], ry), str(row["span"]), fill="#e6edf3", font=_FONTS["15"])
            sw = min(280, int((row["span"] / 1023.0) * 280))
            d.rectangle([cols[4], ry + 3, cols[4] + sw, ry + 15], fill="#3fb950" if row["ok"] else "#f85149")
            d.text((cols[5], ry), "OK" if row["ok"] else "LOW", fill="#3fb950" if row["ok"] else "#f85149", font=_FONTS["15b"])
        else:
            d.text((cols[1], ry), "-", fill="#8b949e", font=_FONTS["15"])
            d.text((cols[2], ry), "-", fill="#8b949e", font=_FONTS["15"])
            d.text((cols[3], ry), "-", fill="#8b949e", font=_FONTS["15"])
            d.text((cols[5], ry), "-", fill="#8b949e", font=_FONTS["15b"])

    fY = r4Y + r4H - 30
    avgL, avgR = s["avgL"], s["avgR"]
    l_txt = f"Left Avg (S0-S3): {avgL['val']} ({'OK' if avgL['ok'] else 'LOW'})" if avgL else "Left Avg (S0-S3): -"
    r_txt = f"Right Avg (S4-S7): {avgR['val']} ({'OK' if avgR['ok'] else 'LOW'})" if avgR else "Right Avg (S4-S7): -"
    d.text((L_X + 16, fY), l_txt, fill=("#3fb950" if avgL["ok"] else "#f85149") if avgL else "#8b949e", font=_FONTS["15b"])
    rw = d.textbbox((0, 0), r_txt, font=_FONTS["15b"])[2]
    d.text((L_X + L_W - 16 - rw, fY), r_txt, fill=("#3fb950" if avgR["ok"] else "#f85149") if avgR else "#8b949e", font=_FONTS["15b"])

    cX, cY, cW, cH = R_X + 12, BODY_Y + 34, R_W - 24, BODY_H - 46
    max_chars = max(20, (cW - 24) // 8)
    wrapped = []
    for text, cls in s["raw_list"]:
        col = CONSOLE_COLORS.get(cls, "#e6edf3")
        for i in range(0, max(1, len(text)), max_chars):
            wrapped.append((text[i:i + max_chars], col))
    max_lines = (cH - 20) // 20
    for idx, (ln, col) in enumerate(wrapped[-max_lines:]):
        d.text((cX + 10, cY + 10 + idx * 20), ln, fill=col, font=_FONTS["14"])

    return img.tobytes()


def _render_map_frame(s: dict, cur_ms: float, end_ms: float, px_per_cm: float) -> bytes:
    W, H = 1280, 720
    img = Image.new("RGB", (W, H), "#090c10")
    d = ImageDraw.Draw(img)

    S = px_per_cm
    cam_x = W / 2.0 - s["x"] * S
    cam_y = H / 2.0 - s["y"] * S

    step = 10.0 * S
    gx = cam_x % step
    while gx < W:
        d.line([(gx, 0), (gx, H)], fill="#161b22", width=1)
        gx += step
    gy = cam_y % step
    while gy < H:
        d.line([(0, gy), (W, gy)], fill="#161b22", width=1)
        gy += step

    d.line([(cam_x - 10, cam_y), (cam_x + 10, cam_y)], fill="#30363d", width=2)
    d.line([(cam_x, cam_y - 10), (cam_x, cam_y + 10)], fill="#30363d", width=2)

    if len(s["bot_path"]) > 1:
        pts = [(cam_x + p[0] * S, cam_y + p[1] * S) for p in s["bot_path"]]
        d.line(pts, fill="#2b5580", width=2)

    if len(s["line_pts"]) > 1:
        seg = []
        lw = max(2, int(0.65 * S))
        for lx, ly, brk in s["line_pts"]:
            if brk and len(seg) > 1:
                d.line(seg, fill="#39c5cf", width=lw)
                seg = []
            seg.append((cam_x + lx * S, cam_y + ly * S))
        if len(seg) > 1:
            d.line(seg, fill="#39c5cf", width=lw)

    _draw_real_bot(d, s, W / 2.0, H / 2.0, S)

    d.rounded_rectangle([14, 14, 540, 56], radius=6, fill="#161b22", outline="#30363d", width=2)
    flash = s["synced"] and s["sync_t"] is not None and 0 <= (cur_ms - s["sync_t"]) <= 350
    d.ellipse([26, 27, 42, 43], fill="#ffffff" if flash else ("#bc8cff" if s["synced"] else "#30363d"))
    d.text((52, 26), f"ERROR 404 MAP  |  {s['status']}  |  {fmt_time(cur_ms)} / {fmt_time(end_ms)}", fill="#e6edf3", font=_FONTS["15b"])

    deg = int(round(((s["theta"] + math.pi / 2.0) * 180.0 / math.pi) % 360))
    hud = f"Dist: {s['dist_cm']:.1f} cm  |  Speed: {s['speed_cms']:.1f} cm/s  |  X: {s['x']:.1f}  Y: {-s['y']:.1f}  Theta: {deg} deg"
    d.rounded_rectangle([14, H - 52, 740, H - 14], radius=6, fill="#161b22", outline="#30363d", width=2)
    d.text((26, H - 41), hud, fill="#39c5cf", font=_FONTS["15b"])

    return img.tobytes()


def run_triple_export(
    events: list[dict],
    cam_frames: list[dict],
    config_data: dict,
    from_sync: bool,
    show_sensor: bool,
    map_zoom: float,
    progress_cb,
) -> str:
    """Render 3 MP4s + package only the 3 MP4s, config.json, and session_data.json."""
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg not found on server PATH")
    if len(events) < 2:
        raise RuntimeError("Not enough telemetry events to export")

    start_idx = 0
    if from_sync:
        for i, ev in enumerate(events):
            if ev["d"].get("type") == "sync":
                start_idx = i

    start_ms = float(events[start_idx]["t"])
    end_ms = float(events[-1]["t"])
    duration_ms = max(500.0, end_ms - start_ms)

    fps = 30
    total_frames = max(1, int(math.ceil((duration_ms / 1000.0) * fps)))
    use_nvenc = has_nvenc(ffmpeg)
    workers = max(2, min(os.cpu_count() or 4, 8))
    max_in_flight = workers * 2

    tmp_dir = tempfile.mkdtemp(prefix="e404_exp_")
    data_mp4 = os.path.join(tmp_dir, "telemetry_data.mp4")
    map_mp4 = os.path.join(tmp_dir, "odometry_map.mp4")
    cam_mp4 = os.path.join(tmp_dir, "camera_feed.mp4")
    zip_path = os.path.join(tmp_dir, "error404_mp4_export.zip")

    state = ReplayState(show_sensor=show_sensor)
    for i in range(start_idx):
        state.apply(events[i])

    p_data = open_ffmpeg_pipe(ffmpeg, 1920, 1080, fps, data_mp4, use_nvenc)
    p_map = open_ffmpeg_pipe(ffmpeg, 1280, 720, fps, map_mp4, use_nvenc)
    p_cam = open_ffmpeg_pipe(ffmpeg, 1280, 720, fps, cam_mp4, use_nvenc)
    stage_label = f"RTX NVENC ({workers}T)" if use_nvenc else f"CPU ({workers}T)"

    cur_idx = start_idx
    cam_idx = -1
    written = 0
    pending = collections.deque()

    try:
        with concurrent.futures.ProcessPoolExecutor(max_workers=workers, initializer=_init_worker) as pool:
            for f in range(total_frames):
                target_t = start_ms + (f * 1000.0 / fps)
                while cur_idx < len(events) and float(events[cur_idx]["t"]) <= target_t:
                    state.apply(events[cur_idx])
                    cur_idx += 1

                while cam_idx + 1 < len(cam_frames) and float(cam_frames[cam_idx + 1]["t"]) <= target_t:
                    cam_idx += 1
                cam_path = cam_frames[cam_idx]["path"] if cam_idx >= 0 else None

                snap = state.snapshot()
                pending.append(pool.submit(_render_triple, (snap, min(target_t, end_ms), end_ms, map_zoom, cam_path)))

                if len(pending) >= max_in_flight:
                    data_b, map_b, cam_b = pending.popleft().result()
                    p_data.stdin.write(data_b)
                    p_map.stdin.write(map_b)
                    p_cam.stdin.write(cam_b)
                    written += 1
                    if written % 4 == 0:
                        progress_cb(int((written / total_frames) * 95), stage_label)

            while pending:
                data_b, map_b, cam_b = pending.popleft().result()
                p_data.stdin.write(data_b)
                p_map.stdin.write(map_b)
                p_cam.stdin.write(cam_b)
                written += 1
                if written % 4 == 0 or written == total_frames:
                    progress_cb(int((written / total_frames) * 95), stage_label)

        for p in (p_data, p_map, p_cam):
            p.stdin.close()
            p.wait()

        progress_cb(97, "Packaging ZIP")
        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_STORED) as zf:
            zf.write(data_mp4, "telemetry_data.mp4")
            zf.write(map_mp4, "odometry_map.mp4")
            zf.write(cam_mp4, "camera_feed.mp4")
            zf.writestr("config.json", json.dumps(config_data, indent=2))
            zf.writestr(
                "session_data.json",
                json.dumps({"version": 1, "events": events}, separators=(",", ":")),
            )

        progress_cb(100, "Done")
        return zip_path
    except Exception:
        for p in (p_data, p_map, p_cam):
            try:
                p.kill()
            except Exception:
                pass
        raise