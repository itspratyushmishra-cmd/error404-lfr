const $ = (id) => document.getElementById(id);

// Pre-build 8 calibration table rows
const labels = ["S0 (Far L)", "S1", "S2", "S3", "S4", "S5", "S6", "S7 (Far R)"];
const calRows = [];
for (let i = 0; i < 8; i++) {
  const tr = document.createElement("tr");
  tr.innerHTML = `
    <td>${labels[i]}</td>
    <td class="c-min">-</td>
    <td class="c-max">-</td>
    <td class="c-span">-</td>
    <td><span class="span-bar-wrap"><span class="span-bar"></span></span></td>
    <td class="c-st">-</td>`;
  $("calBody").appendChild(tr);
  calRows.push({
    min: tr.querySelector(".c-min"),
    max: tr.querySelector(".c-max"),
    span: tr.querySelector(".c-span"),
    bar: tr.querySelector(".span-bar"),
    st: tr.querySelector(".c-st"),
  });
}

// Trace canvas
const canvas = $("trace");
const ctx = canvas.getContext("2d");
let history = new Array(200).fill(3500);

function drawTrace() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = "#262c36";
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();

  ctx.strokeStyle = "#39c5cf";
  ctx.lineWidth = 2;
  ctx.beginPath();
  for (let i = 0; i < history.length; i++) {
    const x = (i / (history.length - 1)) * w;
    const y = h - (history[i] / 7000) * h;
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

// UI Paint Helpers
function statePillClass(st) {
  if (st === "FOLLOW" || st === "RACING") return "pill ok";
  if (st === "JPROBE" || st === "JALIGN" || st === "PIVOT" || st === "CALIBRATING" || st === "WAITING") return "pill warn";
  if (st === "LOST" || st === "STOP" || st === "STOPPED") return "pill err";
  if (st === "CHECKER" || st === "FINISH") return "pill info";
  return "pill";
}

function paintVis(vis) {
  const el = $("visPill");
  if (vis === null || vis === undefined) {
    el.textContent = "VIS: -";
    el.className = "pill";
  } else if (vis) {
    el.textContent = "VIS: 1";
    el.className = "pill ok";
  } else {
    el.textContent = "VIS: 0";
    el.className = "pill err";
  }
}

function paintIR(l, r) {
  const elL = $("irL"), elR = $("irR");
  elL.textContent = l && l !== "-" ? `L: ${l}` : "L: -";
  elL.className = "ir-badge" + (l && l !== "-" ? ` ir-${l}` : "");
  elR.textContent = r && r !== "-" ? `R: ${r}` : "R: -";
  elR.className = "ir-badge" + (r && r !== "-" ? ` ir-${r}` : "");
}

function paintUS(dist) {
  const valEl = $("usVal"), barEl = $("usBar"), noteEl = $("usNote");
  if (dist === null) {
    valEl.textContent = "-";
    valEl.className = "us-val oor";
    barEl.style.width = "0%";
    noteEl.textContent = "Standby";
  } else if (dist < 0) {
    valEl.textContent = "Out of range";
    valEl.className = "us-val oor";
    barEl.style.width = "0%";
    noteEl.textContent = "No echo";
  } else {
    valEl.textContent = `${dist} cm`;
    const pct = Math.min(100, (dist / 400) * 100);
    barEl.style.width = pct + "%";
    if (dist <= 15) {
      valEl.className = "us-val err";
      barEl.style.background = "var(--red)";
      noteEl.textContent = "CLOSE";
    } else if (dist <= 35) {
      valEl.className = "us-val warn";
      barEl.style.background = "var(--amber)";
      noteEl.textContent = "NEAR";
    } else {
      valEl.className = "us-val";
      barEl.style.background = "var(--green)";
      noteEl.textContent = "CLEAR";
    }
  }
}

function paintAct(act, transText = null) {
  const box = $("actBox");
  const mainEl = $("actMain");
  const transEl = $("actTrans");

  if (!act || act === "-") {
    mainEl.textContent = "-";
    transEl.textContent = "";
    box.className = "act-box";
    return;
  }

  mainEl.textContent = act;
  if (transText !== null) transEl.textContent = transText;

  if (act === "STOP" || act === "STOPPED" || act === "LOST" || act === "BACK") box.className = "act-box act-stop";
  else if (act === "STRAIGHT" || act === "FORWARD" || act === "FOLLOW") box.className = "act-box act-go";
  else box.className = "act-box act-turn";
}

function paintMotor(side, dir, pwm) {
  const dirEl = $(side === "L" ? "dirL" : "dirR");
  const pwmEl = $(side === "L" ? "pwmL" : "pwmR");
  const barEl = $(side === "L" ? "pwmBarL" : "pwmBarR");

  if (!dir || dir === "-") {
    dirEl.textContent = "-";
    dirEl.className = "dir-badge";
    pwmEl.textContent = "0";
    barEl.style.width = "0%";
    barEl.style.background = "var(--blue)";
    return;
  }

  dirEl.textContent = dir;
  dirEl.className = "dir-badge" + (dir === "FWD" || dir === "REV" || dir === "BRK" ? ` dir-${dir}` : "");
  const absPwm = Math.min(255, Math.abs(Number(pwm) || 0));
  pwmEl.textContent = pwm;
  barEl.style.width = ((absPwm / 255) * 100) + "%";
  barEl.style.background = dir === "REV" ? "var(--amber)" : (dir === "BRK" ? "var(--red)" : "var(--green)");
}

function paintPID(err, p, i, d, corr) {
  $("pidErr").textContent = err ?? "-";
  $("pidP").textContent = p ?? "-";
  $("pidI").textContent = i ?? "-";
  $("pidD").textContent = d ?? "-";
  $("pidCorr").textContent = corr ?? "-";
}

function paintSyncBadge(synced, mcuMs) {
  const el = $("syncPill");
  if (!synced) {
    el.textContent = "SYNC: WAIT";
    el.className = "pill";
  } else {
    el.textContent = mcuMs !== null && mcuMs !== undefined ? `LED ON (${mcuMs}ms)` : "LED ON";
    el.className = "pill sync-on";
  }
}

// Session History, Sync & Replay State
const MAX_RECORDED = 60000;
const SPEEDS = [1, 0.5, 0.25, 2];
let recorded = [];
let recStart = null;
let syncIdx = null;
let syncRecMs = null;
let syncMcuMs = null;

let isReplay = false;
let isPlaying = false;
let replayIdx = 0;
let speedIdx = 0;
let playClock = 0;
let lastRaf = 0;

let showSensor = false, paused = false;
let isExporting = false;
let exportCancel = false;

function fmtTime(ms) {
  const neg = ms < 0;
  const s = Math.abs(ms) / 1000;
  const m = Math.floor(s / 60);
  const sec = (s % 60).toFixed(1).padStart(4, "0");
  return `${neg ? "-" : ""}${String(m).padStart(2, "0")}:${sec}`;
}

function updateReplayBar() {
  const total = recorded.length;
  const scrub = $("scrubber");
  scrub.disabled = total === 0 || isExporting;
  scrub.max = Math.max(0, total - 1);

  const curIdx = isReplay ? replayIdx : Math.max(0, total - 1);
  scrub.value = curIdx;

  const curMs = total ? recorded[curIdx].t : 0;
  const endMs = total ? recorded[total - 1].t : 0;

  let timeStr = `${fmtTime(curMs)} / ${fmtTime(endMs)}`;
  if (syncRecMs !== null) {
    const rel = curMs - syncRecMs;
    timeStr = `T${rel >= 0 ? "+" : ""}${fmtTime(rel)} | ${fmtTime(endMs)}`;
  }
  $("timeInfo").textContent = `${timeStr} (${total})`;

  // Sync tick position on scrubber
  const tick = $("syncTick");
  if (syncIdx !== null && total > 1) {
    tick.style.display = "block";
    tick.style.left = `${(syncIdx / (total - 1)) * 100}%`;
  } else {
    tick.style.display = "none";
  }

  $("btnJumpSync").disabled = syncIdx === null || isExporting;
  $("btnExport").disabled = total < 2 && !isExporting;
  $("btnPlay").textContent = isPlaying ? "⏸ Pause" : "▶ Play";
  $("btnPlay").classList.toggle("replay-on", isPlaying);

  const liveBtn = $("btnLive");
  if (isReplay) {
    liveBtn.textContent = "↺ LIVE";
    liveBtn.className = "btn replay-on";
  } else {
    liveBtn.textContent = "● LIVE";
    liveBtn.className = "btn live-on";
  }
}

function getRawClassAndVisible(d) {
  if (d.type === "pos")        return [showSensor, "r-pos"];
  if (d.type === "dat")        return [showSensor, "r-dat"];
  if (d.type === "action")     return [true, "r-act"];
  if (d.type === "sync")       return [true, "r-sync"];
  if (d.type === "system")     return [true, "r-ok"];
  if (d.type === "cal_row")    return [true, d.ok ? "r-log" : "r-err"];
  if (d.type === "cal_avg")    return [true, "r-log"];
  if (d.type === "cal_result") return [true, d.ok ? "r-ok" : "r-err"];
  if (d.type === "track")      return [true, d.mode === "UNCERTAIN" ? "r-err" : "r-ok"];
  if (d.type === "status")     return [true, d.status === "STOPPED" ? "r-err" : "r-sys"];
  if (d.type === "sys")        return [true, "r-sys"];
  return [true, "r-log"];
}

function appendRaw(text, cls) {
  if (paused && !isReplay) return;
  const box = $("rawBox");
  const div = document.createElement("div");
  div.className = cls;
  div.textContent = text;
  box.appendChild(div);
  while (box.childElementCount > 200) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

function applyEventToDOM(d) {
  const [vis, cls] = getRawClassAndVisible(d);

  if (d.type === "sync") {
    paintSyncBadge(true, d.mcu_ms);
  } else if (d.type === "pos") {
    $("posVal").textContent = d.pos;
    const err = d.pos - 3500;
    $("errVal").textContent = (err > 0 ? "+" : "") + err;
    $("posMarker").style.left = Math.max(0, Math.min(100, (d.pos / 7000) * 100)) + "%";
    $("hzVal").textContent = d.hz;
    paintVis(d.vis);
    history.push(d.pos);
    history.shift();
    drawTrace();
  } else if (d.type === "dat") {
    $("encL").textContent = d.encL;
    $("encR").textContent = d.encR;
    paintIR(d.irL, d.irR);
    paintUS(d.us);
    paintMotor("L", d.dirL, d.pwmL);
    paintMotor("R", d.dirR, d.pwmR);
    paintPID(d.err, d.pTerm, d.iTerm, d.dTerm, d.corr);
    $("statusPill").textContent = d.st;
    $("statusPill").className = statePillClass(d.st);
    paintAct(d.st);
  } else if (d.type === "action") {
    $("statusPill").textContent = d.to;
    $("statusPill").className = statePillClass(d.to);
    paintAct(d.to, `${d.from} → ${d.to}`);
  } else if (d.type === "system") {
    if (d.status) {
      $("statusPill").textContent = d.status;
      $("statusPill").className = statePillClass(d.status);
    }
  } else if (d.type === "cal_row") {
    const r = calRows[d.idx];
    r.min.textContent = d.min;
    r.max.textContent = d.max;
    r.span.textContent = d.span;
    r.bar.style.width = Math.min(100, (d.span / 1023) * 100) + "%";
    r.bar.className = "span-bar" + (d.ok ? "" : " low");
    r.st.textContent = d.ok ? "OK" : "LOW";
    r.st.style.color = d.ok ? "var(--green)" : "var(--red)";
  } else if (d.type === "cal_avg") {
    const el = d.side === "L" ? $("avgL") : $("avgR");
    el.textContent = `${d.val} (${d.ok ? "OK" : "LOW"})`;
    el.style.color = d.ok ? "var(--green)" : "var(--red)";
  } else if (d.type === "cal_result") {
    $("calPill").textContent = d.ok ? "CAL: OK" : "CAL: LOW";
    $("calPill").className = "pill " + (d.ok ? "ok" : "err");
  } else if (d.type === "track") {
    $("trackPill").textContent = "TRACK: " + d.mode;
    $("trackPill").className = "pill " + (d.mode === "UNCERTAIN" ? "err" : "ok");
  } else if (d.type === "status") {
    $("statusPill").textContent = d.status;
    $("statusPill").className = statePillClass(d.status);
  }

  if (vis && d.raw) appendRaw(d.raw, cls);
}

function buildStateUpTo(targetIdx) {
  const s = {
    pos: 3500, vis: null, hz: 0, status: "IDLE",
    calTxt: "CAL: -", calCls: "pill",
    trackTxt: "TRACK: -", trackCls: "pill",
    synced: false, syncMcuMs: null, syncT: null,
    encL: "-", encR: "-", irL: null, irR: null, usDist: null,
    act: "-", trans: "",
    dirL: "-", pwmL: 0, dirR: "-", pwmR: 0,
    pidErr: "-", pidP: "-", pidI: "-", pidD: "-", pidCorr: "-",
    avgL: null, avgR: null,
    rows: new Array(8).fill(null),
    posList: [],
    rawList: [],
  };

  const end = Math.min(targetIdx, recorded.length - 1);
  for (let i = 0; i <= end; i++) {
    const rec = recorded[i];
    const d = rec.d;
    const [vis, cls] = getRawClassAndVisible(d);
    if (vis && d.raw) {
      s.rawList.push([d.raw, cls]);
      if (s.rawList.length > 200) s.rawList.shift();
    }
    if (d.type === "sync") {
      s.synced = true;
      s.syncMcuMs = d.mcu_ms;
      s.syncT = rec.t;
    } else if (d.type === "pos") {
      s.pos = d.pos; s.vis = d.vis; s.hz = d.hz;
      s.posList.push(d.pos);
      if (s.posList.length > 200) s.posList.shift();
    } else if (d.type === "dat") {
      s.encL = d.encL; s.encR = d.encR;
      s.irL = d.irL; s.irR = d.irR;
      s.usDist = d.us;
      s.dirL = d.dirL; s.pwmL = d.pwmL;
      s.dirR = d.dirR; s.pwmR = d.pwmR;
      s.pidErr = d.err; s.pidP = d.pTerm; s.pidI = d.iTerm; s.pidD = d.dTerm; s.pidCorr = d.corr;
      s.status = d.st;
      s.act = d.st;
    } else if (d.type === "action") {
      s.status = d.to;
      s.act = d.to;
      s.trans = `${d.from} → ${d.to}`;
    } else if (d.type === "system") {
      if (d.status) s.status = d.status;
    } else if (d.type === "cal_row") {
      s.rows[d.idx] = d;
    } else if (d.type === "cal_avg") {
      if (d.side === "L") s.avgL = d; else s.avgR = d;
    } else if (d.type === "cal_result") {
      s.calTxt = d.ok ? "CAL: OK" : "CAL: LOW";
      s.calCls = "pill " + (d.ok ? "ok" : "err");
    } else if (d.type === "track") {
      s.trackTxt = "TRACK: " + d.mode;
      s.trackCls = "pill " + (d.mode === "UNCERTAIN" ? "err" : "ok");
    } else if (d.type === "status") {
      s.status = d.status;
    }
  }
  return s;
}

function renderSnapshotAt(targetIdx) {
  const s = buildStateUpTo(targetIdx);

  $("posVal").textContent = s.pos;
  const err = s.pos - 3500;
  $("errVal").textContent = (err > 0 ? "+" : "") + err;
  $("posMarker").style.left = Math.max(0, Math.min(100, (s.pos / 7000) * 100)) + "%";
  $("hzVal").textContent = s.hz;
  paintVis(s.vis);
  paintSyncBadge(s.synced, s.syncMcuMs);

  $("statusPill").textContent = s.status;
  $("statusPill").className = statePillClass(s.status);
  $("calPill").textContent = s.calTxt;
  $("calPill").className = s.calCls;
  $("trackPill").textContent = s.trackTxt;
  $("trackPill").className = s.trackCls;

  $("encL").textContent = s.encL;
  $("encR").textContent = s.encR;
  paintIR(s.irL, s.irR);
  paintUS(s.usDist);
  paintAct(s.act, s.trans);
  paintMotor("L", s.dirL, s.pwmL);
  paintMotor("R", s.dirR, s.pwmR);
  paintPID(s.pidErr, s.pidP, s.pidI, s.pidD, s.pidCorr);

  for (let i = 0; i < 8; i++) {
    const r = calRows[i], d = s.rows[i];
    if (d) {
      r.min.textContent = d.min; r.max.textContent = d.max; r.span.textContent = d.span;
      r.bar.style.width = Math.min(100, (d.span / 1023) * 100) + "%";
      r.bar.className = "span-bar" + (d.ok ? "" : " low");
      r.st.textContent = d.ok ? "OK" : "LOW";
      r.st.style.color = d.ok ? "var(--green)" : "var(--red)";
    } else {
      r.min.textContent = "-"; r.max.textContent = "-"; r.span.textContent = "-";
      r.bar.style.width = "0%"; r.bar.className = "span-bar";
      r.st.textContent = "-"; r.st.style.color = "";
    }
  }

  $("avgL").textContent = s.avgL ? `${s.avgL.val} (${s.avgL.ok ? "OK" : "LOW"})` : "-";
  $("avgL").style.color = s.avgL ? (s.avgL.ok ? "var(--green)" : "var(--red)") : "";
  $("avgR").textContent = s.avgR ? `${s.avgR.val} (${s.avgR.ok ? "OK" : "LOW"})` : "-";
  $("avgR").style.color = s.avgR ? (s.avgR.ok ? "var(--green)" : "var(--red)") : "";

  history = new Array(200 - s.posList.length).fill(3500).concat(s.posList);
  drawTrace();

  const box = $("rawBox");
  const frag = document.createDocumentFragment();
  for (const [text, cls] of s.rawList) {
    const div = document.createElement("div");
    div.className = cls;
    div.textContent = text;
    frag.appendChild(div);
  }
  box.textContent = "";
  box.appendChild(frag);
  box.scrollTop = box.scrollHeight;
}

// ==================== MP4 VIDEO EXPORT (1280x720 HUD) ====================
function drawHudCard(c, x, y, w, h, title) {
  c.fillStyle = "#161b22";
  c.strokeStyle = "#30363d";
  c.lineWidth = 2;
  c.beginPath();
  c.roundRect(x, y, w, h, 8);
  c.fill();
  c.stroke();

  c.fillStyle = "#8b949e";
  c.font = "bold 13px Consolas, monospace";
  c.fillText(title, x + 14, y + 22);
}

function drawExportFrame(c, s, curMs, endMs) {
  const W = 1280, H = 720;
  c.fillStyle = "#0d1117";
  c.fillRect(0, 0, W, H);

  // Top Header Bar
  c.fillStyle = "#161b22";
  c.strokeStyle = "#30363d";
  c.lineWidth = 2;
  c.beginPath();
  c.roundRect(14, 14, W - 28, 56, 8);
  c.fill();
  c.stroke();

  c.fillStyle = "#e6edf3";
  c.font = "bold 20px Consolas, monospace";
  c.fillText("ERROR 404  |  TELEMETRY RECORDING", 30, 49);

  // Visual LED_ON Sync Indicator (matches physical board LED for DaVinci/editor alignment)
  const ledActive = s.synced;
  const flash = ledActive && s.syncT !== null && (curMs - s.syncT) >= 0 && (curMs - s.syncT) <= 350;
  c.fillStyle = flash ? "#ffffff" : (ledActive ? "#bc8cff" : "#21262d");
  c.beginPath();
  c.arc(540, 42, 11, 0, Math.PI * 2);
  c.fill();
  c.fillStyle = ledActive ? "#bc8cff" : "#8b949e";
  c.font = "bold 16px Consolas, monospace";
  const syncLabel = ledActive
    ? `SYNC LED ON (T+${fmtTime(Math.max(0, curMs - (s.syncT ?? curMs)))})`
    : "SYNC LED: WAIT";
  c.fillText(syncLabel, 560, 48);

  // State & Time on Right
  c.fillStyle = s.status === "FOLLOW" ? "#3fb950" : (s.status === "STOP" || s.status === "LOST" ? "#f85149" : "#d29922");
  c.font = "bold 20px Consolas, monospace";
  c.fillText(`STATE: ${s.status}`, 860, 49);

  c.fillStyle = "#8b949e";
  c.font = "bold 16px Consolas, monospace";
  c.textAlign = "right";
  c.fillText(`${fmtTime(curMs)} / ${fmtTime(endMs)}`, W - 30, 48);
  c.textAlign = "left";

  // Row 1: QTR Position + Rolling Trace
  drawHudCard(c, 14, 84, W - 28, 250, "LIVE QTR POSITION (0 = FAR LEFT | 3500 = CENTER | 7000 = FAR RIGHT)");
  c.fillStyle = "#39c5cf";
  c.font = "bold 46px Consolas, monospace";
  c.fillText(String(s.pos), 30, 150);

  const err = s.pos - 3500;
  c.fillStyle = "#8b949e";
  c.font = "bold 20px Consolas, monospace";
  c.fillText(`ERR: ${err > 0 ? "+" : ""}${err}`, 200, 145);

  c.fillStyle = s.vis ? "#3fb950" : "#f85149";
  c.fillText(`VIS: ${s.vis === null ? "-" : (s.vis ? "1" : "0")}`, 380, 145);

  // Position Bar
  const barX = 30, barY = 164, barW = W - 60, barH = 22;
  c.fillStyle = "#090c10";
  c.strokeStyle = "#30363d";
  c.beginPath();
  c.roundRect(barX, barY, barW, barH, 11);
  c.fill();
  c.stroke();
  c.fillStyle = "#484f58";
  c.fillRect(barX + barW / 2 - 1, barY, 2, barH);
  const mx = barX + Math.max(0, Math.min(1, s.pos / 7000)) * barW;
  c.fillStyle = "#39c5cf";
  c.beginPath();
  c.arc(mx, barY + barH / 2, 9, 0, Math.PI * 2);
  c.fill();

  // Trace Graph
  const tx = 30, ty = 198, tw = W - 60, th = 122;
  c.fillStyle = "#090c10";
  c.fillRect(tx, ty, tw, th);
  c.strokeStyle = "#262c36";
  c.strokeRect(tx, ty, tw, th);
  c.beginPath();
  c.moveTo(tx, ty + th / 2);
  c.lineTo(tx + tw, ty + th / 2);
  c.stroke();

  const pts = new Array(200 - s.posList.length).fill(3500).concat(s.posList);
  c.strokeStyle = "#39c5cf";
  c.lineWidth = 3;
  c.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const px = tx + (i / (pts.length - 1)) * tw;
    const py = ty + th - (pts[i] / 7000) * th;
    if (i === 0) c.moveTo(px, py);
    else c.lineTo(px, py);
  }
  c.stroke();

  // Row 2: Encoders, IR, Ultrasonic, Action/Transition
  const r2Y = 348, r2H = 124;
  drawHudCard(c, 14, r2Y, 300, r2H, "ENCODERS");
  c.fillStyle = "#58a6ff";
  c.font = "bold 28px Consolas, monospace";
  c.fillText(`L: ${s.encL}`, 32, r2Y + 72);
  c.fillText(`R: ${s.encR}`, 175, r2Y + 72);

  drawHudCard(c, 328, r2Y, 280, r2H, "BACKGROUND IR");
  const drawIrBox = (bx, lbl, val) => {
    c.fillStyle = val === "WHITE" ? "#e6edf3" : "#090c10";
    c.strokeStyle = "#6e7681";
    c.beginPath();
    c.roundRect(bx, r2Y + 38, 118, 66, 6);
    c.fill();
    c.stroke();
    c.fillStyle = val === "WHITE" ? "#0d1117" : "#e6edf3";
    c.font = "bold 18px Consolas, monospace";
    c.fillText(`${lbl}: ${val || "-"}`, bx + 12, r2Y + 78);
  };
  drawIrBox(344, "L", s.irL);
  drawIrBox(474, "R", s.irR);

  drawHudCard(c, 622, r2Y, 300, r2H, "ULTRASONIC");
  const usTxt = s.usDist === null ? "-" : (s.usDist < 0 ? "OUT OF RANGE" : `${s.usDist} cm`);
  c.fillStyle = s.usDist > 0 && s.usDist <= 15 ? "#f85149" : (s.usDist > 0 && s.usDist <= 35 ? "#d29922" : "#3fb950");
  c.font = "bold 28px Consolas, monospace";
  c.fillText(usTxt, 638, r2Y + 70);
  c.fillStyle = "#090c10";
  c.fillRect(638, r2Y + 88, 266, 14);
  if (s.usDist > 0) {
    c.fillStyle = s.usDist <= 15 ? "#f85149" : (s.usDist <= 35 ? "#d29922" : "#3fb950");
    c.fillRect(638, r2Y + 88, Math.min(266, (s.usDist / 400) * 266), 14);
  }

  drawHudCard(c, 936, r2Y, 330, r2H, "STATE / ACTION");
  c.fillStyle = s.act === "FOLLOW" ? "#3fb950" : (s.act === "STOP" || s.act === "LOST" ? "#f85149" : "#d29922");
  c.font = "bold 32px Consolas, monospace";
  c.fillText(s.act, 954, r2Y + 72);
  if (s.trans) {
    c.fillStyle = "#39c5cf";
    c.font = "bold 16px Consolas, monospace";
    c.fillText(s.trans, 954, r2Y + 102);
  }

  // Row 3: Motors & PID Telemetry
  const r3Y = 486, r3H = 216;
  drawHudCard(c, 14, r3Y, 520, r3H, "MOTORS (DIRECTION & PWM 0-255)");
  const drawMotorHud = (my, name, dir, pwm) => {
    c.fillStyle = "#e6edf3";
    c.font = "bold 20px Consolas, monospace";
    c.fillText(`${name}  [${dir}]  PWM: ${pwm}`, 32, my);
    c.fillStyle = "#090c10";
    c.fillRect(32, my + 12, 480, 22);
    c.fillStyle = dir === "REV" ? "#d29922" : (dir === "BRK" ? "#f85149" : "#3fb950");
    const w = (Math.min(255, Math.abs(Number(pwm) || 0)) / 255) * 480;
    c.fillRect(32, my + 12, w, 22);
  };
  drawMotorHud(r3Y + 72, "LEFT ", s.dirL, s.pwmL);
  drawMotorHud(r3Y + 150, "RIGHT", s.dirR, s.pwmR);

  drawHudCard(c, 548, r3Y, 718, r3H, "PID TELEMETRY");
  const pidItems = [
    ["ERR", s.pidErr, "#39c5cf"],
    ["P", s.pidP, "#39c5cf"],
    ["I", s.pidI, "#39c5cf"],
    ["D", s.pidD, "#39c5cf"],
    ["OUT", s.pidCorr, "#d29922"],
  ];
  pidItems.forEach(([lbl, val, col], idx) => {
    const bx = 564 + idx * 138;
    c.fillStyle = "#090c10";
    c.strokeStyle = "#262c36";
    c.beginPath();
    c.roundRect(bx, r3Y + 48, 126, 134, 6);
    c.fill();
    c.stroke();
    c.fillStyle = "#8b949e";
    c.font = "bold 16px Consolas, monospace";
    c.fillText(lbl, bx + 14, r3Y + 78);
    c.fillStyle = col;
    c.font = "bold 26px Consolas, monospace";
    c.fillText(String(val), bx + 14, r3Y + 132);
  });
}

async function exportRecordingMP4() {
  if (isExporting) {
    exportCancel = true;
    return;
  }
  if (recorded.length < 2) return;

  isExporting = true;
  exportCancel = false;
  isPlaying = false;

  const btn = $("btnExport");
  btn.classList.add("busy");
  btn.textContent = "Cancel 0%";
  updateReplayBar();

  const snapshot = recorded.slice();
  const startIdx = ($("chkFromSync").checked && syncIdx !== null) ? syncIdx : 0;
  const startMs = snapshot[startIdx].t;
  const endMs = snapshot[snapshot.length - 1].t;
  const totalDur = Math.max(500, endMs - startMs);

  const expCanvas = document.createElement("canvas");
  expCanvas.width = 1280;
  expCanvas.height = 720;
  const expCtx = expCanvas.getContext("2d");

  // Prefer native MP4 if browser supports it; otherwise WebM -> server FFmpeg
  const mimeCandidates = [
    "video/mp4;codecs=avc1.42E01E",
    "video/mp4",
    "video/webm;codecs=vp9",
    "video/webm;codecs=vp8",
    "video/webm",
  ];
  const chosenMime = mimeCandidates.find((m) => MediaRecorder.isTypeSupported(m)) || "";
  const isNativeMp4 = chosenMime.startsWith("video/mp4");

  const stream = expCanvas.captureStream(30);
  const recorder = new MediaRecorder(stream, chosenMime ? { mimeType: chosenMime, videoBitsPerSecond: 4_000_000 } : undefined);
  const chunks = [];
  recorder.ondataavailable = (e) => { if (e.data && e.data.size > 0) chunks.push(e.data); };

  const stopped = new Promise((resolve) => { recorder.onstop = resolve; });
  recorder.start(250);

  let curIdx = startIdx;
  const t0 = performance.now();

  await new Promise((resolve) => {
    function step() {
      if (exportCancel) { resolve(); return; }
      const elapsed = performance.now() - t0;
      const targetT = startMs + elapsed;

      while (curIdx + 1 < snapshot.length && snapshot[curIdx + 1].t <= targetT) {
        curIdx++;
      }

      const st = buildStateUpTo(curIdx);
      drawExportFrame(expCtx, st, Math.min(targetT, endMs), endMs);

      const pct = Math.min(100, Math.round((elapsed / totalDur) * 100));
      btn.textContent = `Cancel ${pct}%`;

      if (elapsed >= totalDur) {
        resolve();
        return;
      }
      requestAnimationFrame(step);
    }
    requestAnimationFrame(step);
  });

  recorder.stop();
  await stopped;
  stream.getTracks().forEach((t) => t.stop());

  if (!exportCancel && chunks.length > 0) {
    const rawBlob = new Blob(chunks, { type: chosenMime || "video/webm" });
    const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);

    if (isNativeMp4) {
      downloadBlob(rawBlob, `telemetry_${stamp}.mp4`);
    } else {
      btn.textContent = "Converting MP4...";
      try {
        const res = await fetch("/api/convert_mp4", {
          method: "POST",
          headers: { "Content-Type": "application/octet-stream" },
          body: rawBlob,
        });
        if (res.ok) {
          const mp4Blob = await res.blob();
          downloadBlob(mp4Blob, `telemetry_${stamp}.mp4`);
        } else {
          downloadBlob(rawBlob, `telemetry_${stamp}.webm`);
        }
      } catch {
        downloadBlob(rawBlob, `telemetry_${stamp}.webm`);
      }
    }
  }

  isExporting = false;
  btn.classList.remove("busy");
  btn.textContent = "⬇ Export MP4";
  updateReplayBar();
}

function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 3000);
}

// ==================== PLAYBACK & CONTROLS ====================
function playbackStep(ts) {
  if (!isPlaying) return;
  const dt = (ts - lastRaf) * SPEEDS[speedIdx];
  lastRaf = ts;
  playClock += dt;

  let advanced = false;
  while (replayIdx + 1 < recorded.length && recorded[replayIdx + 1].t <= playClock) {
    replayIdx++;
    applyEventToDOM(recorded[replayIdx].d);
    advanced = true;
  }

  if (advanced) updateReplayBar();

  if (replayIdx >= recorded.length - 1) {
    isPlaying = false;
    updateReplayBar();
    return;
  }
  requestAnimationFrame(playbackStep);
}

$("btnData").onclick = () => {
  showSensor = !showSensor;
  $("btnData").classList.toggle("on", showSensor);
  if (recorded.length) renderSnapshotAt(isReplay ? replayIdx : recorded.length - 1);
};
$("btnPause").onclick = () => {
  paused = !paused;
  $("btnPause").classList.toggle("on", paused);
  $("btnPause").textContent = paused ? "Resume" : "Pause";
};
$("btnClear").onclick = () => { $("rawBox").textContent = ""; };

$("btnPlay").onclick = () => {
  if (!recorded.length || isExporting) return;
  if (isPlaying) {
    isPlaying = false;
    updateReplayBar();
    return;
  }
  isReplay = true;
  if (replayIdx >= recorded.length - 1) {
    replayIdx = 0;
    renderSnapshotAt(0);
  }
  playClock = recorded[replayIdx].t;
  lastRaf = performance.now();
  isPlaying = true;
  updateReplayBar();
  requestAnimationFrame(playbackStep);
};

$("btnSpeed").onclick = () => {
  speedIdx = (speedIdx + 1) % SPEEDS.length;
  $("btnSpeed").textContent = SPEEDS[speedIdx] + "x";
};

$("btnJumpSync").onclick = () => {
  if (syncIdx === null || isExporting) return;
  isReplay = true;
  isPlaying = false;
  replayIdx = syncIdx;
  renderSnapshotAt(replayIdx);
  updateReplayBar();
};

$("scrubber").oninput = (e) => {
  if (!recorded.length || isExporting) return;
  isReplay = true;
  isPlaying = false;
  replayIdx = Number(e.target.value);
  renderSnapshotAt(replayIdx);
  updateReplayBar();
};

$("btnLive").onclick = () => {
  if (isExporting) return;
  isReplay = false;
  isPlaying = false;
  if (recorded.length) renderSnapshotAt(recorded.length - 1);
  updateReplayBar();
};

$("btnExport").onclick = exportRecordingMP4;

$("btnResetRec").onclick = () => {
  if (isExporting) return;
  recorded = [];
  recStart = null;
  syncIdx = null;
  syncRecMs = null;
  syncMcuMs = null;
  isReplay = false;
  isPlaying = false;
  replayIdx = 0;
  renderSnapshotAt(-1);
  updateReplayBar();
};

// Live SSE Stream
const src = new EventSource("/stream");
src.onopen = () => { $("dot").className = "dot on"; $("connText").textContent = "Live"; };
src.onerror = () => { $("dot").className = "dot"; $("connText").textContent = "Reconnecting..."; };

src.onmessage = (e) => {
  const d = JSON.parse(e.data);
  const now = performance.now();
  if (recStart === null) recStart = now;

  const recTime = now - recStart;
  recorded.push({ t: recTime, d });

  if (d.type === "sync") {
    syncIdx = recorded.length - 1;
    syncRecMs = recTime;
    syncMcuMs = d.mcu_ms;
  }

  if (recorded.length > MAX_RECORDED) {
    recorded.shift();
    if (isReplay && replayIdx > 0) replayIdx--;
    if (syncIdx !== null) {
      syncIdx--;
      if (syncIdx < 0) { syncIdx = null; syncRecMs = null; }
    }
  }

  if (!isReplay) {
    applyEventToDOM(d);
  }
  updateReplayBar();
};

window.addEventListener("resize", drawTrace);
drawTrace();
updateReplayBar();