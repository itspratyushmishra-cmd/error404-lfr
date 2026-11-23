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
    calTxt: "CAL: -", calOk: null,
    trackTxt: "TRACK: -", trackErr: false,
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
      s.calOk = d.ok;
    } else if (d.type === "track") {
      s.trackTxt = "TRACK: " + d.mode;
      s.trackErr = d.mode === "UNCERTAIN";
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
  $("calPill").className = "pill" + (s.calOk === null ? "" : (s.calOk ? " ok" : " err"));
  $("trackPill").textContent = s.trackTxt;
  $("trackPill").className = "pill" + (s.trackTxt === "TRACK: -" ? "" : (s.trackErr ? " err" : " ok"));

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

// ==================== FULL-PAGE 1920x1080 MP4 EXPORT ====================
const CONSOLE_COLORS = {
  "r-log": "#e6edf3",
  "r-pos": "#6e7681",
  "r-dat": "#79c0ff",
  "r-act": "#39c5cf",
  "r-sync": "#bc8cff",
  "r-sys": "#d29922",
  "r-err": "#f85149",
  "r-ok": "#3fb950",
};

function drawCard(c, x, y, w, h, title, rightTitle = "") {
  c.fillStyle = "#161b22";
  c.strokeStyle = "#30363d";
  c.lineWidth = 2;
  c.beginPath();
  c.roundRect(x, y, w, h, 8);
  c.fill();
  c.stroke();

  c.fillStyle = "#8b949e";
  c.font = "bold 14px Consolas, monospace";
  c.fillText(title, x + 14, y + 22);
  if (rightTitle) {
    c.textAlign = "right";
    c.fillText(rightTitle, x + w - 14, y + 22);
    c.textAlign = "left";
  }
}

function drawPill(c, x, y, text, fg, bg, border) {
  c.font = "bold 15px Consolas, monospace";
  const w = c.measureText(text).width + 20;
  c.fillStyle = bg;
  c.strokeStyle = border;
  c.lineWidth = 1.5;
  c.beginPath();
  c.roundRect(x, y, w, 28, 5);
  c.fill();
  c.stroke();
  c.fillStyle = fg;
  c.fillText(text, x + 10, y + 19);
  return w;
}

function wrapText(c, text, maxW) {
  const lines = [];
  let cur = "";
  for (let i = 0; i < text.length; i++) {
    const next = cur + text[i];
    if (c.measureText(next).width > maxW && cur.length > 0) {
      lines.push(cur);
      cur = text[i];
    } else {
      cur = next;
    }
  }
  if (cur) lines.push(cur);
  return lines;
}

function drawExportFrame(c, s, curMs, endMs) {
  const W = 1920, H = 1080;
  c.fillStyle = "#0d1117";
  c.fillRect(0, 0, W, H);

  // ---- TOP BAR ----
  c.fillStyle = "#161b22";
  c.strokeStyle = "#30363d";
  c.lineWidth = 2;
  c.beginPath();
  c.roundRect(14, 14, W - 28, 54, 8);
  c.fill();
  c.stroke();

  let px = 28;
  px += drawPill(c, px, 27, "ERROR 404 MONITOR", "#39c5cf", "#0f3238", "#39c5cf") + 12;

  const stCol = (s.status === "FOLLOW" || s.status === "RACING")
    ? ["#3fb950", "#14361d", "#3fb950"]
    : (s.status === "LOST" || s.status === "STOP" || s.status === "STOPPED")
      ? ["#f85149", "#3d1518", "#f85149"]
      : ["#d29922", "#3b2a0a", "#d29922"];
  px += drawPill(c, px, 27, s.status, stCol[0], stCol[1], stCol[2]) + 10;

  const calCol = s.calOk === null
    ? ["#e6edf3", "#21262d", "#30363d"]
    : (s.calOk ? ["#3fb950", "#14361d", "#3fb950"] : ["#f85149", "#3d1518", "#f85149"]);
  px += drawPill(c, px, 27, s.calTxt, calCol[0], calCol[1], calCol[2]) + 10;

  // LED_ON Sync indicator (flashes white for 350ms at exact LED_ON moment)
  const flash = s.synced && s.syncT !== null && (curMs - s.syncT) >= 0 && (curMs - s.syncT) <= 350;
  const syncTxt = s.synced
    ? `● LED ON (${s.syncMcuMs ?? 0}ms)  T+${fmtTime(Math.max(0, curMs - (s.syncT ?? curMs)))}`
    : "○ SYNC: WAIT";
  drawPill(
    c, px, 27, syncTxt,
    flash ? "#0d1117" : (s.synced ? "#ffffff" : "#8b949e"),
    flash ? "#ffffff" : (s.synced ? "#3b1f5e" : "#21262d"),
    s.synced ? "#bc8cff" : "#30363d"
  );

  // Right side of Top Bar
  c.font = "bold 15px Consolas, monospace";
  const timeStr = `${fmtTime(curMs)} / ${fmtTime(endMs)}`;
  const timeW = c.measureText(timeStr).width + 20;
  const hzStr = `${s.hz} Hz`;
  const hzW = c.measureText(hzStr).width + 20;
  const trkW = c.measureText(s.trackTxt).width + 20;

  let rx = W - 28 - timeW;
  drawPill(c, rx, 27, timeStr, "#e6edf3", "#21262d", "#30363d");
  rx -= hzW + 10;
  drawPill(c, rx, 27, hzStr, "#e6edf3", "#21262d", "#30363d");
  rx -= trkW + 10;
  const trkCol = s.trackTxt === "TRACK: -"
    ? ["#e6edf3", "#21262d", "#30363d"]
    : (s.trackErr ? ["#f85149", "#3d1518", "#f85149"] : ["#3fb950", "#14361d", "#3fb950"]);
  drawPill(c, rx, 27, s.trackTxt, trkCol[0], trkCol[1], trkCol[2]);

  // ---- MAIN SPLIT: LEFT (1130px) | RIGHT CONSOLE (748px) ----
  const L_X = 14, L_W = 1130;
  const R_X = L_X + L_W + 14, R_W = W - R_X - 14;
  const BODY_Y = 80, BODY_H = H - BODY_Y - 14;

  // 1. LIVE QTR POSITION + TRACE (height 265)
  const pY = BODY_Y, pH = 265;
  drawCard(c, L_X, pY, L_W, pH, "LIVE QTR POSITION ([POS])", "0 = Far Left | 3500 = Center | 7000 = Far Right");

  c.fillStyle = "#39c5cf";
  c.font = "bold 44px Consolas, monospace";
  c.fillText(String(s.pos), L_X + 16, pY + 70);

  const err = s.pos - 3500;
  c.fillStyle = "#8b949e";
  c.font = "bold 20px Consolas, monospace";
  c.textAlign = "right";
  c.fillText(`Error: ${err > 0 ? "+" : ""}${err}`, L_X + L_W - 16, pY + 62);
  c.textAlign = "left";

  const visCol = s.vis === null
    ? ["#e6edf3", "#21262d", "#30363d"]
    : (s.vis ? ["#3fb950", "#14361d", "#3fb950"] : ["#f85149", "#3d1518", "#f85149"]);
  drawPill(c, L_X + L_W - 290, pY + 40, `VIS: ${s.vis === null ? "-" : (s.vis ? "1" : "0")}`, visCol[0], visCol[1], visCol[2]);

  // Position Bar
  const bX = L_X + 16, bY = pY + 84, bW = L_W - 32, bH = 20;
  c.fillStyle = "#0d1117";
  c.strokeStyle = "#30363d";
  c.beginPath();
  c.roundRect(bX, bY, bW, bH, 10);
  c.fill();
  c.stroke();
  c.fillStyle = "#484f58";
  c.fillRect(bX + bW / 2 - 1, bY, 2, bH);
  const mx = bX + Math.max(0, Math.min(1, s.pos / 7000)) * bW;
  c.fillStyle = "#39c5cf";
  c.beginPath();
  c.arc(mx, bY + bH / 2, 8, 0, Math.PI * 2);
  c.fill();

  // Trace Graph
  const tX = L_X + 16, tY = pY + 114, tW = L_W - 32, tH = 136;
  c.fillStyle = "#090c10";
  c.fillRect(tX, tY, tW, tH);
  c.strokeStyle = "#262c36";
  c.strokeRect(tX, tY, tW, tH);
  c.beginPath();
  c.moveTo(tX, tY + tH / 2);
  c.lineTo(tX + tW, tY + tH / 2);
  c.stroke();

  const pts = new Array(200 - s.posList.length).fill(3500).concat(s.posList);
  c.strokeStyle = "#39c5cf";
  c.lineWidth = 2.5;
  c.beginPath();
  for (let i = 0; i < pts.length; i++) {
    const gx = tX + (i / (pts.length - 1)) * tW;
    const gy = tY + tH - (pts[i] / 7000) * tH;
    if (i === 0) c.moveTo(gx, gy);
    else c.lineTo(gx, gy);
  }
  c.stroke();

  // 2. PERIPHERALS ROW (height 120)
  const r2Y = pY + pH + 12, r2H = 120;
  const wEnc = 270, wIr = 240, wUs = 275, wAct = L_W - wEnc - wIr - wUs - 36;

  // Encoders
  drawCard(c, L_X, r2Y, wEnc, r2H, "ENCODERS");
  const drawEncBox = (ex, lbl, val) => {
    c.fillStyle = "#0d1117";
    c.strokeStyle = "#262c36";
    c.beginPath();
    c.roundRect(ex, r2Y + 34, 118, 72, 6);
    c.fill();
    c.stroke();
    c.fillStyle = "#8b949e";
    c.font = "bold 12px Consolas, monospace";
    c.textAlign = "center";
    c.fillText(lbl, ex + 59, r2Y + 54);
    c.fillStyle = "#58a6ff";
    c.font = "bold 24px Consolas, monospace";
    c.fillText(String(val), ex + 59, r2Y + 88);
    c.textAlign = "left";
  };
  drawEncBox(L_X + 12, "LEFT", s.encL);
  drawEncBox(L_X + 140, "RIGHT", s.encR);

  // Background IR
  const irX = L_X + wEnc + 12;
  drawCard(c, irX, r2Y, wIr, r2H, "BACKGROUND IR");
  const drawIrBadge = (ix, lbl, val) => {
    const isW = val === "WHITE";
    c.fillStyle = isW ? "#e6edf3" : "#090c10";
    c.strokeStyle = isW ? "#ffffff" : "#6e7681";
    c.beginPath();
    c.roundRect(ix, r2Y + 38, 102, 64, 6);
    c.fill();
    c.stroke();
    c.fillStyle = isW ? "#0d1117" : "#c9d1d9";
    c.font = "bold 16px Consolas, monospace";
    c.textAlign = "center";
    c.fillText(`${lbl}: ${val || "-"}`, ix + 51, r2Y + 76);
    c.textAlign = "left";
  };
  drawIrBadge(irX + 12, "L", s.irL);
  drawIrBadge(irX + 126, "R", s.irR);

  // Ultrasonic
  const usX = irX + wIr + 12;
  drawCard(c, usX, r2Y, wUs, r2H, "ULTRASONIC", "0-400 cm");
  const usTxt = s.usDist === null ? "-" : (s.usDist < 0 ? "Out of range" : `${s.usDist} cm`);
  const usCol = (s.usDist === null || s.usDist < 0)
    ? "#8b949e"
    : (s.usDist <= 15 ? "#f85149" : (s.usDist <= 35 ? "#d29922" : "#3fb950"));
  c.fillStyle = usCol;
  c.font = "bold 26px Consolas, monospace";
  c.fillText(usTxt, usX + 14, r2Y + 68);
  c.fillStyle = "#0d1117";
  c.fillRect(usX + 14, r2Y + 86, wUs - 28, 14);
  if (s.usDist > 0) {
    c.fillStyle = usCol;
    c.fillRect(usX + 14, r2Y + 86, Math.min(wUs - 28, (s.usDist / 400) * (wUs - 28)), 14);
  }

  // State / Action
  const actX = usX + wUs + 12;
  drawCard(c, actX, r2Y, wAct, r2H, "STATE / ACTION");
  const actCol = (s.act === "STOP" || s.act === "STOPPED" || s.act === "LOST" || s.act === "BACK")
    ? ["#f85149", "#3d1518", "#f85149"]
    : (s.act === "STRAIGHT" || s.act === "FORWARD" || s.act === "FOLLOW")
      ? ["#3fb950", "#14361d", "#3fb950"]
      : (s.act === "-" ? ["#8b949e", "#0d1117", "#30363d"] : ["#d29922", "#3b2a0a", "#d29922"]);
  c.fillStyle = actCol[1];
  c.strokeStyle = actCol[2];
  c.beginPath();
  c.roundRect(actX + 12, r2Y + 34, wAct - 24, 72, 6);
  c.fill();
  c.stroke();
  c.fillStyle = actCol[0];
  c.textAlign = "center";
  c.font = "bold 24px Consolas, monospace";
  c.fillText(s.act, actX + wAct / 2, r2Y + (s.trans ? 68 : 78));
  if (s.trans) {
    c.font = "bold 14px Consolas, monospace";
    c.fillText(s.trans, actX + wAct / 2, r2Y + 94);
  }
  c.textAlign = "left";

  // 3. MOTORS & PID ROW (height 125)
  const r3Y = r2Y + r2H + 12, r3H = 125;
  const wMot = 480, wPid = L_W - wMot - 12;

  drawCard(c, L_X, r3Y, wMot, r3H, "MOTORS (DIR & PWM)", "0-255");
  const drawMotorBox = (mx, side, dir, pwm) => {
    c.fillStyle = "#0d1117";
    c.strokeStyle = "#262c36";
    c.beginPath();
    c.roundRect(mx, r3Y + 34, 222, 76, 6);
    c.fill();
    c.stroke();
    c.fillStyle = "#8b949e";
    c.font = "bold 14px Consolas, monospace";
    c.fillText(side, mx + 12, r3Y + 58);
    const dCol = dir === "FWD" ? "#3fb950" : (dir === "REV" ? "#d29922" : (dir === "BRK" ? "#f85149" : "#8b949e"));
    c.fillStyle = dCol;
    c.fillText(`[${dir}]`, mx + 46, r3Y + 58);
    c.fillStyle = "#e6edf3";
    c.textAlign = "right";
    c.font = "bold 18px Consolas, monospace";
    c.fillText(String(pwm), mx + 210, r3Y + 58);
    c.textAlign = "left";

    c.fillStyle = "#161b22";
    c.fillRect(mx + 12, r3Y + 76, 198, 16);
    c.fillStyle = dCol;
    c.fillRect(mx + 12, r3Y + 76, (Math.min(255, Math.abs(Number(pwm) || 0)) / 255) * 198, 16);
  };
  drawMotorBox(L_X + 12, "L", s.dirL, s.pwmL);
  drawMotorBox(L_X + 246, "R", s.dirR, s.pwmR);

  const pidX = L_X + wMot + 12;
  drawCard(c, pidX, r3Y, wPid, r3H, "PID TELEMETRY", "Live Terms");
  const pidItems = [
    ["ERR", s.pidErr, "#39c5cf"],
    ["P", s.pidP, "#39c5cf"],
    ["I", s.pidI, "#39c5cf"],
    ["D", s.pidD, "#39c5cf"],
    ["OUT", s.pidCorr, "#d29922"],
  ];
  const boxW = (wPid - 24 - 4 * 8) / 5;
  pidItems.forEach(([lbl, val, col], idx) => {
    const bx = pidX + 12 + idx * (boxW + 8);
    c.fillStyle = "#0d1117";
    c.strokeStyle = "#262c36";
    c.beginPath();
    c.roundRect(bx, r3Y + 34, boxW, 76, 6);
    c.fill();
    c.stroke();
    c.textAlign = "center";
    c.fillStyle = "#8b949e";
    c.font = "bold 13px Consolas, monospace";
    c.fillText(lbl, bx + boxW / 2, r3Y + 56);
    c.fillStyle = col;
    c.font = "bold 20px Consolas, monospace";
    c.fillText(String(val), bx + boxW / 2, r3Y + 90);
    c.textAlign = "left";
  });

  // 4. QTR SENSOR CALIBRATION REPORT TABLE (fills remaining left height)
  const r4Y = r3Y + r3H + 12, r4H = BODY_Y + BODY_H - r4Y;
  drawCard(c, L_X, r4Y, L_W, r4H, "QTR SENSOR CALIBRATION REPORT", "Min Required Span: 120");

  const cols = [L_X + 16, L_X + 240, L_X + 390, L_X + 540, L_X + 620, L_X + L_W - 20];
  c.fillStyle = "#8b949e";
  c.font = "bold 14px Consolas, monospace";
  c.fillText("Sensor", cols[0], r4Y + 52);
  c.textAlign = "right";
  c.fillText("Min", cols[1], r4Y + 52);
  c.fillText("Max", cols[2], r4Y + 52);
  c.fillText("Span", cols[3], r4Y + 52);
  c.fillText("Status", cols[5], r4Y + 52);
  c.textAlign = "left";
  c.fillText("Bar", cols[4], r4Y + 52);

  c.strokeStyle = "#21262d";
  c.lineWidth = 1;
  c.beginPath();
  c.moveTo(L_X + 14, r4Y + 62);
  c.lineTo(L_X + L_W - 14, r4Y + 62);
  c.stroke();

  const rowH = 38;
  for (let i = 0; i < 8; i++) {
    const ry = r4Y + 90 + i * rowH;
    const d = s.rows[i];
    c.fillStyle = "#e6edf3";
    c.font = "15px Consolas, monospace";
    c.fillText(labels[i], cols[0], ry);

    c.textAlign = "right";
    c.fillText(d ? String(d.min) : "-", cols[1], ry);
    c.fillText(d ? String(d.max) : "-", cols[2], ry);
    c.fillText(d ? String(d.span) : "-", cols[3], ry);
    c.fillStyle = d ? (d.ok ? "#3fb950" : "#f85149") : "#8b949e";
    c.font = "bold 15px Consolas, monospace";
    c.fillText(d ? (d.ok ? "OK" : "LOW") : "-", cols[5], ry);
    c.textAlign = "left";

    // Span Bar
    const barW = 280;
    c.fillStyle = "#0d1117";
    c.fillRect(cols[4], ry - 11, barW, 12);
    if (d) {
      c.fillStyle = d.ok ? "#3fb950" : "#f85149";
      c.fillRect(cols[4], ry - 11, Math.min(barW, (d.span / 1023) * barW), 12);
    }

    c.strokeStyle = "#21262d";
    c.beginPath();
    c.moveTo(L_X + 14, ry + 12);
    c.lineTo(L_X + L_W - 14, ry + 12);
    c.stroke();
  }

  // Calibration Footer
  const fY = r4Y + r4H - 18;
  c.font = "bold 15px Consolas, monospace";
  c.fillStyle = s.avgL ? (s.avgL.ok ? "#3fb950" : "#f85149") : "#8b949e";
  c.fillText(`Left Avg (S0-S3): ${s.avgL ? `${s.avgL.val} (${s.avgL.ok ? "OK" : "LOW"})` : "-"}`, L_X + 16, fY);
  c.textAlign = "right";
  c.fillStyle = s.avgR ? (s.avgR.ok ? "#3fb950" : "#f85149") : "#8b949e";
  c.fillText(`Right Avg (S4-S7): ${s.avgR ? `${s.avgR.val} (${s.avgR.ok ? "OK" : "LOW"})` : "-"}`, L_X + L_W - 16, fY);
  c.textAlign = "left";

  // ---- RIGHT COLUMN: LOGS, TRANSITIONS & REPORTS CONSOLE ----
  drawCard(c, R_X, BODY_Y, R_W, BODY_H, "LOGS, TRANSITIONS & REPORTS");
  const cX = R_X + 12, cY = BODY_Y + 34, cW = R_W - 24, cH = BODY_H - 46;
  c.fillStyle = "#090c10";
  c.strokeStyle = "#262c36";
  c.beginPath();
  c.roundRect(cX, cY, cW, cH, 6);
  c.fill();
  c.stroke();

  c.save();
  c.beginPath();
  c.rect(cX + 6, cY + 6, cW - 12, cH - 12);
  c.clip();

  c.font = "14px Consolas, monospace";
  const wrapped = [];
  for (const [text, cls] of s.rawList) {
    const col = CONSOLE_COLORS[cls] || "#e6edf3";
    for (const ln of wrapText(c, text, cW - 20)) {
      wrapped.push([ln, col]);
    }
  }
  const lineH = 20;
  const maxLines = Math.floor((cH - 16) / lineH);
  const visible = wrapped.slice(-maxLines);
  visible.forEach(([ln, col], idx) => {
    c.fillStyle = col;
    c.fillText(ln, cX + 10, cY + 22 + idx * lineH);
  });
  c.restore();
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
  expCanvas.width = 1920;
  expCanvas.height = 1080;
  const expCtx = expCanvas.getContext("2d");

  const mimeCandidates = [
    "video/webm;codecs=vp9",
    "video/webm;codecs=vp8",
    "video/webm",
    "video/mp4;codecs=avc1.42E01E",
    "video/mp4",
  ];
  const chosenMime = mimeCandidates.find((m) => MediaRecorder.isTypeSupported(m)) || "";

  const stream = expCanvas.captureStream(30);
  const recorder = new MediaRecorder(stream, chosenMime ? { mimeType: chosenMime, videoBitsPerSecond: 6_000_000 } : undefined);
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

    // Always convert through server FFmpeg for a standard H.264 yuv420p MP4
    btn.textContent = "Encoding MP4...";
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
        const ext = chosenMime.startsWith("video/mp4") ? "mp4" : "webm";
        downloadBlob(rawBlob, `telemetry_${stamp}.${ext}`);
      }
    } catch {
      const ext = chosenMime.startsWith("video/mp4") ? "mp4" : "webm";
      downloadBlob(rawBlob, `telemetry_${stamp}.${ext}`);
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