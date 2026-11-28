const $ = (id) => document.getElementById(id);

// ==================== ROBOT GEOMETRY ====================
const CM_PER_TICK         = 16 / 115;
const WHEELBASE_CM        = 17.8;
const SENSOR_OFFSET_CM    = 19;
const SENSOR_WIDTH_CM     = 6.5;
const DEFAULT_MAP_PX_CM   = 6.0;
const MIN_MAP_PX_CM       = 1.2;
const MAX_MAP_PX_CM       = 24.0;

let mapPxPerCm = DEFAULT_MAP_PX_CM;
let followBot = true;
let panOffsetX = 0;
let panOffsetY = 0;
let isDragging = false;
let dragStartX = 0;
let dragStartY = 0;

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

// ==================== 2D ODOMETRY & MAP ====================
const mapCanvas = $("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");
let mapMode = false;

function createOdom() {
  return {
    x: 0, y: 0, theta: -Math.PI / 2,
    distCm: 0, speedCmS: 0,
    prevEncL: null, prevEncR: null, prevDatT: null,
    pos: 3500, vis: false, irL: null, irR: null, synced: false,
    botPath: [{ x: 0, y: 0 }], linePts: [], wasVis: false,
  };
}

let liveOdom = createOdom();

function stepOdom(o, d, tMs) {
  if (d.type === "sync") { o.synced = true; return; }
  if (d.type === "pos") {
    o.pos = d.pos;
    o.vis = Boolean(d.vis);
    if (!o.vis) o.wasVis = false;
    return;
  }
  if (d.type !== "dat") return;

  o.irL = d.irL;
  o.irR = d.irR;

  const encL = Number(d.encL);
  const encR = Number(d.encR);
  if (Number.isNaN(encL) || Number.isNaN(encR)) return;

  if (o.prevEncL === null) {
    o.prevEncL = encL;
    o.prevEncR = encR;
    o.prevDatT = tMs;
    o.speedCmS = 0;
    return;
  }

  const dL = (encL - o.prevEncL) * CM_PER_TICK;
  const dR = (encR - o.prevEncR) * CM_PER_TICK;
  o.prevEncL = encL;
  o.prevEncR = encR;

  const dS = (dL + dR) / 2;
  const dTheta = (dL - dR) / WHEELBASE_CM;
  const midTheta = o.theta + dTheta / 2;

  o.x += dS * Math.cos(midTheta);
  o.y += dS * Math.sin(midTheta);
  o.theta += dTheta;
  o.distCm += Math.abs(dS);

  if (o.prevDatT !== null && tMs !== undefined) {
    const dtSec = (tMs - o.prevDatT) / 1000;
    if (dtSec > 0.005 && dtSec < 2.0) {
      const instSpd = dS / dtSec;
      o.speedCmS = (Math.abs(dS) < 1e-4) ? 0 : (0.55 * instSpd + 0.45 * o.speedCmS);
    } else if (dtSec >= 2.0) {
      o.speedCmS = 0;
    }
    o.prevDatT = tMs;
  }

  const lastP = o.botPath[o.botPath.length - 1];
  if (Math.hypot(o.x - lastP.x, o.y - lastP.y) >= 0.2) {
    o.botPath.push({ x: o.x, y: o.y });
    if (o.botPath.length > 6000) o.botPath.shift();
  }

  if (o.vis) {
    const lat = ((o.pos - 3500) / 3500) * (SENSOR_WIDTH_CM / 2);
    const lx = o.x + Math.cos(o.theta) * SENSOR_OFFSET_CM - Math.sin(o.theta) * lat;
    const ly = o.y + Math.sin(o.theta) * SENSOR_OFFSET_CM + Math.cos(o.theta) * lat;
    const lastL = o.linePts[o.linePts.length - 1];
    if (!lastL || !o.wasVis || Math.hypot(lx - lastL.x, ly - lastL.y) >= 0.2) {
      o.linePts.push({ x: lx, y: ly, brk: !o.wasVis });
      if (o.linePts.length > 8000) o.linePts.shift();
    }
    o.wasVis = true;
  } else {
    o.wasVis = false;
  }
}

function drawRobotIcon(c, o, S = mapPxPerCm) {
  c.save();
  c.translate(o.x * S, o.y * S);
  c.rotate(o.theta + Math.PI / 2);

  c.fillStyle = "#11161d";
  c.strokeStyle = "#8b949e";
  c.lineWidth = 1.6;
  c.beginPath();
  c.roundRect(-9.75 * S, -2.8 * S, 2.25 * S, 5.6 * S, 3);
  c.roundRect(7.5 * S, -2.8 * S, 2.25 * S, 5.6 * S, 3);
  c.fill();
  c.stroke();

  c.fillStyle = "#9a6725";
  c.strokeStyle = "#d49b4b";
  c.lineWidth = 1.6;
  c.beginPath();
  c.roundRect(-7.5 * S, -10.5 * S, 15.0 * S, 13.0 * S, 4);
  c.fill();
  c.stroke();

  c.fillStyle = "#a8732d";
  c.beginPath();
  c.roundRect(-5.0 * S, -19.5 * S, 10.0 * S, 9.2 * S, 4);
  c.fill();
  c.stroke();

  const irLCol = o.irL === "WHITE" ? "#e6edf3" : "#58a6ff";
  const irRCol = o.irR === "WHITE" ? "#e6edf3" : "#58a6ff";
  c.fillStyle = "#1f6feb";
  c.strokeStyle = irLCol;
  c.lineWidth = 1.5;
  c.fillRect(-7.2 * S, -13.2 * S, 2.2 * S, 2.4 * S);
  c.strokeRect(-7.2 * S, -13.2 * S, 2.2 * S, 2.4 * S);
  c.strokeStyle = irRCol;
  c.fillRect(5.0 * S, -13.2 * S, 2.2 * S, 2.4 * S);
  c.strokeRect(5.0 * S, -13.2 * S, 2.2 * S, 2.4 * S);

  c.fillStyle = "#b91c1c";
  c.strokeStyle = "#f85149";
  c.lineWidth = 1.2;
  c.beginPath();
  c.roundRect(-3.25 * S, -19.7 * S, 6.5 * S, 1.4 * S, 2);
  c.fill();
  c.stroke();

  const activeIdx = o.vis ? Math.round((o.pos / 7000) * 7) : -1;
  for (let i = 0; i < 8; i++) {
    const sx = (-SENSOR_WIDTH_CM / 2 + (i / 7) * SENSOR_WIDTH_CM) * S;
    const isHit = o.vis && Math.abs(i - activeIdx) <= 1;
    c.fillStyle = isHit ? "#39c5cf" : "#3d1518";
    c.beginPath();
    c.arc(sx, -19.0 * S, isHit ? Math.max(2.4, 0.36 * S) : Math.max(1.4, 0.2 * S), 0, Math.PI * 2);
    c.fill();
  }

  c.fillStyle = "#1f4e79";
  c.strokeStyle = "#58a6ff";
  c.lineWidth = 1.2;
  c.fillRect(-2.4 * S, -20.6 * S, 4.8 * S, 1.6 * S);
  c.fillStyle = "#c9d1d9";
  c.beginPath();
  c.arc(-1.3 * S, -20.1 * S, Math.max(2, 0.65 * S), 0, Math.PI * 2);
  c.arc(1.3 * S, -20.1 * S, Math.max(2, 0.65 * S), 0, Math.PI * 2);
  c.fill();

  c.fillStyle = "#161b22";
  c.strokeStyle = "#58a6ff";
  c.lineWidth = 1.4;
  c.beginPath();
  c.roundRect(-4.4 * S, -16.8 * S, 8.8 * S, 6.0 * S, 3);
  c.fill();
  c.stroke();

  c.fillStyle = "#b91c1c";
  c.strokeStyle = "#f85149";
  c.lineWidth = 1.2;
  c.fillRect(-1.6 * S, -9.8 * S, 3.2 * S, 2.4 * S);
  c.strokeRect(-1.6 * S, -9.8 * S, 3.2 * S, 2.4 * S);

  c.fillStyle = "#1d4ed8";
  c.strokeStyle = "#58a6ff";
  c.lineWidth = 1.2;
  c.fillRect(-6.4 * S, -6.2 * S, 2.4 * S, 4.7 * S);
  c.strokeRect(-6.4 * S, -6.2 * S, 2.4 * S, 4.7 * S);
  c.fillRect(4.0 * S, -6.2 * S, 2.4 * S, 4.7 * S);
  c.strokeRect(4.0 * S, -6.2 * S, 2.4 * S, 4.7 * S);

  c.fillStyle = "#0d1117";
  c.strokeStyle = "#30363d";
  c.fillRect(-3.5 * S, -6.5 * S, 7.0 * S, 8.0 * S);
  c.strokeRect(-3.5 * S, -6.5 * S, 7.0 * S, 8.0 * S);
  c.fillStyle = "#a855f7";
  c.strokeStyle = "#d8b4fe";
  c.lineWidth = 1;
  for (const bx of [-2.2, 0, 2.2]) {
    c.beginPath();
    c.roundRect((bx - 0.85) * S, -6.0 * S, 1.7 * S, 7.0 * S, 2);
    c.fill();
    c.stroke();
  }

  c.fillStyle = o.synced ? "#bc8cff" : "#3fb950";
  c.beginPath();
  c.arc(0, -13.8 * S, Math.max(2.5, 0.45 * S), 0, Math.PI * 2);
  c.fill();

  c.restore();
}

function updateZoomLabel() {
  const pct = Math.round((mapPxPerCm / DEFAULT_MAP_PX_CM) * 100);
  $("zoomLbl").textContent = `${pct}%`;
  $("btnFollowBot").classList.toggle("on", followBot);
}

function setMapZoom(nextPxPerCm) {
  const clamped = Math.max(MIN_MAP_PX_CM, Math.min(MAX_MAP_PX_CM, nextPxPerCm));
  const ratio = clamped / mapPxPerCm;
  if (!followBot) {
    panOffsetX *= ratio;
    panOffsetY *= ratio;
  }
  mapPxPerCm = clamped;
  updateZoomLabel();
  if (mapMode) drawMap(liveOdom);
}

function drawMap(o = liveOdom) {
  if (!mapMode) return;
  const w = mapCanvas.clientWidth, h = mapCanvas.clientHeight;
  if (w === 0 || h === 0) return;
  if (mapCanvas.width !== w || mapCanvas.height !== h) {
    mapCanvas.width = w;
    mapCanvas.height = h;
  }

  const c = mapCtx;
  const S = mapPxPerCm;
  c.clearRect(0, 0, w, h);

  const camX = w / 2 - o.x * S + (followBot ? 0 : panOffsetX);
  const camY = h / 2 - o.y * S + (followBot ? 0 : panOffsetY);

  const gridStep = 10 * S;
  const offX = ((camX % gridStep) + gridStep) % gridStep;
  const offY = ((camY % gridStep) + gridStep) % gridStep;

  c.strokeStyle = "#161b22";
  c.lineWidth = 1;
  c.beginPath();
  for (let gx = offX; gx < w; gx += gridStep) { c.moveTo(gx, 0); c.lineTo(gx, h); }
  for (let gy = offY; gy < h; gy += gridStep) { c.moveTo(0, gy); c.lineTo(w, gy); }
  c.stroke();

  c.save();
  c.translate(camX, camY);

  c.strokeStyle = "#30363d";
  c.beginPath();
  c.moveTo(-10, 0); c.lineTo(10, 0);
  c.moveTo(0, -10); c.lineTo(0, 10);
  c.stroke();

  if (o.botPath.length > 1) {
    c.strokeStyle = "rgba(88, 166, 255, 0.35)";
    c.lineWidth = 1.5;
    c.setLineDash([4, 4]);
    c.beginPath();
    o.botPath.forEach((p, idx) => {
      if (idx === 0) c.moveTo(p.x * S, p.y * S);
      else c.lineTo(p.x * S, p.y * S);
    });
    c.stroke();
    c.setLineDash([]);
  }

  if (o.linePts.length > 1) {
    c.strokeStyle = "#39c5cf";
    c.lineWidth = Math.max(2, 0.65 * S);
    c.lineCap = "round";
    c.lineJoin = "round";
    c.beginPath();
    o.linePts.forEach((p, idx) => {
      if (idx === 0 || p.brk) c.moveTo(p.x * S, p.y * S);
      else c.lineTo(p.x * S, p.y * S);
    });
    c.stroke();
  }

  drawRobotIcon(c, o, S);
  c.restore();

  const deg = Math.round((((o.theta + Math.PI / 2) * 180) / Math.PI) % 360);
  $("mapHud").textContent =
    `Dist: ${o.distCm.toFixed(1)} cm | Speed: ${o.speedCmS.toFixed(1)} cm/s | X: ${o.x.toFixed(1)} | Y: ${(-o.y).toFixed(1)} | θ: ${deg}°`;
}

mapCanvas.addEventListener("mousedown", (e) => {
  isDragging = true;
  followBot = false;
  dragStartX = e.clientX - panOffsetX;
  dragStartY = e.clientY - panOffsetY;
  mapCanvas.classList.add("dragging");
  updateZoomLabel();
});
window.addEventListener("mousemove", (e) => {
  if (!isDragging) return;
  panOffsetX = e.clientX - dragStartX;
  panOffsetY = e.clientY - dragStartY;
  if (mapMode) drawMap(liveOdom);
});
window.addEventListener("mouseup", () => {
  if (!isDragging) return;
  isDragging = false;
  mapCanvas.classList.remove("dragging");
});

// ==================== UI PAINT HELPERS ====================
function statePillClass(st) {
  if (st === "FOLLOW" || st === "RACING") return "pill ok";
  if (st === "JPROBE" || st === "JALIGN" || st === "PIVOT" || st === "CALIBRATING" || st === "WAITING") return "pill warn";
  if (st === "LOST" || st === "STOP" || st === "STOPPED") return "pill err";
  if (st === "CHECKER" || st === "FINISH") return "pill info";
  return "pill";
}

function paintVis(vis) {
  const el = $("visPill");
  if (vis === null || vis === undefined) { el.textContent = "VIS: -"; el.className = "pill"; }
  else if (vis) { el.textContent = "VIS: 1"; el.className = "pill ok"; }
  else { el.textContent = "VIS: 0"; el.className = "pill err"; }
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
    valEl.textContent = "-"; valEl.className = "us-val oor"; barEl.style.width = "0%"; noteEl.textContent = "Standby";
  } else if (dist < 0) {
    valEl.textContent = "Out of range"; valEl.className = "us-val oor"; barEl.style.width = "0%"; noteEl.textContent = "No echo";
  } else {
    valEl.textContent = `${dist} cm`;
    barEl.style.width = Math.min(100, (dist / 400) * 100) + "%";
    if (dist <= 15) { valEl.className = "us-val err"; barEl.style.background = "var(--red)"; noteEl.textContent = "CLOSE"; }
    else if (dist <= 35) { valEl.className = "us-val warn"; barEl.style.background = "var(--amber)"; noteEl.textContent = "NEAR"; }
    else { valEl.className = "us-val"; barEl.style.background = "var(--green)"; noteEl.textContent = "CLEAR"; }
  }
}

function paintAct(act, transText = null) {
  const box = $("actBox"), mainEl = $("actMain"), transEl = $("actTrans");
  if (!act || act === "-") { mainEl.textContent = "-"; transEl.textContent = ""; box.className = "act-box"; return; }
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
    dirEl.textContent = "-"; dirEl.className = "dir-badge"; pwmEl.textContent = "0";
    barEl.style.width = "0%"; barEl.style.background = "var(--blue)"; return;
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
  if (!synced) { el.textContent = "SYNC: WAIT"; el.className = "pill"; }
  else { el.textContent = mcuMs !== null && mcuMs !== undefined ? `LED ON (${mcuMs}ms)` : "LED ON"; el.className = "pill sync-on"; }
}

// ==================== SESSION HISTORY & REPLAY ====================
const MAX_RECORDED = 60000;
const SPEEDS = [1, 0.5, 0.25, 2];
let recorded = [];
let lastSeq = 0;
let syncIdx = null, syncRecMs = null, syncMcuMs = null;
let isReplay = false, isPlaying = false, replayIdx = 0, speedIdx = 0, playClock = 0, lastRaf = 0;
let showSensor = false, paused = false, isExporting = false;

function fmtTime(ms) {
  const neg = ms < 0;
  const s = Math.abs(ms) / 1000;
  const m = Math.floor(s / 60);
  const sec = (s % 60).toFixed(1).padStart(4, "0");
  return `${neg ? "-" : ""}${String(m).padStart(2, "0")}:${sec}`;
}

function recomputeSyncIndex() {
  syncIdx = null; syncRecMs = null; syncMcuMs = null;
  for (let i = 0; i < recorded.length; i++) {
    if (recorded[i].d.type === "sync") {
      syncIdx = i; syncRecMs = recorded[i].t; syncMcuMs = recorded[i].d.mcu_ms;
    }
  }
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
    timeStr = `T${rel >= 0 ? "+" : ""}${fmtTime(rel)} / ${fmtTime(endMs)}`;
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
  $("btnExport").disabled = total < 2 || isExporting;
  $("btnExportBundle").disabled = isExporting;
  $("btnImportBundle").disabled = isExporting;
  $("btnSaveSession").disabled = total === 0 || isExporting;
  $("btnOpenSession").disabled = isExporting;
  $("btnResetRec").disabled = isExporting;

  $("btnPlay").textContent = isPlaying ? "⏸ Pause" : "▶ Play";
  $("btnPlay").classList.toggle("replay-on", isPlaying);

  const liveBtn = $("btnLive");
  if (isReplay) { liveBtn.textContent = "↺ LIVE"; liveBtn.className = "btn replay-on"; }
  else { liveBtn.textContent = "● LIVE"; liveBtn.className = "btn live-on"; }
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

function applyEventToDOM(d, tMs) {
  const [vis, cls] = getRawClassAndVisible(d);
  stepOdom(liveOdom, d, tMs);

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
    $("spdVal").textContent = liveOdom.speedCmS.toFixed(1);
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
    if (d.status) { $("statusPill").textContent = d.status; $("statusPill").className = statePillClass(d.status); }
  } else if (d.type === "cal_row") {
    const r = calRows[d.idx];
    r.min.textContent = d.min; r.max.textContent = d.max; r.span.textContent = d.span;
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

  if (mapMode && (d.type === "dat" || d.type === "pos" || d.type === "sync")) drawMap(liveOdom);
  if (vis && d.raw) appendRaw(d.raw, cls);
}

function buildStateUpTo(targetIdx) {
  const s = {
    pos: 3500, vis: null, hz: 0, status: "IDLE",
    calTxt: "CAL: -", calOk: null, trackTxt: "TRACK: -", trackErr: false,
    synced: false, syncMcuMs: null, syncT: null,
    encL: "-", encR: "-", irL: null, irR: null, usDist: null, act: "-", trans: "",
    dirL: "-", pwmL: 0, dirR: "-", pwmR: 0,
    pidErr: "-", pidP: "-", pidI: "-", pidD: "-", pidCorr: "-",
    avgL: null, avgR: null, rows: new Array(8).fill(null),
    posList: [], rawList: [], odom: createOdom(),
  };

  const end = Math.min(targetIdx, recorded.length - 1);
  for (let i = 0; i <= end; i++) {
    const rec = recorded[i], d = rec.d;
    stepOdom(s.odom, d, rec.t);
    const [vis, cls] = getRawClassAndVisible(d);
    if (vis && d.raw) { s.rawList.push([d.raw, cls]); if (s.rawList.length > 200) s.rawList.shift(); }
    if (d.type === "sync") { s.synced = true; s.syncMcuMs = d.mcu_ms; s.syncT = rec.t; }
    else if (d.type === "pos") { s.pos = d.pos; s.vis = d.vis; s.hz = d.hz; s.posList.push(d.pos); if (s.posList.length > 200) s.posList.shift(); }
    else if (d.type === "dat") {
      s.encL = d.encL; s.encR = d.encR; s.irL = d.irL; s.irR = d.irR; s.usDist = d.us;
      s.dirL = d.dirL; s.pwmL = d.pwmL; s.dirR = d.dirR; s.pwmR = d.pwmR;
      s.pidErr = d.err; s.pidP = d.pTerm; s.pidI = d.iTerm; s.pidD = d.dTerm; s.pidCorr = d.corr;
      s.status = d.st; s.act = d.st;
    } else if (d.type === "action") { s.status = d.to; s.act = d.to; s.trans = `${d.from} → ${d.to}`; }
    else if (d.type === "system") { if (d.status) s.status = d.status; }
    else if (d.type === "cal_row") { s.rows[d.idx] = d; }
    else if (d.type === "cal_avg") { if (d.side === "L") s.avgL = d; else s.avgR = d; }
    else if (d.type === "cal_result") { s.calTxt = d.ok ? "CAL: OK" : "CAL: LOW"; s.calOk = d.ok; }
    else if (d.type === "track") { s.trackTxt = "TRACK: " + d.mode; s.trackErr = d.mode === "UNCERTAIN"; }
    else if (d.type === "status") { s.status = d.status; }
  }
  return s;
}

function renderSnapshotAt(targetIdx) {
  const s = buildStateUpTo(targetIdx);
  liveOdom = s.odom;

  $("posVal").textContent = s.pos;
  const err = s.pos - 3500;
  $("errVal").textContent = (err > 0 ? "+" : "") + err;
  $("posMarker").style.left = Math.max(0, Math.min(100, (s.pos / 7000) * 100)) + "%";
  $("hzVal").textContent = s.hz;
  $("spdVal").textContent = s.odom.speedCmS.toFixed(1);
  paintVis(s.vis);
  paintSyncBadge(s.synced, s.syncMcuMs);

  $("statusPill").textContent = s.status;
  $("statusPill").className = statePillClass(s.status);
  $("calPill").textContent = s.calTxt;
  $("calPill").className = "pill" + (s.calOk === null ? "" : (s.calOk ? " ok" : " err"));
  $("trackPill").textContent = s.trackTxt;
  $("trackPill").className = "pill" + (s.trackTxt === "TRACK: -" ? "" : (s.trackErr ? " err" : " ok"));

  $("encL").textContent = s.encL; $("encR").textContent = s.encR;
  paintIR(s.irL, s.irR); paintUS(s.usDist); paintAct(s.act, s.trans);
  paintMotor("L", s.dirL, s.pwmL); paintMotor("R", s.dirR, s.pwmR);
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
  if (mapMode) drawMap(liveOdom);

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

async function hydrateSessionFromServer() {
  try {
    const res = await fetch("/api/session");
    if (!res.ok) return;
    const data = await res.json();
    recorded = Array.isArray(data.events) ? data.events : [];
    lastSeq = recorded.length ? recorded[recorded.length - 1].seq : 0;
    isReplay = false; isPlaying = false;
    replayIdx = Math.max(0, recorded.length - 1);
    recomputeSyncIndex();
    renderSnapshotAt(recorded.length - 1);
    updateReplayBar();
  } catch {}
}

// ==================== SERVER-SIDE RTX NVENC TRIPLE-MP4 EXPORT ====================
async function startServerExport() {
  if (isExporting || recorded.length < 2) return;
  isExporting = true;
  isPlaying = false;
  const btn = $("btnExport");
  btn.classList.add("busy");
  btn.textContent = "0% Start";
  updateReplayBar();

  try {
    const res = await fetch("/api/export/start", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        from_sync: $("chkFromSync").checked,
        show_sensor: showSensor,
        map_zoom: mapPxPerCm,
      }),
    });
    const d = await res.json();
    if (!res.ok || !d.ok) {
      alert(d.error || "Failed to start server export.");
      finishExportUI();
      return;
    }

    const jobId = d.job_id;
    const poll = setInterval(async () => {
      try {
        const sRes = await fetch(`/api/export/status/${jobId}`);
        const sData = await sRes.json();
        if (!sData.ok) { clearInterval(poll); alert(sData.error || "Export job lost."); finishExportUI(); return; }
        btn.textContent = `${sData.percent}% ${sData.stage}`;
        if (sData.status === "done") {
          clearInterval(poll);
          window.location.href = `/api/export/download/${jobId}`;
          finishExportUI();
        } else if (sData.status === "error") {
          clearInterval(poll);
          alert(`Export failed: ${sData.error}`);
          finishExportUI();
        }
      } catch {
        clearInterval(poll);
        finishExportUI();
      }
    }, 300);
  } catch {
    alert("Could not reach server for export.");
    finishExportUI();
  }
}

function finishExportUI() {
  isExporting = false;
  const btn = $("btnExport");
  btn.classList.remove("busy");
  btn.textContent = "⬇ 3× MP4";
  updateReplayBar();
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
    applyEventToDOM(recorded[replayIdx].d, recorded[replayIdx].t);
    advanced = true;
  }
  if (advanced) updateReplayBar();
  if (replayIdx >= recorded.length - 1) { isPlaying = false; updateReplayBar(); return; }
  requestAnimationFrame(playbackStep);
}

$("btnToggleView").onclick = () => {
  mapMode = !mapMode;
  $("rawBox").style.display = mapMode ? "none" : "block";
  $("mapWrap").style.display = mapMode ? "block" : "none";
  $("btnResetMap").style.display = mapMode ? "inline-block" : "none";
  $("btnData").style.display = mapMode ? "none" : "inline-block";
  $("btnPause").style.display = mapMode ? "none" : "inline-block";
  $("btnClear").style.display = mapMode ? "none" : "inline-block";
  $("rightTitle").textContent = mapMode ? "2D Odometry & Track Map" : "Logs, Transitions & Reports";
  $("btnToggleView").textContent = mapMode ? "Console View" : "Map View";
  if (mapMode) drawMap(liveOdom);
};

$("btnResetMap").onclick = () => {
  const prevL = liveOdom.prevEncL, prevR = liveOdom.prevEncR, prevT = liveOdom.prevDatT;
  liveOdom = createOdom();
  liveOdom.prevEncL = prevL; liveOdom.prevEncR = prevR; liveOdom.prevDatT = prevT;
  followBot = true; panOffsetX = 0; panOffsetY = 0;
  updateZoomLabel(); drawMap(liveOdom);
};

$("btnFollowBot").onclick = () => {
  followBot = true; panOffsetX = 0; panOffsetY = 0;
  updateZoomLabel(); if (mapMode) drawMap(liveOdom);
};

$("btnZoomIn").onclick = () => setMapZoom(mapPxPerCm * 1.25);
$("btnZoomOut").onclick = () => setMapZoom(mapPxPerCm / 1.25);
$("btnZoomReset").onclick = () => setMapZoom(DEFAULT_MAP_PX_CM);
mapCanvas.addEventListener("wheel", (e) => {
  e.preventDefault();
  setMapZoom(e.deltaY < 0 ? mapPxPerCm * 1.15 : mapPxPerCm / 1.15);
}, { passive: false });

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
  if (isPlaying) { isPlaying = false; updateReplayBar(); return; }
  isReplay = true;
  if (replayIdx >= recorded.length - 1) { replayIdx = 0; renderSnapshotAt(0); }
  playClock = recorded[replayIdx].t;
  lastRaf = performance.now();
  isPlaying = true;
  updateReplayBar();
  requestAnimationFrame(playbackStep);
};

$("btnSpeed").onclick = () => { speedIdx = (speedIdx + 1) % SPEEDS.length; $("btnSpeed").textContent = SPEEDS[speedIdx] + "x"; };
$("btnJumpSync").onclick = () => {
  if (syncIdx === null || isExporting) return;
  isReplay = true; isPlaying = false; replayIdx = syncIdx;
  renderSnapshotAt(replayIdx); updateReplayBar();
};
$("scrubber").oninput = (e) => {
  if (!recorded.length || isExporting) return;
  isReplay = true; isPlaying = false; replayIdx = Number(e.target.value);
  renderSnapshotAt(replayIdx); updateReplayBar();
};
$("btnLive").onclick = () => {
  if (isExporting) return;
  isReplay = false; isPlaying = false;
  if (recorded.length) renderSnapshotAt(recorded.length - 1);
  updateReplayBar();
};

$("btnExport").onclick = startServerExport;

// Full Data + Config + Frames ZIP Export / Import (No MP4)
$("btnExportBundle").onclick = () => {
  window.location.href = "/api/bundle/export";
};

$("btnImportBundle").onclick = () => {
  $("bundleFileInput").value = "";
  $("bundleFileInput").click();
};

$("bundleFileInput").onchange = async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const btn = $("btnImportBundle");
  btn.textContent = "Importing...";
  btn.disabled = true;
  try {
    const res = await fetch("/api/bundle/import", {
      method: "POST",
      headers: { "Content-Type": "application/zip" },
      body: await file.arrayBuffer(),
    });
    const d = await res.json().catch(() => ({}));
    if (!res.ok || !d.ok) {
      alert(d.error || "Failed to import ZIP bundle.");
    }
  } catch {
    alert("Could not upload the selected ZIP file.");
  } finally {
    btn.textContent = "Import ZIP";
    updateReplayBar();
  }
};

// JSON-only Session Export / Import
$("btnSaveSession").onclick = () => { window.location.href = "/api/session/export"; };
$("btnOpenSession").onclick = () => { $("sessionFileInput").value = ""; $("sessionFileInput").click(); };
$("sessionFileInput").onchange = async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  try {
    const res = await fetch("/api/session/import", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: await file.text(),
    });
    if (!res.ok) { const err = await res.json().catch(() => ({})); alert(err.error || "Failed to load session file."); }
  } catch { alert("Could not read the selected file."); }
};
$("btnResetRec").onclick = async () => { if (!isExporting) await fetch("/api/session/clear", { method: "POST" }); };

const src = new EventSource("/stream");
src.onopen = () => { $("dot").className = "dot on"; $("connText").textContent = "Live"; };
src.onerror = () => { $("dot").className = "dot"; $("connText").textContent = "Reconnecting..."; };
src.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  if (msg.event === "session_cleared") {
    recorded = []; lastSeq = 0; syncIdx = null; syncRecMs = null; syncMcuMs = null;
    isReplay = false; isPlaying = false; replayIdx = 0; liveOdom = createOdom();
    renderSnapshotAt(-1); updateReplayBar(); return;
  }
  if (msg.event === "session_reloaded") { hydrateSessionFromServer(); return; }
  if (msg.event === "record" && msg.record) {
    const rec = msg.record;
    if (rec.seq <= lastSeq) return;
    lastSeq = rec.seq;
    recorded.push(rec);
    if (rec.d.type === "sync") { syncIdx = recorded.length - 1; syncRecMs = rec.t; syncMcuMs = rec.d.mcu_ms; }
    if (recorded.length > MAX_RECORDED) {
      recorded.shift();
      if (isReplay && replayIdx > 0) replayIdx--;
      if (syncIdx !== null) { syncIdx--; if (syncIdx < 0) { syncIdx = null; syncRecMs = null; } }
    }
    if (!isReplay) applyEventToDOM(rec.d, rec.t);
    updateReplayBar();
  }
};

window.addEventListener("resize", () => { drawTrace(); if (mapMode) drawMap(liveOdom); });
updateZoomLabel();
drawTrace();
updateReplayBar();
hydrateSessionFromServer();