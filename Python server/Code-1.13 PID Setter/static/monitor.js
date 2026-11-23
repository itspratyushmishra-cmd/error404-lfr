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
  if (st === "FOLLOW") return "pill ok";
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

function paintAct(act) {
  const el = $("actBox");
  if (!act || act === "-") {
    el.textContent = "-";
    el.className = "act-box";
    return;
  }
  el.textContent = act;
  if (act === "STOP" || act === "STOPPED" || act === "LOST" || act === "BACK") el.className = "act-box act-stop";
  else if (act === "STRAIGHT" || act === "FORWARD" || act === "FOLLOW") el.className = "act-box act-go";
  else el.className = "act-box act-turn";
}

// Session History & Replay State
const MAX_RECORDED = 60000;
const SPEEDS = [1, 0.5, 0.25, 2];
let recorded = [];
let recStart = null;
let isReplay = false;
let isPlaying = false;
let replayIdx = 0;
let speedIdx = 0;
let playClock = 0;
let lastRaf = 0;

let showSensor = false, paused = false;

function fmtTime(ms) {
  const s = Math.max(0, ms) / 1000;
  const m = Math.floor(s / 60);
  const sec = (s % 60).toFixed(1).padStart(4, "0");
  return `${String(m).padStart(2, "0")}:${sec}`;
}

function updateReplayBar() {
  const total = recorded.length;
  const scrub = $("scrubber");
  scrub.disabled = total === 0;
  scrub.max = Math.max(0, total - 1);

  const curIdx = isReplay ? replayIdx : Math.max(0, total - 1);
  scrub.value = curIdx;

  const curMs = total ? recorded[curIdx].t : 0;
  const endMs = total ? recorded[total - 1].t : 0;
  $("timeInfo").textContent = `${fmtTime(curMs)} / ${fmtTime(endMs)} (${total})`;

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
  if (d.type === "act")        return [showSensor, "r-dat"];
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

  if (d.type === "pos") {
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
    $("statusPill").textContent = d.st;
    $("statusPill").className = statePillClass(d.st);
    paintAct(d.act || d.st);
  } else if (d.type === "act") {
    paintAct(d.act);
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

function renderSnapshotAt(targetIdx) {
  let pos = 3500, visState = null, hz = 0, status = "IDLE";
  let calTxt = "CAL: -", calCls = "pill";
  let trackTxt = "TRACK: -", trackCls = "pill";
  let encL = "-", encR = "-", irL = null, irR = null, usDist = null, act = "-";
  let avgL = null, avgR = null;
  const rows = new Array(8).fill(null);
  const posList = [];
  const rawList = [];

  const end = Math.min(targetIdx, recorded.length - 1);
  for (let i = 0; i <= end; i++) {
    const d = recorded[i].d;
    const [vis, cls] = getRawClassAndVisible(d);
    if (vis && d.raw) {
      rawList.push([d.raw, cls]);
      if (rawList.length > 200) rawList.shift();
    }
    if (d.type === "pos") {
      pos = d.pos; visState = d.vis; hz = d.hz;
      posList.push(d.pos);
      if (posList.length > 200) posList.shift();
    } else if (d.type === "dat") {
      encL = d.encL; encR = d.encR;
      irL = d.irL; irR = d.irR;
      usDist = d.us;
      status = d.st;
      act = d.act || d.st;
    } else if (d.type === "act") {
      act = d.act;
    } else if (d.type === "cal_row") {
      rows[d.idx] = d;
    } else if (d.type === "cal_avg") {
      if (d.side === "L") avgL = d; else avgR = d;
    } else if (d.type === "cal_result") {
      calTxt = d.ok ? "CAL: OK" : "CAL: LOW";
      calCls = "pill " + (d.ok ? "ok" : "err");
    } else if (d.type === "track") {
      trackTxt = "TRACK: " + d.mode;
      trackCls = "pill " + (d.mode === "UNCERTAIN" ? "err" : "ok");
    } else if (d.type === "status") {
      status = d.status;
    }
  }

  $("posVal").textContent = pos;
  const err = pos - 3500;
  $("errVal").textContent = (err > 0 ? "+" : "") + err;
  $("posMarker").style.left = Math.max(0, Math.min(100, (pos / 7000) * 100)) + "%";
  $("hzVal").textContent = hz;
  paintVis(visState);

  $("statusPill").textContent = status;
  $("statusPill").className = statePillClass(status);
  $("calPill").textContent = calTxt;
  $("calPill").className = calCls;
  $("trackPill").textContent = trackTxt;
  $("trackPill").className = trackCls;

  $("encL").textContent = encL;
  $("encR").textContent = encR;
  paintIR(irL, irR);
  paintUS(usDist);
  paintAct(act);

  for (let i = 0; i < 8; i++) {
    const r = calRows[i], d = rows[i];
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

  $("avgL").textContent = avgL ? `${avgL.val} (${avgL.ok ? "OK" : "LOW"})` : "-";
  $("avgL").style.color = avgL ? (avgL.ok ? "var(--green)" : "var(--red)") : "";
  $("avgR").textContent = avgR ? `${avgR.val} (${avgR.ok ? "OK" : "LOW"})` : "-";
  $("avgR").style.color = avgR ? (avgR.ok ? "var(--green)" : "var(--red)") : "";

  history = new Array(200 - posList.length).fill(3500).concat(posList);
  drawTrace();

  const box = $("rawBox");
  const frag = document.createDocumentFragment();
  for (const [text, cls] of rawList) {
    const div = document.createElement("div");
    div.className = cls;
    div.textContent = text;
    frag.appendChild(div);
  }
  box.textContent = "";
  box.appendChild(frag);
  box.scrollTop = box.scrollHeight;
}

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

// Controls
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
  if (!recorded.length) return;
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

$("scrubber").oninput = (e) => {
  if (!recorded.length) return;
  isReplay = true;
  isPlaying = false;
  replayIdx = Number(e.target.value);
  renderSnapshotAt(replayIdx);
  updateReplayBar();
};

$("btnLive").onclick = () => {
  isReplay = false;
  isPlaying = false;
  if (recorded.length) renderSnapshotAt(recorded.length - 1);
  updateReplayBar();
};

$("btnResetRec").onclick = () => {
  recorded = [];
  recStart = null;
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

  recorded.push({ t: now - recStart, d });
  if (recorded.length > MAX_RECORDED) {
    recorded.shift();
    if (isReplay && replayIdx > 0) replayIdx--;
  }

  if (!isReplay) {
    applyEventToDOM(d);
  }
  updateReplayBar();
};

window.addEventListener("resize", drawTrace);
drawTrace();
updateReplayBar();