const $ = (id) => document.getElementById(id);

const MAX_RECORDED = 60000;
const SPEEDS = [1, 0.5, 0.25, 2];

let recorded = []; // [{ seq, t, d, cam? }]
let lastSeq = 0;
let syncIdx = null, syncRecMs = null, syncMcuMs = null;
let isReplay = false, isPlaying = false, replayIdx = 0;
let speedIdx = 0, playClock = 0, lastRaf = 0;
let isExporting = false;
let lastShownFile = null;

function fmtTime(ms) {
  const neg = ms < 0;
  const s = Math.abs(ms) / 1000;
  const m = Math.floor(s / 60);
  const sec = (s % 60).toFixed(1).padStart(4, "0");
  return `${neg ? "-" : ""}${String(m).padStart(2, "0")}:${sec}`;
}

function statePillClass(st) {
  if (st === "FOLLOW" || st === "RACING") return "pill ok";
  if (st === "JPROBE" || st === "JALIGN" || st === "PIVOT" || st === "CALIBRATING" || st === "WAITING") return "pill warn";
  if (st === "LOST" || st === "STOP" || st === "STOPPED") return "pill err";
  if (st === "CHECKER" || st === "FINISH") return "pill info";
  return "pill";
}

function recomputeSyncIndex() {
  syncIdx = null; syncRecMs = null; syncMcuMs = null;
  let camCount = 0;
  for (let i = 0; i < recorded.length; i++) {
    if (recorded[i].cam) camCount++;
    if (recorded[i].d.type === "sync") {
      syncIdx = i;
      syncRecMs = recorded[i].t;
      syncMcuMs = recorded[i].d.mcu_ms;
    }
  }
  $("camCountPill").textContent = `FRAMES: ${camCount}`;
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
  $("btnResetRec").disabled = isExporting;

  $("btnPlay").textContent = isPlaying ? "⏸ Pause" : "▶ Play";
  $("btnPlay").classList.toggle("replay-on", isPlaying);

  const liveBtn = $("btnLive");
  if (isReplay) { liveBtn.textContent = "↺ LIVE"; liveBtn.className = "btn replay-on"; }
  else { liveBtn.textContent = "● LIVE"; liveBtn.className = "btn live-on"; }
}

function renderFrameAt(targetIdx) {
  let status = "IDLE", pos = "-", err = "-", encL = "-", encR = "-";
  let pwmL = "-", pwmR = "-", us = "-", irL = "-", irR = "-";
  let synced = false, syncMcu = null, camFile = null;

  const end = Math.min(targetIdx, recorded.length - 1);
  for (let i = 0; i <= end; i++) {
    const rec = recorded[i];
    const d = rec.d;
    if (rec.cam) camFile = rec.cam;
    if (d.type === "sync") { synced = true; syncMcu = d.mcu_ms; }
    else if (d.type === "pos") { pos = d.pos; err = d.pos - 3500; }
    else if (d.type === "dat") {
      encL = d.encL; encR = d.encR;
      pwmL = `${d.dirL}:${d.pwmL}`; pwmR = `${d.dirR}:${d.pwmR}`;
      us = d.us < 0 ? "OOR" : `${d.us}cm`;
      irL = d.irL; irR = d.irR;
      status = d.st;
    } else if (d.type === "action") { status = d.to; }
    else if (d.type === "status") { status = d.status; }
  }

  $("statusPill").textContent = status;
  $("statusPill").className = statePillClass(status);
  $("syncPill").textContent = synced ? `LED ON (${syncMcu ?? 0}ms)` : "SYNC: WAIT";
  $("syncPill").className = synced ? "pill sync-on" : "pill";
  $("posVal").textContent = pos;
  $("encVal").textContent = `${encL} / ${encR}`;
  $("pwmVal").textContent = `${pwmL} / ${pwmR}`;
  $("hudOverlay").textContent =
    `State: ${status} | Pos: ${pos} | Err: ${err} | US: ${us} | IR: ${irL} / ${irR}` +
    (camFile ? ` | File: ${camFile}` : "");

  const img = $("replayImg");
  const msg = $("noFrameMsg");
  if (camFile) {
    if (camFile !== lastShownFile) {
      img.src = `/api/cam/file/${encodeURIComponent(camFile)}`;
      lastShownFile = camFile;
    }
    img.style.display = "block";
    msg.style.display = "none";
  } else {
    img.style.display = "none";
    msg.style.display = "block";
    lastShownFile = null;
  }
}

function playbackStep(ts) {
  if (!isPlaying) return;
  const dt = (ts - lastRaf) * SPEEDS[speedIdx];
  lastRaf = ts;
  playClock += dt;

  let advanced = false;
  while (replayIdx + 1 < recorded.length && recorded[replayIdx + 1].t <= playClock) {
    replayIdx++;
    advanced = true;
  }
  if (advanced) {
    renderFrameAt(replayIdx);
    updateReplayBar();
  }
  if (replayIdx >= recorded.length - 1) {
    isPlaying = false;
    updateReplayBar();
    return;
  }
  requestAnimationFrame(playbackStep);
}

async function hydrateFromServer() {
  try {
    const res = await fetch("/api/session");
    if (!res.ok) return;
    const data = await res.json();
    recorded = Array.isArray(data.events) ? data.events : [];
    lastSeq = recorded.length ? recorded[recorded.length - 1].seq : 0;
    isReplay = false;
    isPlaying = false;
    replayIdx = Math.max(0, recorded.length - 1);
    recomputeSyncIndex();
    renderFrameAt(recorded.length - 1);
    updateReplayBar();
  } catch {}
}

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
      body: JSON.stringify({ from_sync: $("chkFromSync").checked, show_sensor: false, map_zoom: 6.0 }),
    });
    const d = await res.json();
    if (!res.ok || !d.ok) { alert(d.error || "Export failed."); finishExportUI(); return; }

    const jobId = d.job_id;
    const poll = setInterval(async () => {
      try {
        const sRes = await fetch(`/api/export/status/${jobId}`);
        const sData = await sRes.json();
        if (!sData.ok) { clearInterval(poll); finishExportUI(); return; }
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
      } catch { clearInterval(poll); finishExportUI(); }
    }, 300);
  } catch { finishExportUI(); }
}

function finishExportUI() {
  isExporting = false;
  const btn = $("btnExport");
  btn.classList.remove("busy");
  btn.textContent = "⬇ 3× MP4";
  updateReplayBar();
}

// Controls
$("btnPlay").onclick = () => {
  if (!recorded.length || isExporting) return;
  if (isPlaying) { isPlaying = false; updateReplayBar(); return; }
  isReplay = true;
  if (replayIdx >= recorded.length - 1) { replayIdx = 0; renderFrameAt(0); }
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
  renderFrameAt(replayIdx); updateReplayBar();
};
$("scrubber").oninput = (e) => {
  if (!recorded.length || isExporting) return;
  isReplay = true; isPlaying = false; replayIdx = Number(e.target.value);
  renderFrameAt(replayIdx); updateReplayBar();
};
$("btnLive").onclick = () => {
  if (isExporting) return;
  isReplay = false; isPlaying = false;
  if (recorded.length) renderFrameAt(recorded.length - 1);
  updateReplayBar();
};
$("btnExport").onclick = startServerExport;
$("btnResetRec").onclick = async () => { if (!isExporting) await fetch("/api/session/clear", { method: "POST" }); };

// Live SSE Stream
const src = new EventSource("/stream");
src.onopen = () => { $("dot").className = "dot on"; $("connText").textContent = "Live"; };
src.onerror = () => { $("dot").className = "dot"; $("connText").textContent = "Reconnecting..."; };
src.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  if (msg.event === "session_cleared") {
    recorded = []; lastSeq = 0; syncIdx = null; syncRecMs = null; syncMcuMs = null;
    isReplay = false; isPlaying = false; replayIdx = 0;
    recomputeSyncIndex(); renderFrameAt(-1); updateReplayBar(); return;
  }
  if (msg.event === "session_reloaded") { hydrateFromServer(); return; }
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
    if (rec.cam) recomputeSyncIndex();
    if (!isReplay) renderFrameAt(recorded.length - 1);
    updateReplayBar();
  }
};

hydrateFromServer();