const $ = (id) => document.getElementById(id);
const video = $("video");
const offCanvas = document.createElement("canvas");
const offCtx = offCanvas.getContext("2d");

let stream = null;
let streaming = false;
let facingMode = "environment";
let rotationDeg = 0; // 0, 90, 180, 270
let wakeLock = null;

function showErr(msg) {
  const box = $("errBox");
  box.textContent = msg;
  box.style.display = "block";
}

function hideErr() {
  $("errBox").style.display = "none";
}

function applyPreviewRotation() {
  video.style.transform = `rotate(${rotationDeg}deg)`;
  const btn = $("btnRotate");
  btn.textContent = `↻ ${rotationDeg}°`;
  btn.classList.toggle("rot-active", rotationDeg !== 0);
}

async function requestWakeLock() {
  try {
    if ("wakeLock" in navigator) {
      wakeLock = await navigator.wakeLock.request("screen");
    }
  } catch {}
}

function releaseWakeLock() {
  if (wakeLock) {
    wakeLock.release().catch(() => {});
    wakeLock = null;
  }
}

async function startCamera() {
  hideErr();
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
    showErr(
      "Camera blocked on HTTP. In Android Chrome, open chrome://flags/#unsafely-treat-insecure-origin-as-secure, add " +
      window.location.origin + ", enable it, and relaunch Chrome."
    );
    return;
  }

  stopCamera();
  const [w, h] = $("resSel").value.split("x").map(Number);

  try {
    stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: { ideal: facingMode },
        width: { ideal: w },
        height: { ideal: h },
        frameRate: { ideal: 60, max: 60 },
      },
      audio: false,
    });
    video.srcObject = stream;
    await video.play();
    streaming = true;
    requestWakeLock();

    $("btnStart").textContent = "Stop Cam";
    $("btnStart").className = "btn danger";
    $("camDot").className = "dot on";
    $("camState").textContent = "Live";
    pumpFrames();
  } catch (err) {
    showErr("Camera error: " + err.message);
  }
}

function stopCamera() {
  streaming = false;
  releaseWakeLock();
  if (stream) {
    stream.getTracks().forEach((t) => t.stop());
    stream = null;
  }
  video.srcObject = null;
  $("btnStart").textContent = "Start Cam";
  $("btnStart").className = "btn primary";
  $("camDot").className = "dot";
  $("camState").textContent = "Stopped";
  $("fpsVal").textContent = "0";
}

async function pumpFrames() {
  while (streaming) {
    if (video.readyState >= 2 && video.videoWidth > 0) {
      // Always output fixed 16:9 landscape frames (no portrait/landscape fluctuation)
      const [outW, outH] = $("resSel").value.split("x").map(Number);
      if (offCanvas.width !== outW || offCanvas.height !== outH) {
        offCanvas.width = outW;
        offCanvas.height = outH;
      }

      const vw = video.videoWidth;
      const vh = video.videoHeight;
      const swap = rotationDeg % 180 !== 0;
      const rotW = swap ? vh : vw;
      const rotH = swap ? vw : vh;

      // Fit rotated frame cleanly inside the landscape canvas
      const scale = Math.min(outW / rotW, outH / rotH);
      const drawW = vw * scale;
      const drawH = vh * scale;

      offCtx.fillStyle = "#090c10";
      offCtx.fillRect(0, 0, outW, outH);
      offCtx.save();
      offCtx.translate(outW / 2, outH / 2);
      offCtx.rotate((rotationDeg * Math.PI) / 180);
      offCtx.drawImage(video, -drawW / 2, -drawH / 2, drawW, drawH);
      offCtx.restore();

      const blob = await new Promise((resolve) => offCanvas.toBlob(resolve, "image/jpeg", 0.68));
      if (blob && streaming) {
        try {
          const res = await fetch("/api/cam/frame", {
            method: "POST",
            headers: { "Content-Type": "image/jpeg" },
            body: blob,
          });
          if (res.ok) {
            const d = await res.json();
            $("camState").textContent = "Live";
            $("fpsVal").textContent = d.fps;
            $("recPill").textContent = `SAVED: ${d.frames}`;
            $("recPill").className = d.recording ? "pill ok" : "pill warn";
          }
        } catch {
          $("camState").textContent = "Reconnecting...";
          await new Promise((r) => setTimeout(r, 250));
        }
      }
    } else {
      await new Promise((r) => setTimeout(r, 50));
    }
  }
}

$("btnStart").onclick = () => {
  if (streaming) stopCamera();
  else startCamera();
};

$("btnFlip").onclick = () => {
  facingMode = facingMode === "environment" ? "user" : "environment";
  if (streaming) startCamera();
};

$("btnRotate").onclick = () => {
  rotationDeg = (rotationDeg + 90) % 360;
  applyPreviewRotation();
};

$("resSel").onchange = () => {
  if (streaming) startCamera();
};

applyPreviewRotation();