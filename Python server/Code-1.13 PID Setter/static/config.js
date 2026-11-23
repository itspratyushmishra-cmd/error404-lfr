const $ = (id) => document.getElementById(id);

const KEYS = [
  "Kp", "Ki", "Kd",
  "speedStraight", "speedGentle", "speedMedium", "speedSharp", "speedMinBase",
  "probeSpeed", "alignSpeed", "pivotSpeed", "searchSpeed",
  "trackMode", "junctionPolicy", "stopAtFinish", "autoStart", "uturnDir",
  "leftInvert", "rightInvert", "encLeftInvert", "encRightInvert"
];

function collectForm() {
  const data = {};
  for (const k of KEYS) {
    data[k] = $(k).value;
  }
  return data;
}

function populateForm(cfg) {
  for (const k of KEYS) {
    if (cfg[k] !== undefined) $(k).value = cfg[k];
  }
  updatePreview();
}

function updatePreview() {
  const vals = KEYS.map((k) => $(k).value || "0").join(",");
  $("packetPreview").textContent = `$CFG,${vals}`;
}

function showMsg(text, color) {
  const el = $("cfgMsg");
  el.textContent = text;
  el.style.color = color;
}

async function pollStatus() {
  try {
    const res = await fetch("/api/status");
    const d = await res.json();
    $("espDot").className = d.esp_connected ? "dot on" : "dot";
    $("espText").textContent = d.esp_connected ? "ESP Connected" : "ESP Disconnected";
  } catch {
    $("espDot").className = "dot";
    $("espText").textContent = "Server Offline";
  }
}

async function loadConfig() {
  const res = await fetch("/api/config");
  const d = await res.json();
  populateForm(d.config);
}

for (const k of KEYS) {
  $(k).addEventListener("input", updatePreview);
  $(k).addEventListener("change", updatePreview);
}

$("btnSave").onclick = async () => {
  const res = await fetch("/api/config", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(collectForm()),
  });
  const d = await res.json();
  if (d.ok) {
    populateForm(d.config);
    showMsg("Saved on server", "var(--green)");
  } else {
    showMsg(d.error || "Save failed", "var(--red)");
  }
};

$("btnSend").onclick = async () => {
  showMsg("Sending...", "var(--amber)");
  const res = await fetch("/api/config/send", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(collectForm()),
  });
  const d = await res.json();
  if (d.ok) {
    showMsg("Saved & Sent to ESP", "var(--green)");
  } else {
    showMsg(d.error || "Send failed", "var(--red)");
  }
};

$("btnReset").onclick = async () => {
  const res = await fetch("/api/config/reset", { method: "POST" });
  const d = await res.json();
  if (d.ok) {
    populateForm(d.config);
    showMsg("Reset to defaults", "var(--amber)");
  }
};

loadConfig();
pollStatus();
setInterval(pollStatus, 2000);