const $ = (id) => document.getElementById(id);

const KEYS = [
  "Kp", "Ki", "Kd", "speedScale",
  "speedStraight", "speedGentle", "speedMedium", "speedSharp", "speedMinBase",
  "probeSpeed", "alignSpeed", "pivotSpeed", "searchSpeed",
  "trackMode", "junctionPolicy", "stopAtFinish", "autoStart", "uturnDir",
  "leftInvert", "rightInvert", "encLeftInvert", "encRightInvert"
];

const FLOAT_KEYS = new Set(["Kp", "Ki", "Kd", "speedScale"]);

function collectForm() {
  const data = {};
  for (const k of KEYS) {
    const raw = $(k).value;
    data[k] = FLOAT_KEYS.has(k) ? parseFloat(raw) : parseInt(raw, 10);
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
  const vals = KEYS.map((k) => {
    const raw = $(k).value || "0";
    return FLOAT_KEYS.has(k) ? String(Math.round((parseFloat(raw) || 0) * 10000)) : raw;
  }).join(",");
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

$("btnDownloadCfg").onclick = () => {
  const data = collectForm();
  const blob = new Blob([JSON.stringify(data, null, 2)], { type: "application/json" });
  const stamp = new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = `robot_config_${stamp}.json`;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
  showMsg("Config file downloaded", "var(--cyan)");
};

$("btnUploadCfg").onclick = () => {
  $("cfgFileInput").value = "";
  $("cfgFileInput").click();
};

$("cfgFileInput").onchange = async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  try {
    const parsed = JSON.parse(await file.text());
    const cfgObj = parsed.config && typeof parsed.config === "object" ? parsed.config : parsed;
    const res = await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(cfgObj),
    });
    const d = await res.json();
    if (d.ok) {
      populateForm(d.config);
      showMsg(`Loaded ${file.name}`, "var(--green)");
    } else {
      showMsg(d.error || "Invalid config file", "var(--red)");
    }
  } catch {
    showMsg("Failed to parse JSON file", "var(--red)");
  }
};

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