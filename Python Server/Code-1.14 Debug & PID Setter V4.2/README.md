# Error 404 — Line-Follower Telemetry, Odometry & Video Dashboard

A local Python Flask dashboard for real-time wireless telemetry, remote PID/speed configuration, 2D encoder odometry mapping, synchronized landscape phone camera capture, full-session ZIP backup/restore, and GPU-accelerated triple-MP4 export for an Arduino Mega + ESP8266 line-following robot.

---

## Features

* **Live Telemetry Monitor (`/`):** Real-time QTR-8A line position & rolling trace, encoder counts, calculated speed (cm/s), background IR sensors, ultrasonic distance, motor direction/PWM bars, PID breakdown (`ERR`, `P`, `I`, `D`, `OUT`), blue-highlighted state transitions, and 8-sensor calibration report.
* **Interactive 2D Odometry Map:** Toggleable view in the right panel with a scale top-down icon of the physical robot, live track-line drawing (when `Vis == 1`), drag-to-pan, mouse-wheel/button zoom, and **Follow** lock.
* **Remote Configuration (`/config`):** Edit all 22 robot parameters, save/load local `.json` config files, and push `$CFG` packets over TCP. Saved configs are automatically sent 2 seconds after the ESP connects.
* **Phone Camera & Synced Capture (`/phone`, `/camera`, `/replay`):**
  * `/phone`: Mobile-first rear/front camera streamer running at max FPS with **Flip**, **Rotate 90°** (`0° / 90° / 180° / 270°`), resolution selector, Screen Wake Lock, and **locked 16:9 landscape output** so feeds never fluctuate between portrait and landscape.
  * `/camera`: Live max-FPS MJPEG viewer on PC.
  * **Event-Locked Recording:** Camera snapshots are saved to disk on every incoming event—including pre-sync setup, calibration, and live telemetry frames (`pos`, `dat`, `sync`, `action`).
  * `/replay`: Scrubbable timeline player with a fixed 16:9 landscape stage, synced telemetry HUD, and `[SYNC]` marker.
* **Persistent Cross-Device Sessions & Full ZIP Backup:**
  * All telemetry (`session_data.jsonl`) and camera frames (`cam_frames/`) persist on the server across reloads and devices.
  * **Export ZIP / Import ZIP:** Download or restore a complete session archive (`config.json`, `session_data.json`, and `cam_frames/`) without MP4s.
  * **Save JSON / Open JSON / Clear:** Export/import telemetry JSON only, or wipe the current session.
* **Server-Side RTX NVENC MP4 Export (`⬇ 3× MP4`):** Multi-core cached frame renderer piped to FFmpeg (`h264_nvenc` on NVIDIA GPUs with automatic `libx264` CPU fallback) showing live `%` progress. Downloads a `.zip` containing **only** these 5 files:
  1. `telemetry_data.mp4` (1920×1080 full dashboard + console)
  2. `odometry_map.mp4` (1280×720 dedicated 2D map)
  3. `camera_feed.mp4` (1280×720 synced landscape phone camera video)
  4. `config.json` (active robot configuration)
  5. `session_data.json` (full monitor telemetry & replay log)
  * **Sync Checkbox:** When **ticked**, all three MP4s start at the exact `[SYNC] LED_ON` timestamp. When **unticked**, all pre-sync frames (calibration/setup) are included from the start of the session.

---

## Folder Structure

```text
esp_dashboard/
├── app.py                  # Flask server, TCP listener, SSE stream, session, bundle & camera APIs
├── video_exporter.py       # Multi-core + RTX NVENC triple-MP4 + data bundle exporter
├── README.md               # Project documentation
├── config.json             # Auto-created: saved robot configuration
├── session_data.jsonl      # Auto-created: persistent telemetry log
├── cam_frames/             # Auto-created: event-synced camera JPEGs + index.jsonl
├── static/
│   ├── style.css           # Shared desktop UI stylesheet
│   ├── monitor.js          # Telemetry monitor, 2D odometry map, ZIP import/export & replay client
│   ├── config.js           # Config form, scaling preview, local JSON save/upload
│   ├── phone_cam.js        # Mobile camera capture, 90° rotation & locked-landscape max-FPS uploader
│   └── cam_replay.js       # Synced camera frame timeline player
└── templates/
    ├── index.html          # Monitor page (/)
    ├── config.html         # Config page (/config)
    ├── cam_view.html       # PC live camera viewer (/camera)
    ├── cam_replay.html     # Camera frame replay (/replay)
    └── phone_cam.html      # Mobile camera page (/phone)