/*
  ============================================================
  Robotic Arm Controller — ESP32 + PCA9685 + Web Dashboard
  v2.0 — Slow startup sweep + Hard angle limits
  ============================================================
  Libraries required (install via Arduino Library Manager):
    - Adafruit PWM Servo Driver Library  (Adafruit)
    - ESPAsyncWebServer                  (me-no-dev)
    - AsyncTCP                           (me-no-dev)

  PCA9685 Wiring:
    ESP32 SDA  → PCA9685 SDA  (GPIO 21)
    ESP32 SCL  → PCA9685 SCL  (GPIO 22)
    ESP32 3.3V → PCA9685 VCC
    ESP32 GND  → PCA9685 GND
    External 5V/6V Power Supply → PCA9685 V+  (servo power)
    External GND               → PCA9685 GND  (common ground with ESP32)

  Servo Channels on PCA9685:
    Channel 0 → Base Rotation
    Channel 1 → Shoulder
    Channel 2 → Elbow
    Channel 3 → Wrist
    Channel 4 → Gripper
  ============================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ── WiFi Credentials ─────────────────────────────────────────
const char* WIFI_SSID = "Motridox";
const char* WIFI_PASS = "Bassim@8371";

// ── PCA9685 ───────────────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN_US  500       // pulse width at 0°  (µs)
#define SERVO_MAX_US  2500      // pulse width at 180° (µs)
#define OSC_FREQ      25000000  // PCA9685 oscillator frequency
#define PWM_FREQ_HZ   50        // standard servo PWM frequency

// ── Servo Channel Assignments ─────────────────────────────────
#define CH_BASE      0
#define CH_SHOULDER  1
#define CH_ELBOW     2
#define CH_WRIST     3
#define CH_GRIPPER   4
#define NUM_SERVOS   5

// ╔══════════════════════════════════════════════════════════════╗
// ║        SERVO CONFIGURATION — EDIT THIS SECTION ONLY         ║
// ╠══════════════════════════════════════════════════════════════╣
// ║  minAngle / maxAngle : HARD LIMITS — physical stops.        ║
// ║    The firmware will NEVER command beyond these values,     ║
// ║    regardless of what the web dashboard or serial sends.    ║
// ║  homeAngle : safe neutral/rest position ("Home All" btn).   ║
// ╚══════════════════════════════════════════════════════════════╝

struct ServoConfig {
  const char* name;
  const char* icon;    // web dashboard display
  const char* sub;     // web dashboard subtitle
  int  channel;
  int  minAngle;       // ← HARD LIMIT  (physical stop)
  int  maxAngle;       // ← HARD LIMIT  (physical stop)
  int  homeAngle;      // safe neutral position
};

ServoConfig servos[NUM_SERVOS] = {
  //   name         icon   subtitle               ch           min  max  home
  { "Base",     "🔄", "Rotation / Yaw",      CH_BASE,       0, 180,  90 },
  { "Shoulder", "💪", "Joint 1 / Lift",      CH_SHOULDER,   0, 180,  90 },
  { "Elbow",    "🦾", "Joint 2 / Reach",     CH_ELBOW,      0, 180,  90 },
  { "Wrist",    "🤚", "Joint 3 / Tilt",      CH_WRIST,      0, 180,  90 },
  { "Gripper",  "✊", "End Effector",        CH_GRIPPER,    0,  90,  45 },
};

// ── Startup Target Angles ─────────────────────────────────────
// Where servos slowly sweep to on power-up (from 0°).
// Update these after calibration. Values are clamped to hard limits.
//
//                   Base  Shoulder  Elbow  Wrist  Gripper
int startupAngles[NUM_SERVOS] = { 90,   180,     0,    90,     30 };

// ── Speed Settings ────────────────────────────────────────────
#define SERVO_SPEED_STARTUP  30   // °/s  startup sweep (lower = slower/safer)
#define SERVO_SPEED          60   // °/s  runtime (web dashboard commands)

// ─────────────────────────────────────────────────────────────
// DO NOT edit below unless you know what you're doing.
// Hard limits are enforced through clampAngle() which is called
// in every write path — writeServoNow(), setServoAngle(),
// sweepBlocking(), and the /set HTTP handler.
// ─────────────────────────────────────────────────────────────

// Runtime state — currentAngles starts at 0 because PCA9685
// outputs 0 duty cycle on power-up, so servos physically sit at 0°.
int currentAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
int targetAngles[NUM_SERVOS]  = { 0, 0, 0, 0, 0 };
unsigned long lastStepMs = 0;

// ── Web Server ────────────────────────────────────────────────
AsyncWebServer server(80);

// ── Helper: angle → PCA9685 tick count ───────────────────────
uint16_t angleToPwm(int angle, int minAngle, int maxAngle) {
  angle = constrain(angle, minAngle, maxAngle);
  float fraction   = (float)(angle - minAngle) / (maxAngle - minAngle);
  float us         = SERVO_MIN_US + fraction * (SERVO_MAX_US - SERVO_MIN_US);
  float ticksPerUs = (4096.0f * PWM_FREQ_HZ) / 1000000.0f;
  return (uint16_t)(us * ticksPerUs);
}

// ── Central hard-limit clamp ──────────────────────────────────
// Single enforcement point. Every angle value passes through here.
int clampAngle(int ch, int angle) {
  return constrain(angle, servos[ch].minAngle, servos[ch].maxAngle);
}

// Write angle directly to PCA9685 (no smoothing). Clamps to hard limits.
void writeServoNow(int ch, int angle) {
  angle = clampAngle(ch, angle);
  uint16_t tick = angleToPwm(angle, servos[ch].minAngle, servos[ch].maxAngle);
  pca.setPWM(servos[ch].channel, 0, tick);
  currentAngles[ch] = angle;
}

// Queue a target — loop() sweeper will move there gradually.
// Clamps to hard limits before storing.
void setServoAngle(int ch, int angle) {
  targetAngles[ch] = clampAngle(ch, angle);
}

// ── Non-blocking smooth sweep — call from loop() ─────────────
void updateServos() {
  unsigned long now = millis();
  unsigned long dt  = now - lastStepMs;
  if (dt < 20) return;   // ~50 Hz update rate
  lastStepMs = now;

  float maxStep = (SERVO_SPEED * dt) / 1000.0f;

  for (int i = 0; i < NUM_SERVOS; i++) {
    int cur = currentAngles[i];
    int tgt = targetAngles[i];
    if (cur == tgt) continue;

    int diff = tgt - cur;
    int step = (abs(diff) <= (int)maxStep) ? diff
                                           : (diff > 0 ? (int)maxStep : -(int)maxStep);
    if (step == 0) step = (diff > 0) ? 1 : -1;

    writeServoNow(i, cur + step);
  }
}

// ── Blocking slow sweep (used during setup only) ──────────────
// Moves all servos simultaneously to destAngles[] at the given
// speed. Blocks until all arrive. Clamps all targets to hard limits.
void sweepBlocking(int destAngles[], int speedDegPerSec) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetAngles[i] = clampAngle(i, destAngles[i]);
  }

  unsigned long stepTime = millis();
  bool allDone = false;

  while (!allDone) {
    unsigned long now = millis();
    unsigned long dt  = now - stepTime;

    if (dt >= 20) {
      stepTime = now;
      allDone  = true;
      float maxStep = (speedDegPerSec * dt) / 1000.0f;
      if (maxStep < 1.0f) maxStep = 1.0f;

      for (int i = 0; i < NUM_SERVOS; i++) {
        int cur = currentAngles[i];
        int tgt = targetAngles[i];
        if (cur == tgt) continue;

        allDone = false;
        int diff = tgt - cur;
        int step = (abs(diff) <= (int)maxStep) ? diff
                                               : (diff > 0 ? (int)maxStep : -(int)maxStep);
        if (step == 0) step = (diff > 0) ? 1 : -1;

        writeServoNow(i, cur + step);
        Serial.printf("  %s → %d°\n", servos[i].name, currentAngles[i]);
      }
    }

    delay(1);
  }
}

void moveAllHome() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, servos[i].homeAngle);
  }
}

// ── /config JSON builder ──────────────────────────────────────
// Returns per-servo config including hard limits.
// The web dashboard fetches this on load so slider ranges
// always match the firmware limits exactly.
String buildConfigJson() {
  String json = "{\"servos\":[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += "{";
    json += "\"id\":"      + String(i)                        + ",";
    json += "\"name\":\""  + String(servos[i].name)           + "\",";
    json += "\"icon\":\""  + String(servos[i].icon)           + "\",";
    json += "\"sub\":\""   + String(servos[i].sub)            + "\",";
    json += "\"min\":"     + String(servos[i].minAngle)       + ",";
    json += "\"max\":"     + String(servos[i].maxAngle)       + ",";
    json += "\"home\":"    + String(servos[i].homeAngle)      + ",";
    json += "\"current\":" + String(currentAngles[i]);
    json += "}";
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "]}";
  return json;
}

// ── HTML Dashboard ────────────────────────────────────────────
// Slider min/max values are loaded from /config on page load,
// so they automatically reflect the firmware hard limits.
// Hard limit badges (MIN x° / MAX x°) are shown on each card.
const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robotic Arm Control</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600;800&display=swap');

  :root {
    --bg:      #0a0c10;
    --surface: #111520;
    --border:  #1e2a40;
    --accent:  #00d4ff;
    --accent2: #ff6b35;
    --text:    #c8d8f0;
    --dim:     #4a5a78;
    --glow:    0 0 18px rgba(0,212,255,0.35);
  }

  * { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    font-family: 'Exo 2', sans-serif;
    color: var(--text);
    min-height: 100vh;
    padding: 20px;
    background-image:
      radial-gradient(ellipse at 20% 10%, rgba(0,212,255,0.05) 0%, transparent 50%),
      radial-gradient(ellipse at 80% 90%, rgba(255,107,53,0.05) 0%, transparent 50%);
  }

  header {
    text-align: center;
    margin-bottom: 32px;
    padding-bottom: 20px;
    border-bottom: 1px solid var(--border);
  }

  .logo-line {
    display: flex; align-items: center;
    justify-content: center; gap: 14px; margin-bottom: 6px;
  }

  .arm-icon { font-size: 2rem; filter: drop-shadow(0 0 8px var(--accent)); }

  h1 {
    font-size: 2rem; font-weight: 800;
    letter-spacing: 3px; text-transform: uppercase;
    background: linear-gradient(90deg, var(--accent), #60efff);
    -webkit-background-clip: text; -webkit-text-fill-color: transparent;
  }

  .subtitle {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.72rem; color: var(--dim);
    letter-spacing: 4px; text-transform: uppercase;
  }

  .status-bar {
    display: flex; align-items: center; justify-content: center;
    gap: 8px; margin-top: 12px;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.75rem; color: var(--dim);
  }

  .status-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: #22c55e; box-shadow: 0 0 8px #22c55e;
    animation: pulse 2s infinite;
  }

  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.4} }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(290px, 1fr));
    gap: 16px; max-width: 1100px; margin: 0 auto 24px;
  }

  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px; padding: 22px;
    position: relative; overflow: hidden;
    transition: border-color 0.3s, box-shadow 0.3s;
  }

  .card::before {
    content: ''; position: absolute;
    top: 0; left: 0; right: 0; height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0; transition: opacity 0.3s;
  }

  .card:hover { border-color: var(--accent); box-shadow: var(--glow); }
  .card:hover::before { opacity: 1; }
  .gripper-card:hover { border-color: var(--accent2); box-shadow: 0 0 18px rgba(255,107,53,0.35); }

  .card-header {
    display: flex; align-items: center;
    justify-content: space-between; margin-bottom: 12px;
  }

  .joint-label { display: flex; align-items: center; gap: 10px; }

  .joint-icon {
    width: 36px; height: 36px; border-radius: 8px;
    background: rgba(0,212,255,0.1);
    border: 1px solid rgba(0,212,255,0.25);
    display: flex; align-items: center;
    justify-content: center; font-size: 1.1rem;
  }

  .gripper-card .joint-icon {
    background: rgba(255,107,53,0.1);
    border-color: rgba(255,107,53,0.25);
  }

  .joint-name {
    font-weight: 600; font-size: 1rem;
    letter-spacing: 1px; text-transform: uppercase; color: #e0eeff;
  }

  .joint-sub {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.65rem; color: var(--dim); letter-spacing: 1px;
  }

  .angle-display {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.6rem; font-weight: bold;
    color: var(--accent);
    text-shadow: 0 0 12px rgba(0,212,255,0.5);
    min-width: 60px; text-align: right;
  }

  .gripper-card .angle-display {
    color: var(--accent2);
    text-shadow: 0 0 12px rgba(255,107,53,0.5);
  }

  /* Hard limit badges */
  .limit-row {
    display: flex; gap: 6px; margin-bottom: 12px;
  }

  .limit-badge {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.6rem; padding: 2px 8px;
    border-radius: 4px; letter-spacing: 1px;
  }

  .badge-min {
    border: 1px solid rgba(255, 107, 53, 0.5);
    color: rgba(255, 107, 53, 0.8);
    background: rgba(255, 107, 53, 0.06);
  }

  .badge-max {
    border: 1px solid rgba(0, 212, 255, 0.5);
    color: rgba(0, 212, 255, 0.8);
    background: rgba(0, 212, 255, 0.06);
  }

  .badge-home {
    border: 1px solid rgba(34, 197, 94, 0.4);
    color: rgba(34, 197, 94, 0.7);
    background: rgba(34, 197, 94, 0.05);
  }

  input[type=range] {
    -webkit-appearance: none;
    width: 100%; height: 6px;
    border-radius: 3px; background: var(--border);
    outline: none; cursor: pointer; margin-bottom: 5px;
  }

  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 20px; height: 20px; border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 10px rgba(0,212,255,0.6);
    cursor: pointer; transition: transform 0.15s, box-shadow 0.15s;
  }

  input[type=range]::-webkit-slider-thumb:hover {
    transform: scale(1.25);
    box-shadow: 0 0 18px rgba(0,212,255,0.9);
  }

  .gripper-card input[type=range]::-webkit-slider-thumb {
    background: var(--accent2);
    box-shadow: 0 0 10px rgba(255,107,53,0.6);
  }

  .gripper-card input[type=range]::-webkit-slider-thumb:hover {
    box-shadow: 0 0 18px rgba(255,107,53,0.9);
  }

  .range-labels {
    display: flex; justify-content: space-between;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.62rem; color: var(--dim); margin-bottom: 12px;
  }

  .btn-row { display: flex; gap: 7px; }

  .btn {
    flex: 1; padding: 7px 0;
    border-radius: 7px; border: 1px solid var(--border);
    background: transparent; color: var(--dim);
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.68rem; letter-spacing: 1px;
    cursor: pointer; transition: all 0.2s;
  }

  .btn:hover {
    border-color: var(--accent); color: var(--accent);
    background: rgba(0,212,255,0.07);
  }

  .actions {
    display: flex; gap: 12px; justify-content: center;
    max-width: 1100px; margin: 0 auto; flex-wrap: wrap;
  }

  .action-btn {
    padding: 12px 28px; border-radius: 10px; border: 1px solid;
    font-family: 'Exo 2', sans-serif; font-weight: 600;
    font-size: 0.9rem; letter-spacing: 2px; text-transform: uppercase;
    cursor: pointer; transition: all 0.25s;
  }

  .btn-home  { border-color: var(--accent);  color: var(--accent);  background: rgba(0,212,255,0.08); }
  .btn-home:hover { background: rgba(0,212,255,0.2); box-shadow: var(--glow); }
  .btn-open  { border-color: #22c55e; color: #22c55e; background: rgba(34,197,94,0.08); }
  .btn-open:hover  { background: rgba(34,197,94,0.2); }
  .btn-close { border-color: var(--accent2); color: var(--accent2); background: rgba(255,107,53,0.08); }
  .btn-close:hover { background: rgba(255,107,53,0.2); }

  .toast {
    position: fixed; bottom: 24px; right: 24px;
    background: var(--surface); border: 1px solid var(--accent);
    color: var(--accent); font-family: 'Share Tech Mono', monospace;
    font-size: 0.78rem; padding: 10px 18px; border-radius: 8px;
    box-shadow: var(--glow); opacity: 0; transform: translateY(12px);
    transition: all 0.3s; pointer-events: none;
  }

  .toast.show { opacity: 1; transform: translateY(0); }

  #loading {
    text-align: center; padding: 60px;
    font-family: 'Share Tech Mono', monospace;
    color: var(--dim); letter-spacing: 3px;
  }
</style>
</head>
<body>

<header>
  <div class="logo-line">
    <span class="arm-icon">🦾</span>
    <h1>Arm Control</h1>
  </div>
  <div class="subtitle">ESP32 · PCA9685 · 5-DOF · Hard Limits Active</div>
  <div class="status-bar">
    <div class="status-dot"></div>
    <span>CONNECTED &mdash; LIVE CONTROL</span>
  </div>
</header>

<div class="grid" id="servoGrid">
  <div id="loading">LOADING CONFIG...</div>
</div>

<div class="actions">
  <button class="action-btn btn-home"  onclick="homeAll()">⌂ Home All</button>
  <button class="action-btn btn-open"  onclick="setGripper('min')">◇ Open Gripper</button>
  <button class="action-btn btn-close" onclick="setGripper('max')">◆ Close Gripper</button>
</div>

<div class="toast" id="toast"></div>

<script>
let joints = [];

// Fetch servo config from firmware — includes hard limits.
// Sliders are built after this so min/max always match firmware.
fetch('/config')
  .then(r => r.json())
  .then(data => {
    joints = data.servos.map(s => ({
      id:        s.id,
      name:      s.name,
      icon:      s.icon,
      sub:       s.sub,
      min:       s.min,
      max:       s.max,
      home:      s.home,
      val:       s.current,
      isGripper: (s.id === data.servos.length - 1)
    }));
    buildUI();
  })
  .catch(() => {
    document.getElementById('loading').textContent = '⚠ Failed to load config from firmware';
  });

function buildUI() {
  const grid = document.getElementById('servoGrid');
  grid.innerHTML = '';
  joints.forEach(j => {
    const card = document.createElement('div');
    card.className = `card ${j.isGripper ? 'gripper-card' : ''}`;
    card.innerHTML = `
      <div class="card-header">
        <div class="joint-label">
          <div class="joint-icon">${j.icon}</div>
          <div>
            <div class="joint-name">${j.name}</div>
            <div class="joint-sub">${j.sub}</div>
          </div>
        </div>
        <div class="angle-display" id="disp${j.id}">${j.val}°</div>
      </div>
      <div class="limit-row">
        <span class="limit-badge badge-min">MIN ${j.min}°</span>
        <span class="limit-badge badge-max">MAX ${j.max}°</span>
        <span class="limit-badge badge-home">HOME ${j.home}°</span>
      </div>
      <input type="range" id="sl${j.id}" min="${j.min}" max="${j.max}" value="${j.val}"
        oninput="onSlide(${j.id}, this.value)"
        onchange="sendAngle(${j.id}, this.value)">
      <div class="range-labels"><span>${j.min}°</span><span>${j.max}°</span></div>
      <div class="btn-row">
        <button class="btn" onclick="nudge(${j.id}, -10)">−10°</button>
        <button class="btn" onclick="nudge(${j.id},  -1)">− 1°</button>
        <button class="btn" onclick="centerJoint(${j.id})">CTR</button>
        <button class="btn" onclick="nudge(${j.id},  +1)">+ 1°</button>
        <button class="btn" onclick="nudge(${j.id}, +10)">+10°</button>
      </div>
    `;
    grid.appendChild(card);
  });
}

function onSlide(id, val) {
  document.getElementById(`disp${id}`).textContent = val + '°';
}

function sendAngle(id, val) {
  val = parseInt(val);
  const j = joints[id];
  // Client-side clamp (belt + suspenders — firmware also clamps)
  val = Math.min(j.max, Math.max(j.min, val));
  j.val = val;
  fetch(`/set?ch=${id}&angle=${val}`)
    .then(r => r.text())
    .then(() => showToast(`${j.name} → ${val}°`))
    .catch(() => showToast('⚠ Connection error', true));
}

function nudge(id, delta) {
  const j = joints[id];
  const newVal = Math.min(j.max, Math.max(j.min, j.val + delta));
  document.getElementById(`sl${id}`).value = newVal;
  onSlide(id, newVal);
  sendAngle(id, newVal);
}

function centerJoint(id) {
  const j = joints[id];
  const mid = Math.floor((j.min + j.max) / 2);
  document.getElementById(`sl${id}`).value = mid;
  onSlide(id, mid);
  sendAngle(id, mid);
}

function homeAll() {
  fetch('/home').then(() => {
    joints.forEach(j => {
      document.getElementById(`sl${j.id}`).value = j.home;
      onSlide(j.id, j.home);
      j.val = j.home;
    });
    showToast('All joints → Home');
  });
}

function setGripper(mode) {
  const g = joints[joints.length - 1];
  const angle = (mode === 'min') ? g.min : g.max;
  document.getElementById(`sl${g.id}`).value = angle;
  onSlide(g.id, angle);
  sendAngle(g.id, angle);
}

let toastTimer;
function showToast(msg, isErr = false) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.borderColor = isErr ? '#ef4444' : 'var(--accent)';
  t.style.color       = isErr ? '#ef4444' : 'var(--accent)';
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 1800);
}
</script>
</body>
</html>
)=====";

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // PCA9685 init
  pca.begin();
  pca.setOscillatorFrequency(OSC_FREQ);
  pca.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

  // Write explicit 0° to all servos — clean valid pulse, prevents jitter
  Serial.println("\n[INIT] Settling servos at 0°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeServoNow(i, 0);
    targetAngles[i] = 0;
  }
  delay(300);

  // Slow blocking sweep from 0° to startup positions
  Serial.println("[INIT] Sweeping to startup angles (slow)...");
  sweepBlocking(startupAngles, SERVO_SPEED_STARTUP);
  Serial.println("[INIT] All servos at startup position. Ready.");

  // WiFi
  Serial.printf("\nConnecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // ── HTTP Routes ───────────────────────────────────────────

  // Web dashboard
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML_PAGE);
  });

  // Servo config + hard limits (fetched by web dashboard on load)
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildConfigJson());
  });

  // /set?ch=0&angle=90 — queue a smooth move (hard limits enforced)
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("ch") && req->hasParam("angle")) {
      int ch    = req->getParam("ch")->value().toInt();
      int angle = req->getParam("angle")->value().toInt();
      if (ch >= 0 && ch < NUM_SERVOS) {
        int clamped = clampAngle(ch, angle);
        setServoAngle(ch, angle);  // internally calls clampAngle
        Serial.printf("[SET] %s req=%d° → clamped=%d°\n",
                      servos[ch].name, angle, clamped);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // /home — move all servos to homeAngle (smooth)
  server.on("/home", HTTP_GET, [](AsyncWebServerRequest* req) {
    moveAllHome();
    Serial.println("[HOME] All servos → home");
    req->send(200, "text/plain", "OK");
  });

  // /angles — current positions as JSON
  server.on("/angles", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json = "{";
    for (int i = 0; i < NUM_SERVOS; i++) {
      json += "\"" + String(servos[i].name) + "\":" + String(currentAngles[i]);
      if (i < NUM_SERVOS - 1) json += ",";
    }
    json += "}";
    req->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started.");

  lastStepMs = millis();
}

void loop() {
  updateServos();   // non-blocking smooth sweep toward target angles
}
