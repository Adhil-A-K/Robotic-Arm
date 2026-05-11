/*
  ============================================================
  Robotic Arm Calibration Tool — ESP32 + PCA9685
  v1.0 — Per-servo control + preset sequencing
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
#include <ArduinoJson.h>

// ── Network (STA only for calibration dashboard) ────────────
static const char *STA_SSID      = "Motridox";
static const char *STA_PASSWORD  = "Bassim@8371";

// ── PCA9685 ───────────────────────────────────────────────────
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500
#define OSC_FREQ      25000000
#define PWM_FREQ_HZ   50

// ── Servo Channel Assignments ─────────────────────────────────
#define CH_BASE      0
#define CH_SHOULDER  1
#define CH_ELBOW     2
#define CH_WRIST     3
#define CH_GRIPPER   4
#define NUM_SERVOS   5

// ── Servo Config (with per-servo speed) ──────────────────────
struct ServoConfig {
  const char* name;
  const char* icon;
  const char* sub;
  int  channel;
  int  minAngle;
  int  maxAngle;
  int  homeAngle;
  int  speed;      // °/s
  int  easeMinMs;  // minimum move time ms
};

ServoConfig servos[NUM_SERVOS] = {
  //   name         icon   subtitle               ch           min  max  home  speed  easeMin
  { "Base",     "🔄", "Rotation / Yaw",      CH_BASE,       0, 180,  95,    60,    120 },
  { "Shoulder", "💪", "Joint 1 / Lift",      CH_SHOULDER,   0, 180, 180,    60,    120 },
  { "Elbow",    "🦾", "Joint 2 / Reach",     CH_ELBOW,      0, 180,   0,    60,    120 },
  { "Wrist",    "🤚", "Joint 3 / Tilt",      CH_WRIST,     30, 180,  55,    60,    120 },
  { "Gripper",  "✊", "End Effector",        CH_GRIPPER,    0, 180, 110,    60,    120 },
};

// ── Runtime State ─────────────────────────────────────────────
int currentAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
int targetAngles[NUM_SERVOS]  = { 0, 0, 0, 0, 0 };
int motionStartAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
unsigned long motionStartMs[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
unsigned long motionDurationMs[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
unsigned long lastStepMs = 0;

// ── Preset Sequencing ──────────────────────────────────────────
#define MAX_PRESETS 20
struct Preset {
  String name;
  int angles[NUM_SERVOS];
};
Preset presets[MAX_PRESETS];
int presetCount = 0;
bool sequenceRunning = false;
int currentPresetIndex = 0;
unsigned long sequenceLastMoveMs = 0;
bool sequenceAtPreset = false;
// Sequence execution delay (ms) — adjustable via UI
int sequenceDelayMs = 1000;

// ── Web Server ────────────────────────────────────────────────
AsyncWebServer server(80);

// ── Helper: angle → PCA9685 tick ─────────────────────────────
uint16_t angleToPwm(int angle, int minAngle, int maxAngle) {
  angle = constrain(angle, minAngle, maxAngle);
  float fraction   = (float)(angle - minAngle) / (maxAngle - minAngle);
  float us         = SERVO_MIN_US + fraction * (SERVO_MAX_US - SERVO_MIN_US);
  float ticksPerUs = (4096.0f * PWM_FREQ_HZ) / 1000000.0f;
  return (uint16_t)(us * ticksPerUs);
}

int clampAngle(int ch, int angle) {
  return constrain(angle, servos[ch].minAngle, servos[ch].maxAngle);
}

String jsonEscape(const String& input) {
  String out;
  out.reserve(input.length() + 8);

  for (size_t i = 0; i < input.length(); i++) {
    char c = input[i];
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        out += c;
        break;
    }
  }

  return out;
}

void writeServoNow(int ch, int angle) {
  angle = clampAngle(ch, angle);
  uint16_t tick = angleToPwm(angle, servos[ch].minAngle, servos[ch].maxAngle);
  pca.setPWM(servos[ch].channel, 0, tick);
  currentAngles[ch] = angle;
}

void setServoAngle(int ch, int angle) {
  int clamped = clampAngle(ch, angle);
  if (clamped == targetAngles[ch]) return;

  int start = currentAngles[ch];
  targetAngles[ch] = clamped;
  motionStartAngles[ch] = start;
  motionStartMs[ch] = millis();

  int travel = abs(clamped - start);
  if (travel <= 1) {
    motionDurationMs[ch] = 0;
    return;
  }

  unsigned long duration = (unsigned long)((1000.0f * travel) / servos[ch].speed);
  if (duration < (unsigned long)servos[ch].easeMinMs) 
    duration = servos[ch].easeMinMs;
  motionDurationMs[ch] = duration;
}

void setAllAngles(int angles[NUM_SERVOS]) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, angles[i]);
  }
}

bool allAtTarget() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (abs(currentAngles[i] - targetAngles[i]) > 2) return false;
  }
  return true;
}

// ── Non-blocking eased motion (per-servo speed) ──────────────
void updateServos() {
  unsigned long now = millis();
  unsigned long dt  = now - lastStepMs;
  if (dt < 20) return;
  lastStepMs = now;

  for (int i = 0; i < NUM_SERVOS; i++) {
    int cur = currentAngles[i];
    int tgt = targetAngles[i];
    if (cur == tgt) continue;

    unsigned long dur = motionDurationMs[i];
    if (dur == 0) {
      writeServoNow(i, tgt);
      continue;
    }

    unsigned long elapsed = now - motionStartMs[i];
    if (elapsed >= dur) {
      writeServoNow(i, tgt);
      motionDurationMs[i] = 0;
      continue;
    }

    float t = (float)elapsed / (float)dur;
    float eased = t * t * (3.0f - 2.0f * t);

    int start = motionStartAngles[i];
    float interp = start + (tgt - start) * eased;
    int next = (int)(interp + (interp >= 0 ? 0.5f : -0.5f));
    next = clampAngle(i, next);

    if (next == cur) next += (tgt > cur) ? 1 : -1;
    next = clampAngle(i, next);

    writeServoNow(i, next);
  }
}

// ── Preset Management ────────────────────────────────────────
void savePreset(String name, int angles[NUM_SERVOS]) {
  if (presetCount >= MAX_PRESETS) return;
  
  presets[presetCount].name = name;
  for (int i = 0; i < NUM_SERVOS; i++) {
    presets[presetCount].angles[i] = angles[i];
  }
  presetCount++;
}

void overwritePreset(int index, String name, int angles[NUM_SERVOS]) {
  if (index < 0 || index >= presetCount) return;
  presets[index].name = name;
  for (int i = 0; i < NUM_SERVOS; i++) {
    presets[index].angles[i] = angles[i];
  }
}

void deletePreset(int index) {
  if (index < 0 || index >= presetCount) return;
  for (int i = index; i < presetCount - 1; i++) {
    presets[i] = presets[i + 1];
  }
  presetCount--;
}

// ── Sequence Execution ────────────────────────────────────────
void startSequence() {
  if (presetCount == 0 || sequenceRunning) return;
  sequenceRunning = true;
  currentPresetIndex = 0;
  sequenceAtPreset = false;
  setAllAngles(presets[0].angles);
}

void runSequence() {
  if (!sequenceRunning) return;

  if (!allAtTarget()) {
    sequenceAtPreset = false;
    return;
  }

  unsigned long now = millis();
  if (!sequenceAtPreset) {
    sequenceAtPreset = true;
    sequenceLastMoveMs = now;
    return;
  }

  if (now - sequenceLastMoveMs < (unsigned long)sequenceDelayMs) return;

  currentPresetIndex++;
  if (currentPresetIndex >= presetCount) {
    sequenceRunning = false;
    currentPresetIndex = 0;
    sequenceAtPreset = false;
    return;
  }

  setAllAngles(presets[currentPresetIndex].angles);
  sequenceAtPreset = false;
}

// ── JSON builders ────────────────────────────────────────────
String buildConfigJson() {
  String json = "{\"servos\":[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += "{";
    json += "\"id\":"      + String(i)                   + ",";
    json += "\"name\":\""  + String(servos[i].name)      + "\",";
    json += "\"icon\":\""  + String(servos[i].icon)      + "\",";
    json += "\"sub\":\""   + String(servos[i].sub)       + "\",";
    json += "\"min\":"     + String(servos[i].minAngle)  + ",";
    json += "\"max\":"     + String(servos[i].maxAngle)  + ",";
    json += "\"home\":"    + String(servos[i].homeAngle) + ",";
    json += "\"speed\":"    + String(servos[i].speed)      + ",";
    json += "\"easeMin\":"  + String(servos[i].easeMinMs)  + ",";
    json += "\"current\":" + String(currentAngles[i]);
    json += "}";
    if (i < NUM_SERVOS - 1) json += ",";
  }

  json += "],\"presets\":[";
  for (int p = 0; p < presetCount; p++) {
    json += "{\"id\":" + String(p) + ",\"name\":\"" + jsonEscape(presets[p].name) + "\",\"angles\":[";
    for (int i = 0; i < NUM_SERVOS; i++) {
      json += String(presets[p].angles[i]);
      if (i < NUM_SERVOS - 1) json += ",";
    }
    json += "]}";
    if (p < presetCount - 1) json += ",";
  }

  json += "],\"sequenceRunning\":" + String(sequenceRunning ? "true" : "false") + ",";
  json += "\"sequenceDelayMs\":" + String(sequenceDelayMs) + ",";
  json += "\"currentPresetIndex\":" + String(currentPresetIndex);
  json += "}";
  return json;
}

String buildStatusJson() {
  String json = "{";
  json += "\"busy\":" + String(sequenceRunning ? "true" : "false") + ",";
  json += "\"current_preset\":" + String(currentPresetIndex) + ",";
  json += "\"angles\":[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += String(currentAngles[i]);
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "]}";
  return json;
}

// ── HTML Dashboard (reuses RoboticArm.ino style) ─────────────
const char HTML_PAGE[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Calibration Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Exo+2:wght@300;600;800&display=swap');

  :root {
    --bg:      #0a0c10;
    --surface: #111520;
    --border:  #1e2a40;
    --accent:  #00d4ff;
    --accent2: #ff6b35;
    --green:   #22c55e;
    --yellow:  #f59e0b;
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
    text-align: center; margin-bottom: 32px;
    padding-bottom: 20px; border-bottom: 1px solid var(--border);
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

  .section-title {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.7rem; letter-spacing: 4px;
    text-transform: uppercase; color: var(--dim);
    margin: 28px auto 14px;
    max-width: 1100px;
    padding-bottom: 6px;
    border-bottom: 1px solid var(--border);
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
    gap: 16px; 
    max-width: 1100px; 
    margin: 0 auto 24px;
    min-height: 200px;
  }
  
  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px; 
    padding: 22px;
    position: relative;
    overflow: hidden;
    transition: border-color 0.3s, box-shadow 0.3s;
    min-height: 300px;
  }
  .card::before {
    content: ''; position: absolute;
    top: 0; left: 0; right: 0; height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0; transition: opacity 0.3s;
  }
  .card:hover { border-color: var(--accent); box-shadow: var(--glow); }
  .card:hover::before { opacity: 1; }

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
  .joint-name { font-weight: 600; font-size: 1rem; letter-spacing: 1px; text-transform: uppercase; color: #e0eeff; }
  .joint-sub { font-family: 'Share Tech Mono', monospace; font-size: 0.65rem; color: var(--dim); letter-spacing: 1px; }
  .angle-display {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.6rem; font-weight: bold;
    color: var(--accent);
    text-shadow: 0 0 12px rgba(0,212,255,0.5);
    min-width: 60px; text-align: right;
  }

  .input-row {
    display: flex; gap: 8px; margin-bottom: 12px;
    align-items: center;
  }
  .input-row label {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.65rem; color: var(--dim);
    text-transform: uppercase; letter-spacing: 1px;
    min-width: 60px;
  }
  input[type="number"] {
    background: rgba(0,212,255,0.05); border: 1px solid rgba(0,212,255,0.2);
    border-radius: 6px; padding: 4px 8px;
    font-family: 'Share Tech Mono', monospace; font-size: 0.75rem;
    color: var(--accent); text-align: center;
    width: 80px;
  }
  input[type="number"]:focus {
    outline: none; border-color: var(--accent);
    background: rgba(0,212,255,0.1);
  }

  input[type=range] {
    -webkit-appearance: none; width: 100%; height: 6px;
    border-radius: 3px; background: var(--border);
    outline: none; cursor: pointer; margin-bottom: 5px;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 20px; height: 20px; border-radius: 50%;
    background: var(--accent); box-shadow: 0 0 10px rgba(0,212,255,0.6);
    cursor: pointer; transition: transform 0.15s, box-shadow 0.15s;
  }
  input[type=range]::-webkit-slider-thumb:hover { transform: scale(1.25); box-shadow: 0 0 18px rgba(0,212,255,0.9); }

  .range-labels {
    display: flex; justify-content: space-between;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.62rem; color: var(--dim); margin-bottom: 12px;
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
  .btn-save { border-color: var(--green); color: var(--green); background: rgba(34,197,94,0.08); }
  .btn-save:hover { background: rgba(34,197,94,0.2); }
  .btn-run { border-color: var(--accent); color: var(--accent); background: rgba(0,212,255,0.08); }
  .btn-run:hover { background: rgba(0,212,255,0.2); box-shadow: var(--glow); }
  .btn-run:disabled {
    opacity: 0.45;
    cursor: not-allowed;
    box-shadow: none;
  }

  /* Preset Table */
  .preset-table-container {
    max-width: 1100px; margin: 0 auto 24px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 14px; padding: 24px;
  }

  .preset-controls {
    display: flex; gap: 12px; margin-bottom: 16px;
    align-items: center;
    flex-wrap: wrap;
  }
  
  #presetName {
    flex: 1; min-width: 200px;
    padding: 8px 12px;
    background: rgba(0,212,255,0.05); border: 1px solid rgba(0,212,255,0.2);
    border-radius: 8px; color: var(--accent);
    font-family: 'Share Tech Mono', monospace; font-size: 0.8rem;
  }
  
  .delay-control {
    display: flex; align-items: center; gap: 8px;
    font-family: 'Share Tech Mono', monospace; font-size: 0.7rem;
    color: var(--dim);
  }
  
  #delaySlider {
    width: 120px;
  }
  
  .delay-value {
    min-width: 40px; text-align: right;
    color: var(--accent);
  }
  #presetName:focus { outline: none; border-color: var(--accent); }

  .table-wrapper {
    overflow-x: auto;
  }
  table {
    width: 100%; border-collapse: collapse;
    font-family: 'Share Tech Mono', monospace; font-size: 0.7rem;
  }
  th {
    padding: 8px 12px; text-align: left;
    color: var(--dim); text-transform: uppercase; letter-spacing: 1px;
    border-bottom: 1px solid var(--border);
  }
  td {
    padding: 8px 12px; border-bottom: 1px solid rgba(255,255,255,0.05);
  }
  .preset-running {
    background: rgba(34,197,94,0.15);
    border-left: 3px solid var(--green);
  }
  
  .preset-id {
    font-family: 'Share Tech Mono', monospace;
    font-weight: bold;
    transition: color 0.2s;
  }
  
  .preset-id.running {
    color: var(--green);
  }
  
  .preset-id.idle {
    color: var(--dim);
  }

  .btn-table {
    background: transparent;
    border: 1px solid var(--border);
    border-radius: 6px;
    color: var(--accent);
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.68rem;
    padding: 4px 8px;
    margin-right: 6px;
    cursor: pointer;
    transition: all 0.2s;
  }

  .btn-table:hover {
    border-color: var(--accent);
    background: rgba(0,212,255,0.12);
  }
  .btn-delete:hover { border-color: #ef4444; color: #ef4444; }

  .toast {
    position: fixed; bottom: 24px; right: 24px;
    background: var(--surface); border: 1px solid var(--accent);
    color: var(--accent); font-family: 'Share Tech Mono', monospace;
    font-size: 0.78rem; padding: 10px 18px; border-radius: 8px;
    box-shadow: var(--glow); opacity: 0; transform: translateY(12px);
    transition: all 0.3s; pointer-events: none; z-index: 100;
  }
  .toast.show { opacity: 1; transform: translateY(0); }
  .toast.error { border-color: #ef4444; color: #ef4444; }

  #loading { text-align: center; padding: 60px; font-family: 'Share Tech Mono', monospace; color: var(--dim); letter-spacing: 3px; }
</style>
</head>
<body>

<header>
  <div class="logo-line">
    <span class="arm-icon">🔧</span>
    <h1>Calibration Tool</h1>
  </div>
  <div class="subtitle">ESP32 · PCA9685 · Per-Servo Control</div>
</header>

<!-- ── Servo Controls ───────────────────────────────────────── -->
<div class="section-title">🎛 Per-Servo Control</div>
<div class="grid" id="servoGrid"></div>

<!-- ── Preset Sequencing ───────────────────────────────────── -->
<div class="section-title">📍 Preset Sequencing</div>
<div class="preset-table-container">
  <div class="preset-controls">
    <input type="text" id="presetName" placeholder="Enter preset name...">
    <button class="action-btn btn-save" onclick="saveCurrentPreset()">💾 Save Preset</button>
    <button class="action-btn btn-run" id="btnRun" onclick="runSequence()">▶ Run Sequence</button>
    <div class="delay-control">
      <label>DELAY</label>
      <input type="range" id="delaySlider" min="200" max="2000" step="100" value="1000"
             oninput="updateDelayValue(this.value)" onchange="setSequenceDelay(this.value)">
      <span class="delay-value" id="delayValue">1000</span>ms
    </div>
  </div>
  <div class="table-wrapper">
    <table id="presetTable">
      <thead>
        <tr>
          <th style="width:60px">ID</th>
          <th>Name</th>
          <th style="width:300px">Angles (Base, Shoulder, Elbow, Wrist, Gripper)</th>
          <th style="width:120px">Actions</th>
        </tr>
      </thead>
      <tbody id="presetTableBody">
        <tr><td colspan="4" id="loading">LOADING PRESETS...</td></tr>
      </tbody>
    </table>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
let joints = [];
let presets = [];
let seqPoll = null;
let toastTimer = null;

function fetchJson(url) {
  return fetch(url).then((r) => {
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    return r.json();
  });
}

function formatAngles(angles) {
  const names = ['Base', 'Shoulder', 'Elbow', 'Wrist', 'Gripper'];
  return angles.map((a, i) => `${names[i]}:${a}°`).join(' | ');
}

function escapeHtml(text) {
  return String(text)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#39;');
}

function setSeqRunning(running) {
  const btn = document.getElementById('btnRun');
  if (btn) btn.disabled = running;
}

function updateRunningRow(currentIdx, running) {
  const rows = document.querySelectorAll('#presetTableBody tr[data-preset-idx]');
  rows.forEach((row) => {
    row.classList.remove('preset-running');
    const idCell = row.querySelector('.preset-id');
    if (idCell) {
      idCell.classList.remove('running');
      idCell.classList.add('idle');
    }
  });

  if (!running) return;
  const active = document.querySelector(`#presetTableBody tr[data-preset-idx="${currentIdx}"]`);
  if (!active) return;
  active.classList.add('preset-running');
  const idCell = active.querySelector('.preset-id');
  if (idCell) {
    idCell.classList.remove('idle');
    idCell.classList.add('running');
  }
}

function updateDelayValue(val) {
  document.getElementById('delayValue').textContent = String(val);
}

function setDelayControl(ms) {
  const slider = document.getElementById('delaySlider');
  if (!slider) return;
  slider.value = ms;
  updateDelayValue(ms);
}

function buildServoUI() {
  const grid = document.getElementById('servoGrid');
  if (!grid) return;
  grid.innerHTML = '';

  if (!joints.length) {
    grid.innerHTML = '<div id="loading">No joints available</div>';
    return;
  }

  joints.forEach((j) => {
    const card = document.createElement('div');
    card.className = 'card';
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

      <div class="input-row">
        <label>MIN</label>
        <input type="number" id="min${j.id}" value="${j.min}"
               onchange="updateServoLimits(${j.id})" min="0" max="180">
        <label>MAX</label>
        <input type="number" id="max${j.id}" value="${j.max}"
               onchange="updateServoLimits(${j.id})" min="0" max="180">
      </div>

      <div class="input-row">
        <label>SPEED</label>
        <input type="number" id="speed${j.id}" value="${j.speed}"
               onchange="updateServoSpeed(${j.id})" min="10" max="200">
        <label>EASE</label>
        <input type="number" id="ease${j.id}" value="${j.easeMin}"
               onchange="updateServoEase(${j.id})" min="50" max="500">
      </div>

      <input type="range" id="sl${j.id}" min="${j.min}" max="${j.max}" value="${j.val}"
        oninput="onSlide(${j.id}, this.value)"
        onchange="sendAngle(${j.id}, this.value)">
      <div class="range-labels"><span>${j.min}°</span><span>${j.max}°</span></div>
    `;
    grid.appendChild(card);
  });
}

function buildPresetTable(currentIdx = -1, running = false) {
  const tbody = document.getElementById('presetTableBody');
  if (!tbody) return;

  tbody.innerHTML = '';
  if (!presets.length) {
    tbody.innerHTML = '<tr><td colspan="4" style="text-align:center;color:var(--dim)">No presets saved yet</td></tr>';
    return;
  }

  presets.forEach((p, idx) => {
    const row = document.createElement('tr');
    row.dataset.presetIdx = String(idx);

    const isRunning = running && idx === currentIdx;
    const idClass = `preset-id ${isRunning ? 'running' : 'idle'}`;
    if (isRunning) row.classList.add('preset-running');

    row.innerHTML = `
      <td class="${idClass}">${idx}</td>
      <td>${escapeHtml(p.name)}</td>
      <td>${formatAngles(p.angles)}</td>
      <td>
        <button class="btn-table" onclick="gotoPreset(${idx})">▶ Go To</button>
        <button class="btn-table" onclick="overwritePreset(${idx})">💾 Save</button>
        <button class="btn-table btn-delete" onclick="deletePreset(${idx})">🗑 Delete</button>
      </td>
    `;
    tbody.appendChild(row);
  });
}

function loadConfigAndRender() {
  fetchJson('/config')
    .then((data) => {
      joints = (data.servos || []).map((s) => ({
        id: s.id,
        name: s.name,
        icon: s.icon,
        sub: s.sub,
        min: s.min,
        max: s.max,
        home: s.home,
        val: s.current,
        speed: s.speed,
        easeMin: s.easeMin
      }));

      presets = data.presets || [];
      setDelayControl(data.sequenceDelayMs || 1000);
      buildServoUI();
      buildPresetTable(data.currentPresetIndex ?? -1, Boolean(data.sequenceRunning));
      setSeqRunning(Boolean(data.sequenceRunning));

      if (!seqPoll) seqPoll = setInterval(pollStatus, 500);
    })
    .catch(() => {
      const loading = document.getElementById('loading');
      if (loading) loading.textContent = '⚠ Failed to load config';
      showToast('⚠ Failed to load config', true);
    });
}

function updateServoLimits(id) {
  const minInput = document.getElementById(`min${id}`);
  const maxInput = document.getElementById(`max${id}`);
  const slider = document.getElementById(`sl${id}`);
  if (!minInput || !maxInput || !slider) return;

  const min = parseInt(minInput.value, 10);
  const max = parseInt(maxInput.value, 10);

  if (!Number.isInteger(min) || !Number.isInteger(max) || min < 0 || max > 180 || min >= max) {
    showToast('⚠ Invalid range: min < max and 0≤value≤180', true);
    return;
  }

  joints[id].min = min;
  joints[id].max = max;
  slider.min = String(min);
  slider.max = String(max);

  let val = parseInt(slider.value, 10);
  val = Math.min(max, Math.max(min, val));
  slider.value = String(val);
  onSlide(id, val);
  sendAngle(id, val);
  showToast(`${joints[id].name} limits: ${min}°–${max}°`);
}

function updateServoSpeed(id) {
  const input = document.getElementById(`speed${id}`);
  if (!input) return;

  const speed = parseInt(input.value, 10);
  if (!Number.isInteger(speed) || speed < 10 || speed > 200) {
    showToast('⚠ Speed must be 10–200°/s', true);
    return;
  }

  fetch(`/set_speed?id=${id}&speed=${speed}`)
    .then((r) => {
      if (!r.ok) throw new Error('Bad speed');
      joints[id].speed = speed;
      showToast(`${joints[id].name} speed → ${speed}°/s`);
    })
    .catch(() => showToast('⚠ Speed update failed', true));
}

function updateServoEase(id) {
  const input = document.getElementById(`ease${id}`);
  if (!input) return;

  const ease = parseInt(input.value, 10);
  if (!Number.isInteger(ease) || ease < 50 || ease > 500) {
    showToast('⚠ Ease must be 50–500ms', true);
    return;
  }

  fetch(`/set_ease?id=${id}&ease=${ease}`)
    .then((r) => {
      if (!r.ok) throw new Error('Bad ease');
      joints[id].easeMin = ease;
      showToast(`${joints[id].name} ease → ${ease}ms`);
    })
    .catch(() => showToast('⚠ Ease update failed', true));
}

function setSequenceDelay(ms) {
  const delay = parseInt(ms, 10);
  if (!Number.isInteger(delay) || delay < 200 || delay > 2000) {
    showToast('⚠ Delay must be 200–2000ms', true);
    return;
  }

  fetch(`/set_delay?ms=${delay}`)
    .then((r) => {
      if (!r.ok) throw new Error('Delay failed');
      updateDelayValue(delay);
      showToast(`Sequence delay → ${delay}ms`);
    })
    .catch(() => showToast('⚠ Delay update failed', true));
}

function gotoPreset(idx) {
  if (!presets[idx]) return;
  fetch(`/goto_preset?id=${idx}`)
    .then((r) => {
      if (!r.ok) throw new Error('Go To failed');
      showToast(`Moving to preset: ${presets[idx].name}`);
    })
    .catch(() => showToast('⚠ Go To preset failed', true));
}

function saveCurrentPreset() {
  const nameEl = document.getElementById('presetName');
  if (!nameEl) return;
  const name = nameEl.value.trim();

  if (!name) {
    showToast('⚠ Enter a preset name', true);
    return;
  }

  const angles = joints.map((j) => j.val);
  fetch(`/save_preset?name=${encodeURIComponent(name)}&a0=${angles[0]}&a1=${angles[1]}&a2=${angles[2]}&a3=${angles[3]}&a4=${angles[4]}`)
    .then((r) => {
      if (!r.ok) throw new Error('Save failed');
      return r.json();
    })
    .then((data) => {
      presets = data.presets || [];
      buildPresetTable(data.currentPresetIndex ?? -1, Boolean(data.sequenceRunning));
      nameEl.value = '';
      showToast(`Saved: ${name}`);
    })
    .catch(() => showToast('⚠ Save failed', true));
}

function overwritePreset(idx) {
  if (!presets[idx]) return;

  const name = prompt('Enter new name (or keep current):', presets[idx].name);
  if (!name) return;

  const angles = joints.map((j) => j.val);
  fetch(`/overwrite_preset?id=${idx}&name=${encodeURIComponent(name)}&a0=${angles[0]}&a1=${angles[1]}&a2=${angles[2]}&a3=${angles[3]}&a4=${angles[4]}`)
    .then((r) => {
      if (!r.ok) throw new Error('Overwrite failed');
      return r.json();
    })
    .then((data) => {
      presets = data.presets || [];
      buildPresetTable(data.currentPresetIndex ?? -1, Boolean(data.sequenceRunning));
      showToast(`Updated preset ${idx}`);
    })
    .catch(() => showToast('⚠ Overwrite failed', true));
}

function deletePreset(idx) {
  if (!presets[idx]) return;
  if (!confirm(`Delete preset ${idx}: ${presets[idx].name}?`)) return;

  fetch(`/delete_preset?id=${idx}`)
    .then((r) => {
      if (!r.ok) throw new Error('Delete failed');
      return r.json();
    })
    .then((data) => {
      presets = data.presets || [];
      buildPresetTable(data.currentPresetIndex ?? -1, Boolean(data.sequenceRunning));
      showToast(`Deleted preset ${idx}`);
    })
    .catch(() => showToast('⚠ Delete failed', true));
}

function runSequence() {
  fetch('/run_sequence')
    .then((r) => r.text())
    .then((t) => {
      if (t === 'OK') {
        setSeqRunning(true);
        showToast('Sequence started');
      } else {
        showToast(`⚠ ${t}`, true);
      }
      pollStatus();
    })
    .catch(() => showToast('⚠ Connection error', true));
}

function pollStatus() {
  fetchJson('/status')
    .then((s) => {
      const running = Boolean(s.busy);
      setSeqRunning(running);
      updateRunningRow(s.current_preset, running);

      if (Array.isArray(s.angles)) {
        s.angles.forEach((a, i) => {
          const sl = document.getElementById(`sl${i}`);
          const dp = document.getElementById(`disp${i}`);
          if (!sl || !dp) return;

          if (!sl.matches(':active')) {
            sl.value = String(a);
            joints[i].val = a;
            dp.textContent = `${a}°`;
          }
        });
      }
    })
    .catch(() => {});
}

function onSlide(id, val) {
  const angle = parseInt(val, 10);
  joints[id].val = angle;
  const display = document.getElementById(`disp${id}`);
  if (display) display.textContent = `${angle}°`;
}

function sendAngle(id, val) {
  let angle = parseInt(val, 10);
  const joint = joints[id];
  angle = Math.min(joint.max, Math.max(joint.min, angle));
  joint.val = angle;

  fetch(`/set?ch=${id}&angle=${angle}`)
    .then((r) => {
      if (!r.ok) throw new Error('Set failed');
      showToast(`${joint.name} → ${angle}°`);
    })
    .catch(() => showToast('⚠ Connection error', true));
}

function showToast(msg, isErr = false) {
  const t = document.getElementById('toast');
  if (!t) return;

  t.textContent = msg;
  t.className = `toast ${isErr ? 'error' : ''}`;
  t.classList.add('show');

  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 2200);
}

loadConfigAndRender();
</script>
</body>
</html>
)=====";

// ── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pca.begin();
  pca.setOscillatorFrequency(OSC_FREQ);
  pca.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

  // Sync to home angles
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeServoNow(i, servos[i].homeAngle);
    targetAngles[i] = servos[i].homeAngle;
  }
  delay(120);

  // STA WiFi
  Serial.println("\n[NET] Connecting to WiFi...");
  WiFi.begin(STA_SSID, STA_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[NET] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

  // HTTP Routes
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML_PAGE);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildConfigJson());
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildStatusJson());
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("ch") && req->hasParam("angle")) {
      int ch = req->getParam("ch")->value().toInt();
      int angle = req->getParam("angle")->value().toInt();
      if (ch >= 0 && ch < NUM_SERVOS) {
        setServoAngle(ch, angle);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.on("/set_speed", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id") && req->hasParam("speed")) {
      int id = req->getParam("id")->value().toInt();
      int speed = req->getParam("speed")->value().toInt();
      if (id >= 0 && id < NUM_SERVOS && speed >= 10 && speed <= 200) {
        servos[id].speed = speed;
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.on("/set_ease", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id") && req->hasParam("ease")) {
      int id = req->getParam("id")->value().toInt();
      int ease = req->getParam("ease")->value().toInt();
      if (id >= 0 && id < NUM_SERVOS && ease >= 50 && ease <= 500) {
        servos[id].easeMinMs = ease;
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.on("/save_preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("name") && req->hasParam("a0")) {
      String name = req->getParam("name")->value();
      int angles[NUM_SERVOS];
      const char* keys[] = {"a0","a1","a2","a3","a4"};
      bool ok = true;
      for (int i = 0; i < NUM_SERVOS; i++) {
        if (req->hasParam(keys[i])) {
          angles[i] = req->getParam(keys[i])->value().toInt();
        } else { ok = false; break; }
      }
      if (ok) {
        savePreset(name, angles);
        req->send(200, "application/json", buildConfigJson());
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.on("/overwrite_preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id") && req->hasParam("name") && req->hasParam("a0")) {
      int id = req->getParam("id")->value().toInt();
      String name = req->getParam("name")->value();
      int angles[NUM_SERVOS];
      const char* keys[] = {"a0","a1","a2","a3","a4"};
      bool ok = true;
      for (int i = 0; i < NUM_SERVOS; i++) {
        if (req->hasParam(keys[i])) {
          angles[i] = req->getParam(keys[i])->value().toInt();
        } else { ok = false; break; }
      }
      if (ok && id >= 0 && id < presetCount) {
        overwritePreset(id, name, angles);
        req->send(200, "application/json", buildConfigJson());
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.on("/delete_preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id")) {
      int id = req->getParam("id")->value().toInt();
      if (id >= 0 && id < presetCount) {
        deletePreset(id);
        req->send(200, "application/json", buildConfigJson());
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // Add endpoint to set sequence delay
  server.on("/set_delay", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("ms")) {
      int ms = req->getParam("ms")->value().toInt();
      if (ms >= 200 && ms <= 2000) {
        sequenceDelayMs = ms;
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });
  
  // Add endpoint to go to a specific preset
  server.on("/goto_preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id")) {
      int id = req->getParam("id")->value().toInt();
      if (id >= 0 && id < presetCount) {
        setAllAngles(presets[id].angles);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // Add endpoint to run sequence
  server.on("/run_sequence", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (presetCount == 0) {
      req->send(200, "text/plain", "No presets");
      return;
    }
    startSequence();
    req->send(200, "text/plain", "OK");
  });
  
  server.begin();
  Serial.println("Web server started.");
  lastStepMs = millis();
}

void loop() {
  updateServos();
  runSequence();
}