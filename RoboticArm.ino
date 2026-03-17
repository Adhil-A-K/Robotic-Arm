/*
  ============================================================
  Robotic Arm Controller — ESP32 + PCA9685 + Web Dashboard
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

// Servo pulse limits (microseconds → tune per your servos)
#define SERVO_MIN_US  500    // ~0°
#define SERVO_MAX_US  2500   // ~180°
#define OSC_FREQ      25000000  // PCA9685 oscillator frequency

// Servo channel assignments
#define CH_BASE      0
#define CH_SHOULDER  1
#define CH_ELBOW     2
#define CH_WRIST     3
#define CH_GRIPPER   4

// Angle limits per joint (degrees)
struct ServoConfig {
  const char* name;
  int channel;
  int minAngle;
  int maxAngle;
  int homeAngle;
};

ServoConfig servos[] = {
  { "Base",     CH_BASE,     0, 180,  90 },
  { "Shoulder", CH_SHOULDER, 0, 180,  90 },
  { "Elbow",    CH_ELBOW,    0, 180,  90 },
  { "Wrist",    CH_WRIST,    0, 180,  90 },
  { "Gripper",  CH_GRIPPER,  0, 90,   45 },
};

// ── Smooth Motion ─────────────────────────────────────────────
#define PWM_FREQ_HZ   50

// Startup sweep speed (degrees per second) — slow and gentle
#define SERVO_SPEED_STARTUP  30   // °/s  ← change to taste (lower = slower)

// Runtime speed for web dashboard commands
#define SERVO_SPEED           60  // °/s

// NOTE: on power-up, PCA9685 outputs 0 duty cycle → servos physically
// snap to 0°. We initialise currentAngles to 0 to reflect reality,
// then sweep slowly to home during setup().
int currentAngles[5]  = { 0, 0, 0, 0, 0 };
int targetAngles[5]   = { 0, 0, 0, 0, 0 };
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

// Write a raw angle directly to the PCA9685 (no smoothing)
void writeServoNow(int ch, int angle) {
  ServoConfig& s = servos[ch];
  angle = constrain(angle, s.minAngle, s.maxAngle);
  uint16_t tick = angleToPwm(angle, s.minAngle, s.maxAngle);
  pca.setPWM(s.channel, 0, tick);
  currentAngles[ch] = angle;
}

// Queue a target — the loop() sweeper will move there gradually
void setServoAngle(int ch, int angle) {
  ServoConfig& s = servos[ch];
  targetAngles[ch] = constrain(angle, s.minAngle, s.maxAngle);
}

// ── Non-blocking smooth sweep (called from loop()) ────────────
// Uses SERVO_SPEED (runtime speed).
void updateServos() {
  unsigned long now = millis();
  unsigned long dt  = now - lastStepMs;
  if (dt < 20) return;              // update at ~50 Hz
  lastStepMs = now;

  float maxStep = (SERVO_SPEED * dt) / 1000.0f;

  for (int i = 0; i < 5; i++) {
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

// ── Blocking slow sweep used during setup() ───────────────────
// Sweeps all servos from their current position to destAngles[]
// at SERVO_SPEED_STARTUP degrees/second. Blocks until complete.
void sweepBlocking(int destAngles[], int speedDegPerSec) {
  // Set targets
  for (int i = 0; i < 5; i++) {
    targetAngles[i] = constrain(destAngles[i], servos[i].minAngle, servos[i].maxAngle);
  }

  unsigned long stepTime = millis();
  bool allDone = false;

  while (!allDone) {
    unsigned long now = millis();
    unsigned long dt  = now - stepTime;

    if (dt >= 20) {                       // step at ~50 Hz
      stepTime = now;
      allDone  = true;
      float maxStep = (speedDegPerSec * dt) / 1000.0f;
      if (maxStep < 1.0f) maxStep = 1.0f; // always move at least 1°

      for (int i = 0; i < 5; i++) {
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
  for (int i = 0; i < 5; i++) {
    targetAngles[i] = servos[i].homeAngle;
  }
}

// ── HTML Dashboard (stored in flash) ─────────────────────────
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
    --bg:       #0a0c10;
    --surface:  #111520;
    --border:   #1e2a40;
    --accent:   #00d4ff;
    --accent2:  #ff6b35;
    --warn:     #ffcc00;
    --text:     #c8d8f0;
    --dim:      #4a5a78;
    --glow:     0 0 18px rgba(0,212,255,0.35);
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
    position: relative;
  }

  .logo-line {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 14px;
    margin-bottom: 6px;
  }

  .arm-icon {
    font-size: 2rem;
    filter: drop-shadow(0 0 8px var(--accent));
  }

  h1 {
    font-size: 2rem;
    font-weight: 800;
    letter-spacing: 3px;
    text-transform: uppercase;
    background: linear-gradient(90deg, var(--accent), #60efff);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
  }

  .subtitle {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.72rem;
    color: var(--dim);
    letter-spacing: 4px;
    text-transform: uppercase;
  }

  .status-bar {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    margin-top: 12px;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.75rem;
    color: var(--dim);
  }

  .status-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: #22c55e;
    box-shadow: 0 0 8px #22c55e;
    animation: pulse 2s infinite;
  }

  @keyframes pulse {
    0%,100% { opacity:1; } 50% { opacity:0.4; }
  }

  .grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 16px;
    max-width: 1100px;
    margin: 0 auto 24px;
  }

  .card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 22px;
    position: relative;
    overflow: hidden;
    transition: border-color 0.3s, box-shadow 0.3s;
  }

  .card::before {
    content: '';
    position: absolute;
    top: 0; left: 0; right: 0;
    height: 2px;
    background: linear-gradient(90deg, transparent, var(--accent), transparent);
    opacity: 0;
    transition: opacity 0.3s;
  }

  .card:hover {
    border-color: var(--accent);
    box-shadow: var(--glow);
  }
  .card:hover::before { opacity: 1; }

  .card-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    margin-bottom: 18px;
  }

  .joint-label {
    display: flex;
    align-items: center;
    gap: 10px;
  }

  .joint-icon {
    width: 36px; height: 36px;
    border-radius: 8px;
    background: rgba(0,212,255,0.1);
    border: 1px solid rgba(0,212,255,0.25);
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 1.1rem;
  }

  .joint-name {
    font-weight: 600;
    font-size: 1rem;
    letter-spacing: 1px;
    text-transform: uppercase;
    color: #e0eeff;
  }

  .joint-sub {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.65rem;
    color: var(--dim);
    letter-spacing: 1px;
  }

  .angle-display {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.6rem;
    font-weight: bold;
    color: var(--accent);
    text-shadow: 0 0 12px rgba(0,212,255,0.5);
    min-width: 60px;
    text-align: right;
  }

  .range-wrap {
    position: relative;
    margin-bottom: 14px;
  }

  input[type=range] {
    -webkit-appearance: none;
    width: 100%;
    height: 6px;
    border-radius: 3px;
    background: var(--border);
    outline: none;
    cursor: pointer;
  }

  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 20px; height: 20px;
    border-radius: 50%;
    background: var(--accent);
    box-shadow: 0 0 10px rgba(0,212,255,0.6);
    cursor: pointer;
    transition: transform 0.15s, box-shadow 0.15s;
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
    display: flex;
    justify-content: space-between;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.62rem;
    color: var(--dim);
    margin-top: 5px;
  }

  .btn-row {
    display: flex;
    gap: 10px;
    margin-top: 6px;
  }

  .btn {
    flex: 1;
    padding: 7px 0;
    border-radius: 7px;
    border: 1px solid var(--border);
    background: transparent;
    color: var(--dim);
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.7rem;
    letter-spacing: 1px;
    cursor: pointer;
    transition: all 0.2s;
  }

  .btn:hover {
    border-color: var(--accent);
    color: var(--accent);
    background: rgba(0,212,255,0.07);
  }

  .actions {
    display: flex;
    gap: 12px;
    justify-content: center;
    max-width: 1100px;
    margin: 0 auto;
    flex-wrap: wrap;
  }

  .action-btn {
    padding: 12px 28px;
    border-radius: 10px;
    border: 1px solid;
    font-family: 'Exo 2', sans-serif;
    font-weight: 600;
    font-size: 0.9rem;
    letter-spacing: 2px;
    text-transform: uppercase;
    cursor: pointer;
    transition: all 0.25s;
  }

  .btn-home {
    border-color: var(--accent);
    color: var(--accent);
    background: rgba(0,212,255,0.08);
  }
  .btn-home:hover {
    background: rgba(0,212,255,0.2);
    box-shadow: var(--glow);
  }

  .btn-open {
    border-color: #22c55e;
    color: #22c55e;
    background: rgba(34,197,94,0.08);
  }
  .btn-open:hover { background: rgba(34,197,94,0.2); }

  .btn-close {
    border-color: var(--accent2);
    color: var(--accent2);
    background: rgba(255,107,53,0.08);
  }
  .btn-close:hover { background: rgba(255,107,53,0.2); }

  .toast {
    position: fixed;
    bottom: 24px; right: 24px;
    background: var(--surface);
    border: 1px solid var(--accent);
    color: var(--accent);
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.78rem;
    padding: 10px 18px;
    border-radius: 8px;
    box-shadow: var(--glow);
    opacity: 0;
    transform: translateY(12px);
    transition: all 0.3s;
    pointer-events: none;
  }

  .toast.show {
    opacity: 1;
    transform: translateY(0);
  }
</style>
</head>
<body>

<header>
  <div class="logo-line">
    <span class="arm-icon">🦾</span>
    <h1>Arm Control</h1>
  </div>
  <div class="subtitle">ESP32 · PCA9685 · 5-DOF Robotic Arm</div>
  <div class="status-bar">
    <div class="status-dot"></div>
    <span>CONNECTED &mdash; LIVE CONTROL</span>
  </div>
</header>

<div class="grid" id="servoGrid"></div>

<div class="actions">
  <button class="action-btn btn-home" onclick="homeAll()">⌂ Home All</button>
  <button class="action-btn btn-open"  onclick="setGripper(0)">◇ Open Gripper</button>
  <button class="action-btn btn-close" onclick="setGripper(90)">◆ Close Gripper</button>
</div>

<div class="toast" id="toast"></div>

<script>
const joints = [
  { id:0, name:"Base",     sub:"Rotation / Yaw",   icon:"🔄", min:0, max:180, val:90,  cls:""             },
  { id:1, name:"Shoulder", sub:"Joint 1 / Lift",    icon:"💪", min:0, max:180, val:90,  cls:""             },
  { id:2, name:"Elbow",    sub:"Joint 2 / Reach",   icon:"🦾", min:0, max:180, val:90,  cls:""             },
  { id:3, name:"Wrist",    sub:"Joint 3 / Tilt",    icon:"🤚", min:0, max:180, val:90,  cls:""             },
  { id:4, name:"Gripper",  sub:"End Effector",      icon:"✊", min:0, max:90,  val:45,  cls:"gripper-card" },
];

const grid = document.getElementById('servoGrid');

joints.forEach(j => {
  const card = document.createElement('div');
  card.className = `card ${j.cls}`;
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
    <div class="range-wrap">
      <input type="range" id="sl${j.id}" min="${j.min}" max="${j.max}" value="${j.val}"
        oninput="onSlide(${j.id}, this.value)"
        onchange="sendAngle(${j.id}, this.value)">
    </div>
    <div class="range-labels"><span>${j.min}°</span><span>${j.max}°</span></div>
    <div class="btn-row">
      <button class="btn" onclick="nudge(${j.id}, -10)">− 10°</button>
      <button class="btn" onclick="nudge(${j.id}, -1)">− 1°</button>
      <button class="btn" onclick="centerJoint(${j.id})">CTR</button>
      <button class="btn" onclick="nudge(${j.id}, +1)">+ 1°</button>
      <button class="btn" onclick="nudge(${j.id}, +10)">+ 10°</button>
    </div>
  `;
  grid.appendChild(card);
});

function onSlide(id, val) {
  document.getElementById(`disp${id}`).textContent = val + '°';
}

function sendAngle(id, val) {
  val = parseInt(val);
  joints[id].val = val;
  fetch(`/set?ch=${id}&angle=${val}`)
    .then(r => r.text())
    .then(() => showToast(`${joints[id].name} → ${val}°`))
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
      const mid = Math.floor((j.min + j.max) / 2);
      document.getElementById(`sl${j.id}`).value = mid;
      onSlide(j.id, mid);
      j.val = mid;
    });
    showToast('All joints → Home');
  });
}

function setGripper(angle) {
  document.getElementById('sl4').value = angle;
  onSlide(4, angle);
  sendAngle(4, angle);
}

let toastTimer;
function showToast(msg, isErr=false) {
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
  pca.setPWMFreq(50);   // 50 Hz for standard servos
  delay(10);

  // ── Slow startup sweep ────────────────────────────────────
  //
  // When the PCA9685 first powers on it outputs 0 duty cycle,
  // which commands servos to 0°. The old code called writeServoNow()
  // directly which caused an instant snap. We fix this by:
  //
  //  1. Explicitly write 0° to every servo so the PCA9685 is
  //     sending a valid 0° pulse (prevents undefined jitter).
  //  2. Acknowledge that currentAngles = 0° (matches physics).
  //  3. Slowly sweep from 0° → home position using sweepBlocking().
  //
  Serial.println("\n[INIT] Writing 0° to all servos (settling at rest)...");
  for (int i = 0; i < 5; i++) {
    writeServoNow(i, 0);         // explicit valid 0° pulse
    targetAngles[i] = 0;
    // currentAngles[i] is already 0 (initialised at top)
  }
  delay(300);  // short settle before we start moving

  Serial.println("[INIT] Slow sweep: 0° → Home position...");
  int homePos[5];
  for (int i = 0; i < 5; i++) homePos[i] = servos[i].homeAngle;
  sweepBlocking(homePos, SERVO_SPEED_STARTUP);

  Serial.println("[INIT] All servos at home. Starting WiFi...");

  // ── WiFi ──────────────────────────────────────────────────
  Serial.printf("Connecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // ── Routes ────────────────────────────────────────────────
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML_PAGE);
  });

  // /set?ch=0&angle=90
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("ch") && req->hasParam("angle")) {
      int ch    = req->getParam("ch")->value().toInt();
      int angle = req->getParam("angle")->value().toInt();
      if (ch >= 0 && ch < 5) {
        setServoAngle(ch, angle);
        Serial.printf("[SET] %s → %d°\n", servos[ch].name, angle);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // /home — move all to home (smooth, via loop())
  server.on("/home", HTTP_GET, [](AsyncWebServerRequest* req) {
    moveAllHome();
    Serial.println("[HOME] All servos homed");
    req->send(200, "text/plain", "OK");
  });

  // /angles — return current angles as JSON
  server.on("/angles", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json = "{";
    for (int i = 0; i < 5; i++) {
      json += "\"" + String(servos[i].name) + "\":" + String(currentAngles[i]);
      if (i < 4) json += ",";
    }
    json += "}";
    req->send(200, "application/json", json);
  });

  server.begin();
  Serial.println("Web server started.");

  // Sync lastStepMs so the first updateServos() call starts cleanly
  lastStepMs = millis();
}

void loop() {
  updateServos();   // non-blocking smooth sweep toward target angles
}
