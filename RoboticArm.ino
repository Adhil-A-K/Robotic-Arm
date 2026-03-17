/*
  ============================================================
  Robotic Arm Controller — ESP32 + PCA9685 + Web Dashboard
  v3.0 — Presets + Pick-and-Drop sequences
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

// ── WiFi Credentials ─────────────────────────────────────────
const char* WIFI_SSID = "Motridox";
const char* WIFI_PASS = "Bassim@8371";

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

// ── Servo Config ──────────────────────────────────────────────
struct ServoConfig {
  const char* name;
  const char* icon;
  const char* sub;
  int  channel;
  int  minAngle;
  int  maxAngle;
  int  homeAngle;
};

ServoConfig servos[NUM_SERVOS] = {
  { "Base",     "🔄", "Rotation / Yaw",      CH_BASE,       0, 180,  90 },
  { "Shoulder", "💪", "Joint 1 / Lift",      CH_SHOULDER,   0, 180,  90 },
  { "Elbow",    "🦾", "Joint 2 / Reach",     CH_ELBOW,      0, 180,  90 },
  { "Wrist",    "🤚", "Joint 3 / Tilt",      CH_WRIST,      0, 180,  90 },
  { "Gripper",  "✊", "End Effector",        CH_GRIPPER,    0,  90,  45 },
};

// ── Startup Target Angles ─────────────────────────────────────
//                   Base  Shoulder  Elbow  Wrist  Gripper
int startupAngles[NUM_SERVOS] = { 90, 180, 0, 90, 30 };

// ── Speed Settings ────────────────────────────────────────────
#define SERVO_SPEED_STARTUP  30   // °/s  startup sweep
#define SERVO_SPEED          60   // °/s  runtime

// ── Gripper open/close angles ─────────────────────────────────
#define GRIPPER_OPEN    0    // fully open
#define GRIPPER_CLOSE   85   // fully closed (gripping)

// ╔══════════════════════════════════════════════════════════════╗
// ║                  PRESET LOCATIONS                           ║
// ║                                                             ║
// ║  4 presets: pickup1, pickup2, drop1, drop2                  ║
// ║  Angles: { Base, Shoulder, Elbow, Wrist, Gripper }          ║
// ║                                                             ║
// ║  These are placeholder values — calibrate using the web     ║
// ║  dashboard's "Save as Preset" buttons, then hard-code here. ║
// ╚══════════════════════════════════════════════════════════════╝

struct Preset {
  const char* name;
  const char* label;
  int angles[NUM_SERVOS];   // Base, Shoulder, Elbow, Wrist, Gripper
};

// NOTE: Gripper angle in presets = arm position (gripper open while moving,
//       gripper closes/opens during pick/drop — handled by the sequence).
//       The Gripper value here is the "approach" gripper state (open = 0).
Preset presets[4] = {
  { "pickup1", "Pickup 1",  { 45,  130,  60,  90,  GRIPPER_OPEN } },
  { "pickup2", "Pickup 2",  { 135, 130,  60,  90,  GRIPPER_OPEN } },
  { "drop1",   "Drop 1",    { 45,  100,  80,  90,  GRIPPER_OPEN } },
  { "drop2",   "Drop 2",    { 135, 100,  80,  90,  GRIPPER_OPEN } },
};

// Safe transit height (arm raised safely before moving between positions)
int transitAngles[NUM_SERVOS] = { 90, 160, 20, 90, GRIPPER_OPEN };

// ── Runtime State ─────────────────────────────────────────────
int currentAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
int targetAngles[NUM_SERVOS]  = { 0, 0, 0, 0, 0 };
unsigned long lastStepMs = 0;

// Sequence state machine
bool        seqRunning  = false;
int         seqStep     = 0;
int         seqObj      = 0;   // 0 = object1, 1 = object2
bool        seqWaiting  = false;
unsigned long seqWaitUntil = 0;

// Sequence steps enum
enum SeqStep {
  SEQ_TRANSIT_UP = 0,
  SEQ_GOTO_PICKUP,
  SEQ_CLOSE_GRIPPER,
  SEQ_WAIT_GRIP,
  SEQ_TRANSIT_WITH_OBJECT,
  SEQ_GOTO_DROP,
  SEQ_OPEN_GRIPPER,
  SEQ_WAIT_RELEASE,
  SEQ_TRANSIT_FINAL,
  SEQ_DONE
};

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

void writeServoNow(int ch, int angle) {
  angle = clampAngle(ch, angle);
  uint16_t tick = angleToPwm(angle, servos[ch].minAngle, servos[ch].maxAngle);
  pca.setPWM(servos[ch].channel, 0, tick);
  currentAngles[ch] = angle;
}

void setServoAngle(int ch, int angle) {
  targetAngles[ch] = clampAngle(ch, angle);
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

// ── Non-blocking smooth sweep ─────────────────────────────────
void updateServos() {
  unsigned long now = millis();
  unsigned long dt  = now - lastStepMs;
  if (dt < 20) return;
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

// ── Blocking sweep (setup only) ───────────────────────────────
void sweepBlocking(int destAngles[], int speedDegPerSec) {
  for (int i = 0; i < NUM_SERVOS; i++) targetAngles[i] = clampAngle(i, destAngles[i]);
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
      }
    }
    delay(1);
  }
}

void moveAllHome() {
  for (int i = 0; i < NUM_SERVOS; i++) setServoAngle(i, servos[i].homeAngle);
}

// ── Pick-and-Drop Sequence State Machine ──────────────────────
// Called from loop() — non-blocking step execution.
void runSequence() {
  if (!seqRunning) return;

  // If we're in a timed wait, check it
  if (seqWaiting) {
    if (millis() < seqWaitUntil) return;
    seqWaiting = false;
    seqStep++;
  }

  // If arm is still moving, wait for it
  if (!allAtTarget()) return;

  int pickupIdx = seqObj;       // 0 = pickup1, 1 = pickup2
  int dropIdx   = seqObj + 2;   // 2 = drop1,   3 = drop2

  switch (seqStep) {

    case SEQ_TRANSIT_UP:
      Serial.printf("[SEQ] Step 1 — Raise to transit height\n");
      setAllAngles(transitAngles);
      seqStep++;
      break;

    case SEQ_GOTO_PICKUP:
      Serial.printf("[SEQ] Step 2 — Move to pickup%d\n", pickupIdx + 1);
      setAllAngles(presets[pickupIdx].angles);
      setServoAngle(CH_GRIPPER, GRIPPER_OPEN);
      seqStep++;
      break;

    case SEQ_CLOSE_GRIPPER:
      Serial.printf("[SEQ] Step 3 — Close gripper\n");
      setServoAngle(CH_GRIPPER, GRIPPER_CLOSE);
      seqWaiting   = true;
      seqWaitUntil = millis() + 600;   // 600ms settle
      break;

    case SEQ_WAIT_GRIP:
      // handled above in seqWaiting check → auto-advances
      seqStep++;
      break;

    case SEQ_TRANSIT_WITH_OBJECT:
      Serial.printf("[SEQ] Step 4 — Transit with object (gripper stays closed)\n");
      setAllAngles(transitAngles);
      setServoAngle(CH_GRIPPER, GRIPPER_CLOSE);  // keep closed during transit
      seqStep++;
      break;

    case SEQ_GOTO_DROP:
      Serial.printf("[SEQ] Step 5 — Move to drop%d\n", dropIdx - 1);
      setAllAngles(presets[dropIdx].angles);
      setServoAngle(CH_GRIPPER, GRIPPER_CLOSE);  // keep closed
      seqStep++;
      break;

    case SEQ_OPEN_GRIPPER:
      Serial.printf("[SEQ] Step 6 — Open gripper (drop object)\n");
      setServoAngle(CH_GRIPPER, GRIPPER_OPEN);
      seqWaiting   = true;
      seqWaitUntil = millis() + 500;
      break;

    case SEQ_WAIT_RELEASE:
      seqStep++;
      break;

    case SEQ_TRANSIT_FINAL:
      Serial.printf("[SEQ] Step 7 — Return to transit height\n");
      setAllAngles(transitAngles);
      seqStep++;
      break;

    case SEQ_DONE:
      Serial.printf("[SEQ] Done — Object %d delivered.\n", seqObj + 1);
      seqRunning = false;
      seqStep    = 0;
      break;
  }
}

// ── JSON builders ─────────────────────────────────────────────
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
    json += "\"current\":" + String(currentAngles[i]);
    json += "}";
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "],\"presets\":[";
  for (int p = 0; p < 4; p++) {
    json += "{\"id\":" + String(p) + ",\"name\":\"" + presets[p].name + "\",\"label\":\"" + presets[p].label + "\",\"angles\":[";
    for (int i = 0; i < NUM_SERVOS; i++) {
      json += String(presets[p].angles[i]);
      if (i < NUM_SERVOS - 1) json += ",";
    }
    json += "]}";
    if (p < 3) json += ",";
  }
  json += "],\"seqRunning\":" + String(seqRunning ? "true" : "false") + "}";
  return json;
}

// ── HTML Dashboard ────────────────────────────────────────────
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
  .status-bar {
    display: flex; align-items: center; justify-content: center;
    gap: 8px; margin-top: 12px;
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.75rem; color: var(--dim);
  }
  .status-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: var(--green); box-shadow: 0 0 8px var(--green);
    animation: pulse 2s infinite;
  }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:0.4} }

  /* Section headers */
  .section-title {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.7rem; letter-spacing: 4px;
    text-transform: uppercase; color: var(--dim);
    margin: 28px auto 14px;
    max-width: 1100px;
    padding-bottom: 6px;
    border-bottom: 1px solid var(--border);
  }

  /* Servo cards */
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
  .gripper-card .joint-icon { background: rgba(255,107,53,0.1); border-color: rgba(255,107,53,0.25); }
  .joint-name { font-weight: 600; font-size: 1rem; letter-spacing: 1px; text-transform: uppercase; color: #e0eeff; }
  .joint-sub { font-family: 'Share Tech Mono', monospace; font-size: 0.65rem; color: var(--dim); letter-spacing: 1px; }
  .angle-display {
    font-family: 'Share Tech Mono', monospace;
    font-size: 1.6rem; font-weight: bold;
    color: var(--accent);
    text-shadow: 0 0 12px rgba(0,212,255,0.5);
    min-width: 60px; text-align: right;
  }
  .gripper-card .angle-display { color: var(--accent2); text-shadow: 0 0 12px rgba(255,107,53,0.5); }

  .limit-row { display: flex; gap: 6px; margin-bottom: 12px; }
  .limit-badge {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.6rem; padding: 2px 8px;
    border-radius: 4px; letter-spacing: 1px;
  }
  .badge-min { border: 1px solid rgba(255,107,53,0.5); color: rgba(255,107,53,0.8); background: rgba(255,107,53,0.06); }
  .badge-max { border: 1px solid rgba(0,212,255,0.5);  color: rgba(0,212,255,0.8);  background: rgba(0,212,255,0.06); }
  .badge-home { border: 1px solid rgba(34,197,94,0.4); color: rgba(34,197,94,0.7);  background: rgba(34,197,94,0.05); }

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
  .gripper-card input[type=range]::-webkit-slider-thumb { background: var(--accent2); box-shadow: 0 0 10px rgba(255,107,53,0.6); }
  .gripper-card input[type=range]::-webkit-slider-thumb:hover { box-shadow: 0 0 18px rgba(255,107,53,0.9); }

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
  .btn:hover { border-color: var(--accent); color: var(--accent); background: rgba(0,212,255,0.07); }

  /* Global action buttons */
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
  .btn-open  { border-color: var(--green); color: var(--green); background: rgba(34,197,94,0.08); }
  .btn-open:hover  { background: rgba(34,197,94,0.2); }
  .btn-close { border-color: var(--accent2); color: var(--accent2); background: rgba(255,107,53,0.08); }
  .btn-close:hover { background: rgba(255,107,53,0.2); }

  /* ── Preset Section ────────────────────────────────────────── */
  .preset-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(230px, 1fr));
    gap: 14px; max-width: 1100px; margin: 0 auto 24px;
  }

  .preset-card {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 12px; padding: 18px;
    transition: border-color 0.3s, box-shadow 0.3s;
  }

  .preset-card.pickup { border-left: 3px solid var(--green); }
  .preset-card.drop   { border-left: 3px solid var(--accent2); }

  .preset-title {
    font-weight: 700; font-size: 0.95rem;
    text-transform: uppercase; letter-spacing: 2px;
    margin-bottom: 4px;
  }
  .preset-card.pickup .preset-title { color: var(--green); }
  .preset-card.drop   .preset-title { color: var(--accent2); }

  .preset-angles {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.65rem; color: var(--dim);
    letter-spacing: 1px; margin-bottom: 14px;
    line-height: 1.8;
  }

  .preset-btn-row { display: flex; gap: 8px; }

  .preset-btn {
    flex: 1; padding: 9px 0;
    border-radius: 8px; border: 1px solid var(--border);
    background: transparent; color: var(--dim);
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.7rem; letter-spacing: 1px;
    cursor: pointer; transition: all 0.2s;
  }
  .preset-btn:hover { border-color: var(--accent); color: var(--accent); background: rgba(0,212,255,0.07); }

  .preset-btn.save-btn:hover { border-color: var(--yellow); color: var(--yellow); background: rgba(245,158,11,0.08); }

  /* Saved indicator */
  .saved-flash {
    animation: saved 0.8s ease-out;
  }
  @keyframes saved {
    0%   { border-color: var(--yellow); box-shadow: 0 0 14px rgba(245,158,11,0.6); }
    100% { border-color: var(--border); box-shadow: none; }
  }

  /* ── Pick & Drop Section ───────────────────────────────────── */
  .pickdrop-panel {
    max-width: 1100px; margin: 0 auto 24px;
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 14px; padding: 24px;
  }

  .pd-title {
    font-weight: 700; font-size: 1rem;
    text-transform: uppercase; letter-spacing: 3px;
    color: var(--accent); margin-bottom: 6px;
  }

  .pd-sub {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.68rem; color: var(--dim);
    letter-spacing: 1px; margin-bottom: 18px;
  }

  .pd-buttons { display: flex; gap: 14px; flex-wrap: wrap; }

  .pd-btn {
    flex: 1; min-width: 180px; padding: 16px 20px;
    border-radius: 12px; border: 1px solid;
    font-family: 'Exo 2', sans-serif; font-weight: 700;
    font-size: 1rem; letter-spacing: 2px; text-transform: uppercase;
    cursor: pointer; transition: all 0.25s;
    display: flex; flex-direction: column; align-items: center; gap: 4px;
  }
  .pd-btn .pd-icon { font-size: 1.5rem; }
  .pd-btn .pd-label { font-size: 0.75rem; letter-spacing: 3px; color: var(--dim); font-weight: 400; }

  .pd-obj1 { border-color: var(--green);   color: var(--green);   background: rgba(34,197,94,0.08); }
  .pd-obj1:hover { background: rgba(34,197,94,0.2); box-shadow: 0 0 18px rgba(34,197,94,0.3); }
  .pd-obj2 { border-color: var(--yellow);  color: var(--yellow);  background: rgba(245,158,11,0.08); }
  .pd-obj2:hover { background: rgba(245,158,11,0.2); box-shadow: 0 0 18px rgba(245,158,11,0.3); }
  .pd-stop { border-color: #ef4444; color: #ef4444; background: rgba(239,68,68,0.08); }
  .pd-stop:hover { background: rgba(239,68,68,0.2); }

  .pd-btn:disabled, .pd-btn[disabled] {
    opacity: 0.35; cursor: not-allowed; pointer-events: none;
  }

  .seq-status {
    font-family: 'Share Tech Mono', monospace;
    font-size: 0.72rem; color: var(--dim);
    margin-top: 14px; letter-spacing: 1px;
  }
  .seq-status.running { color: var(--green); }

  /* Toast */
  .toast {
    position: fixed; bottom: 24px; right: 24px;
    background: var(--surface); border: 1px solid var(--accent);
    color: var(--accent); font-family: 'Share Tech Mono', monospace;
    font-size: 0.78rem; padding: 10px 18px; border-radius: 8px;
    box-shadow: var(--glow); opacity: 0; transform: translateY(12px);
    transition: all 0.3s; pointer-events: none; z-index: 100;
  }
  .toast.show { opacity: 1; transform: translateY(0); }

  #loading { text-align: center; padding: 60px; font-family: 'Share Tech Mono', monospace; color: var(--dim); letter-spacing: 3px; }
</style>
</head>
<body>

<header>
  <div class="logo-line">
    <span class="arm-icon">🦾</span>
    <h1>Arm Control</h1>
  </div>
  <div class="subtitle">ESP32 · PCA9685 · 5-DOF · v3.0</div>
  <div class="status-bar">
    <div class="status-dot"></div>
    <span>CONNECTED &mdash; LIVE CONTROL</span>
  </div>
</header>

<!-- ── Pick & Drop ──────────────────────────────────────────── -->
<div class="section-title">⚡ Pick &amp; Drop Sequences</div>
<div class="pickdrop-panel">
  <div class="pd-title">Automated Sequences</div>
  <div class="pd-sub">Arm moves to pickup → grabs → transits → drops into basket. Uses preset positions below.</div>
  <div class="pd-buttons">
    <button class="pd-btn pd-obj1" id="btnObj1" onclick="runPickDrop(0)">
      <span class="pd-icon">📦</span>
      <span>Pick &amp; Drop Object 1</span>
      <span class="pd-label">Pickup 1 → Drop 1</span>
    </button>
    <button class="pd-btn pd-obj2" id="btnObj2" onclick="runPickDrop(1)">
      <span class="pd-icon">📦</span>
      <span>Pick &amp; Drop Object 2</span>
      <span class="pd-label">Pickup 2 → Drop 2</span>
    </button>
    <button class="pd-btn pd-stop" id="btnStop" onclick="stopSeq()">
      <span class="pd-icon">🛑</span>
      <span>Stop</span>
      <span class="pd-label">Emergency Stop</span>
    </button>
  </div>
  <div class="seq-status" id="seqStatus">IDLE — Ready</div>
</div>

<!-- ── Presets ──────────────────────────────────────────────── -->
<div class="section-title">📍 Preset Positions</div>
<div class="preset-grid" id="presetGrid">
  <div id="loading">LOADING PRESETS...</div>
</div>

<!-- ── Servo Controls ───────────────────────────────────────── -->
<div class="section-title">🎛 Manual Control</div>
<div class="grid" id="servoGrid"></div>

<div class="actions">
  <button class="action-btn btn-home"  onclick="homeAll()">⌂ Home All</button>
  <button class="action-btn btn-open"  onclick="setGripper('min')">◇ Open Gripper</button>
  <button class="action-btn btn-close" onclick="setGripper('max')">◆ Close Gripper</button>
</div>

<div class="toast" id="toast"></div>

<script>
let joints  = [];
let presets = [];
let seqPoll = null;

// Load config (joints + presets)
fetch('/config')
  .then(r => r.json())
  .then(data => {
    joints  = data.servos.map((s, idx) => ({
      id: s.id, name: s.name, icon: s.icon, sub: s.sub,
      min: s.min, max: s.max, home: s.home, val: s.current,
      isGripper: idx === data.servos.length - 1
    }));
    presets = data.presets;
    buildServoUI();
    buildPresetUI();
    if (data.seqRunning) setSeqRunning(true);
  })
  .catch(() => {
    document.getElementById('loading').textContent = '⚠ Failed to load config';
  });

// ── Servo UI ─────────────────────────────────────────────────
function buildServoUI() {
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

// ── Preset UI ────────────────────────────────────────────────
function buildPresetUI() {
  const grid = document.getElementById('presetGrid');
  grid.innerHTML = '';
  presets.forEach(p => {
    const isPickup = p.name.startsWith('pickup');
    const card = document.createElement('div');
    card.className = `preset-card ${isPickup ? 'pickup' : 'drop'}`;
    card.id = `preset-card-${p.id}`;
    card.innerHTML = `
      <div class="preset-title">${p.label}</div>
      <div class="preset-angles" id="preset-angles-${p.id}">${formatAngles(p.angles)}</div>
      <div class="preset-btn-row">
        <button class="preset-btn" onclick="gotoPreset(${p.id})">▶ Go To</button>
        <button class="preset-btn save-btn" onclick="savePreset(${p.id})">💾 Save Current</button>
      </div>
    `;
    grid.appendChild(card);
  });
}

function formatAngles(angles) {
  const names = ['Base', 'Shoulder', 'Elbow', 'Wrist', 'Gripper'];
  return names.map((n, i) => `${n}: ${angles[i]}°`).join(' &nbsp;|&nbsp; ');
}

// Go to a preset position
function gotoPreset(pid) {
  fetch(`/goto_preset?id=${pid}`)
    .then(() => showToast(`Moving to ${presets[pid].label}...`))
    .catch(() => showToast('⚠ Connection error', true));
}

// Save current slider positions as preset (temporary — shown on UI only)
// To permanently save: note the values and update firmware
function savePreset(pid) {
  const angles = joints.map(j => j.val);
  presets[pid].angles = angles;

  // Update display
  document.getElementById(`preset-angles-${pid}`).innerHTML = formatAngles(angles);

  // Flash card
  const card = document.getElementById(`preset-card-${pid}`);
  card.classList.add('saved-flash');
  setTimeout(() => card.classList.remove('saved-flash'), 900);

  // Send to firmware so it can use it immediately for sequences
  fetch(`/save_preset?id=${pid}&a0=${angles[0]}&a1=${angles[1]}&a2=${angles[2]}&a3=${angles[3]}&a4=${angles[4]}`)
    .then(() => showToast(`Saved! ${presets[pid].label} = [${angles.join(', ')}]`))
    .catch(() => showToast('⚠ Save failed', true));
}

// ── Pick & Drop ──────────────────────────────────────────────
function runPickDrop(obj) {
  fetch(`/pickdrop?obj=${obj}`)
    .then(r => r.text())
    .then(t => {
      if (t === 'OK') {
        setSeqRunning(true);
        showToast(`Sequence started: Object ${obj + 1}`);
      } else {
        showToast('⚠ ' + t, true);
      }
    })
    .catch(() => showToast('⚠ Connection error', true));
}

function stopSeq() {
  fetch('/stop_seq')
    .then(() => {
      setSeqRunning(false);
      showToast('Sequence stopped');
    });
}

function setSeqRunning(running) {
  document.getElementById('btnObj1').disabled = running;
  document.getElementById('btnObj2').disabled = running;
  const status = document.getElementById('seqStatus');
  if (running) {
    status.textContent = '▶ RUNNING — Sequence in progress...';
    status.className = 'seq-status running';
    if (!seqPoll) seqPoll = setInterval(pollSeqStatus, 800);
  } else {
    status.textContent = 'IDLE — Ready';
    status.className = 'seq-status';
    if (seqPoll) { clearInterval(seqPoll); seqPoll = null; }
  }
}

function pollSeqStatus() {
  fetch('/config')
    .then(r => r.json())
    .then(data => {
      if (!data.seqRunning) setSeqRunning(false);
    })
    .catch(() => {});
}

// ── Servo controls ───────────────────────────────────────────
function onSlide(id, val) {
  document.getElementById(`disp${id}`).textContent = val + '°';
}
function sendAngle(id, val) {
  val = parseInt(val);
  const j = joints[id];
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

// Toast
let toastTimer;
function showToast(msg, isErr = false) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.borderColor = isErr ? '#ef4444' : 'var(--accent)';
  t.style.color       = isErr ? '#ef4444' : 'var(--accent)';
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 2200);
}
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

  Serial.println("\n[INIT] Settling servos at 0°...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeServoNow(i, 0);
    targetAngles[i] = 0;
  }
  delay(300);

  Serial.println("[INIT] Sweeping to startup angles...");
  sweepBlocking(startupAngles, SERVO_SPEED_STARTUP);
  Serial.println("[INIT] Ready.");

  // WiFi
  Serial.printf("\nConnecting to %s", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

  // ── HTTP Routes ───────────────────────────────────────────

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML_PAGE);
  });

  server.on("/config", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildConfigJson());
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("ch") && req->hasParam("angle")) {
      int ch    = req->getParam("ch")->value().toInt();
      int angle = req->getParam("angle")->value().toInt();
      if (ch >= 0 && ch < NUM_SERVOS) {
        setServoAngle(ch, angle);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.on("/home", HTTP_GET, [](AsyncWebServerRequest* req) {
    moveAllHome();
    req->send(200, "text/plain", "OK");
  });

  server.on("/angles", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json = "{";
    for (int i = 0; i < NUM_SERVOS; i++) {
      json += "\"" + String(servos[i].name) + "\":" + String(currentAngles[i]);
      if (i < NUM_SERVOS - 1) json += ",";
    }
    json += "}";
    req->send(200, "application/json", json);
  });

  // /goto_preset?id=0 — move arm to a preset position
  server.on("/goto_preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id")) {
      int id = req->getParam("id")->value().toInt();
      if (id >= 0 && id < 4) {
        setAllAngles(presets[id].angles);
        Serial.printf("[PRESET] Moving to %s\n", presets[id].label);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // /save_preset?id=0&a0=45&a1=130&a2=60&a3=90&a4=0
  // Saves a preset in RAM (survives session, not power cycle — hard-code after calibration)
  server.on("/save_preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id")) {
      int id = req->getParam("id")->value().toInt();
      if (id >= 0 && id < 4) {
        int a[NUM_SERVOS];
        bool ok = true;
        const char* keys[] = { "a0","a1","a2","a3","a4" };
        for (int i = 0; i < NUM_SERVOS; i++) {
          if (req->hasParam(keys[i])) {
            a[i] = clampAngle(i, req->getParam(keys[i])->value().toInt());
          } else { ok = false; break; }
        }
        if (ok) {
          for (int i = 0; i < NUM_SERVOS; i++) presets[id].angles[i] = a[i];
          Serial.printf("[PRESET] Saved preset %d: [%d,%d,%d,%d,%d]\n",
                        id, a[0], a[1], a[2], a[3], a[4]);
          req->send(200, "text/plain", "OK");
          return;
        }
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // /pickdrop?obj=0  (0 = object1, 1 = object2)
  server.on("/pickdrop", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (seqRunning) {
      req->send(200, "text/plain", "Already running");
      return;
    }
    if (req->hasParam("obj")) {
      int obj = req->getParam("obj")->value().toInt();
      if (obj == 0 || obj == 1) {
        seqObj     = obj;
        seqStep    = 0;
        seqRunning = true;
        Serial.printf("[SEQ] Starting pick-and-drop for object %d\n", obj + 1);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // /stop_seq — emergency stop
  server.on("/stop_seq", HTTP_GET, [](AsyncWebServerRequest* req) {
    seqRunning = false;
    seqStep    = 0;
    // Halt all servos at current position
    for (int i = 0; i < NUM_SERVOS; i++) targetAngles[i] = currentAngles[i];
    Serial.println("[SEQ] STOPPED by user");
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
