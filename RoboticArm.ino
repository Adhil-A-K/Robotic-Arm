/*
  ============================================================
  Robotic Arm Controller — ESP32 + PCA9685 + Web Dashboard
  v3.0 — Presets + Pick/Drop State Machine
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

#define CH_BASE      0
#define CH_SHOULDER  1
#define CH_ELBOW     2
#define CH_WRIST     3
#define CH_GRIPPER   4
#define NUM_SERVOS   5

// ╔══════════════════════════════════════════════════════════════╗
// ║             SERVO CONFIGURATION — EDIT HERE                 ║
// ╚══════════════════════════════════════════════════════════════╝
struct ServoConfig {
  const char* name;
  const char* icon;
  const char* sub;
  int  channel;
  int  minAngle;   // HARD LIMIT — physical stop
  int  maxAngle;   // HARD LIMIT — physical stop
  int  homeAngle;
};

ServoConfig servos[NUM_SERVOS] = {
  { "Base",     "🔄", "Rotation / Yaw",  CH_BASE,      0, 180,  90 },
  { "Shoulder", "💪", "Joint 1 / Lift",  CH_SHOULDER,  0, 180,  90 },
  { "Elbow",    "🦾", "Joint 2 / Reach", CH_ELBOW,     0, 180,  90 },
  { "Wrist",    "🤚", "Joint 3 / Tilt",  CH_WRIST,     0, 180,  90 },
  { "Gripper",  "✊", "End Effector",    CH_GRIPPER,   0,  90,  45 },
};

// ── Startup angles (where arm sweeps to on boot) ──────────────
//                      Base Shoulder Elbow Wrist Gripper
int startupAngles[NUM_SERVOS] = { 90, 180, 0, 90, 30 };

// ── Speed ─────────────────────────────────────────────────────
#define SERVO_SPEED_STARTUP  30   // °/s  on boot
#define SERVO_SPEED          60   // °/s  runtime

// ╔══════════════════════════════════════════════════════════════╗
// ║                    PRESET LOCATIONS                         ║
// ║                                                             ║
// ║  4 presets: PICKUP_1, PICKUP_2, DROP_1, DROP_2             ║
// ║  Angles: { Base, Shoulder, Elbow, Wrist, Gripper }          ║
// ║                                                             ║
// ║  GRIPPER CONVENTION:                                        ║
// ║    GRIPPER_OPEN  (0)  = open  / release object             ║
// ║    GRIPPER_CLOSE (90) = closed / gripping object           ║
// ║                                                             ║
// ║  These are placeholder values — calibrate via web UI,      ║
// ║  then paste the exported values here and re-flash.         ║
// ╚══════════════════════════════════════════════════════════════╝
#define GRIPPER_OPEN   0
#define GRIPPER_CLOSE  90
#define NUM_PRESETS    4

// Preset index constants
#define PRESET_PICKUP_1  0
#define PRESET_PICKUP_2  1
#define PRESET_DROP_1    2
#define PRESET_DROP_2    3

const char* presetNames[NUM_PRESETS] = {
  "Pickup 1", "Pickup 2", "Drop 1", "Drop 2"
};
const char* presetIcons[NUM_PRESETS] = { "📦", "📦", "🗑️", "🗑️" };

// { Base, Shoulder, Elbow, Wrist, Gripper }
// Gripper is FORCED by the state machine — value here is ignored at runtime.
// It is included so "Save Preset" captures the full arm pose for reference.
int hardcodedPresets[NUM_PRESETS][NUM_SERVOS] = {
  //  Base  Shoulder  Elbow  Wrist  Gripper
  {   60,     120,     60,    90,     0  },   // PICKUP_1
  {  120,     120,     60,    90,     0  },   // PICKUP_2
  {   60,      60,    120,    90,     0  },   // DROP_1
  {  120,      60,    120,    90,     0  },   // DROP_2
};

// Runtime preset storage (starts as hardcoded, updated via web UI)
int presets[NUM_PRESETS][NUM_SERVOS];

// ─────────────────────────────────────────────────────────────
// Runtime state
// ─────────────────────────────────────────────────────────────
int currentAngles[NUM_SERVOS] = { 0, 0, 0, 0, 0 };
int targetAngles[NUM_SERVOS]  = { 0, 0, 0, 0, 0 };
unsigned long lastStepMs = 0;

// ── Pick/Drop state machine ───────────────────────────────────
enum TaskState {
  TASK_IDLE,
  TASK_OPEN_GRIPPER,     // open gripper before approaching pickup
  TASK_GOTO_PICKUP,      // move arm to pickup location
  TASK_CLOSE_GRIPPER,    // close gripper to grab
  TASK_GRIP_SETTLE,      // wait briefly for grip to settle
  TASK_GOTO_DROP,        // move arm to drop location
  TASK_OPEN_DROP,        // open gripper to release
  TASK_RELEASE_SETTLE,   // wait briefly
  TASK_GOTO_HOME,        // return to home position
  TASK_DONE
};

TaskState taskState = TASK_IDLE;
int       taskObject  = -1;    // 0 = object1, 1 = object2
unsigned long taskTimerMs = 0;
String    taskStatusMsg = "Idle";

// ── Web Server ────────────────────────────────────────────────
AsyncWebServer server(80);

// ─────────────────────────────────────────────────────────────
// Core servo helpers
// ─────────────────────────────────────────────────────────────
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
  pca.setPWM(servos[ch].channel, 0,
             angleToPwm(angle, servos[ch].minAngle, servos[ch].maxAngle));
  currentAngles[ch] = angle;
}

void setServoAngle(int ch, int angle) {
  targetAngles[ch] = clampAngle(ch, angle);
}

// Returns true when all servos have reached their targets
bool allServosAtTarget() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (currentAngles[i] != targetAngles[i]) return false;
  }
  return true;
}

void moveToPreset(int idx) {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, presets[idx][i]);
  }
}

void moveAllHome() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServoAngle(i, servos[i].homeAngle);
  }
}

// Non-blocking smooth sweep — called from loop()
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

// Blocking sweep for setup()
void sweepBlocking(int destAngles[], int speedDegPerSec) {
  for (int i = 0; i < NUM_SERVOS; i++)
    targetAngles[i] = clampAngle(i, destAngles[i]);

  unsigned long stepTime = millis();
  bool allDone = false;
  while (!allDone) {
    unsigned long now = millis();
    unsigned long dt  = now - stepTime;
    if (dt >= 20) {
      stepTime = now;
      allDone  = true;
      float maxStep = max((speedDegPerSec * (int)dt) / 1000.0f, 1.0f);
      for (int i = 0; i < NUM_SERVOS; i++) {
        int cur = currentAngles[i], tgt = targetAngles[i];
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

// ─────────────────────────────────────────────────────────────
// Pick/Drop state machine — runs in loop()
// ─────────────────────────────────────────────────────────────
void startPickDrop(int objIndex) {
  if (taskState != TASK_IDLE) return;
  taskObject = objIndex;   // 0 or 1
  taskState  = TASK_OPEN_GRIPPER;
  taskStatusMsg = "Starting: open gripper";
  Serial.printf("[TASK] Pick/drop object %d — starting\n", objIndex + 1);
}

void updateTask() {
  if (taskState == TASK_IDLE || taskState == TASK_DONE) return;

  switch (taskState) {

    case TASK_OPEN_GRIPPER:
      setServoAngle(CH_GRIPPER, GRIPPER_OPEN);
      taskState = TASK_GOTO_PICKUP;
      taskStatusMsg = "Moving to pickup " + String(taskObject + 1);
      break;

    case TASK_GOTO_PICKUP:
      // Queue pickup preset (gripper channel will be overridden anyway)
      moveToPreset(taskObject == 0 ? PRESET_PICKUP_1 : PRESET_PICKUP_2);
      setServoAngle(CH_GRIPPER, GRIPPER_OPEN);  // keep open during approach
      // Wait until arm arrives
      if (allServosAtTarget()) {
        taskState = TASK_CLOSE_GRIPPER;
        taskStatusMsg = "Closing gripper";
      }
      break;

    case TASK_CLOSE_GRIPPER:
      setServoAngle(CH_GRIPPER, GRIPPER_CLOSE);
      taskState     = TASK_GRIP_SETTLE;
      taskTimerMs   = millis();
      taskStatusMsg = "Gripping...";
      break;

    case TASK_GRIP_SETTLE:
      if (millis() - taskTimerMs >= 600) {
        taskState = TASK_GOTO_DROP;
        taskStatusMsg = "Moving to drop " + String(taskObject + 1);
      }
      break;

    case TASK_GOTO_DROP:
      moveToPreset(taskObject == 0 ? PRESET_DROP_1 : PRESET_DROP_2);
      setServoAngle(CH_GRIPPER, GRIPPER_CLOSE);  // keep closed during transit
      if (allServosAtTarget()) {
        taskState = TASK_OPEN_DROP;
        taskStatusMsg = "Releasing object";
      }
      break;

    case TASK_OPEN_DROP:
      setServoAngle(CH_GRIPPER, GRIPPER_OPEN);
      taskState   = TASK_RELEASE_SETTLE;
      taskTimerMs = millis();
      break;

    case TASK_RELEASE_SETTLE:
      if (millis() - taskTimerMs >= 500) {
        taskState = TASK_GOTO_HOME;
        taskStatusMsg = "Returning home";
      }
      break;

    case TASK_GOTO_HOME:
      moveAllHome();
      if (allServosAtTarget()) {
        taskState = TASK_DONE;
        taskStatusMsg = "Done — object " + String(taskObject + 1) + " delivered!";
        Serial.printf("[TASK] Object %d delivered.\n", taskObject + 1);
      }
      break;

    case TASK_DONE:
      taskState = TASK_IDLE;
      break;

    default: break;
  }
}

// ─────────────────────────────────────────────────────────────
// JSON builders
// ─────────────────────────────────────────────────────────────
String buildConfigJson() {
  String json = "{\"servos\":[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += "{";
    json += "\"id\":"      + String(i)                  + ",";
    json += "\"name\":\""  + String(servos[i].name)     + "\",";
    json += "\"icon\":\""  + String(servos[i].icon)     + "\",";
    json += "\"sub\":\""   + String(servos[i].sub)      + "\",";
    json += "\"min\":"     + String(servos[i].minAngle) + ",";
    json += "\"max\":"     + String(servos[i].maxAngle) + ",";
    json += "\"home\":"    + String(servos[i].homeAngle)+ ",";
    json += "\"current\":" + String(currentAngles[i]);
    json += "}";
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "],\"presets\":[";
  for (int p = 0; p < NUM_PRESETS; p++) {
    json += "{";
    json += "\"id\":"     + String(p)               + ",";
    json += "\"name\":\"" + String(presetNames[p])  + "\",";
    json += "\"icon\":\"" + String(presetIcons[p])  + "\",";
    json += "\"angles\":[";
    for (int i = 0; i < NUM_SERVOS; i++) {
      json += String(presets[p][i]);
      if (i < NUM_SERVOS - 1) json += ",";
    }
    json += "]}";
    if (p < NUM_PRESETS - 1) json += ",";
  }
  json += "]}";
  return json;
}

String buildStatusJson() {
  String json = "{";
  json += "\"task\":\"" + taskStatusMsg + "\",";
  json += "\"busy\":"   + String(taskState != TASK_IDLE && taskState != TASK_DONE ? "true" : "false") + ",";
  json += "\"angles\":[";
  for (int i = 0; i < NUM_SERVOS; i++) {
    json += String(currentAngles[i]);
    if (i < NUM_SERVOS - 1) json += ",";
  }
  json += "]}";
  return json;
}

// ─────────────────────────────────────────────────────────────
// HTML (loaded from flash)
// ─────────────────────────────────────────────────────────────
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
    --bg:#0a0c10;--surface:#111520;--surface2:#161c2e;
    --border:#1e2a40;--accent:#00d4ff;--accent2:#ff6b35;
    --green:#22c55e;--yellow:#facc15;
    --text:#c8d8f0;--dim:#4a5a78;
    --glow:0 0 18px rgba(0,212,255,0.35);
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);font-family:'Exo 2',sans-serif;color:var(--text);min-height:100vh;padding:20px;
    background-image:radial-gradient(ellipse at 20% 10%,rgba(0,212,255,.05) 0%,transparent 50%),
    radial-gradient(ellipse at 80% 90%,rgba(255,107,53,.05) 0%,transparent 50%);}
  header{text-align:center;margin-bottom:28px;padding-bottom:18px;border-bottom:1px solid var(--border);}
  .logo-line{display:flex;align-items:center;justify-content:center;gap:14px;margin-bottom:6px;}
  .arm-icon{font-size:2rem;filter:drop-shadow(0 0 8px var(--accent));}
  h1{font-size:1.9rem;font-weight:800;letter-spacing:3px;text-transform:uppercase;
    background:linear-gradient(90deg,var(--accent),#60efff);-webkit-background-clip:text;-webkit-text-fill-color:transparent;}
  .subtitle{font-family:'Share Tech Mono',monospace;font-size:.72rem;color:var(--dim);letter-spacing:4px;text-transform:uppercase;}
  .status-bar{display:flex;align-items:center;justify-content:center;gap:8px;margin-top:10px;
    font-family:'Share Tech Mono',monospace;font-size:.75rem;color:var(--dim);}
  .status-dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 8px var(--green);animation:pulse 2s infinite;}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}

  /* ── Section headings ── */
  .section-title{
    font-family:'Share Tech Mono',monospace;font-size:.7rem;letter-spacing:4px;
    text-transform:uppercase;color:var(--dim);margin:28px 0 12px;
    display:flex;align-items:center;gap:10px;max-width:1100px;margin-left:auto;margin-right:auto;
  }
  .section-title::after{content:'';flex:1;height:1px;background:var(--border);}

  /* ── Servo cards grid ── */
  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(290px,1fr));
    gap:16px;max-width:1100px;margin:0 auto 8px;}
  .card{background:var(--surface);border:1px solid var(--border);border-radius:12px;padding:20px;
    position:relative;overflow:hidden;transition:border-color .3s,box-shadow .3s;}
  .card::before{content:'';position:absolute;top:0;left:0;right:0;height:2px;
    background:linear-gradient(90deg,transparent,var(--accent),transparent);opacity:0;transition:opacity .3s;}
  .card:hover{border-color:var(--accent);box-shadow:var(--glow);}
  .card:hover::before{opacity:1;}
  .gripper-card:hover{border-color:var(--accent2);box-shadow:0 0 18px rgba(255,107,53,.35);}
  .card-header{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px;}
  .joint-label{display:flex;align-items:center;gap:10px;}
  .joint-icon{width:36px;height:36px;border-radius:8px;background:rgba(0,212,255,.1);
    border:1px solid rgba(0,212,255,.25);display:flex;align-items:center;justify-content:center;font-size:1.1rem;}
  .gripper-card .joint-icon{background:rgba(255,107,53,.1);border-color:rgba(255,107,53,.25);}
  .joint-name{font-weight:600;font-size:1rem;letter-spacing:1px;text-transform:uppercase;color:#e0eeff;}
  .joint-sub{font-family:'Share Tech Mono',monospace;font-size:.65rem;color:var(--dim);}
  .angle-display{font-family:'Share Tech Mono',monospace;font-size:1.6rem;font-weight:bold;
    color:var(--accent);text-shadow:0 0 12px rgba(0,212,255,.5);min-width:60px;text-align:right;}
  .gripper-card .angle-display{color:var(--accent2);text-shadow:0 0 12px rgba(255,107,53,.5);}
  .limit-row{display:flex;gap:6px;margin-bottom:10px;flex-wrap:wrap;}
  .limit-badge{font-family:'Share Tech Mono',monospace;font-size:.6rem;padding:2px 7px;border-radius:4px;letter-spacing:1px;}
  .badge-min{border:1px solid rgba(255,107,53,.5);color:rgba(255,107,53,.8);background:rgba(255,107,53,.06);}
  .badge-max{border:1px solid rgba(0,212,255,.5);color:rgba(0,212,255,.8);background:rgba(0,212,255,.06);}
  .badge-home{border:1px solid rgba(34,197,94,.4);color:rgba(34,197,94,.7);background:rgba(34,197,94,.05);}
  input[type=range]{-webkit-appearance:none;width:100%;height:6px;border-radius:3px;
    background:var(--border);outline:none;cursor:pointer;margin-bottom:4px;}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:20px;height:20px;border-radius:50%;
    background:var(--accent);box-shadow:0 0 10px rgba(0,212,255,.6);cursor:pointer;transition:transform .15s,box-shadow .15s;}
  input[type=range]::-webkit-slider-thumb:hover{transform:scale(1.25);box-shadow:0 0 18px rgba(0,212,255,.9);}
  .gripper-card input[type=range]::-webkit-slider-thumb{background:var(--accent2);box-shadow:0 0 10px rgba(255,107,53,.6);}
  .gripper-card input[type=range]::-webkit-slider-thumb:hover{box-shadow:0 0 18px rgba(255,107,53,.9);}
  .range-labels{display:flex;justify-content:space-between;font-family:'Share Tech Mono',monospace;
    font-size:.62rem;color:var(--dim);margin-bottom:10px;}
  .btn-row{display:flex;gap:6px;}
  .btn{flex:1;padding:6px 0;border-radius:7px;border:1px solid var(--border);background:transparent;
    color:var(--dim);font-family:'Share Tech Mono',monospace;font-size:.68rem;letter-spacing:1px;
    cursor:pointer;transition:all .2s;}
  .btn:hover{border-color:var(--accent);color:var(--accent);background:rgba(0,212,255,.07);}

  /* ── Global action buttons ── */
  .actions{display:flex;gap:10px;justify-content:center;max-width:1100px;margin:0 auto 0;flex-wrap:wrap;}
  .action-btn{padding:11px 24px;border-radius:10px;border:1px solid;font-family:'Exo 2',sans-serif;
    font-weight:600;font-size:.88rem;letter-spacing:2px;text-transform:uppercase;cursor:pointer;transition:all .25s;}
  .btn-home{border-color:var(--accent);color:var(--accent);background:rgba(0,212,255,.08);}
  .btn-home:hover{background:rgba(0,212,255,.2);box-shadow:var(--glow);}
  .btn-open{border-color:var(--green);color:var(--green);background:rgba(34,197,94,.08);}
  .btn-open:hover{background:rgba(34,197,94,.2);}
  .btn-close{border-color:var(--accent2);color:var(--accent2);background:rgba(255,107,53,.08);}
  .btn-close:hover{background:rgba(255,107,53,.2);}

  /* ── Preset cards ── */
  .preset-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));
    gap:14px;max-width:1100px;margin:0 auto;}
  .preset-card{background:var(--surface2);border:1px solid var(--border);border-radius:12px;padding:18px;
    transition:border-color .3s,box-shadow .3s;}
  .preset-card:hover{border-color:rgba(0,212,255,.4);}
  .preset-header{display:flex;align-items:center;gap:10px;margin-bottom:14px;}
  .preset-icon{font-size:1.4rem;}
  .preset-name{font-weight:700;font-size:.95rem;letter-spacing:1px;text-transform:uppercase;color:#e0eeff;}
  .preset-tag{font-family:'Share Tech Mono',monospace;font-size:.6rem;color:var(--dim);}
  .preset-angles{display:flex;flex-wrap:wrap;gap:5px;margin-bottom:14px;}
  .angle-pill{font-family:'Share Tech Mono',monospace;font-size:.65rem;padding:3px 8px;
    border-radius:4px;border:1px solid var(--border);color:var(--dim);background:rgba(255,255,255,.02);}
  .angle-pill span{color:var(--text);}
  .preset-btns{display:flex;gap:8px;}
  .preset-btn{flex:1;padding:8px 0;border-radius:8px;border:1px solid;
    font-family:'Share Tech Mono',monospace;font-size:.7rem;letter-spacing:1px;
    cursor:pointer;transition:all .2s;text-transform:uppercase;}
  .btn-goto{border-color:rgba(0,212,255,.5);color:var(--accent);background:rgba(0,212,255,.06);}
  .btn-goto:hover{background:rgba(0,212,255,.15);border-color:var(--accent);}
  .btn-save{border-color:rgba(250,204,21,.5);color:var(--yellow);background:rgba(250,204,21,.05);}
  .btn-save:hover{background:rgba(250,204,21,.15);border-color:var(--yellow);}
  .btn-save.saved{border-color:rgba(34,197,94,.7);color:var(--green);background:rgba(34,197,94,.1);}

  /* ── Pick/Drop panel ── */
  .pickdrop-panel{max-width:1100px;margin:0 auto;background:var(--surface2);
    border:1px solid var(--border);border-radius:12px;padding:20px;}
  .pd-title{font-weight:700;font-size:1rem;letter-spacing:2px;text-transform:uppercase;
    color:#e0eeff;margin-bottom:6px;}
  .pd-desc{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);margin-bottom:16px;}
  .pd-btns{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:16px;}
  .pd-btn{padding:12px 24px;border-radius:10px;border:1px solid;font-family:'Exo 2',sans-serif;
    font-weight:700;font-size:.9rem;letter-spacing:2px;text-transform:uppercase;cursor:pointer;transition:all .25s;}
  .pd-btn-1{border-color:#818cf8;color:#818cf8;background:rgba(129,140,248,.08);}
  .pd-btn-1:hover{background:rgba(129,140,248,.2);}
  .pd-btn-2{border-color:#f472b6;color:#f472b6;background:rgba(244,114,182,.08);}
  .pd-btn-2:hover{background:rgba(244,114,182,.2);}
  .pd-btn:disabled{opacity:.35;cursor:not-allowed;}
  .task-status{font-family:'Share Tech Mono',monospace;font-size:.78rem;padding:10px 14px;
    border-radius:8px;background:rgba(0,0,0,.25);border:1px solid var(--border);
    color:var(--text);display:flex;align-items:center;gap:8px;}
  .task-dot{width:8px;height:8px;border-radius:50%;background:var(--dim);flex-shrink:0;}
  .task-dot.busy{background:#facc15;box-shadow:0 0 8px #facc15;animation:pulse 1s infinite;}
  .task-dot.done{background:var(--green);box-shadow:0 0 8px var(--green);}

  /* ── Export modal ── */
  .modal-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.75);
    z-index:100;align-items:center;justify-content:center;}
  .modal-overlay.show{display:flex;}
  .modal{background:var(--surface);border:1px solid var(--border);border-radius:14px;
    padding:28px;max-width:700px;width:90%;max-height:85vh;overflow-y:auto;
    box-shadow:0 0 40px rgba(0,0,0,.6);}
  .modal-title{font-weight:800;font-size:1.1rem;letter-spacing:2px;text-transform:uppercase;
    color:var(--accent);margin-bottom:6px;}
  .modal-sub{font-family:'Share Tech Mono',monospace;font-size:.7rem;color:var(--dim);margin-bottom:16px;}
  .modal pre{background:rgba(0,0,0,.4);border:1px solid var(--border);border-radius:8px;
    padding:16px;font-family:'Share Tech Mono',monospace;font-size:.78rem;
    color:#a0e0ff;overflow-x:auto;white-space:pre;margin-bottom:16px;}
  .modal-btns{display:flex;gap:10px;justify-content:flex-end;}
  .modal-btn{padding:9px 20px;border-radius:8px;border:1px solid;
    font-family:'Exo 2',sans-serif;font-weight:600;font-size:.85rem;
    letter-spacing:1px;cursor:pointer;transition:all .2s;}
  .btn-copy{border-color:var(--accent);color:var(--accent);background:rgba(0,212,255,.08);}
  .btn-copy:hover{background:rgba(0,212,255,.2);}
  .btn-modal-close{border-color:var(--border);color:var(--dim);background:transparent;}
  .btn-modal-close:hover{border-color:var(--text);color:var(--text);}

  .toast{position:fixed;bottom:24px;right:24px;background:var(--surface);
    border:1px solid var(--accent);color:var(--accent);font-family:'Share Tech Mono',monospace;
    font-size:.78rem;padding:10px 18px;border-radius:8px;box-shadow:var(--glow);
    opacity:0;transform:translateY(12px);transition:all .3s;pointer-events:none;}
  .toast.show{opacity:1;transform:translateY(0);}
  #loading{text-align:center;padding:60px;font-family:'Share Tech Mono',monospace;color:var(--dim);letter-spacing:3px;}
</style>
</head>
<body>

<header>
  <div class="logo-line">
    <span class="arm-icon">🦾</span>
    <h1>Arm Control</h1>
  </div>
  <div class="subtitle">ESP32 · PCA9685 · 5-DOF · Presets + Pick/Drop</div>
  <div class="status-bar"><div class="status-dot"></div><span>CONNECTED — LIVE CONTROL</span></div>
</header>

<!-- ── Servo sliders ── -->
<div class="section-title">Joint Control</div>
<div class="grid" id="servoGrid"><div id="loading">LOADING CONFIG...</div></div>

<br>
<div class="actions">
  <button class="action-btn btn-home"  onclick="homeAll()">⌂ Home All</button>
  <button class="action-btn btn-open"  onclick="setGripper('min')">◇ Open Gripper</button>
  <button class="action-btn btn-close" onclick="setGripper('max')">◆ Close Gripper</button>
</div>

<!-- ── Presets ── -->
<div class="section-title">Preset Locations</div>
<div class="preset-grid" id="presetGrid"></div>

<br>
<div style="text-align:center;max-width:1100px;margin:0 auto;">
  <button class="action-btn" style="border-color:#a78bfa;color:#a78bfa;background:rgba(167,139,250,.08);"
    onclick="showExport()">⬇ Export Presets to Code</button>
</div>

<!-- ── Pick / Drop ── -->
<div class="section-title">Pick &amp; Drop</div>
<div class="pickdrop-panel">
  <div class="pd-title">🤖 Automated Pick &amp; Drop</div>
  <div class="pd-desc">Runs full sequence: open gripper → pickup → grab → transit → drop → home</div>
  <div class="pd-btns">
    <button class="pd-btn pd-btn-1" id="pdBtn1" onclick="pickDrop(1)">📦 Object 1</button>
    <button class="pd-btn pd-btn-2" id="pdBtn2" onclick="pickDrop(2)">📦 Object 2</button>
  </div>
  <div class="task-status">
    <div class="task-dot" id="taskDot"></div>
    <span id="taskStatus">Idle</span>
  </div>
</div>

<!-- ── Export modal ── -->
<div class="modal-overlay" id="exportModal">
  <div class="modal">
    <div class="modal-title">📋 Export Presets to Code</div>
    <div class="modal-sub">Copy this into RoboticArm.ino → flash → presets are permanently saved</div>
    <pre id="exportCode"></pre>
    <div class="modal-btns">
      <button class="modal-btn btn-copy" onclick="copyExport()">Copy to Clipboard</button>
      <button class="modal-btn btn-modal-close" onclick="closeExport()">Close</button>
    </div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
let joints   = [];
let presets  = [];
let pollTimer = null;

// ── Boot: load config from firmware ──────────────────────────
fetch('/config')
  .then(r => r.json())
  .then(data => {
    joints  = data.servos;
    presets = data.presets.map(p => ({ ...p, angles: [...p.angles] }));
    joints.forEach(j => j.val = j.current);
    buildServoUI();
    buildPresetUI();
    startPoll();
  })
  .catch(() => {
    document.getElementById('loading').textContent = '⚠ Failed to load config';
  });

// ── Servo UI ──────────────────────────────────────────────────
function buildServoUI() {
  const grid = document.getElementById('servoGrid');
  grid.innerHTML = '';
  joints.forEach(j => {
    const isGripper = j.id === joints.length - 1;
    const card = document.createElement('div');
    card.className = 'card' + (isGripper ? ' gripper-card' : '');
    card.innerHTML = `
      <div class="card-header">
        <div class="joint-label">
          <div class="joint-icon">${j.icon}</div>
          <div><div class="joint-name">${j.name}</div><div class="joint-sub">${j.sub}</div></div>
        </div>
        <div class="angle-display" id="disp${j.id}">${j.val}°</div>
      </div>
      <div class="limit-row">
        <span class="limit-badge badge-min">MIN ${j.min}°</span>
        <span class="limit-badge badge-max">MAX ${j.max}°</span>
        <span class="limit-badge badge-home">HOME ${j.home}°</span>
      </div>
      <input type="range" id="sl${j.id}" min="${j.min}" max="${j.max}" value="${j.val}"
        oninput="onSlide(${j.id},this.value)" onchange="sendAngle(${j.id},this.value)">
      <div class="range-labels"><span>${j.min}°</span><span>${j.max}°</span></div>
      <div class="btn-row">
        <button class="btn" onclick="nudge(${j.id},-10)">−10°</button>
        <button class="btn" onclick="nudge(${j.id}, -1)">− 1°</button>
        <button class="btn" onclick="centerJoint(${j.id})">CTR</button>
        <button class="btn" onclick="nudge(${j.id}, +1)">+ 1°</button>
        <button class="btn" onclick="nudge(${j.id},+10)">+10°</button>
      </div>`;
    grid.appendChild(card);
  });
}

function onSlide(id, val) {
  document.getElementById('disp'+id).textContent = val + '°';
}
function sendAngle(id, val) {
  val = parseInt(val);
  joints[id].val = val;
  fetch('/set?ch='+id+'&angle='+val)
    .then(()=>showToast(joints[id].name+' → '+val+'°'))
    .catch(()=>showToast('⚠ Connection error',true));
}
function nudge(id, delta) {
  const j = joints[id];
  const v = Math.min(j.max, Math.max(j.min, j.val+delta));
  document.getElementById('sl'+id).value = v;
  onSlide(id, v); sendAngle(id, v);
}
function centerJoint(id) {
  const j = joints[id];
  const mid = Math.floor((j.min+j.max)/2);
  document.getElementById('sl'+id).value = mid;
  onSlide(id, mid); sendAngle(id, mid);
}
function homeAll() {
  fetch('/home').then(()=>{
    joints.forEach(j=>{
      document.getElementById('sl'+j.id).value = j.home;
      onSlide(j.id, j.home); j.val = j.home;
    });
    showToast('All joints → Home');
  });
}
function setGripper(mode) {
  const g = joints[joints.length-1];
  const angle = mode==='min' ? g.min : g.max;
  document.getElementById('sl'+g.id).value = angle;
  onSlide(g.id, angle); sendAngle(g.id, angle);
}

// ── Preset UI ─────────────────────────────────────────────────
function buildPresetUI() {
  const grid = document.getElementById('presetGrid');
  grid.innerHTML = '';
  presets.forEach(p => {
    const card = document.createElement('div');
    card.className = 'preset-card';
    card.id = 'preset-card-' + p.id;
    card.innerHTML = presetCardHTML(p);
    grid.appendChild(card);
  });
}

function presetCardHTML(p) {
  const names = ['Base','Shoulder','Elbow','Wrist','Gripper'];
  const pills = p.angles.map((a,i) =>
    `<div class="angle-pill">${names[i].substring(0,3)}: <span>${a}°</span></div>`
  ).join('');
  return `
    <div class="preset-header">
      <div class="preset-icon">${p.icon}</div>
      <div>
        <div class="preset-name">${p.name}</div>
        <div class="preset-tag">PRESET ${p.id+1} · ${p.id<2?'PICKUP':'DROP'}</div>
      </div>
    </div>
    <div class="preset-angles">${pills}</div>
    <div class="preset-btns">
      <button class="preset-btn btn-goto" onclick="gotoPreset(${p.id})">▶ Go To</button>
      <button class="preset-btn btn-save" id="save-btn-${p.id}" onclick="savePreset(${p.id})">● Save Current</button>
    </div>`;
}

function gotoPreset(id) {
  fetch('/preset?id='+id)
    .then(()=>showToast('Moving to '+presets[id].name))
    .catch(()=>showToast('⚠ Error',true));
}

function savePreset(id) {
  // Capture current slider values as the new preset
  const angles = joints.map(j => j.val);
  presets[id].angles = [...angles];

  // Send to firmware
  const body = JSON.stringify({ id: id, angles: angles });
  fetch('/savepreset', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body
  }).then(()=>{
    // Refresh card display
    document.getElementById('preset-card-'+id).innerHTML = presetCardHTML(presets[id]);
    const btn = document.getElementById('save-btn-'+id);
    if(btn){ btn.textContent = '✓ Saved'; btn.classList.add('saved'); }
    showToast(presets[id].name+' saved!');
  }).catch(()=>showToast('⚠ Save failed',true));
}

// ── Pick/Drop ─────────────────────────────────────────────────
function pickDrop(obj) {
  const busy = document.getElementById('taskDot').classList.contains('busy');
  if(busy){ showToast('Task already running!', true); return; }
  fetch('/pickdrop?obj='+(obj-1))
    .then(()=>showToast('Task started: Object '+obj))
    .catch(()=>showToast('⚠ Error',true));
}

// ── Status poll ───────────────────────────────────────────────
function startPoll() {
  pollTimer = setInterval(pollStatus, 500);
}

function pollStatus() {
  fetch('/status')
    .then(r=>r.json())
    .then(s=>{
      document.getElementById('taskStatus').textContent = s.task;
      const dot = document.getElementById('taskDot');
      dot.className = 'task-dot' + (s.busy ? ' busy' : ' done');
      const btns = [document.getElementById('pdBtn1'), document.getElementById('pdBtn2')];
      btns.forEach(b=>{ if(b) b.disabled = s.busy; });
      // Update slider displays from live angles
      s.angles.forEach((a,i)=>{
        const sl = document.getElementById('sl'+i);
        const dp = document.getElementById('disp'+i);
        if(sl && !sl.matches(':active')){ sl.value = a; }
        if(dp && !sl.matches(':active')){ dp.textContent = a+'°'; joints[i].val = a; }
      });
    })
    .catch(()=>{});
}

// ── Export to code ─────────────────────────────────────────────
function showExport() {
  const names = presets.map(p=>p.name.toUpperCase().replace(/ /g,'_'));
  let code = '// ── Paste this into RoboticArm.ino to hard-code current presets ──\n';
  code += '// { Base, Shoulder, Elbow, Wrist, Gripper }\n\n';
  code += 'int hardcodedPresets[NUM_PRESETS][NUM_SERVOS] = {\n';
  presets.forEach((p,i)=>{
    const padded = p.angles.map(a=>String(a).padStart(4,' ')).join(', ');
    code += '  { '+padded+' },   // '+p.name+(i<presets.length-1?'':'')+'\n';
  });
  code += '};';
  document.getElementById('exportCode').textContent = code;
  document.getElementById('exportModal').classList.add('show');
}
function closeExport() { document.getElementById('exportModal').classList.remove('show'); }
function copyExport() {
  const text = document.getElementById('exportCode').textContent;
  navigator.clipboard.writeText(text)
    .then(()=>showToast('Copied to clipboard!'))
    .catch(()=>showToast('⚠ Copy failed — select manually',true));
}

// ── Toast ─────────────────────────────────────────────────────
let toastTimer;
function showToast(msg, isErr=false) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.borderColor = isErr?'#ef4444':'var(--accent)';
  t.style.color       = isErr?'#ef4444':'var(--accent)';
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(()=>t.classList.remove('show'),2000);
}
</script>
</body>
</html>
)=====";

// ─────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Copy hardcoded presets into runtime array
  for (int p = 0; p < NUM_PRESETS; p++)
    for (int i = 0; i < NUM_SERVOS; i++)
      presets[p][i] = hardcodedPresets[p][i];

  // PCA9685 init
  pca.begin();
  pca.setOscillatorFrequency(OSC_FREQ);
  pca.setPWMFreq(PWM_FREQ_HZ);
  delay(10);

  // Settle at 0° then slow sweep to startup position
  Serial.println("\n[INIT] Settling servos at 0°...");
  for (int i = 0; i < NUM_SERVOS; i++) { writeServoNow(i, 0); targetAngles[i] = 0; }
  delay(300);

  Serial.println("[INIT] Slow sweep to startup angles...");
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

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", buildStatusJson());
  });

  // /set?ch=0&angle=90
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("ch") && req->hasParam("angle")) {
      int ch    = req->getParam("ch")->value().toInt();
      int angle = req->getParam("angle")->value().toInt();
      if (ch >= 0 && ch < NUM_SERVOS) {
        setServoAngle(ch, angle);
        Serial.printf("[SET] %s → %d°\n", servos[ch].name, clampAngle(ch, angle));
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

  // /preset?id=0  — move arm to a preset (smooth)
  server.on("/preset", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (req->hasParam("id")) {
      int id = req->getParam("id")->value().toInt();
      if (id >= 0 && id < NUM_PRESETS) {
        moveToPreset(id);
        Serial.printf("[PRESET] Moving to: %s\n", presetNames[id]);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  // POST /savepreset  { "id": 0, "angles": [90,120,60,90,0] }
  server.on("/savepreset", HTTP_POST,
    [](AsyncWebServerRequest* req) {},
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, data, len) == DeserializationError::Ok) {
        int id = doc["id"];
        if (id >= 0 && id < NUM_PRESETS) {
          JsonArray arr = doc["angles"];
          for (int i = 0; i < NUM_SERVOS && i < (int)arr.size(); i++)
            presets[id][i] = clampAngle(i, (int)arr[i]);
          Serial.printf("[SAVE] Preset %d (%s) updated: ", id, presetNames[id]);
          for (int i = 0; i < NUM_SERVOS; i++) Serial.printf("%d ", presets[id][i]);
          Serial.println();
          req->send(200, "text/plain", "OK");
          return;
        }
      }
      req->send(400, "text/plain", "Bad Request");
    }
  );

  // /pickdrop?obj=0  — trigger full pick/drop sequence (0 or 1)
  server.on("/pickdrop", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (taskState != TASK_IDLE) {
      req->send(409, "text/plain", "Busy");
      return;
    }
    if (req->hasParam("obj")) {
      int obj = req->getParam("obj")->value().toInt();
      if (obj == 0 || obj == 1) {
        startPickDrop(obj);
        req->send(200, "text/plain", "OK");
        return;
      }
    }
    req->send(400, "text/plain", "Bad Request");
  });

  server.begin();
  Serial.println("Web server started.");
  lastStepMs = millis();
}

// ─────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────
void loop() {
  updateServos();
  updateTask();
}
