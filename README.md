# Robotic-Arm (ESP32 + PCA9685)

Robotic arm firmware for **THE-V** waste sorting flow:
- Detect **Paper** or **Plastic** from app
- Trigger automated pick-and-drop sequence
- Ignore duplicate app detections while sequence is running
- Web dashboard for manual control + preset calibration

---

## Current Behavior (v3)

### Waste classes
- `0` = **Paper**
- `1` = **Plastic**

### Sequence flow
For each detection/manual trigger:
1. Raise to transit height
2. Move to pickup preset
3. Close gripper
4. Transit with object
5. Move to drop preset
6. Open gripper
7. Return to transit

### Duplicate-signal protection
If app keeps sending detections repeatedly:
- While sequence is running → new `/update` requests are ignored (`reason: busy`)
- After completion → short re-arm delay ignores trailing spam (`reason: rearming`)

This prevents double-pick for one object.

---

## Servo Configuration (current)

```cpp
ServoConfig servos[NUM_SERVOS] = {
  //   name         icon   subtitle               ch           min  max  home
  { "Base",     "🔄", "Rotation / Yaw",      CH_BASE,       0, 180,  90 },
  { "Shoulder", "💪", "Joint 1 / Lift",      CH_SHOULDER,   0, 180, 180 },
  { "Elbow",    "🦾", "Joint 2 / Reach",     CH_ELBOW,      0, 180,   0 },
  { "Wrist",    "🤚", "Joint 3 / Tilt",      CH_WRIST,     30, 180,  90 },
  { "Gripper",  "✊", "End Effector",        CH_GRIPPER,    0,  90,  30 },
};

int startupAngles[NUM_SERVOS] = { 90, 180, 0, 90, 30 };
```

### Important tuning constants
```cpp
#define SERVO_SPEED_STARTUP   30   // boot sweep speed
#define SERVO_SPEED           60   // runtime speed

#define GRIPPER_OPEN           0
#define GRIPPER_CLOSE         90

#define SEQ_GRIP_SETTLE_MS   900
#define SEQ_RELEASE_SETTLE_MS 700
#define SEQ_REARM_DELAY_MS  1200
```

---

## Presets (Calibrate in dashboard, then hard-code)

Current placeholders:
```cpp
Preset presets[4] = {
  { "pickup1", "Paper — Pickup",   { 45, 130,  60,  90, 0 } },
  { "pickup2", "Plastic — Pickup", {135, 130,  60,  90, 0 } },
  { "drop1",   "Paper — Drop",     { 45, 100,  80,  90, 0 } },
  { "drop2",   "Plastic — Drop",   {135, 100,  80,  90, 0 } },
};

int transitAngles[NUM_SERVOS] = { 90, 160, 20, 90, 0 };
```

### Calibration workflow
1. Open dashboard (`http://<esp-ip>/`)
2. Move arm with sliders to desired pose
3. Click **Save Current** for each preset
4. Test via manual Pick & Drop buttons
5. When stable, copy values into firmware `presets[]` and flash

---

## Network + API

ESP32 now uses **friend-style AP + optional STA dual mode**:

- **AP always ON** (stable app fallback)
  - SSID: `WasteBin_AP`
  - Password: `12345678`
  - IP: `192.168.4.1` (fixed)
- **STA optional** (`ENABLE_STA=true` by default)
  - Router SSID: `Motridox`
  - Router Password: `Bassim@8371`

So your friend can either:
1. Connect phone directly to ESP AP (`192.168.4.1`), or
2. Use router path when STA is connected.

### Endpoints
- `GET /` → Web dashboard
- `GET /ping` → Health check
- `GET /config` → Servo + preset config for UI
- `GET /status` → Live state (`busy`, task label, angles, last detection)
- `POST /update` → **App detection trigger**
- `GET /pickdrop?obj=0|1` → Manual trigger (0 Paper, 1 Plastic)
- `GET /stop_seq` → Emergency stop
- `GET /set?ch=<0..4>&angle=<deg>` → Manual joint target
- `GET /home` → Move to home angles
- `GET /goto_preset?id=<0..3>` → Move to preset
- `GET /save_preset?id=...&a0=...&a1=...&a2=...&a3=...&a4=...` → Save preset in RAM

---

## App Payload Format (recommended)

### Preferred (from app)
```json
{ "bin_type": 0 }
```
or
```json
{ "bin_type": 1 }
```

### Also accepted (fallback while app is evolving)
- JSON fields containing text like `paper` / `plastic`
  - keys checked: `status`, `type`, `label`, `material`, `waste_type`, `bin_name`, `class`
- Raw text body: `"0"`, `"1"`, `"paper"`, `"plastic"`
- `"metal"` is temporarily mapped to **paper** for compatibility

---

## Quick test commands

For direct AP mode, use `192.168.4.1`.
(If using router STA mode, replace `<ESP_IP>` with router-assigned ESP IP.)

### Ping
```bash
curl -s http://<ESP_IP>/ping
```

### Trigger Paper
```bash
curl -s -X POST http://<ESP_IP>/update \
  -H 'Content-Type: application/json' \
  -d '{"bin_type":0}'
```

### Trigger Plastic
```bash
curl -s -X POST http://<ESP_IP>/update \
  -H 'Content-Type: application/json' \
  -d '{"bin_type":1}'
```

### Live status
```bash
curl -s http://<ESP_IP>/status
```

---

## Bring-up checklist (send this to app dev)

1. ESP32 and phone are on same Wi-Fi
2. App hits `GET /ping` first
3. On detection, app sends `POST /update` with `bin_type` (0/1)
4. App should treat response as authoritative:
   - `accepted=true` → sequence started
   - `ignored=true, reason=busy|rearming` → do nothing/retry later
5. Optional: app polls `/status` for UI feedback

---

## Notes
- Keep mechanics safe: start with slow speeds and wide clearances.
- If arm knocks objects while moving between stations, increase `transitAngles` shoulder/elbow lift.
- Do not remove hard servo min/max limits unless re-calibrated physically.
