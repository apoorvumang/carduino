#include <Arduino.h>
#include <BLEGamepadClient.h>

XboxController controller;

// ====== Pin wiring (your setup) ======
static const int PIN_THROTTLE_L = 26;  // Left motor throttle (DAC2)
static const int PIN_DIR_L = 27;       // Left motor direction (digital)

static const int PIN_THROTTLE_R = 25;  // Right motor throttle (DAC1)
static const int PIN_DIR_R = 21;       // Right motor direction (digital)

// ====== Direction logic ======
static const bool DIR_HIGH_IS_FORWARD = true;

// If one motor is physically flipped (common in differential drive),
// set these to true to invert just that motor.
static const bool INVERT_LEFT_MOTOR = false;
static const bool INVERT_RIGHT_MOTOR = true;

// ====== Controls tuning ======
static const float DEADZONE_THROTTLE = 0.08f;  // stick deadzone
static const float DEADZONE_TURN = 0.08f;

static const float MAX_CMD = 1.00f;  // clamp commands to [-1..1]
static const uint8_t DAC_MAX = 255;  // full-scale DAC output (0..255)

// How “strong” turning is relative to throttle.
// 1.0 = full mix, 0.6 = gentler steering.
static const float TURN_GAIN = 0.85f;

// In-place turn speed when button held (0..1)
static const float PIVOT_SPEED = 0.65f;

// Simple ramp limiting (smooths jerk)
static const float SLEW_PER_LOOP = 0.06f;  // max change per loop (0..1). Lower = smoother

// ====== helpers ======
static float applyDeadzone(float x, float dz) {
  if (fabsf(x) < dz) return 0.0f;
  // Rescale so it ramps from 0 at dz to 1 at 1
  float sign = (x >= 0) ? 1.0f : -1.0f;
  float mag = (fabsf(x) - dz) / (1.0f - dz);
  if (mag > 1.0f) mag = 1.0f;
  return sign * mag;
}

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float slewLimit(float target, float current, float step) {
  float delta = target - current;
  if (delta > step) delta = step;
  if (delta < -step) delta = -step;
  return current + delta;
}

static void setMotor(int pinThrottle, int pinDir, float cmd, bool invertMotor) {
  // cmd is [-1..1], sign = direction, magnitude = throttle
  cmd = clampf(cmd, -1.0f, 1.0f);
  if (invertMotor) cmd = -cmd;

  bool forward = (cmd >= 0.0f);
  float mag = fabsf(cmd);  // 0..1

  // Direction pin
  bool dirLevel = DIR_HIGH_IS_FORWARD ? forward : !forward;
  digitalWrite(pinDir, dirLevel ? HIGH : LOW);

  // Throttle to DAC
  uint8_t dacVal = (uint8_t)lroundf(mag * (float)DAC_MAX);
  dacWrite(pinThrottle, dacVal);
}

void stopAll() {
  // 0 throttle on both motors (keep dir pins as-is, doesn’t matter)
  dacWrite(PIN_THROTTLE_L, 0);
  dacWrite(PIN_THROTTLE_R, 0);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nBooting...");

  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);

  // safe defaults
  digitalWrite(PIN_DIR_L, DIR_HIGH_IS_FORWARD ? HIGH : LOW);
  digitalWrite(PIN_DIR_R, DIR_HIGH_IS_FORWARD ? HIGH : LOW);
  stopAll();

  controller.begin();
  Serial.println("BLEGamepadClient started. Put Xbox controller in pairing mode.");
}

void loop() {
  static bool lastConnected = false;

  bool connected = controller.isConnected();
  if (connected != lastConnected) {
    lastConnected = connected;
    Serial.println(connected ? "✅ Controller connected!" : "❌ Controller disconnected.");
  }

  if (!connected) {
    stopAll();
    delay(120);
    return;
  }

  XboxControlsState s;
  controller.read(&s);

  // Sticks in your print look like -1..+1 floats
  // You wanted:
  // - Right joystick: throttle (forward/back)
  // - Left joystick: direction (left/right)
  float throttle = -s.rightStickY;  // invert so pushing stick forward = +throttle
  float turn = s.leftStickX;

  throttle = applyDeadzone(throttle, DEADZONE_THROTTLE);
  turn = applyDeadzone(turn, DEADZONE_TURN);

  // In-place turning with buttons:
  // X = spin left, B = spin right
  if (s.buttonX) {
    throttle = 0.0f;
    turn = -PIVOT_SPEED;
  } else if (s.buttonB) {
    throttle = 0.0f;
    turn = +PIVOT_SPEED;
  }

  // Differential drive mix
  // Basic mix:
  // left  = throttle + turn
  // right = throttle - turn
  //
  // TURN_GAIN tames steering.
  float leftTarget = throttle + (turn * TURN_GAIN);
  float rightTarget = throttle - (turn * TURN_GAIN);

  // Normalize so neither exceeds [-1..1]
  float maxMag = fmaxf(fabsf(leftTarget), fabsf(rightTarget));
  if (maxMag > MAX_CMD) {
    leftTarget /= maxMag;
    rightTarget /= maxMag;
  }

  // Slew limiting for smoothness
  static float leftCmd = 0.0f;
  static float rightCmd = 0.0f;
  leftCmd = slewLimit(leftTarget, leftCmd, SLEW_PER_LOOP);
  rightCmd = slewLimit(rightTarget, rightCmd, SLEW_PER_LOOP);

  // Output to motors
  setMotor(PIN_THROTTLE_L, PIN_DIR_L, leftCmd, INVERT_LEFT_MOTOR);
  setMotor(PIN_THROTTLE_R, PIN_DIR_R, rightCmd, INVERT_RIGHT_MOTOR);

  // Debug (optional)
  Serial.printf("thr=%.2f turn=%.2f | L=%.2f R=%.2f | X:%d B:%d\n",
                throttle, turn, leftCmd, rightCmd, s.buttonX, s.buttonB);

  delay(40);
}
