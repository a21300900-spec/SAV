// anchor_mpc_bno085_arduino.ino
// Virtual anchor with MPC for ESP32 Dev Module, GPS SparkFun 17285, Adafruit BNO085, motor, and servo control.

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BNO08x.h>

// -------------------- Pin configuration --------------------
static constexpr uint8_t PIN_GPS_RX = 16;     // GPS TX -> ESP32 RX2
static constexpr uint8_t PIN_GPS_TX = 17;     // GPS RX -> ESP32 TX2
static constexpr uint32_t GPS_BAUD = 9600;

static constexpr uint8_t PIN_IMU_SDA = 21;
static constexpr uint8_t PIN_IMU_SCL = 22;
static constexpr uint8_t BNO085_ADDR = 0x4A;

static constexpr uint8_t PIN_JOY_VRX = 34;
static constexpr uint8_t PIN_JOY_VRY = 35;
static constexpr uint8_t PIN_JOY_SW  = 32;

static constexpr uint8_t PIN_MOTOR_PWM = 33;
static constexpr uint8_t PIN_MOTOR_DIR = 26;

static constexpr uint8_t PIN_SERVO_PWM = 25;

static constexpr uint8_t PIN_ANCHOR_BTN = 14;

// Reserved LoRa pins (unused yet)
static constexpr uint8_t PIN_LORA_MOSI = 23;
static constexpr uint8_t PIN_LORA_MISO = 19;
static constexpr uint8_t PIN_LORA_SCLK = 18;
static constexpr uint8_t PIN_LORA_CS   = 5;
static constexpr uint8_t PIN_LORA_RST  = 27;
static constexpr uint8_t PIN_LORA_DIO0 = 4;

// -------------------- LEDC configuration --------------------
static constexpr uint8_t CH_MOTOR = 0;
static constexpr uint8_t CH_SERVO = 1;
static constexpr uint32_t MOTOR_FREQ = 1800;    // Hz
static constexpr uint8_t MOTOR_RES_BITS = 13;    // >=13 bits
static constexpr uint32_t SERVO_FREQ = 50;       // Hz
static constexpr uint8_t SERVO_RES_BITS = 16;

static constexpr float DUTY_MIN = 0.07f;
static constexpr float DUTY_MAX = 0.94f;
static constexpr uint32_t MOTOR_GUARD_MS = 3000;

static constexpr uint16_t SERVO_US_MIN = 1000;
static constexpr uint16_t SERVO_US_NEU = 1500;
static constexpr uint16_t SERVO_US_MAX = 2000;
static constexpr float SERVO_DEADZONE = 0.05f;

// -------------------- Sensor and filter parameters --------------------
static constexpr uint16_t BNO_RATE_HZ = 100;
static constexpr float GPS_MIN_SPEED = 0.5f;   // m/s for GPS yaw update

// -------------------- MPC configuration --------------------
static constexpr float MPC_DT = 0.2f;
static constexpr int   MPC_NP = 20;
static constexpr float MPC_W_POS = 4.0f;
static constexpr float MPC_W_PSI = 1.0f;
static constexpr float MPC_W_U   = 0.1f;
static constexpr float MPC_W_DU  = 1.0f;
static constexpr float MPC_DELTA_MAX = 0.45f;   // rad
static constexpr float MPC_DELTA_MID = 0.25f;   // rad
static constexpr float MPC_SPEED_SCALE = 3.0f;  // convert duty fraction to m/s target

// Hysteresis thresholds
static constexpr float ANCHOR_ERR_HIGH = 0.6f;  // meters
static constexpr float ANCHOR_ERR_LOW  = 0.3f;  // meters
static constexpr uint8_t ANCHOR_HIGH_COUNT = 3;
static constexpr uint8_t ANCHOR_LOW_COUNT  = 5;

// -------------------- Global state structures --------------------
struct State {
  float px;
  float py;
  float psi;
  float v;
};

struct Ref {
  float px;
  float py;
  float psi;
};

struct Cmd {
  float delta;
  float tau;
  bool reverse;
};

// -------------------- Hardware objects --------------------
HardwareSerial SerialGPS(2);
TinyGPSPlus gps;
Adafruit_BNO08x bno(BNO08X_RESET);

// -------------------- Motor control globals --------------------
static bool motor_on = false;
static bool motor_dir_reverse = false;
static float motor_duty = 0.0f;  // 0-1 range

// -------------------- Hold and anchor state --------------------
static bool speed_hold_active = false;
static float speed_hold_duty = 0.0f;
static bool speed_hold_reverse = false;

static bool anchor_active = false;
static bool anchor_button_last = true;
static uint32_t anchor_button_last_ms = 0;
static double anchor_lat = 0.0;
static double anchor_lon = 0.0;
static float anchor_yaw = 0.0f;

// -------------------- Sensor data --------------------
static bool gps_fix = false;
static double gps_lat = 0.0;
static double gps_lon = 0.0;
static float gps_speed = 0.0f;
static float gps_cog = 0.0f;
static float enu_x = 0.0f;
static float enu_y = 0.0f;

static float latest_yaw = 0.0f;
static float latest_gyro_z = 0.0f;
static uint32_t last_imu_ms = 0;

// -------------------- Kalman filter --------------------
static float kf_x[2] = {0.0f, 0.0f};  // [psi, bias]
static float kf_P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static float kf_Q_psi = 1e-3f;
static float kf_Q_bias = 1e-5f;
static float kf_R_yaw = 1e-2f;
static float kf_R_gps = 4e-2f;

// -------------------- MPC command --------------------
static Cmd mpc_cmd = {0.0f, 0.0f, false};
static bool mpc_enabled = false;
static uint8_t anchor_high_counter = 0;
static uint8_t anchor_low_counter = 0;

// -------------------- Function declarations --------------------
void motorEnable(bool on);
bool motorIsOn();
bool motorGetDir();
float motorGetDuty01();
bool motorSetDuty01(float d01);
bool motorSetDir(bool reverse);

bool servoWriteUS(uint16_t us);
float readJoystickDir();
bool readHoldButton();
void updateAnchorButton();

bool bnoBegin(uint8_t addr = BNO085_ADDR, uint16_t rateHz = BNO_RATE_HZ);
bool bnoRead(float *yaw_rad, float *gyro_z_rad_s, uint32_t *ts_ms);

void kfInit(float q_psi, float q_bias, float r_yaw, float r_gps);
void kfPredict(float gyro_z, float dt);
void kfUpdateYaw(float yaw_meas);
void kfUpdateGPS(float cog_rad);
float kfGetYaw();

bool gpsPoll();
bool gpsGet(float *lat, float *lon, float *speed_mps, float *cog_rad, bool *fix);
void toENU(double lat, double lon, double lat0, double lon0, float *ex, float *ny);

void mpcStep(const State &x, const Ref &r, Cmd *out);

void taskSensors();
void taskInput();
void taskMPC();
void taskActuators();

float wrapAngle(float a);
float wrapDiff(float a);
float clampf(float v, float lo, float hi);

// -------------------- Motor control implementation --------------------
void motorEnable(bool on) {
  if (on == motor_on) {
    return;
  }
  if (!on) {
    ledcWrite(CH_MOTOR, 0);
    motor_duty = 0.0f;
    motor_on = false;
  } else {
    motor_on = true;
  }
}

bool motorIsOn() {
  return motor_on;
}

bool motorGetDir() {
  return motor_dir_reverse;
}

float motorGetDuty01() {
  return motor_duty;
}

bool motorSetDuty01(float d01) {
  if (d01 <= 0.0f) {
    motor_duty = 0.0f;
    ledcWrite(CH_MOTOR, 0);
    return true;
  }
  if (!motor_on) {
    motorEnable(true);
  }
  float duty = clampf(d01, DUTY_MIN, DUTY_MAX);
  uint32_t maxDuty = (1UL << MOTOR_RES_BITS) - 1;
  uint32_t dutyCounts = static_cast<uint32_t>(duty * maxDuty);
  ledcWrite(CH_MOTOR, dutyCounts);
  motor_duty = duty;
  return true;
}

bool motorSetDir(bool reverse) {
  if (reverse == motor_dir_reverse) {
    return true;
  }
  if (motor_duty > 0.0f) {
    motorSetDuty01(0.0f);
    delay(MOTOR_GUARD_MS);
  }
  motor_dir_reverse = reverse;
  digitalWrite(PIN_MOTOR_DIR, reverse ? HIGH : LOW);
  return true;
}

// -------------------- Servo implementation --------------------
bool servoWriteUS(uint16_t us) {
  if (us < SERVO_US_MIN) us = SERVO_US_MIN;
  if (us > SERVO_US_MAX) us = SERVO_US_MAX;
  uint32_t maxDuty = (1UL << SERVO_RES_BITS) - 1;
  float duty = (static_cast<float>(us) / 20000.0f) * maxDuty;
  ledcWrite(CH_SERVO, static_cast<uint32_t>(duty));
  return true;
}

// -------------------- Joystick input --------------------
float readJoystickDir() {
  static constexpr int AVG_SAMPLES = 16;
  uint32_t acc = 0;
  for (int i = 0; i < AVG_SAMPLES; ++i) {
    acc += analogRead(PIN_JOY_VRX);
  }
  float avg = static_cast<float>(acc) / AVG_SAMPLES;
  float normalized = (avg / 4095.0f) * 2.0f - 1.0f; // [-1, 1]
  if (fabsf(normalized) < SERVO_DEADZONE) {
    normalized = 0.0f;
  }
  return clampf(normalized, -1.0f, 1.0f);
}

bool readHoldButton() {
  static bool lastState = HIGH;
  static uint32_t lastDebounce = 0;
  bool state = digitalRead(PIN_JOY_SW);
  uint32_t now = millis();
  if (state != lastState) {
    if (now - lastDebounce >= 30) {
      lastDebounce = now;
      lastState = state;
      if (state == LOW) {
        speed_hold_active = !speed_hold_active;
        if (speed_hold_active) {
          speed_hold_duty = motor_duty;
          speed_hold_reverse = motor_dir_reverse;
        } else {
          speed_hold_duty = 0.0f;
        }
      }
    }
  }
  return speed_hold_active;
}

void updateAnchorButton() {
  bool state = digitalRead(PIN_ANCHOR_BTN);
  uint32_t now = millis();
  if (state != anchor_button_last) {
    if (now - anchor_button_last_ms >= 30) {
      anchor_button_last_ms = now;
      anchor_button_last = state;
      if (state == LOW && gps_fix) {
        anchor_active = !anchor_active;
        if (anchor_active) {
          anchor_lat = gps_lat;
          anchor_lon = gps_lon;
          anchor_yaw = latest_yaw;
          anchor_high_counter = 0;
          anchor_low_counter = 0;
          Serial.println(F("Anchor: stored current position."));
        } else {
          Serial.println(F("Anchor: disabled."));
        }
      }
    }
  }
}

// -------------------- BNO085 helpers --------------------
bool bnoBegin(uint8_t addr, uint16_t rateHz) {
  if (!bno.begin_I2C(addr)) {
    Serial.println(F("BNO085 init failed"));
    return false;
  }
  bno.enableReport(SH2_ROTATION_VECTOR, rateHz);
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, rateHz);
  last_imu_ms = millis();
  Serial.println(F("BNO085 ready"));
  return true;
}

bool bnoRead(float *yaw_rad, float *gyro_z_rad_s, uint32_t *ts_ms) {
  bool updated = false;
  sh2_SensorValue_t sensorValue;
  while (bno.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue.un.rotationVector.real;
      float qx = sensorValue.un.rotationVector.i;
      float qy = sensorValue.un.rotationVector.j;
      float qz = sensorValue.un.rotationVector.k;
      float yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
      latest_yaw = wrapAngle(yaw);
      updated = true;
    } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      latest_gyro_z = sensorValue.un.gyroscope.z;
    }
    last_imu_ms = millis();
  }
  if (updated) {
    if (yaw_rad) *yaw_rad = latest_yaw;
    if (gyro_z_rad_s) *gyro_z_rad_s = latest_gyro_z;
    if (ts_ms) *ts_ms = last_imu_ms;
    return true;
  }
  return false;
}

// -------------------- Kalman filter --------------------
void kfInit(float q_psi, float q_bias, float r_yaw, float r_gps) {
  kf_x[0] = 0.0f;
  kf_x[1] = 0.0f;
  kf_P[0][0] = 1.0f;
  kf_P[0][1] = 0.0f;
  kf_P[1][0] = 0.0f;
  kf_P[1][1] = 1.0f;
  kf_Q_psi = q_psi;
  kf_Q_bias = q_bias;
  kf_R_yaw = r_yaw;
  kf_R_gps = r_gps;
}

void kfPredict(float gyro_z, float dt) {
  if (dt <= 0.0f || dt > 0.2f) {
    return;
  }
  float psi = kf_x[0];
  float bias = kf_x[1];
  psi += (gyro_z - bias) * dt;
  psi = wrapAngle(psi);
  kf_x[0] = psi;
  // State transition F = [[1, -dt], [0, 1]]
  float P00 = kf_P[0][0];
  float P01 = kf_P[0][1];
  float P10 = kf_P[1][0];
  float P11 = kf_P[1][1];
  float F00 = 1.0f;
  float F01 = -dt;
  float F10 = 0.0f;
  float F11 = 1.0f;
  float Q00 = kf_Q_psi * dt;
  float Q11 = kf_Q_bias * dt;
  float newP00 = F00 * (F00 * P00 + F01 * P10) + F01 * (F00 * P01 + F01 * P11) + Q00;
  float newP01 = F00 * (F10 * P00 + F11 * P10) + F01 * (F10 * P01 + F11 * P11);
  float newP10 = F10 * (F00 * P00 + F01 * P10) + F11 * (F00 * P01 + F01 * P11);
  float newP11 = F10 * (F10 * P00 + F11 * P10) + F11 * (F10 * P01 + F11 * P11) + Q11;
  kf_P[0][0] = newP00;
  kf_P[0][1] = newP01;
  kf_P[1][0] = newP10;
  kf_P[1][1] = newP11;
}

void kfUpdateYaw(float yaw_meas) {
  float H0 = 1.0f;
  float H1 = 0.0f;
  float S = H0 * (H0 * kf_P[0][0] + H1 * kf_P[1][0]) + kf_R_yaw;
  if (S <= 0.0f) {
    return;
  }
  float K0 = (kf_P[0][0] * H0 + kf_P[0][1] * H1) / S;
  float K1 = (kf_P[1][0] * H0 + kf_P[1][1] * H1) / S;
  float y = wrapDiff(yaw_meas - kf_x[0]);
  kf_x[0] = wrapAngle(kf_x[0] + K0 * y);
  kf_x[1] = kf_x[1] + K1 * y;
  float P00 = kf_P[0][0];
  float P01 = kf_P[0][1];
  float P10 = kf_P[1][0];
  float P11 = kf_P[1][1];
  kf_P[0][0] = P00 - K0 * H0 * P00;
  kf_P[0][1] = P01 - K0 * H0 * P01;
  kf_P[1][0] = P10 - K1 * H0 * P00;
  kf_P[1][1] = P11 - K1 * H0 * P01;
}

void kfUpdateGPS(float cog_rad) {
  float H0 = 1.0f;
  float H1 = 0.0f;
  float S = H0 * (H0 * kf_P[0][0] + H1 * kf_P[1][0]) + kf_R_gps;
  if (S <= 0.0f) {
    return;
  }
  float K0 = (kf_P[0][0] * H0 + kf_P[0][1] * H1) / S;
  float K1 = (kf_P[1][0] * H0 + kf_P[1][1] * H1) / S;
  float y = wrapDiff(cog_rad - kf_x[0]);
  kf_x[0] = wrapAngle(kf_x[0] + K0 * y);
  kf_x[1] = kf_x[1] + K1 * y;
  float P00 = kf_P[0][0];
  float P01 = kf_P[0][1];
  float P10 = kf_P[1][0];
  float P11 = kf_P[1][1];
  kf_P[0][0] = P00 - K0 * H0 * P00;
  kf_P[0][1] = P01 - K0 * H0 * P01;
  kf_P[1][0] = P10 - K1 * H0 * P00;
  kf_P[1][1] = P11 - K1 * H0 * P01;
}

float kfGetYaw() {
  return wrapAngle(kf_x[0]);
}

// -------------------- GPS helpers --------------------
bool gpsPoll() {
  bool updated = false;
  while (SerialGPS.available() > 0) {
    char c = SerialGPS.read();
    if (gps.encode(c)) {
      updated = true;
    }
  }
  return updated;
}

bool gpsGet(float *lat, float *lon, float *speed_mps, float *cog_rad, bool *fix) {
  bool hasFix = gps.location.isValid() && gps.location.age() < 2000 && gps.hdop.isValid();
  bool fresh = gps.location.isUpdated() || gps.speed.isUpdated() || gps.course.isUpdated();
  if (fix) *fix = hasFix;
  if (!hasFix) {
    return false;
  }
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  float speed = gps.speed.mps();
  float cog = gps.course.deg() * DEG_TO_RAD;
  if (lat) *lat = latitude;
  if (lon) *lon = longitude;
  if (speed_mps) *speed_mps = speed;
  if (cog_rad) *cog_rad = cog;
  return fresh;
}

void toENU(double lat, double lon, double lat0, double lon0, float *ex, float *ny) {
  static constexpr double R = 6378137.0; // Earth radius
  double latRad = lat * DEG_TO_RAD;
  double lonRad = lon * DEG_TO_RAD;
  double lat0Rad = lat0 * DEG_TO_RAD;
  double lon0Rad = lon0 * DEG_TO_RAD;
  double dLat = latRad - lat0Rad;
  double dLon = lonRad - lon0Rad;
  double meanLat = 0.5 * (latRad + lat0Rad);
  float east = static_cast<float>(R * dLon * cos(meanLat));
  float north = static_cast<float>(R * dLat);
  if (ex) *ex = east;
  if (ny) *ny = north;
}

// -------------------- MPC implementation --------------------
void mpcStep(const State &x, const Ref &r, Cmd *out) {
  static const float deltaOptions[5] = {
    -MPC_DELTA_MAX, -MPC_DELTA_MID, 0.0f, MPC_DELTA_MID, MPC_DELTA_MAX
  };
  static const float tauOptions[7] = {0.0f, 0.08f, 0.12f, 0.16f, 0.24f, 0.32f, 0.40f};
  float bestCost = 1e9f;
  Cmd best = {0.0f, 0.0f, false};
  float currentDelta = 0.0f;
  float currentTau = 0.0f;
  for (int revIdx = 0; revIdx < 2; ++revIdx) {
    bool reverse = (revIdx == 1);
    if (!motor_on && reverse) {
      // prefer forward when starting from stop unless reverse explicitly better
    }
    for (float deltaCmd : deltaOptions) {
      for (float tauCmd : tauOptions) {
        State s = x;
        float prevDelta = currentDelta;
        float prevTau = currentTau;
        float cost = 0.0f;
        for (int k = 0; k < MPC_NP; ++k) {
          float stepDelta = deltaCmd;
          float tau = tauCmd;
          float velSign = reverse ? -1.0f : 1.0f;
          float v_body = s.v * velSign;
          float beta = stepDelta; // small-angle approximation
          s.px += v_body * cosf(s.psi) * MPC_DT;
          s.py += v_body * sinf(s.psi) * MPC_DT;
          s.psi = wrapAngle(s.psi + v_body * tanf(beta) * MPC_DT * 0.8f);
          float targetSpeed = tau * MPC_SPEED_SCALE;
          float accel = (targetSpeed - s.v) * 0.5f;
          s.v = clampf(s.v + accel * MPC_DT, 0.0f, 4.0f);
          float ex = s.px - r.px;
          float ey = s.py - r.py;
          float errPsi = wrapDiff(s.psi - r.psi);
          cost += MPC_W_POS * (ex * ex + ey * ey);
          cost += MPC_W_PSI * (errPsi * errPsi);
          cost += MPC_W_U * (stepDelta * stepDelta + tau * tau);
          float dDelta = stepDelta - prevDelta;
          float dTau = tau - prevTau;
          cost += MPC_W_DU * (dDelta * dDelta + dTau * dTau);
          prevDelta = stepDelta;
          prevTau = tau;
        }
        if (cost < bestCost) {
          bestCost = cost;
          best.delta = deltaCmd;
          best.tau = tauCmd;
          best.reverse = reverse;
        }
      }
    }
  }
  if (out) {
    *out = best;
  }
}

// -------------------- Utility functions --------------------
float wrapAngle(float a) {
  while (a > PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

float wrapDiff(float a) {
  return wrapAngle(a);
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// -------------------- Task implementations --------------------
void taskSensors() {
  float yaw;
  float gyro;
  uint32_t ts;
  bool hasYaw = bnoRead(&yaw, &gyro, &ts);
  static uint32_t lastPredictMs = millis();
  uint32_t nowMs = millis();
  float dt = (nowMs - lastPredictMs) * 0.001f;
  if (dt > 0.0005f) {
    if (dt > 0.05f) dt = 0.05f;
    kfPredict(latest_gyro_z, dt);
    lastPredictMs = nowMs;
  }
  if (hasYaw) {
    kfUpdateYaw(yaw);
  }
  if (gpsPoll()) {
    bool fix;
    if (gpsGet(&gps_lat, &gps_lon, &gps_speed, &gps_cog, &fix)) {
      gps_fix = fix;
      if (gps_fix) {
        if (anchor_active) {
          toENU(gps_lat, gps_lon, anchor_lat, anchor_lon, &enu_x, &enu_y);
        }
        if (gps_speed > GPS_MIN_SPEED) {
          kfUpdateGPS(gps_cog);
        }
      }
    } else {
      gps_fix = fix;
    }
  }
}

void taskInput() {
  readHoldButton();
  updateAnchorButton();
}

void taskMPC() {
  if (!anchor_active || !gps_fix) {
    mpc_enabled = false;
    mpc_cmd = {0.0f, 0.0f, motor_dir_reverse};
    return;
  }
  State x;
  x.px = enu_x;
  x.py = enu_y;
  x.psi = kfGetYaw();
  x.v = gps_speed;
  Ref r;
  r.px = 0.0f;
  r.py = 0.0f;
  r.psi = anchor_yaw;
  mpcStep(x, r, &mpc_cmd);
  float err = sqrtf(x.px * x.px + x.py * x.py);
  if (err > ANCHOR_ERR_HIGH) {
    if (anchor_high_counter < 255) anchor_high_counter++;
    anchor_low_counter = 0;
  } else if (err < ANCHOR_ERR_LOW) {
    if (anchor_low_counter < 255) anchor_low_counter++;
    if (anchor_high_counter > 0) anchor_high_counter--;
  } else {
    anchor_high_counter = 0;
    anchor_low_counter = 0;
  }
  if (anchor_high_counter >= ANCHOR_HIGH_COUNT) {
    mpc_enabled = true;
  }
  if (anchor_low_counter >= ANCHOR_LOW_COUNT) {
    mpc_enabled = false;
  }
}

void taskActuators() {
  float joystick = readJoystickDir();
  float servoTargetUs = SERVO_US_NEU;
  bool manualServo = fabsf(joystick) > 0.001f;
  if (mpc_enabled && !manualServo) {
    float delta = clampf(mpc_cmd.delta, -MPC_DELTA_MAX, MPC_DELTA_MAX);
    float us = SERVO_US_NEU + (delta / MPC_DELTA_MAX) * 450.0f;
    servoTargetUs = static_cast<uint16_t>(SERVO_US_NEU + clampf(us - SERVO_US_NEU, -450.0f, 450.0f));
  } else {
    servoTargetUs = SERVO_US_NEU + static_cast<int16_t>(joystick * 450.0f);
  }
  servoWriteUS(static_cast<uint16_t>(servoTargetUs));

  bool hold = speed_hold_active;
  float desiredDuty = 0.0f;
  bool desiredReverse = motor_dir_reverse;

  if (hold) {
    desiredDuty = speed_hold_duty;
    desiredReverse = speed_hold_reverse;
  } else if (mpc_enabled) {
    desiredDuty = mpc_cmd.tau;
    desiredReverse = mpc_cmd.reverse;
  } else {
    desiredDuty = 0.0f;
    desiredReverse = motor_dir_reverse;
  }

  if (!gps_fix) {
    desiredDuty = 0.0f;
  }

  if (desiredDuty <= 0.0f) {
    motorSetDuty01(0.0f);
  } else {
    if (motor_dir_reverse != desiredReverse) {
      motorSetDir(desiredReverse);
    }
    motorSetDuty01(desiredDuty);
  }

  if (motor_duty > 0.0f) {
    motorEnable(true);
  } else {
    motorEnable(false);
  }
}

// -------------------- Setup and loop --------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println(F("Virtual anchor MPC starting..."));

  pinMode(PIN_MOTOR_DIR, OUTPUT);
  digitalWrite(PIN_MOTOR_DIR, LOW);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  pinMode(PIN_ANCHOR_BTN, INPUT_PULLUP);

  analogReadResolution(12);

  ledcSetup(CH_MOTOR, MOTOR_FREQ, MOTOR_RES_BITS);
  ledcAttachPin(PIN_MOTOR_PWM, CH_MOTOR);
  ledcWrite(CH_MOTOR, 0);

  ledcSetup(CH_SERVO, SERVO_FREQ, SERVO_RES_BITS);
  ledcAttachPin(PIN_SERVO_PWM, CH_SERVO);
  servoWriteUS(SERVO_US_NEU);

  Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL, 400000);
  Wire.setClock(400000);

  if (!bnoBegin()) {
    Serial.println(F("IMU not detected, continuing with fallback yaw"));
  }

  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Serial.println(F("GPS serial started"));

  kfInit(1e-3f, 1e-5f, 5e-2f, 1.5e-1f);
}

void loop() {
  static uint32_t lastSensors = 0;
  static uint32_t lastInput = 0;
  static uint32_t lastMPC = 0;
  static uint32_t lastAct = 0;
  uint32_t now = millis();

  if (now - lastSensors >= 10) { // ~100 Hz
    taskSensors();
    lastSensors = now;
  }
  if (now - lastInput >= 20) {
    taskInput();
    lastInput = now;
  }
  if (now - lastMPC >= 200) { // 5 Hz
    taskMPC();
    lastMPC = now;
    Serial.print(F("State: " ));
    Serial.print(enu_x, 2);
    Serial.print(F(","));
    Serial.print(enu_y, 2);
    Serial.print(F(" m | yaw="));
    Serial.print(kfGetYaw(), 2);
    Serial.print(F(" rad | v="));
    Serial.print(gps_speed, 2);
    Serial.print(F(" m/s | duty="));
    Serial.print(motor_duty, 2);
    Serial.print(F(" | reverse="));
    Serial.print(motor_dir_reverse ? F("Y") : F("N"));
    Serial.print(F(" | mpc="));
    Serial.println(mpc_enabled ? F("ON") : F("OFF"));
  }
  if (now - lastAct >= 20) {
    taskActuators();
    lastAct = now;
  }
}

// Upload steps (Arduino IDE 2.x): select "ESP32 Dev Module", choose serial port, set Upload Speed 921600, then click Upload.
// Example log trace: "Virtual anchor MPC starting..." -> "BNO085 ready" -> "GPS serial started" -> "Anchor: stored current position." -> "State: 0.95,0.40 m | yaw=0.12 rad | v=0.32 m/s | duty=0.16 | reverse=N | mpc=ON" -> (error grows) -> motor duty goes to 0, wait 3000 ms, "State: ... | reverse=Y | mpc=ON" (safe inversion).
