/*
  XIAO ESP32-C6 Quadruped + Direct-Servo Arm + Claw Controller (PCA9685 + UDP)
  + IMU Stabilization Overlay + HC-SR04 Front Obstacle Freeze
  ------------------------------------------------------------------------------
  Surgical integration goals preserved from the original controller:
    - Quadruped walking gait logic preserved
    - Demo / autonomous time-based sequence preserved
    - Wi-Fi UDP command/control preserved
    - IMU leveling overlay preserved
    - Direct arm servo control preserved

  Added HC-SR04 behavior:
    - Front-mounted sonar on D0 (TRIG) and D1 (ECHO)
    - Stop threshold declared near the top of the sketch
    - When obstacle distance falls below threshold, robot FREEZES in-place
      by holding all servos at their last commanded angles
    - Demo mode clock is paused while frozen, then resumes from the same time
    - UDP packets received while frozen are ignored (not queued)
    - In manual mode, after unfreeze, robot only executes commands received
      after unfreezing; otherwise legs return to idle stance while the arm
      remains at its last pose until explicit command "0"

  PATCH (arm demo interpolation):
    - Demo arm steps now interpolate smoothly from current armActive to
      the next demo target over DEMO_ARM_INTERP_DURATION_SEC using the
      same easeInOut mechanism as the "0" home-return transition.
    - Only the arm-in-demo logic is changed. Everything else is unchanged.

  Notes:
    - Leg servos still use 0..270 deg with 135 deg center
    - Arm servos and claw use 0..180 deg with 45 deg center
    - IMU leveling overlay is applied only to the legs
    - Pin naming and setup sequence aligned to the user's XIAO ESP32-C6 adapted sketch
*/

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// -------------------- DEBUG --------------------
#define DEBUG_RX 1
#define DEBUG_IMU 0

// -------------------- WALK SPEED --------------------
static const float WALK_SPEED_DEFAULT_MULT = 2.50f;
static const float WALK_SPEED_MIN_MULT     = 0.25f;
static const float WALK_SPEED_MAX_MULT     = 3.00f;
static const float WALK_SPEED_STEP_MULT    = 0.10f;

enum Command : uint8_t {
  IDLE = 0,
  FORWARD = 1,
  BACKWARD = 2,
  TURN_LEFT = 3,
  TURN_RIGHT = 4
};

enum GaitMode : uint8_t {
  GAIT_TROT = 0,
  GAIT_CRAWL = 1
};

struct Keyframe;
struct LegState;

static void advanceToNextNonzero(LegState& L);
static void tickLeg(LegState& L, float dtSec);
static void setCommandTrajectories(Command cmd);
static float readSonarDistanceCm();
static float computeSonarMotionScale(float distanceCm);
static void updateSonarObstacleState();
static void onSonarFreezeChanged(bool freezeNow);

// ===============================
// 1) WiFi / UDP CONFIG (multi-AP auto-connect)
// ===============================
WiFiMulti wifiMulti;

#define WIFI_NET1_SSID "Tazwar's HONOR X9b 5G"
#define WIFI_NET1_PASS "b3qivk5g_z37utr76"

#define WIFI_NET2_SSID "Error404"
#define WIFI_NET2_PASS "12345670"

#define WIFI_NET3_SSID "TP-Link_729E"
#define WIFI_NET3_PASS "27278178"

const int UDP_PORT = 5555;

WiFiUDP udp;
char rxBuf[128];

static const uint32_t CMD_TIMEOUT_MS = 1000;
static uint32_t lastWiFiCheckMs = 0;
static const uint32_t WIFI_CHECK_INTERVAL_MS = 1000;

// ===============================
// 2) I2C BUS (Shared for XIAO ESP32-C6)
// ===============================
static const int I2C_SDA = D4;
static const int I2C_SCL = D5;

// ===============================
// 2B) HC-SR04 FRONT SONAR CONFIG
// ===============================
static const int SONAR_TRIG_PIN = D0;
static const int SONAR_ECHO_PIN = D1;

static const float SONAR_SLOW_START_DISTANCE_CM = 60.0f;
static const float SONAR_STOP_DISTANCE_CM       = 30.0f;

static const uint32_t SONAR_SAMPLE_INTERVAL_MS = 50;
static const uint32_t SONAR_ECHO_TIMEOUT_US    = 30000;
static const float SONAR_SOUND_CM_PER_US       = 0.0343f;
static const float SONAR_NO_ECHO_DISTANCE_CM   = 999.0f;

// ===============================
// 3) PCA9685 / SERVO CONFIG
// ===============================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

static const int SERVO_FREQ = 50;
static const int MIN_US = 550;
static const int MAX_US = 2500;

static const float LEG_SERVO_CENTER_DEG = 135.0f;
static const float LEG_SERVO_MIN_DEG    = 0.0f;
static const float LEG_SERVO_MAX_DEG    = 270.0f;

static const float ARM_SERVO_CENTER_DEG = 45.0f;
static const float ARM_SERVO_MIN_DEG    = 0.0f;
static const float ARM_SERVO_MAX_DEG    = 180.0f;

// ===============================
// 4) ROBOT CONFIG
// ===============================
static float R1 = 1.0f;
static float R2 = 1.0f;

static const float REST_X = 1.8f;
static const float REST_Y = 0.0f;
static const float REST_Z = 0.0f;

static const float LAT_MIN_DEG      = -134.0f;
static const float LAT_MAX_DEG      = +134.0f;
static const float SHOULDER_MIN_DEG = -134.0f;
static const float SHOULDER_MAX_DEG = +134.0f;
static const float ELBOW_MIN_DEG    = -134.0f;
static const float ELBOW_MAX_DEG    = +134.0f;

static const bool MODEL_USES_RELATIVE_ELBOW_ANGLE = true;

static int8_t LAT_SIGN[4]      = { +1, -1, -1, +1 };
static int8_t SHOULDER_SIGN[4] = { -1, +1, +1, -1 };
static int8_t ELBOW_SIGN[4]    = { +1, -1, -1, +1 };

static float LAT_OFFSET_DEG[4]      = { 13.0, -32.0, -30.0, 13.0 };
static float SHOULDER_OFFSET_DEG[4] = { -3.0, -10.0, -2.0, -18.0 };
static float ELBOW_OFFSET_DEG[4]    = { -10.0, -13.0, -6.0, -13.0 };

static uint8_t SERVO_CH[4][3] = {
  { 0,  1,  2 },
  { 3,  4,  5 },
  { 6,  7,  8 },
  { 9, 10, 11 }
};

struct Vec3 { float x, y, z; };

// ===============================
// 4B) ARM CONFIG (DIRECT SERVO CONTROL)
// ===============================
static const uint8_t ARM_LAT_CH      = 12;
static const uint8_t ARM_SHOULDER_CH = 13;
static const uint8_t ARM_ELBOW_CH    = 14;
static const uint8_t ARM_CLAW_CH     = 15;

static const float ARM_LAT_REL_MIN_DEG      = -45.0f;
static const float ARM_LAT_REL_MAX_DEG      = +135.0f;
static const float ARM_SHOULDER_REL_MIN_DEG = -180.0f;
static const float ARM_SHOULDER_REL_MAX_DEG = 0.0f;
static const float ARM_ELBOW_REL_MIN_DEG    = 0.0f;
static const float ARM_ELBOW_REL_MAX_DEG    = +135.0f;
static const float ARM_CLAW_REL_MIN_DEG     = -40.0f;
static const float ARM_CLAW_REL_MAX_DEG     = +64.0f;

static const float ARM_STEP_DEG = 1.0f;
static const float ARM_SLEW_DEG_PER_SEC = 35.0f;

// ---- PATCH 1: Duration for pose/home transitions ----
static float POSE_INTERP_DURATION_SEC = 1.20f;

// ---- PATCH 1: Duration for demo arm step interpolation (slow & smooth) ----
static const float DEMO_ARM_INTERP_DURATION_SEC = 3.0f;

static const int TOTAL_SERVO_CHANNELS = 16;
static float lastServoDeg[TOTAL_SERVO_CHANNELS] = {0};
static bool  lastServoDegValid[TOTAL_SERVO_CHANNELS] = {false};

static float ARM_LAT_OFFSET_DEG      = 79.0f;
static float ARM_SHOULDER_OFFSET_DEG = 102.0f;
static float ARM_ELBOW_OFFSET_DEG    = -43.0f;
static float ARM_CLAW_OFFSET_DEG     = 23.0f;

struct ArmState {
  float latRelDeg;
  float shoulderRelDeg;
  float elbowRelDeg;
  float clawRelDeg;
};

// ===============================
// 5) GAIT TIMING
// ===============================
static int   INTERP_POINTS_BASE = 48;
static int   MICROSTEP_MS       = 10;
static float GAIT_SPEED         = WALK_SPEED_DEFAULT_MULT;
static const float GAIT_SPEED_MIN  = WALK_SPEED_MIN_MULT;
static const float GAIT_SPEED_MAX  = WALK_SPEED_MAX_MULT;
static const float GAIT_SPEED_STEP = WALK_SPEED_STEP_MULT;

static float CMD_SPEED[5] = { 1.0f,1.0f,1.0f,1.0f,1.0f };
static const uint32_t KEY_DEBOUNCE_MS = 250;

static const char* CMD_NAME[5] = {
  "IDLE", "FORWARD", "BACKWARD", "TURN_LEFT", "TURN_RIGHT"
};

static const char* GAIT_MODE_NAME[2] = {
  "TROT", "CRAWL"
};

static GaitMode gaitMode = GAIT_TROT;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline float rad2deg(float r) { return r * 57.29577951308232f; }

static inline bool vecAlmostEq(const Vec3& a, const Vec3& b, float eps=1e-6f) {
  return (fabsf(a.x - b.x) <= eps) && (fabsf(a.y - b.y) <= eps) && (fabsf(a.z - b.z) <= eps);
}
static inline float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (fabsf(in_max - in_min) < 1e-9f) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// ===============================
// 6) EXPLICIT GAIT KEYFRAMES
// ===============================
struct Keyframe { float x, y, z, scale; };
#define LEN(a) (int)(sizeof(a)/sizeof(a[0]))

// ---- TROT GAITS ----
static const Keyframe TROT_FORWARD_FR[] = {{1.8f,0.1f,0.0f,1.0f},{1.3f,-0.2f,0.0f,1.0f},{1.8f,-0.5f,0.0f,1.0f},{1.8f,-0.2f,0.0f,1.0f}};
static const Keyframe TROT_FORWARD_FL[] = {{1.8f,-0.5f,0.0f,1.0f},{1.8f,-0.2f,0.0f,1.0f},{1.8f,0.1f,0.0f,1.0f},{1.3f,-0.2f,0.0f,1.0f}};
static const Keyframe TROT_FORWARD_BL[] = {{1.8f,0.6f,0.0f,1.0f},{1.3f,0.3f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.3f,0.0f,1.0f}};
static const Keyframe TROT_FORWARD_BR[] = {{1.8f,0.0f,0.0f,1.0f},{1.8f,0.3f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f},{1.3f,0.3f,0.0f,1.0f}};

static const Keyframe TROT_BACKWARD_FR[] = {{1.8f,-0.6f,0.0f,1.0f},{1.4f,-0.3f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,-0.3f,0.0f,1.0f}};
static const Keyframe TROT_BACKWARD_FL[] = {{1.8f,0.0f,0.0f,1.0f},{1.8f,-0.3f,0.0f,1.0f},{1.8f,-0.6f,0.0f,1.0f},{1.4f,-0.3f,0.0f,1.0f}};
static const Keyframe TROT_BACKWARD_BL[] = {{1.8f,0.0f,0.0f,1.0f},{1.4f,0.3f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f},{1.8f,0.3f,0.0f,1.0f}};
static const Keyframe TROT_BACKWARD_BR[] = {{1.8f,0.6f,0.0f,1.0f},{1.8f,0.3f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.4f,0.3f,0.0f,1.0f}};

static const Keyframe TROT_TURN_LEFT_FR[] = {{1.7f,0.0f,0.3f,1.0f},{1.7f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.0f,-0.1f,1.0f},{1.8f,0.0f,-0.2f,1.0f},{1.8f,0.0f,-0.3f,1.0f},{1.4f,0.0f,-0.1f,1.0f}};
static const Keyframe TROT_TURN_LEFT_FL[] = {{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.3f,1.0f},{1.4f,0.0f,0.1f,1.0f},{1.8f,0.0f,-0.3f,1.0f},{1.7f,0.0f,-0.2f,1.0f},{1.7f,0.0f,-0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f}};
static const Keyframe TROT_TURN_LEFT_BL[] = {{1.8f,0.2f,0.3f,1.0f},{1.4f,0.2f,0.1f,1.0f},{1.8f,0.2f,-0.3f,1.0f},{1.8f,0.2f,-0.2f,1.0f},{1.8f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,0.1f,1.0f},{1.7f,0.2f,0.2f,1.0f}};
static const Keyframe TROT_TURN_LEFT_BR[] = {{1.8f,0.2f,0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,-0.1f,1.0f},{1.7f,0.2f,-0.2f,1.0f},{1.8f,0.2f,-0.3f,1.0f},{1.4f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.3f,1.0f},{1.8f,0.2f,0.2f,1.0f}};

static const Keyframe TROT_TURN_RIGHT_FR[] = {{1.7f,0.0f,-0.3f,1.0f},{1.7f,0.0f,-0.2f,1.0f},{1.8f,0.0f,-0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.3f,1.0f},{1.4f,0.0f,0.1f,1.0f}};
static const Keyframe TROT_TURN_RIGHT_FL[] = {{1.8f,0.0f,-0.1f,1.0f},{1.8f,0.0f,-0.2f,1.0f},{1.8f,0.0f,-0.3f,1.0f},{1.4f,0.0f,-0.1f,1.0f},{1.7f,0.0f,0.3f,1.0f},{1.7f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f}};
static const Keyframe TROT_TURN_RIGHT_BL[] = {{1.8f,0.2f,-0.3f,1.0f},{1.4f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.3f,1.0f},{1.8f,0.2f,0.2f,1.0f},{1.8f,0.2f,0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,-0.1f,1.0f},{1.7f,0.2f,-0.2f,1.0f}};
static const Keyframe TROT_TURN_RIGHT_BR[] = {{1.8f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,0.1f,1.0f},{1.7f,0.2f,0.2f,1.0f},{1.8f,0.2f,0.3f,1.0f},{1.4f,0.2f,0.1f,1.0f},{1.8f,0.2f,-0.3f,1.0f},{1.8f,0.2f,-0.2f,1.0f}};

// ---- CRAWL GAITS ----
static const Keyframe CRAWL_BACKWARD_FR[] = {{1.8f,0.6f,0.0f,1.0f},{1.7f,0.4f,0.0f,1.0f},{1.7f,0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,-0.2f,0.0f,1.0f},{1.8f,-0.4f,0.0f,1.0f},{1.8f,-0.6f,0.0f,1.0f},{1.4f,-0.2f,0.0f,1.0f}};
static const Keyframe CRAWL_BACKWARD_FL[] = {{1.8f,-0.2f,0.0f,1.0f},{1.8f,-0.4f,0.0f,1.0f},{1.8f,-0.6f,0.0f,1.0f},{1.4f,-0.2f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f},{1.7f,0.4f,0.0f,1.0f},{1.7f,0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f}};
static const Keyframe CRAWL_BACKWARD_BL[] = {{1.7f,-0.4f,0.0f,1.0f},{1.4f,0.0f,0.0f,1.0f},{1.8f,0.8f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f},{1.8f,0.4f,0.0f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.7f,-0.2f,0.0f,1.0f}};
static const Keyframe CRAWL_BACKWARD_BR[] = {{1.8f,0.4f,0.0f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.7f,-0.2f,0.0f,1.0f},{1.7f,-0.4f,0.0f,1.0f},{1.4f,0.0f,0.0f,1.0f},{1.8f,0.8f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f}};

static const Keyframe CRAWL_FORWARD_FR[] = {{1.4f,-0.2f,0.0f,1.0f},{1.8f,-0.6f,0.0f,1.0f},{1.8f,-0.4f,0.0f,1.0f},{1.8f,-0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.7f,0.2f,0.0f,1.0f},{1.7f,0.4f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f}};
static const Keyframe CRAWL_FORWARD_FL[] = {{1.8f,0.0f,0.0f,1.0f},{1.7f,0.2f,0.0f,1.0f},{1.7f,0.4f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f},{1.4f,-0.2f,0.0f,1.0f},{1.8f,-0.6f,0.0f,1.0f},{1.8f,-0.4f,0.0f,1.0f},{1.8f,-0.2f,0.0f,1.0f}};
static const Keyframe CRAWL_FORWARD_BL[] = {{1.7f,-0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.8f,0.4f,0.0f,1.0f},{1.8f,0.6f,0.0f,1.0f},{1.8f,0.8f,0.0f,1.0f},{1.4f,0.0f,0.0f,1.0f},{1.7f,-0.4f,0.0f,1.0f}};
static const Keyframe CRAWL_FORWARD_BR[] = {{1.8f,0.6f,0.0f,1.0f},{1.8f,0.8f,0.0f,1.0f},{1.4f,0.0f,0.0f,1.0f},{1.7f,-0.4f,0.0f,1.0f},{1.7f,-0.2f,0.0f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.8f,0.4f,0.0f,1.0f}};

static const Keyframe CRAWL_TURN_LEFT_FR[] = {{1.7f,0.0f,0.3f,1.0f},{1.7f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.0f,-0.1f,1.0f},{1.8f,0.0f,-0.2f,1.0f},{1.8f,0.0f,-0.3f,1.0f},{1.4f,0.0f,-0.1f,1.0f}};
static const Keyframe CRAWL_TURN_LEFT_FL[] = {{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.3f,1.0f},{1.4f,0.0f,0.1f,1.0f},{1.8f,0.0f,-0.3f,1.0f},{1.7f,0.0f,-0.2f,1.0f},{1.7f,0.0f,-0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f}};
static const Keyframe CRAWL_TURN_LEFT_BL[] = {{1.8f,0.2f,0.3f,1.0f},{1.4f,0.2f,0.1f,1.0f},{1.8f,0.2f,-0.3f,1.0f},{1.8f,0.2f,-0.2f,1.0f},{1.8f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,0.1f,1.0f},{1.7f,0.2f,0.2f,1.0f}};
static const Keyframe CRAWL_TURN_LEFT_BR[] = {{1.8f,0.2f,0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,-0.1f,1.0f},{1.7f,0.2f,-0.2f,1.0f},{1.8f,0.2f,-0.3f,1.0f},{1.4f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.3f,1.0f},{1.8f,0.2f,0.2f,1.0f}};

static const Keyframe CRAWL_TURN_RIGHT_FR[] = {{1.7f,0.0f,-0.3f,1.0f},{1.7f,0.0f,-0.2f,1.0f},{1.8f,0.0f,-0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f},{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.3f,1.0f},{1.4f,0.0f,0.1f,1.0f}};
static const Keyframe CRAWL_TURN_RIGHT_FL[] = {{1.8f,0.0f,-0.1f,1.0f},{1.8f,0.0f,-0.2f,1.0f},{1.8f,0.0f,-0.3f,1.0f},{1.4f,0.0f,-0.1f,1.0f},{1.7f,0.0f,0.3f,1.0f},{1.7f,0.0f,0.2f,1.0f},{1.8f,0.0f,0.1f,1.0f},{1.8f,0.0f,0.0f,1.0f}};
static const Keyframe CRAWL_TURN_RIGHT_BL[] = {{1.8f,0.2f,-0.3f,1.0f},{1.4f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.3f,1.0f},{1.8f,0.2f,0.2f,1.0f},{1.8f,0.2f,0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,-0.1f,1.0f},{1.7f,0.2f,-0.2f,1.0f}};
static const Keyframe CRAWL_TURN_RIGHT_BR[] = {{1.8f,0.2f,-0.1f,1.0f},{1.8f,0.2f,0.0f,1.0f},{1.7f,0.2f,0.1f,1.0f},{1.7f,0.2f,0.2f,1.0f},{1.8f,0.2f,0.3f,1.0f},{1.4f,0.2f,0.1f,1.0f},{1.8f,0.2f,-0.3f,1.0f},{1.8f,0.2f,-0.2f,1.0f}};

struct Traj { const Keyframe* frames; int len; };

static const Traj TROT_GAITS[5][4] = {
  { {nullptr,0}, {nullptr,0}, {nullptr,0}, {nullptr,0} },
  { {TROT_FORWARD_FR, LEN(TROT_FORWARD_FR)}, {TROT_FORWARD_FL, LEN(TROT_FORWARD_FL)}, {TROT_FORWARD_BL, LEN(TROT_FORWARD_BL)}, {TROT_FORWARD_BR, LEN(TROT_FORWARD_BR)} },
  { {TROT_BACKWARD_FR, LEN(TROT_BACKWARD_FR)}, {TROT_BACKWARD_FL, LEN(TROT_BACKWARD_FL)}, {TROT_BACKWARD_BL, LEN(TROT_BACKWARD_BL)}, {TROT_BACKWARD_BR, LEN(TROT_BACKWARD_BR)} },
  { {TROT_TURN_LEFT_FR, LEN(TROT_TURN_LEFT_FR)}, {TROT_TURN_LEFT_FL, LEN(TROT_TURN_LEFT_FL)}, {TROT_TURN_LEFT_BL, LEN(TROT_TURN_LEFT_BL)}, {TROT_TURN_LEFT_BR, LEN(TROT_TURN_LEFT_BR)} },
  { {TROT_TURN_RIGHT_FR, LEN(TROT_TURN_RIGHT_FR)}, {TROT_TURN_RIGHT_FL, LEN(TROT_TURN_RIGHT_FL)}, {TROT_TURN_RIGHT_BL, LEN(TROT_TURN_RIGHT_BL)}, {TROT_TURN_RIGHT_BR, LEN(TROT_TURN_RIGHT_BR)} }
};

static const Traj CRAWL_GAITS[5][4] = {
  { {nullptr,0}, {nullptr,0}, {nullptr,0}, {nullptr,0} },
  { {CRAWL_FORWARD_FR, LEN(CRAWL_FORWARD_FR)}, {CRAWL_FORWARD_FL, LEN(CRAWL_FORWARD_FL)}, {CRAWL_FORWARD_BL, LEN(CRAWL_FORWARD_BL)}, {CRAWL_FORWARD_BR, LEN(CRAWL_FORWARD_BR)} },
  { {CRAWL_BACKWARD_FR, LEN(CRAWL_BACKWARD_FR)}, {CRAWL_BACKWARD_FL, LEN(CRAWL_BACKWARD_FL)}, {CRAWL_BACKWARD_BL, LEN(CRAWL_BACKWARD_BL)}, {CRAWL_BACKWARD_BR, LEN(CRAWL_BACKWARD_BR)} },
  { {CRAWL_TURN_LEFT_FR, LEN(CRAWL_TURN_LEFT_FR)}, {CRAWL_TURN_LEFT_FL, LEN(CRAWL_TURN_LEFT_FL)}, {CRAWL_TURN_LEFT_BL, LEN(CRAWL_TURN_LEFT_BL)}, {CRAWL_TURN_LEFT_BR, LEN(CRAWL_TURN_LEFT_BR)} },
  { {CRAWL_TURN_RIGHT_FR, LEN(CRAWL_TURN_RIGHT_FR)}, {CRAWL_TURN_RIGHT_FL, LEN(CRAWL_TURN_RIGHT_FL)}, {CRAWL_TURN_RIGHT_BL, LEN(CRAWL_TURN_RIGHT_BL)}, {CRAWL_TURN_RIGHT_BR, LEN(CRAWL_TURN_RIGHT_BR)} }
};

#define ACTIVE_TRAJ(cmd, legIdx) (((gaitMode == GAIT_CRAWL) ? CRAWL_GAITS : TROT_GAITS)[(int)(cmd)][(legIdx)])

// ===============================
// 7) DEMO PATH
// ===============================
struct AutoStep {
  Command cmd;
  float armLatRelDeg;
  float armShoulderRelDeg;
  float armElbowRelDeg;
  float clawRelDeg;
  float durationSec;
};

static const AutoStep DEMO_PATH[] = {
  {FORWARD,     0.0f,   0.0f,  0.0f,   0.0f,  5.0f},
  {IDLE,       45.0f,  -90.0f,  90.0f,  0.0f,  5.0f},
  {IDLE,      -45.0f,  -90.0f,  90.0f,  0.0f,  5.0f},
  {IDLE,        0.0f,   0.0f,   0.0f,   0.0f,  3.0f},
  {FORWARD,     0.0f,   0.0f,   0.0f,   0.0f, 5.0f},
};
static const int DEMO_LEN = LEN(DEMO_PATH);

// ===============================
// 8) LEG STATE MACHINE
// ===============================
struct LegState {
  Vec3 cur, segStart, segEnd;
  float segElapsed, segDur;
  const Keyframe* traj;
  int trajLen, trajIdx;
  bool holdOnComplete;
  Command activeCmd;

  int   kfAtCur;
  int   swingIdx;
  bool  isSwing;

  float smoothBlend;
  float dyApplied;
  bool  seedPhase;
};

static LegState legs[4];

static inline float baseSegmentSec() {
  return (float)INTERP_POINTS_BASE * ((float)MICROSTEP_MS / 1000.0f);
}
static inline float effectiveSegDuration(Command cmd, float scale) {
  float cmdSpeed = CMD_SPEED[(int)cmd];
  float speed = clampf(GAIT_SPEED, GAIT_SPEED_MIN, GAIT_SPEED_MAX);
  return (baseSegmentSec() * scale) / (speed * cmdSpeed);
}

static int computeSwingIdxFromTraj(const Keyframe* traj, int len) {
  if (!traj || len <= 0) return -1;
  float dMin = 1e30f, dMax = -1e30f;
  int idxMin = 0;
  for (int i = 0; i < len; i++) {
    float x = traj[i].x, y = traj[i].y, z = traj[i].z;
    float d = sqrtf(x*x + y*y + z*z);
    if (d < dMin) { dMin = d; idxMin = i; }
    if (d > dMax) { dMax = d; }
  }
  if (fabsf(dMax - dMin) < 1e-5f) return -1;
  return idxMin;
}

static bool isSwingSegment(int startIdx, int endIdx, int swingIdx) {
  if (swingIdx < 0) return false;
  return (startIdx == swingIdx) || (endIdx == swingIdx);
}

static void advanceToNextNonzero(LegState& L) {
  if (!L.traj || L.trajLen <= 0) return;

  Vec3 start = L.cur;
  const Keyframe& kf = L.traj[L.trajIdx];
  Vec3 end{kf.x, kf.y, kf.z};

  L.segStart   = start;
  L.segEnd     = end;
  L.segElapsed = 0.0f;
  L.segDur     = effectiveSegDuration(L.activeCmd, kf.scale);

  int startIdx = L.kfAtCur;
  int endIdx   = L.trajIdx;
  L.isSwing = isSwingSegment(startIdx, endIdx, L.swingIdx);
}

static float computeSeedDurationForCommand(Command cmd) {
  float seedDur = baseSegmentSec();
  bool gotAny = false;

  for (int i = 0; i < 4; i++) {
    const Traj& T = ACTIVE_TRAJ(cmd, i);
    if (T.frames && T.len > 0) {
      float d = effectiveSegDuration(cmd, T.frames[0].scale);
      if (!gotAny || d > seedDur) seedDur = d;
      gotAny = true;
    }
  }
  return seedDur;
}

static void setCommandTrajectories(Command cmd) {
  if (cmd == IDLE) {
    for (int i = 0; i < 4; i++) {
      LegState& L = legs[i];
      L.activeCmd = IDLE;
      L.traj = nullptr;
      L.trajLen = 0;
      L.trajIdx = 0;
      L.holdOnComplete = true;
      L.segStart = L.cur;
      L.segEnd = Vec3{REST_X, REST_Y, REST_Z};
      L.segElapsed = 0.0f;
      L.segDur = effectiveSegDuration(IDLE, 1.0f);
      L.kfAtCur = -1;
      L.swingIdx = -1;
      L.isSwing = false;
      L.seedPhase = false;
    }
    return;
  }

  float seedDur = computeSeedDurationForCommand(cmd);

  for (int i = 0; i < 4; i++) {
    const Traj& T = ACTIVE_TRAJ(cmd, i);
    LegState& L = legs[i];

    L.activeCmd = cmd;
    L.traj = T.frames;
    L.trajLen = T.len;
    L.trajIdx = 0;
    L.holdOnComplete = false;
    L.swingIdx = computeSwingIdxFromTraj(L.traj, L.trajLen);
    L.kfAtCur = -1;
    L.isSwing = false;
    L.seedPhase = true;

    L.segStart = L.cur;

    const Keyframe& kf0 = L.traj[0];
    L.segEnd = Vec3{kf0.x, kf0.y, kf0.z};
    L.segElapsed = 0.0f;
    L.segDur = seedDur;
  }
}

static void tickLeg(LegState& L, float dtSec) {
  float remaining = dtSec;

  while (remaining > 0.0f) {
    float timeLeft = fmaxf(1e-9f, L.segDur - L.segElapsed);
    float consume = fminf(remaining, timeLeft);
    L.segElapsed += consume;
    remaining -= consume;

    float t = fminf(1.0f, L.segElapsed / fmaxf(1e-9f, L.segDur));
    L.cur.x = L.segStart.x + (L.segEnd.x - L.segStart.x) * t;
    L.cur.y = L.segStart.y + (L.segEnd.y - L.segStart.y) * t;
    L.cur.z = L.segStart.z + (L.segEnd.z - L.segStart.z) * t;

    if (L.segElapsed + 1e-6f < L.segDur) break;
    if (L.holdOnComplete) break;

    if (L.traj && L.trajLen > 0) {
      if (L.seedPhase) {
        L.seedPhase = false;
        L.kfAtCur = 0;
        L.trajIdx = (L.trajLen > 1) ? 1 : 0;
        advanceToNextNonzero(L);
      } else {
        L.kfAtCur = L.trajIdx;
        L.trajIdx = (L.trajIdx + 1) % L.trajLen;
        advanceToNextNonzero(L);
      }
    } else {
      break;
    }
  }
}

// ===============================
// 9) LEG IK ONLY
// ===============================
static const float IK_EPS = 1e-6f;

static void planar2LinkIK(float x, float y, float L1, float L2,
                          float& shoulder, float& elbow,
                          float& d, float& dClamped) {
  d = hypotf(x, y);
  dClamped = clampf(d, fabsf(L1 - L2) + IK_EPS, (L1 + L2) - IK_EPS);
  float cosKnee = (dClamped*dClamped - L1*L1 - L2*L2) / (2.0f * L1 * L2);
  cosKnee = clampf(cosKnee, -1.0f, 1.0f);
  elbow = -acosf(cosKnee);
  float k1 = L1 + L2 * cosf(elbow);
  float k2 = L2 * sinf(elbow);
  shoulder = atan2f(y, x) - atan2f(k2, k1);
}

static void ik3dEqualLinks(const Vec3& tgt,
                           float& lat, float& shoulder, float& elbow,
                           bool& outOfReach) {
  float x = tgt.x, y = tgt.y, z = tgt.z;
  lat = atan2f(z, x);
  float r = hypotf(x, z);
  float d, dClamped;
  planar2LinkIK(r, y, R1, R2, shoulder, elbow, d, dClamped);
  outOfReach = (fabsf(d - dClamped) > 1e-5f);
}

// ===============================
// 10) SERVO OUTPUT
// ===============================
static void writeLegServoDeg(uint8_t ch, float deg) {
  deg = clampf(deg, LEG_SERVO_MIN_DEG, LEG_SERVO_MAX_DEG);
  float usF = mapFloat(deg, 0.0f, 270.0f, (float)MIN_US, (float)MAX_US);
  int us = (int)lroundf(usF);
  pwm.writeMicroseconds(ch, us);
  if (ch < TOTAL_SERVO_CHANNELS) {
    lastServoDeg[ch] = deg;
    lastServoDegValid[ch] = true;
  }
}

static void writeArmServoDeg(uint8_t ch, float deg) {
  deg = clampf(deg, ARM_SERVO_MIN_DEG, ARM_SERVO_MAX_DEG);
  float usF = mapFloat(deg, 0.0f, 180.0f, (float)MIN_US, (float)MAX_US);
  int us = (int)lroundf(usF);
  pwm.writeMicroseconds(ch, us);
  if (ch < TOTAL_SERVO_CHANNELS) {
    lastServoDeg[ch] = deg;
    lastServoDegValid[ch] = true;
  }
}

static void allServosAdjustedTPose() {
  for (int leg = 0; leg < 4; leg++) {
    float latServo = LEG_SERVO_CENTER_DEG + LAT_OFFSET_DEG[leg];
    float shServo  = LEG_SERVO_CENTER_DEG + SHOULDER_OFFSET_DEG[leg];
    float elServo  = LEG_SERVO_CENTER_DEG + ELBOW_OFFSET_DEG[leg];
    writeLegServoDeg(SERVO_CH[leg][0], latServo);
    writeLegServoDeg(SERVO_CH[leg][1], shServo);
    writeLegServoDeg(SERVO_CH[leg][2], elServo);
  }

  writeArmServoDeg(ARM_LAT_CH,      ARM_SERVO_CENTER_DEG + ARM_LAT_OFFSET_DEG);
  writeArmServoDeg(ARM_SHOULDER_CH, ARM_SERVO_CENTER_DEG + ARM_SHOULDER_OFFSET_DEG);
  writeArmServoDeg(ARM_ELBOW_CH,    ARM_SERVO_CENTER_DEG + ARM_ELBOW_OFFSET_DEG);
  writeArmServoDeg(ARM_CLAW_CH,     ARM_SERVO_CENTER_DEG + ARM_CLAW_OFFSET_DEG);
}

// ===============================
// 11) IMU (ADXL345 + ITG3205) + LEVELING OVERLAY
// ===============================
static const uint8_t ADXL345_ADDR = 0x53;
static const uint8_t ITG3205_ADDR = 0x68;
static const uint8_t IMU_MAP[3]  = { 0, 1, 2 };
static const int8_t  IMU_SIGN[3] = { +1, +1, +1 };
static const float IMU_CF_TAU_SEC = 0.5f;

static float imuRollDeg  = 0.0f;
static float imuPitchDeg = 0.0f;
static float imuYawDeg   = 0.0f;
static float gyroBiasX = 0.0f;
static float gyroBiasY = 0.0f;
static float gyroBiasZ = 0.0f;
static bool imuOk = false;

static const float LEVEL_KP_ROLL  = 0.060f;
static const float LEVEL_KD_ROLL  = 0.0045f;
static const float LEVEL_KP_PITCH = 0.060f;
static const float LEVEL_KD_PITCH = 0.0045f;

static const float LEVEL_ROLL_SIGN  = +1.0f;
static const float LEVEL_PITCH_SIGN = +1.0f;
static const float LEVEL_DEADBAND_IDLE_DEG = 1.0f;
static const float LEVEL_DEADBAND_MOVE_DEG = 0.5f;
static const float LEVEL_MAX_DY = 0.25f;
static const float LEVEL_SWING_BLEND = 0.25f;
static const float BLEND_TAU_SEC = 0.12f;
static const float DY_SLEW_RATE  = 2.0f;

static const int8_t LEG_LEFT_SIGN[4]  = { -1, +1, +1, -1 };
static const int8_t LEG_FRONT_SIGN[4] = { +1, +1, -1, -1 };

static bool i2cWrite8(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t val) {
  w.beginTransmission(addr);
  w.write(reg);
  w.write(val);
  return (w.endTransmission() == 0);
}

static bool i2cReadBytes(TwoWire& w, uint8_t addr, uint8_t reg, uint8_t* out, uint8_t len) {
  w.beginTransmission(addr);
  w.write(reg);
  if (w.endTransmission(false) != 0) return false;
  uint8_t got = w.requestFrom((int)addr, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) out[i] = (uint8_t)w.read();
  return true;
}

static bool imuInitADXL345() {
  if (!i2cWrite8(Wire, ADXL345_ADDR, 0x2C, 0x0A)) return false;
  if (!i2cWrite8(Wire, ADXL345_ADDR, 0x31, 0x08)) return false;
  if (!i2cWrite8(Wire, ADXL345_ADDR, 0x2D, 0x08)) return false;
  return true;
}

static bool imuInitITG3205() {
  if (!i2cWrite8(Wire, ITG3205_ADDR, 0x3E, 0x00)) return false;
  if (!i2cWrite8(Wire, ITG3205_ADDR, 0x16, 0x1B)) return false;
  if (!i2cWrite8(Wire, ITG3205_ADDR, 0x15, 0x09)) return false;
  return true;
}

static bool imuBegin() {
  bool okA = imuInitADXL345();
  bool okG = imuInitITG3205();
  return okA && okG;
}

static bool imuReadAccelG(float& ax, float& ay, float& az) {
  uint8_t b[6];
  if (!i2cReadBytes(Wire, ADXL345_ADDR, 0x32, b, 6)) return false;
  int16_t rx = (int16_t)((b[1] << 8) | b[0]);
  int16_t ry = (int16_t)((b[3] << 8) | b[2]);
  int16_t rz = (int16_t)((b[5] << 8) | b[4]);

  int16_t raw[3] = { rx, ry, rz };
  float bx = (float)IMU_SIGN[0] * (float)raw[ IMU_MAP[0] ];
  float by = (float)IMU_SIGN[1] * (float)raw[ IMU_MAP[1] ];
  float bz = (float)IMU_SIGN[2] * (float)raw[ IMU_MAP[2] ];

  const float LSB_TO_G = 0.0039f;
  ax = bx * LSB_TO_G;
  ay = by * LSB_TO_G;
  az = bz * LSB_TO_G;
  return true;
}

static bool imuReadGyroDPS(float& gx, float& gy, float& gz) {
  uint8_t b[6];
  if (!i2cReadBytes(Wire, ITG3205_ADDR, 0x1D, b, 6)) return false;
  int16_t rx = (int16_t)((b[0] << 8) | b[1]);
  int16_t ry = (int16_t)((b[2] << 8) | b[3]);
  int16_t rz = (int16_t)((b[4] << 8) | b[5]);

  int16_t raw[3] = { rx, ry, rz };
  float bx = (float)IMU_SIGN[0] * (float)raw[ IMU_MAP[0] ];
  float by = (float)IMU_SIGN[1] * (float)raw[ IMU_MAP[1] ];
  float bz = (float)IMU_SIGN[2] * (float)raw[ IMU_MAP[2] ];

  const float LSB_TO_DPS = 1.0f / 14.375f;
  gx = bx * LSB_TO_DPS;
  gy = by * LSB_TO_DPS;
  gz = bz * LSB_TO_DPS;
  return true;
}

static void imuCalibrateGyroBias(int samples = 400) {
  float sumX = 0, sumY = 0, sumZ = 0;
  int good = 0;
  for (int i = 0; i < samples; i++) {
    float gx, gy, gz;
    if (imuReadGyroDPS(gx, gy, gz)) {
      sumX += gx; sumY += gy; sumZ += gz;
      good++;
    }
    delay(2);
  }
  if (good > 10) {
    gyroBiasX = sumX / good;
    gyroBiasY = sumY / good;
    gyroBiasZ = sumZ / good;
  }
}

static void imuUpdate(float dtSec, float& outRollRate, float& outPitchRate, float& outYawRate) {
  float ax, ay, az;
  float gx, gy, gz;

  bool okA = imuReadAccelG(ax, ay, az);
  bool okG = imuReadGyroDPS(gx, gy, gz);

  if (!okA || !okG || dtSec <= 0) {
    outRollRate = outPitchRate = outYawRate = 0.0f;
    return;
  }

  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

  float rollAcc  = rad2deg(atan2f(ay, az));
  float pitchAcc = rad2deg(atan2f(-ax, sqrtf(ay*ay + az*az)));

  float rollGyro  = imuRollDeg  + gx * dtSec;
  float pitchGyro = imuPitchDeg + gy * dtSec;
  float yawGyro   = imuYawDeg   + gz * dtSec;

  float alpha = IMU_CF_TAU_SEC / (IMU_CF_TAU_SEC + dtSec);
  imuRollDeg  = alpha * rollGyro  + (1.0f - alpha) * rollAcc;
  imuPitchDeg = alpha * pitchGyro + (1.0f - alpha) * pitchAcc;
  imuYawDeg   = yawGyro;

  outRollRate  = gx;
  outPitchRate = gy;
  outYawRate   = gz;
}

static void updateLevelingOverlay(Command cmd,
                                 float dtSec,
                                 float rollDeg, float pitchDeg,
                                 float rollRateDps, float pitchRateDps) {
  if (!imuOk) {
    for (int i = 0; i < 4; i++) {
      float maxStep = DY_SLEW_RATE * dtSec;
      float delta = 0.0f - legs[i].dyApplied;
      delta = clampf(delta, -maxStep, +maxStep);
      legs[i].dyApplied += delta;
      legs[i].smoothBlend = 1.0f;
    }
    return;
  }

  float deadband = (cmd == IDLE) ? LEVEL_DEADBAND_IDLE_DEG : LEVEL_DEADBAND_MOVE_DEG;

  float eRoll  = -rollDeg;
  float ePitch = -pitchDeg;

  if (fabsf(eRoll) < deadband)  eRoll = 0.0f;
  if (fabsf(ePitch) < deadband) ePitch = 0.0f;

  float rollCorr  = LEVEL_ROLL_SIGN  * (LEVEL_KP_ROLL  * eRoll  + LEVEL_KD_ROLL  * (-rollRateDps));
  float pitchCorr = LEVEL_PITCH_SIGN * (LEVEL_KP_PITCH * ePitch + LEVEL_KD_PITCH * (-pitchRateDps));

  float aBlend = BLEND_TAU_SEC / (BLEND_TAU_SEC + dtSec);
  float oneMinusABlend = (1.0f - aBlend);

  for (int i = 0; i < 4; i++) {
    float targetBlend = 1.0f;
    if (cmd != IDLE) {
      targetBlend = legs[i].isSwing ? LEVEL_SWING_BLEND : 1.0f;
    }
    legs[i].smoothBlend = aBlend * legs[i].smoothBlend + oneMinusABlend * targetBlend;
    float dyRaw = (float)LEG_LEFT_SIGN[i] * rollCorr + (float)LEG_FRONT_SIGN[i] * pitchCorr;
    float dyDesired = dyRaw * legs[i].smoothBlend;
    dyDesired = clampf(dyDesired, -LEVEL_MAX_DY, +LEVEL_MAX_DY);
    float maxStep = DY_SLEW_RATE * dtSec;
    float delta = dyDesired - legs[i].dyApplied;
    delta = clampf(delta, -maxStep, +maxStep);
    legs[i].dyApplied += delta;
  }
}

// ===============================
// 12) STATE
// ===============================
static bool calibrationMode = true;
static bool demoActive = false;

static Command activeCmd = IDLE;
static Command manualCmd = IDLE;

static ArmState armManual = { 0.0f, 0.0f, 0.0f, 0.0f };
static ArmState armActive = { 0.0f, 0.0f, 0.0f, 0.0f };

static uint32_t lastPacketMs = 0;
static uint32_t lastCtrlMs = 0;

static int demoIdx = 0;
static float demoElapsed = 0.0f;

static uint32_t lastZms = 0;
static uint32_t lastXms = 0;
static uint32_t lastMms = 0;
static uint32_t lastPms = 0;
static uint32_t lastStatusMs = 0;
static uint32_t lastReachWarnMs[4] = {0,0,0,0};

static bool sonarAvoidanceEnabled = false;
static bool sonarFrozen = false;
static bool sonarObstacleDetected = false;
static float sonarDistanceCm = SONAR_NO_ECHO_DISTANCE_CM;
static float sonarMotionScale = 1.0f;
static uint32_t lastSonarSampleMs = 0;
static uint32_t lastSonarPrintMs = 0;
static uint32_t lastFrozenCmdIgnoreMs = 0;

// ---- PATCH 2: durationSec field added to ArmTransitionState ----
struct ArmTransitionState {
  bool active;
  float elapsedSec;
  float durationSec;   // <-- PATCH 2: per-transition duration
  ArmState start;
  ArmState target;
};

struct PoseTransitionState {
  bool active;
  bool toCalibration;
  float elapsedSec;
  float legStartDeg[4][3];
  float legTargetDeg[4][3];
  float armStartDeg[4];
  float armTargetDeg[4];
};

// ---- PATCH 3: initializer updated with durationSec = 0 ----
static ArmTransitionState armHomeTransition = {
  false, 0.0f, 0.0f,
  {0.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f}
};

static PoseTransitionState poseTransition = {
  false, true, 0.0f,
  {{0.0f}},
  {{0.0f}},
  {0.0f, 0.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 0.0f, 0.0f}
};

// ===============================
// 10B) ARM HELPERS (DIRECT SERVO CONTROL)
// ===============================
static void armSetHome(ArmState& A) {
  A.latRelDeg = 0.0f;
  A.shoulderRelDeg = 0.0f;
  A.elbowRelDeg = 0.0f;
  A.clawRelDeg = 0.0f;
}

static void copyArmState(ArmState& dst, const ArmState& src) {
  dst.latRelDeg = src.latRelDeg;
  dst.shoulderRelDeg = src.shoulderRelDeg;
  dst.elbowRelDeg = src.elbowRelDeg;
  dst.clawRelDeg = src.clawRelDeg;
}

static void clampArmState(ArmState& A) {
  A.latRelDeg      = clampf(A.latRelDeg,      ARM_LAT_REL_MIN_DEG,      ARM_LAT_REL_MAX_DEG);
  A.shoulderRelDeg = clampf(A.shoulderRelDeg, ARM_SHOULDER_REL_MIN_DEG, ARM_SHOULDER_REL_MAX_DEG);
  A.elbowRelDeg    = clampf(A.elbowRelDeg,    ARM_ELBOW_REL_MIN_DEG,    ARM_ELBOW_REL_MAX_DEG);
  A.clawRelDeg     = clampf(A.clawRelDeg,     ARM_CLAW_REL_MIN_DEG,     ARM_CLAW_REL_MAX_DEG);
}

static void applyArmManualStep(float dLat, float dShoulder, float dElbow, float dClaw) {
  armHomeTransition.active = false;

  armManual.latRelDeg      += dLat;
  armManual.shoulderRelDeg += dShoulder;
  armManual.elbowRelDeg    += dElbow;
  armManual.clawRelDeg     += dClaw;
  clampArmState(armManual);
}

static float moveTowardDeg(float current, float target, float maxStep) {
  float diff = target - current;
  if (fabsf(diff) <= maxStep) return target;
  return current + ((diff > 0.0f) ? maxStep : -maxStep);
}

static void smoothArmTowardManual(float dtSec) {
  float maxStep = ARM_SLEW_DEG_PER_SEC * fmaxf(0.0f, dtSec);

  armActive.latRelDeg      = moveTowardDeg(armActive.latRelDeg,      armManual.latRelDeg,      maxStep);
  armActive.shoulderRelDeg = moveTowardDeg(armActive.shoulderRelDeg, armManual.shoulderRelDeg, maxStep);
  armActive.elbowRelDeg    = moveTowardDeg(armActive.elbowRelDeg,    armManual.elbowRelDeg,    maxStep);
  armActive.clawRelDeg     = moveTowardDeg(armActive.clawRelDeg,     armManual.clawRelDeg,     maxStep);

  clampArmState(armActive);
}

static float unitFromInt(int v) {
  if (v > 0) return 1.0f;
  if (v < 0) return -1.0f;
  return 0.0f;
}

// ---- PATCH 4: applyDemoArmStep now starts a smooth easeInOut transition ----
// Instead of snapping armActive instantly, we begin an armHomeTransition
// from wherever the arm currently is to the demo step's target pose.
// Duration is DEMO_ARM_INTERP_DURATION_SEC (slow & smooth).
static void applyDemoArmStep(int idx) {
  if (idx < 0 || idx >= DEMO_LEN) return;

  // Start position = wherever the arm is right now
  copyArmState(armHomeTransition.start, armActive);

  // Target = this demo step's arm pose
  armHomeTransition.target.latRelDeg      = DEMO_PATH[idx].armLatRelDeg;
  armHomeTransition.target.shoulderRelDeg = DEMO_PATH[idx].armShoulderRelDeg;
  armHomeTransition.target.elbowRelDeg    = DEMO_PATH[idx].armElbowRelDeg;
  armHomeTransition.target.clawRelDeg     = DEMO_PATH[idx].clawRelDeg;
  clampArmState(armHomeTransition.target);

  armHomeTransition.elapsedSec  = 0.0f;
  armHomeTransition.durationSec = DEMO_ARM_INTERP_DURATION_SEC;  // slow
  armHomeTransition.active      = true;

  Serial.printf("[Arm Demo] Interpolating to step %d: lat=%.1f sh=%.1f el=%.1f claw=%.1f over %.1fs\n",
                idx,
                armHomeTransition.target.latRelDeg,
                armHomeTransition.target.shoulderRelDeg,
                armHomeTransition.target.elbowRelDeg,
                armHomeTransition.target.clawRelDeg,
                armHomeTransition.durationSec);
}

static void updateActiveArmTargetFromMode() {
  if (calibrationMode) {
    armSetHome(armActive);
    return;
  }
  // Demo arm is now handled exclusively via applyDemoArmStep() + armHomeTransition.
  // This function is intentionally left without a demo branch.
}

static void driveArmServosFromState(const ArmState& A) {
  float servoLat = ARM_SERVO_CENTER_DEG + A.latRelDeg + ARM_LAT_OFFSET_DEG;
  float servoSh  = ARM_SERVO_CENTER_DEG + A.shoulderRelDeg + ARM_SHOULDER_OFFSET_DEG;
  float servoEl  = ARM_SERVO_CENTER_DEG + A.elbowRelDeg + ARM_ELBOW_OFFSET_DEG;
  float servoCl  = ARM_SERVO_CENTER_DEG + A.clawRelDeg + ARM_CLAW_OFFSET_DEG;

  writeArmServoDeg(ARM_LAT_CH,      servoLat);
  writeArmServoDeg(ARM_SHOULDER_CH, servoSh);
  writeArmServoDeg(ARM_ELBOW_CH,    servoEl);
  writeArmServoDeg(ARM_CLAW_CH,     servoCl);
}

static inline float easeInOut01(float t) {
  t = clampf(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

static float getTrackedServoDeg(uint8_t ch, float fallbackDeg) {
  if (ch < TOTAL_SERVO_CHANNELS && lastServoDegValid[ch]) return lastServoDeg[ch];
  return fallbackDeg;
}

static void computeLegServoAnglesForTarget(int legIdx, const Vec3& tgt,
                                           float& servoLat, float& servoSh, float& servoEl,
                                           bool& outOfReach) {
  float lat, sh, el;
  ik3dEqualLinks(tgt, lat, sh, el, outOfReach);

  float latDeg = rad2deg(lat);
  float shDeg  = rad2deg(sh);
  float elDeg  = rad2deg(el);

  latDeg = clampf(latDeg, LAT_MIN_DEG, LAT_MAX_DEG);
  shDeg  = clampf(shDeg,  SHOULDER_MIN_DEG, SHOULDER_MAX_DEG);
  elDeg  = clampf(elDeg,  ELBOW_MIN_DEG, ELBOW_MAX_DEG);

  float elbowUsedDeg = elDeg;
  if (!MODEL_USES_RELATIVE_ELBOW_ANGLE) elbowUsedDeg = shDeg + elDeg;

  float outLatDeg = (float)LAT_SIGN[legIdx] * latDeg + LAT_OFFSET_DEG[legIdx];
  float outShDeg  = (float)SHOULDER_SIGN[legIdx] * shDeg + SHOULDER_OFFSET_DEG[legIdx];
  float outElDeg  = (float)ELBOW_SIGN[legIdx] * elbowUsedDeg + ELBOW_OFFSET_DEG[legIdx];

  servoLat = LEG_SERVO_CENTER_DEG + outLatDeg;
  servoSh  = LEG_SERVO_CENTER_DEG + outShDeg;
  servoEl  = LEG_SERVO_CENTER_DEG + outElDeg;
}

static void computeIdleLegServoTargets(float outDeg[4][3]) {
  const Vec3 idleTgt = {REST_X, REST_Y, REST_Z};
  for (int leg = 0; leg < 4; leg++) {
    bool outOfReach = false;
    computeLegServoAnglesForTarget(leg, idleTgt,
                                   outDeg[leg][0], outDeg[leg][1], outDeg[leg][2],
                                   outOfReach);
  }
}

static void computeTPoseServoTargets(float outLegDeg[4][3], float outArmDeg[4]) {
  for (int leg = 0; leg < 4; leg++) {
    outLegDeg[leg][0] = LEG_SERVO_CENTER_DEG + LAT_OFFSET_DEG[leg];
    outLegDeg[leg][1] = LEG_SERVO_CENTER_DEG + SHOULDER_OFFSET_DEG[leg];
    outLegDeg[leg][2] = LEG_SERVO_CENTER_DEG + ELBOW_OFFSET_DEG[leg];
  }

  outArmDeg[0] = ARM_SERVO_CENTER_DEG + ARM_LAT_OFFSET_DEG;
  outArmDeg[1] = ARM_SERVO_CENTER_DEG + ARM_SHOULDER_OFFSET_DEG;
  outArmDeg[2] = ARM_SERVO_CENTER_DEG + ARM_ELBOW_OFFSET_DEG;
  outArmDeg[3] = ARM_SERVO_CENTER_DEG + ARM_CLAW_OFFSET_DEG;
}

static void computeArmHomeServoTargets(float outArmDeg[4]) {
  outArmDeg[0] = ARM_SERVO_CENTER_DEG + ARM_LAT_OFFSET_DEG;
  outArmDeg[1] = ARM_SERVO_CENTER_DEG + ARM_SHOULDER_OFFSET_DEG;
  outArmDeg[2] = ARM_SERVO_CENTER_DEG + ARM_ELBOW_OFFSET_DEG;
  outArmDeg[3] = ARM_SERVO_CENTER_DEG + ARM_CLAW_OFFSET_DEG;
}

static void captureArmStateFromTrackedServos(ArmState& A) {
  A.latRelDeg      = getTrackedServoDeg(ARM_LAT_CH,      ARM_SERVO_CENTER_DEG + ARM_LAT_OFFSET_DEG)      - ARM_SERVO_CENTER_DEG - ARM_LAT_OFFSET_DEG;
  A.shoulderRelDeg = getTrackedServoDeg(ARM_SHOULDER_CH, ARM_SERVO_CENTER_DEG + ARM_SHOULDER_OFFSET_DEG) - ARM_SERVO_CENTER_DEG - ARM_SHOULDER_OFFSET_DEG;
  A.elbowRelDeg    = getTrackedServoDeg(ARM_ELBOW_CH,    ARM_SERVO_CENTER_DEG + ARM_ELBOW_OFFSET_DEG)    - ARM_SERVO_CENTER_DEG - ARM_ELBOW_OFFSET_DEG;
  A.clawRelDeg     = getTrackedServoDeg(ARM_CLAW_CH,     ARM_SERVO_CENTER_DEG + ARM_CLAW_OFFSET_DEG)     - ARM_SERVO_CENTER_DEG - ARM_CLAW_OFFSET_DEG;
  clampArmState(A);
}

// ---- PATCH 5: beginArmHomeTransition sets durationSec = POSE_INTERP_DURATION_SEC ----
static void beginArmHomeTransition() {
  armSetHome(armManual);
  captureArmStateFromTrackedServos(armHomeTransition.start);
  armSetHome(armHomeTransition.target);
  armHomeTransition.elapsedSec  = 0.0f;
  armHomeTransition.durationSec = POSE_INTERP_DURATION_SEC;  // <-- PATCH 5
  armHomeTransition.active      = true;
}

// ---- PATCH 6: tickArmHomeTransition uses armHomeTransition.durationSec ----
static void tickArmHomeTransition(float dtSec) {
  if (!armHomeTransition.active) return;

  // Use per-transition duration; fall back to POSE_INTERP_DURATION_SEC if unset
  float duration = fmaxf(0.01f,
    (armHomeTransition.durationSec > 0.0f)
      ? armHomeTransition.durationSec
      : POSE_INTERP_DURATION_SEC);

  armHomeTransition.elapsedSec += dtSec;
  float t = easeInOut01(armHomeTransition.elapsedSec / duration);

  armActive.latRelDeg      = armHomeTransition.start.latRelDeg      + (armHomeTransition.target.latRelDeg      - armHomeTransition.start.latRelDeg)      * t;
  armActive.shoulderRelDeg = armHomeTransition.start.shoulderRelDeg + (armHomeTransition.target.shoulderRelDeg - armHomeTransition.start.shoulderRelDeg) * t;
  armActive.elbowRelDeg    = armHomeTransition.start.elbowRelDeg    + (armHomeTransition.target.elbowRelDeg    - armHomeTransition.start.elbowRelDeg)    * t;
  armActive.clawRelDeg     = armHomeTransition.start.clawRelDeg     + (armHomeTransition.target.clawRelDeg     - armHomeTransition.start.clawRelDeg)     * t;
  clampArmState(armActive);

  if (armHomeTransition.elapsedSec >= duration) {
    armHomeTransition.active = false;
    copyArmState(armActive, armHomeTransition.target);
    copyArmState(armManual, armHomeTransition.target);
  }
}

static void beginPoseTransition(bool toCalibration) {
  armHomeTransition.active = false;
  poseTransition.active = true;
  poseTransition.toCalibration = toCalibration;
  poseTransition.elapsedSec = 0.0f;

  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      uint8_t ch = SERVO_CH[leg][joint];
      float fallback = LEG_SERVO_CENTER_DEG;
      if (joint == 0) fallback += LAT_OFFSET_DEG[leg];
      else if (joint == 1) fallback += SHOULDER_OFFSET_DEG[leg];
      else fallback += ELBOW_OFFSET_DEG[leg];
      poseTransition.legStartDeg[leg][joint] = getTrackedServoDeg(ch, fallback);
    }
  }

  poseTransition.armStartDeg[0] = getTrackedServoDeg(ARM_LAT_CH,      ARM_SERVO_CENTER_DEG + ARM_LAT_OFFSET_DEG);
  poseTransition.armStartDeg[1] = getTrackedServoDeg(ARM_SHOULDER_CH, ARM_SERVO_CENTER_DEG + ARM_SHOULDER_OFFSET_DEG);
  poseTransition.armStartDeg[2] = getTrackedServoDeg(ARM_ELBOW_CH,    ARM_SERVO_CENTER_DEG + ARM_ELBOW_OFFSET_DEG);
  poseTransition.armStartDeg[3] = getTrackedServoDeg(ARM_CLAW_CH,     ARM_SERVO_CENTER_DEG + ARM_CLAW_OFFSET_DEG);

  if (toCalibration) {
    computeTPoseServoTargets(poseTransition.legTargetDeg, poseTransition.armTargetDeg);
  } else {
    computeIdleLegServoTargets(poseTransition.legTargetDeg);
    computeArmHomeServoTargets(poseTransition.armTargetDeg);
  }
}

static void tickPoseTransition(float dtSec) {
  if (!poseTransition.active) return;

  float duration = fmaxf(0.01f, POSE_INTERP_DURATION_SEC);
  poseTransition.elapsedSec += dtSec;
  float t = easeInOut01(poseTransition.elapsedSec / duration);

  for (int leg = 0; leg < 4; leg++) {
    float servoLat = poseTransition.legStartDeg[leg][0] + (poseTransition.legTargetDeg[leg][0] - poseTransition.legStartDeg[leg][0]) * t;
    float servoSh  = poseTransition.legStartDeg[leg][1] + (poseTransition.legTargetDeg[leg][1] - poseTransition.legStartDeg[leg][1]) * t;
    float servoEl  = poseTransition.legStartDeg[leg][2] + (poseTransition.legTargetDeg[leg][2] - poseTransition.legStartDeg[leg][2]) * t;

    writeLegServoDeg(SERVO_CH[leg][0], servoLat);
    writeLegServoDeg(SERVO_CH[leg][1], servoSh);
    writeLegServoDeg(SERVO_CH[leg][2], servoEl);
  }

  float servoLat = poseTransition.armStartDeg[0] + (poseTransition.armTargetDeg[0] - poseTransition.armStartDeg[0]) * t;
  float servoSh  = poseTransition.armStartDeg[1] + (poseTransition.armTargetDeg[1] - poseTransition.armStartDeg[1]) * t;
  float servoEl  = poseTransition.armStartDeg[2] + (poseTransition.armTargetDeg[2] - poseTransition.armStartDeg[2]) * t;
  float servoCl  = poseTransition.armStartDeg[3] + (poseTransition.armTargetDeg[3] - poseTransition.armStartDeg[3]) * t;

  writeArmServoDeg(ARM_LAT_CH,      servoLat);
  writeArmServoDeg(ARM_SHOULDER_CH, servoSh);
  writeArmServoDeg(ARM_ELBOW_CH,    servoEl);
  writeArmServoDeg(ARM_CLAW_CH,     servoCl);

  armActive.latRelDeg      = servoLat - ARM_SERVO_CENTER_DEG - ARM_LAT_OFFSET_DEG;
  armActive.shoulderRelDeg = servoSh  - ARM_SERVO_CENTER_DEG - ARM_SHOULDER_OFFSET_DEG;
  armActive.elbowRelDeg    = servoEl  - ARM_SERVO_CENTER_DEG - ARM_ELBOW_OFFSET_DEG;
  armActive.clawRelDeg     = servoCl  - ARM_SERVO_CENTER_DEG - ARM_CLAW_OFFSET_DEG;
  clampArmState(armActive);

  if (poseTransition.elapsedSec >= duration) {
    poseTransition.active = false;
    armSetHome(armManual);
    armSetHome(armActive);

    if (!poseTransition.toCalibration) {
      for (int i = 0; i < 4; i++) {
        legs[i].cur = Vec3{REST_X, REST_Y, REST_Z};
        legs[i].segStart = legs[i].cur;
        legs[i].segEnd   = legs[i].cur;
        legs[i].segElapsed = 0.0f;
        legs[i].segDur = baseSegmentSec();
        legs[i].traj = nullptr;
        legs[i].trajLen = 0;
        legs[i].trajIdx = 0;
        legs[i].holdOnComplete = true;
        legs[i].activeCmd = IDLE;
        legs[i].kfAtCur = -1;
        legs[i].swingIdx = -1;
        legs[i].isSwing = false;
        legs[i].smoothBlend = 1.0f;
        legs[i].dyApplied = 0.0f;
        legs[i].seedPhase = false;
      }
      setCommandTrajectories(IDLE);
    }
  }
}

static float readSonarDistanceCm() {
  digitalWrite(SONAR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(SONAR_ECHO_PIN, HIGH, SONAR_ECHO_TIMEOUT_US);
  if (durationUs == 0) return SONAR_NO_ECHO_DISTANCE_CM;

  float distanceCm = (durationUs * SONAR_SOUND_CM_PER_US) * 0.5f;
  if (distanceCm < 0.0f) distanceCm = SONAR_NO_ECHO_DISTANCE_CM;
  return distanceCm;
}

static void onSonarFreezeChanged(bool freezeNow) {
  sonarFrozen = freezeNow;

  if (freezeNow) {
    manualCmd = IDLE;
    uint32_t now = millis();
    Serial.printf("[SONAR] FREEZE ON | distance=%.1f cm <= %.1f cm\n",
                  sonarDistanceCm, SONAR_STOP_DISTANCE_CM);
    lastSonarPrintMs = now;
  } else {
    uint32_t now = millis();
    Serial.printf("[SONAR] FREEZE OFF | distance=%.1f cm > %.1f cm | ramp=%.2f\n",
                  sonarDistanceCm, SONAR_STOP_DISTANCE_CM, sonarMotionScale);
    lastSonarPrintMs = now;
  }
}

static float computeSonarMotionScale(float distanceCm) {
  if (distanceCm >= SONAR_NO_ECHO_DISTANCE_CM) return 1.0f;
  if (distanceCm <= SONAR_STOP_DISTANCE_CM) return 0.0f;
  if (distanceCm >= SONAR_SLOW_START_DISTANCE_CM) return 1.0f;

  const float span = SONAR_SLOW_START_DISTANCE_CM - SONAR_STOP_DISTANCE_CM;
  if (span <= 1e-6f) return (distanceCm > SONAR_STOP_DISTANCE_CM) ? 1.0f : 0.0f;

  return clampf((distanceCm - SONAR_STOP_DISTANCE_CM) / span, 0.0f, 1.0f);
}

static void setSonarAvoidanceEnabled(bool enabled) {
  sonarAvoidanceEnabled = enabled;

  if (!sonarAvoidanceEnabled) {
    sonarObstacleDetected = false;
    sonarDistanceCm = SONAR_NO_ECHO_DISTANCE_CM;
    sonarMotionScale = 1.0f;
    if (sonarFrozen) {
      onSonarFreezeChanged(false);
    }
    Serial.println("[SONAR] Obstacle avoidance: DISABLED");
  } else {
    lastSonarSampleMs = millis() - SONAR_SAMPLE_INTERVAL_MS;
    lastSonarPrintMs = 0;
    Serial.println("[SONAR] Obstacle avoidance: ENABLED");
  }
}

static void updateSonarObstacleState() {
  if (!sonarAvoidanceEnabled) {
    sonarObstacleDetected = false;
    sonarDistanceCm = SONAR_NO_ECHO_DISTANCE_CM;
    sonarMotionScale = 1.0f;
    if (sonarFrozen) onSonarFreezeChanged(false);
    return;
  }

  uint32_t now = millis();
  if ((now - lastSonarSampleMs) < SONAR_SAMPLE_INTERVAL_MS) return;
  lastSonarSampleMs = now;

  sonarDistanceCm = readSonarDistanceCm();
  sonarMotionScale = computeSonarMotionScale(sonarDistanceCm);

  const bool shouldFreeze = (sonarMotionScale <= 0.0f);
  sonarObstacleDetected = (sonarDistanceCm < SONAR_SLOW_START_DISTANCE_CM);

  if (shouldFreeze != sonarFrozen) {
    onSonarFreezeChanged(shouldFreeze);
  } else if (now - lastSonarPrintMs > 1500) {
    lastSonarPrintMs = now;
    Serial.printf("[SONAR] distance=%.1f cm | ramp=%.2f | freeze=%s\n",
                  sonarDistanceCm, sonarMotionScale, sonarFrozen ? "ON" : "OFF");
  }
}

// ===============================
// 13) STRING SANITIZER
// ===============================
static void trimAndUpperInPlace(char* s) {
  if (!s) return;
  int start = 0;
  while (s[start] && (s[start] == ' ' || s[start] == '\t' || s[start] == '\r' || s[start] == '\n')) start++;
  if (start > 0) {
    int i = 0;
    while (s[start]) { s[i++] = s[start++]; }
    s[i] = 0;
  }
  int n = (int)strlen(s);
  while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t' || s[n-1] == '\r' || s[n-1] == '\n')) {
    s[n-1] = 0; n--;
  }
  for (int i = 0; s[i]; i++) {
    if (s[i] >= 'a' && s[i] <= 'z') s[i] = (char)(s[i] - 'a' + 'A');
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("==============================================================");
  Serial.println("[UDP] Commands:");
  Serial.println("  W/A/S/D : hold-to-walk (manual)");
  Serial.println("  IDLE    : stop walking");
  Serial.println("  Z       : toggle Calibration (Adjusted T-Pose)");
  Serial.println("  X       : toggle Demo sequence");
  Serial.println("  M       : toggle gait mode (TROT / CRAWL)");
  Serial.println("  - / +   : gait speed down/up");
  Serial.println("  1 / 2   : ARM lateral  - / +");
  Serial.println("  3 / 4   : ARM shoulder - / +");
  Serial.println("  5 / 6   : ARM elbow    - / +");
  Serial.println("  7 / 8   : ARM claw     - / +");
  Serial.println("  0       : ARM home / calibrated zero pose");
  Serial.println("  P       : toggle sonar obstacle avoidance ON/OFF");
  Serial.printf("  SONAR   : front HC-SR04 ramp %.1f->%.1f cm, full stop at %.1f cm on D0/D1 when enabled\n",
                SONAR_SLOW_START_DISTANCE_CM, SONAR_STOP_DISTANCE_CM, SONAR_STOP_DISTANCE_CM);
  Serial.println("==============================================================");
  Serial.println();
}

// ===============================
// 14) UDP PARSER
// ===============================
static void handlePacket(char* msg) {
  if (!msg || !msg[0]) return;
  trimAndUpperInPlace(msg);
  if (!msg[0]) return;

  if (strcmp(msg, "P") == 0) {
    Serial.println("P");
    uint32_t now = millis();
    if (now - lastPms >= KEY_DEBOUNCE_MS) {
      lastPms = now;
      setSonarAvoidanceEnabled(!sonarAvoidanceEnabled);
    }
    return;
  }

  if (sonarFrozen) {
    uint32_t now = millis();
    if (now - lastFrozenCmdIgnoreMs > 250) {
      lastFrozenCmdIgnoreMs = now;
      Serial.printf("[SONAR] Ignored UDP command while frozen: '%s'\n", msg);
    }
    return;
  }

  if (strcmp(msg, "Z") == 0) {
    uint32_t now = millis();
    if (now - lastZms >= KEY_DEBOUNCE_MS) {
      lastZms = now;
      calibrationMode = !calibrationMode;
      demoActive = false;
      manualCmd = IDLE;
      activeCmd = IDLE;
      demoIdx = 0;
      demoElapsed = 0.0f;
      setCommandTrajectories(IDLE);
      armSetHome(armManual);
      beginPoseTransition(calibrationMode);

      Serial.printf("[Mode] Calibration: %s\n", calibrationMode ? "ON" : "OFF");
      if (calibrationMode) {
        Serial.printf("[Info] Smooth transition -> Adjusted T-Pose + Arm home over %.2fs.\n", POSE_INTERP_DURATION_SEC);
      } else {
        Serial.printf("[Info] Smooth transition -> Idle legs + Arm home over %.2fs.\n", POSE_INTERP_DURATION_SEC);
      }
    }
    return;
  }

  if (strcmp(msg, "C") == 0 || strcmp(msg, "CAL_ADJ") == 0) {
    return;
  }

  if (strcmp(msg, "X") == 0) {
    uint32_t now = millis();
    if (poseTransition.active) {
      Serial.println("[Demo] Ignored X because pose transition is active.");
      return;
    }
    if (calibrationMode) {
      Serial.println("[Demo] Ignored X because Calibration is ON.");
      return;
    }
    if (now - lastXms >= KEY_DEBOUNCE_MS) {
      lastXms = now;

      if (!demoActive && (manualCmd != IDLE || activeCmd != IDLE)) {
        Serial.println("[Demo] Ignored X because robot is not IDLE.");
        return;
      }

      demoActive = !demoActive;
      demoIdx = 0;
      demoElapsed = 0.0f;

      if (demoActive && DEMO_LEN > 0) {
        activeCmd = DEMO_PATH[0].cmd;
        setCommandTrajectories(activeCmd);
        applyDemoArmStep(0);   // now starts smooth transition
        Serial.printf("[Demo] ON -> Step 0/%d | cmd=%s | armTarget=(%.1f, %.1f, %.1f, %.1f) | interp=%.1fs\n",
                      DEMO_LEN - 1,
                      CMD_NAME[(int)activeCmd],
                      armHomeTransition.target.latRelDeg,
                      armHomeTransition.target.shoulderRelDeg,
                      armHomeTransition.target.elbowRelDeg,
                      armHomeTransition.target.clawRelDeg,
                      DEMO_ARM_INTERP_DURATION_SEC);
      } else {
        demoActive = false;
        activeCmd = IDLE;
        setCommandTrajectories(IDLE);
        copyArmState(armManual, armActive);
        Serial.println("[Demo] OFF -> IDLE (manual ready).");
      }
    }
    return;
  }

  if (strcmp(msg, "M") == 0) {
    uint32_t now = millis();
    if (now - lastMms >= KEY_DEBOUNCE_MS) {
      lastMms = now;
      gaitMode = (gaitMode == GAIT_TROT) ? GAIT_CRAWL : GAIT_TROT;
      Serial.printf("[Gait] Mode = %s\n", GAIT_MODE_NAME[(int)gaitMode]);

      if (activeCmd != IDLE) {
        setCommandTrajectories(activeCmd);
      }
    }
    return;
  }

  if (strcmp(msg, "SPD_DN") == 0 || strcmp(msg, "-") == 0 || strcmp(msg, "[") == 0) {
    GAIT_SPEED = fmaxf(GAIT_SPEED_MIN, GAIT_SPEED - GAIT_SPEED_STEP);
    Serial.printf("[Gait] Speed = %.2fx\n", GAIT_SPEED);
    return;
  }
  if (strcmp(msg, "SPD_UP") == 0 || strcmp(msg, "+") == 0 || strcmp(msg, "]") == 0) {
    GAIT_SPEED = fminf(GAIT_SPEED_MAX, GAIT_SPEED + GAIT_SPEED_STEP);
    Serial.printf("[Gait] Speed = %.2fx\n", GAIT_SPEED);
    return;
  }

  if (poseTransition.active) {
    return;
  }

  if (calibrationMode) {
    if (strcmp(msg, "1") == 0 || strcmp(msg, "2") == 0 ||
        strcmp(msg, "3") == 0 || strcmp(msg, "4") == 0 ||
        strcmp(msg, "5") == 0 || strcmp(msg, "6") == 0 ||
        strcmp(msg, "7") == 0 || strcmp(msg, "8") == 0 ||
        strcmp(msg, "0") == 0 ||
        strcmp(msg, "W") == 0 || strcmp(msg, "S") == 0 ||
        strcmp(msg, "A") == 0 || strcmp(msg, "D") == 0 ||
        strcmp(msg, "IDLE") == 0) {
      return;
    }
  }

  if (demoActive) {
    if (strcmp(msg, "1") == 0 || strcmp(msg, "2") == 0 ||
        strcmp(msg, "3") == 0 || strcmp(msg, "4") == 0 ||
        strcmp(msg, "5") == 0 || strcmp(msg, "6") == 0 ||
        strcmp(msg, "7") == 0 || strcmp(msg, "8") == 0 ||
        strcmp(msg, "0") == 0 ||
        strcmp(msg, "W") == 0 || strcmp(msg, "S") == 0 ||
        strcmp(msg, "A") == 0 || strcmp(msg, "D") == 0 ||
        strcmp(msg, "IDLE") == 0) {
      return;
    }
  }

  int armLatDir = 0, armShoulderDir = 0, armElbowDir = 0, armClawDir = 0;
  if (sscanf(msg, "ARM:%d,%d,%d,%d", &armLatDir, &armShoulderDir, &armElbowDir, &armClawDir) == 4 ||
      sscanf(msg, "ARM %d %d %d %d", &armLatDir, &armShoulderDir, &armElbowDir, &armClawDir) == 4) {
    applyArmManualStep(ARM_STEP_DEG * unitFromInt(armLatDir),
                       ARM_STEP_DEG * unitFromInt(armShoulderDir),
                       ARM_STEP_DEG * unitFromInt(armElbowDir),
                       ARM_STEP_DEG * unitFromInt(armClawDir));
    return;
  }

  if (strcmp(msg, "1") == 0) { applyArmManualStep(-ARM_STEP_DEG, 0.0f, 0.0f, 0.0f); return; }
  if (strcmp(msg, "2") == 0) { applyArmManualStep(+ARM_STEP_DEG, 0.0f, 0.0f, 0.0f); return; }
  if (strcmp(msg, "3") == 0) { applyArmManualStep(0.0f, -ARM_STEP_DEG, 0.0f, 0.0f); return; }
  if (strcmp(msg, "4") == 0) { applyArmManualStep(0.0f, +ARM_STEP_DEG, 0.0f, 0.0f); return; }
  if (strcmp(msg, "5") == 0) { applyArmManualStep(0.0f, 0.0f, -ARM_STEP_DEG, 0.0f); return; }
  if (strcmp(msg, "6") == 0) { applyArmManualStep(0.0f, 0.0f, +ARM_STEP_DEG, 0.0f); return; }
  if (strcmp(msg, "7") == 0) { applyArmManualStep(0.0f, 0.0f, 0.0f, -ARM_STEP_DEG); return; }
  if (strcmp(msg, "8") == 0) { applyArmManualStep(0.0f, 0.0f, 0.0f, +ARM_STEP_DEG); return; }
  if (strcmp(msg, "0") == 0) {
    beginArmHomeTransition();
    Serial.printf("[Arm] HOME -> smooth return to calibrated zero pose over %.2fs.\n", POSE_INTERP_DURATION_SEC);
    return;
  }

  if (strcmp(msg, "W") == 0) manualCmd = FORWARD;
  else if (strcmp(msg, "S") == 0) manualCmd = BACKWARD;
  else if (strcmp(msg, "A") == 0) manualCmd = TURN_LEFT;
  else if (strcmp(msg, "D") == 0) manualCmd = TURN_RIGHT;
  else if (strcmp(msg, "IDLE") == 0) manualCmd = IDLE;
  else {
    Serial.printf("[Warn] Unknown packet after sanitize: '%s'\n", msg);
  }
}

// ===============================
// 15) SETUP / LOOP
// ===============================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}

  Serial.println("\n\n=== SYSTEM INITIALIZING ===");

  pinMode(SONAR_TRIG_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  Serial.printf("[HW] HC-SR04 ready | TRIG=D0 | ECHO=D1 | ramp %.1f->%.1f cm\n",
                SONAR_SLOW_START_DISTANCE_CM, SONAR_STOP_DISTANCE_CM);
  Serial.println("[SONAR] Obstacle avoidance default: DISABLED (press/send P to enable)");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  allServosAdjustedTPose();
  Serial.println("[HW] PCA9685 ready. Boot pose = Adjusted T-Pose for legs + arm.");

  Serial.println("[HW] Initializing IMU on shared I2C bus ...");
  imuOk = imuBegin();
  if (!imuOk) {
    Serial.println("[ERR] IMU init failed (ADXL345/ITG3205). Leveling disabled.");
  } else {
    imuCalibrateGyroBias();
    Serial.printf("[IMU] Gyro bias (deg/s): bx=%.4f by=%.4f bz=%.4f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
    Serial.println("[IMU] Leveling overlay READY (roll/pitch on legs only).");
  }

  for (int i = 0; i < 4; i++) {
    legs[i].cur = Vec3{REST_X, REST_Y, REST_Z};
    legs[i].segStart = legs[i].cur;
    legs[i].segEnd   = legs[i].cur;
    legs[i].segElapsed = 0.0f;
    legs[i].segDur = baseSegmentSec();
    legs[i].traj = nullptr;
    legs[i].trajLen = 0;
    legs[i].trajIdx = 0;
    legs[i].holdOnComplete = true;
    legs[i].activeCmd = IDLE;
    legs[i].kfAtCur = -1;
    legs[i].swingIdx = -1;
    legs[i].isSwing = false;
    legs[i].smoothBlend = 1.0f;
    legs[i].dyApplied = 0.0f;
    legs[i].seedPhase = false;
  }
  setCommandTrajectories(IDLE);

  armSetHome(armManual);
  armSetHome(armActive);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  wifiMulti.addAP(WIFI_NET1_SSID, WIFI_NET1_PASS);
  wifiMulti.addAP(WIFI_NET2_SSID, WIFI_NET2_PASS);
  wifiMulti.addAP(WIFI_NET3_SSID, WIFI_NET3_PASS);

  Serial.println("[WiFi] Connecting using WiFiMulti...");
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\n[WiFi] CONNECTED");
  Serial.print("[WiFi] SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("[WiFi] IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);

  printHelp();
  Serial.printf("[Mode] Calibration: ON | SonarAvoid=%s | Gait=%s | WalkSpeed=%.2fx | ArmHomeRel=(%.1f, %.1f, %.1f, %.1f)\n",
                sonarAvoidanceEnabled ? "ON" : "OFF",
                GAIT_MODE_NAME[(int)gaitMode],
                GAIT_SPEED,
                armActive.latRelDeg,
                armActive.shoulderRelDeg,
                armActive.elbowRelDeg,
                armActive.clawRelDeg);

  lastCtrlMs = millis();
  lastPacketMs = millis();
  lastWiFiCheckMs = millis();
}

void loop() {
  updateSonarObstacleState();

  uint32_t wifiNow = millis();
  if (wifiNow - lastWiFiCheckMs >= WIFI_CHECK_INTERVAL_MS) {
    lastWiFiCheckMs = wifiNow;
    uint8_t wifiStatus = wifiMulti.run();
    static bool wifiWasConnected = false;
    bool wifiConnected = (wifiStatus == WL_CONNECTED);
    if (wifiConnected && !wifiWasConnected) {
      Serial.print("[WiFi] Connected to: ");
      Serial.println(WiFi.SSID());
      Serial.print("[WiFi] IP: ");
      Serial.println(WiFi.localIP());
    } else if (!wifiConnected && wifiWasConnected) {
      Serial.println("[WiFi] Lost connection. WiFiMulti is searching for another known AP...");
    }
    wifiWasConnected = wifiConnected;
  }

  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(rxBuf, sizeof(rxBuf) - 1);
    if (len > 0) rxBuf[len] = 0;

#if DEBUG_RX
    Serial.printf("[RX] len=%d raw='%s'\n", len, rxBuf);
#endif

    lastPacketMs = millis();
    handlePacket(rxBuf);
  }

  if (!demoActive && !calibrationMode && !sonarFrozen) {
    uint32_t now = millis();
    if (now - lastPacketMs > CMD_TIMEOUT_MS) {
      manualCmd = IDLE;
    }
  }

  uint32_t nowMs = millis();
  if (nowMs - lastCtrlMs < (uint32_t)MICROSTEP_MS) return;

  float dtSec = (nowMs - lastCtrlMs) / 1000.0f;
  lastCtrlMs = nowMs;

  if (sonarFrozen) {
    // Full freeze: hold all servos at last angles, advance nothing.
  } else if (poseTransition.active) {
    tickPoseTransition(dtSec);
  } else if (calibrationMode) {
    allServosAdjustedTPose();
    armSetHome(armActive);
  } else {
    const float sonarMotionDtSec = dtSec * sonarMotionScale;

    if (demoActive) {
      demoElapsed += sonarMotionDtSec;
      if (demoElapsed >= DEMO_PATH[demoIdx].durationSec) {
        demoIdx++;
        demoElapsed = 0.0f;
        if (demoIdx >= DEMO_LEN) {
          demoActive = false;
          activeCmd = IDLE;
          setCommandTrajectories(IDLE);
          copyArmState(armManual, armActive);
          Serial.println("[Demo] Completed all steps -> Demo OFF -> IDLE.");
        } else {
          Command newCmd = DEMO_PATH[demoIdx].cmd;
          if (newCmd != activeCmd) {
            activeCmd = newCmd;
            setCommandTrajectories(activeCmd);
          }
          applyDemoArmStep(demoIdx);   // starts smooth transition to new pose
          Serial.printf("[Demo] Step %d/%d | cmd=%s | armTarget=(%.1f, %.1f, %.1f, %.1f) | interp=%.1fs\n",
                        demoIdx, DEMO_LEN - 1,
                        CMD_NAME[(int)activeCmd],
                        armHomeTransition.target.latRelDeg,
                        armHomeTransition.target.shoulderRelDeg,
                        armHomeTransition.target.elbowRelDeg,
                        armHomeTransition.target.clawRelDeg,
                        DEMO_ARM_INTERP_DURATION_SEC);
        }
      }
    } else {
      if (manualCmd != activeCmd) {
        activeCmd = manualCmd;
        setCommandTrajectories(activeCmd);
        Serial.printf("[Manual] walk cmd=%s\n", CMD_NAME[(int)activeCmd]);
      }
    }

    for (int i = 0; i < 4; i++) tickLeg(legs[i], sonarMotionDtSec);

    float rollRate=0, pitchRate=0, yawRate=0;
    if (imuOk) imuUpdate(dtSec, rollRate, pitchRate, yawRate);

    updateLevelingOverlay(activeCmd, dtSec, imuRollDeg, imuPitchDeg, rollRate, pitchRate);

    for (int i = 0; i < 4; i++) {
      Vec3 tgt = legs[i].cur;
      tgt.y += legs[i].dyApplied;

      float servoLat, servoSh, servoEl;
      bool outOfReach = false;
      computeLegServoAnglesForTarget(i, tgt, servoLat, servoSh, servoEl, outOfReach);

      if (outOfReach && (nowMs - lastReachWarnMs[i] > 1000)) {
        lastReachWarnMs[i] = nowMs;
        Serial.printf("[Warn][IK] Leg %d: target out of reach; clamped by IK.\n", i);
      }

      writeLegServoDeg(SERVO_CH[i][0], servoLat);
      writeLegServoDeg(SERVO_CH[i][1], servoSh);
      writeLegServoDeg(SERVO_CH[i][2], servoEl);
    }

    // ---- PATCH 7: arm update in loop ----
    // armHomeTransition handles BOTH demo interpolation AND "0" home transitions.
    // When demo is active and no transition is running, arm holds at last position.
    // Manual mode uses smoothArmTowardManual as before.
    if (armHomeTransition.active) {
      tickArmHomeTransition(dtSec);
    } else if (!demoActive) {
      smoothArmTowardManual(dtSec);
    }
    // demoActive + no active transition = arm holds at completed pose automatically

    driveArmServosFromState(armActive);
  }

  if (nowMs - lastStatusMs > 800) {
    lastStatusMs = nowMs;
    Serial.printf("[Status] Cal=%s | Demo=%s | SonarAvoid=%s | Freeze=%s | Sonar=%.1fcm | Ramp=%.2f | PoseMove=%s | ArmMove=%s | Gait=%s | Cmd=%s | Speed=%.2fx | roll=%.2f pitch=%.2f | ArmRel=(%.1f, %.1f, %.1f, %.1f)\n",
                  calibrationMode ? "ON":"OFF",
                  demoActive ? "ON":"OFF",
                  sonarAvoidanceEnabled ? "ON":"OFF",
                  sonarFrozen ? "ON":"OFF",
                  sonarDistanceCm,
                  sonarMotionScale,
                  poseTransition.active ? "ON":"OFF",
                  armHomeTransition.active ? "ON":"OFF",
                  GAIT_MODE_NAME[(int)gaitMode],
                  CMD_NAME[(int)activeCmd],
                  GAIT_SPEED,
                  imuRollDeg,
                  imuPitchDeg,
                  armActive.latRelDeg,
                  armActive.shoulderRelDeg,
                  armActive.elbowRelDeg,
                  armActive.clawRelDeg);
  }
}