// ============================================================================
// esp_drive — micro-ROS firmware v1 (connection test) — Re4Life robot
// Board: classic ESP32 DevKitC. Library: micro_ros_arduino, JAZZY release.
//
// ROS interface (contract v4 §3b):
//   sub  /cmd_vel          geometry_msgs/Twist   (reliable)  -> diff-drive IK
//   sub  /e_stop           std_msgs/Bool         (reliable)  -> active brake
//   pub  /wheel_telemetry  sensor_msgs/JointState (best-effort, 5 Hz)
//                          name=[wheel_left,wheel_right],
//                          position=cumulative m, velocity=rad/s
//   pub  /battery_voltage  std_msgs/Float32      (best-effort, 1 Hz)
//                          V1-TEMPORARY: replaced by robot_interfaces/MotorStatus
//                          in v2 (needs micro-ROS lib rebuild — see firmware README)
//
// SAFETY (the reason this board exists):
//   * WATCHDOG: no /cmd_vel for 500 ms  -> zero current. ALWAYS ON.
//     (The legacy firmware had this commented out. Never again.)
//   * /e_stop true -> VESC active brake, latched until false. Release != resume:
//     motors stay idle until a fresh /cmd_vel arrives.
//   * Agent disconnected -> motors stopped (independent of the watchdog).
//
// V1 NOT included (deferred arm / custom msgs): /servo_cmd, /servo_state,
// /motor_status. Servos are initialized to their start positions and held.
//
// IMPORTANT: USB Serial IS the micro-ROS transport. Never Serial.print() here.
// Status is on the onboard LED: slow blink = waiting for agent, solid = connected.
//
// Timestamps are left zero (no time sync in v1); base_controller stamps on
// arrival. v2: rmw_uros_sync_session for real stamps (needed when ultrasonic
// data feeds collision_monitor).
// ============================================================================

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32.h>

#include <Arduino.h>
#include <VescUart.h>
#include <ESP32Servo.h>

// ── Mechanical constants (contract §3b — compiled in on purpose) ────────────
static const float WHEEL_DIAMETER_M   = 0.165f;   // 6.5" hoverboard wheel
static const int   POLE_PAIRS         = 15;       // mech rpm * 15 = ERPM
static const float MAX_MECH_RPM       = 120.0f;
// !!! PLACEHOLDER — MEASURE center-to-center of the wheel contact patches.
// Must match base_controller's wheel_track parameter.
static const float WHEEL_TRACK_M      = 0.50f;    // TODO MEASURE
// Right motor is mounted mirrored: forward = negative ERPM. The sign flip
// lives HERE and nowhere else (contract §3b / legacy bug #4).
static const float RIGHT_SIGN         = -1.0f;

static const float MM_PER_TACH_TICK   = (165.0f * PI) / (POLE_PAIRS * 6.0f);

// ── Timing / safety ──────────────────────────────────────────────────────────
static const uint32_t WATCHDOG_TIMEOUT_MS   = 500;   // MANDATORY
static const uint32_t MOTOR_UPDATE_MS       = 65;    // ramp tick (legacy value)
static const uint32_t TELEMETRY_MS          = 200;   // 5 Hz JointState
static const uint32_t BATTERY_MS            = 1000;
static const float    MAX_ERPM_DELTA_PER_S  = 150.0f * 15.0f; // legacy ramp, spike backstop
static const float    BRAKE_CURRENT_A       = 8.0f;  // e_stop active brake

#define LED_PIN 2

// ── VESC wiring (legacy pins kept) ───────────────────────────────────────────
static const int RX1 = 16, TX1 = 17;   // left VESC
static const int RX2 = 22, TX2 = 21;   // right VESC
HardwareSerial VescSerial1(1);
HardwareSerial VescSerial2(2);
VescUart vescLeft;
VescUart vescRight;

// ── Servos: deferred arm — initialized and held, no ROS interface in v1 ─────
class AsyncServo {
  Servo _s; int _pin, _cur, _tgt, _min, _max; unsigned long _last; int _step;
public:
  AsyncServo(int pin, int mn, int mx, int start)
    : _pin(pin), _cur(constrain(start, mn, mx)), _tgt(_cur),
      _min(mn), _max(mx), _last(0), _step(40) {}
  void begin() { _s.setPeriodHertz(50); _s.attach(_pin, 500, 2500); _s.write(_cur); }
  void update() {
    if (_cur == _tgt) return;
    if (millis() - _last >= (unsigned long)_step) {
      _last = millis();
      _cur += (_cur < _tgt) ? 1 : -1;
      _s.write(_cur);
    }
  }
};
AsyncServo baseServo(13, 60, 120, 90);
AsyncServo elbowServo(27, 10, 170, 10);
AsyncServo gripperServo(26, 40, 100, 100);

// ── Drive state ──────────────────────────────────────────────────────────────
volatile float targetErpmL = 0, targetErpmR = 0;     // ROS-frame (forward +)
float commandedErpmL = 0, commandedErpmR = 0;
volatile uint32_t lastCmdMs = 0;
volatile bool eStop = false;
long  startTachoL = 0, startTachoR = 0;
bool  haveTachoL = false, haveTachoR = false;
uint32_t lastMotorMs = 0, lastTelemMs = 0, lastBattMs = 0;

// ── micro-ROS entities ───────────────────────────────────────────────────────
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_e_stop;
rcl_publisher_t pub_joint;
rcl_publisher_t pub_batt;
geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Bool msg_e_stop;
sensor_msgs__msg__JointState msg_joint;
std_msgs__msg__Float32 msg_batt;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;

#define RCCHECK(fn)  { if ((fn) != RCL_RET_OK) return false; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t _t = -1; \
  if (_t == -1) _t = uxr_millis(); \
  if ((int64_t)(uxr_millis() - _t) > (MS)) { X; _t = uxr_millis(); } \
} while (0)

// ── Motor helpers ────────────────────────────────────────────────────────────
void stopMotorsHard() {
  vescLeft.setCurrent(0.0f);
  vescRight.setCurrent(0.0f);
  targetErpmL = 0; targetErpmR = 0;
  commandedErpmL = 0; commandedErpmR = 0;
}

void applyBrake() {
  vescLeft.setBrakeCurrent(BRAKE_CURRENT_A);
  vescRight.setBrakeCurrent(BRAKE_CURRENT_A);
  targetErpmL = 0; targetErpmR = 0;
  commandedErpmL = 0; commandedErpmR = 0;
}

// ── ROS callbacks ────────────────────────────────────────────────────────────
void onCmdVel(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  if (eStop) return;                       // locked until /e_stop false
  const float v = (float)m->linear.x;      // m/s
  const float w = (float)m->angular.z;     // rad/s
  float vL = v - w * WHEEL_TRACK_M * 0.5f;
  float vR = v + w * WHEEL_TRACK_M * 0.5f;
  // m/s -> mech rpm -> ERPM, clamped
  const float mps2rpm = 60.0f / (PI * WHEEL_DIAMETER_M);
  float rpmL = constrain(vL * mps2rpm, -MAX_MECH_RPM, MAX_MECH_RPM);
  float rpmR = constrain(vR * mps2rpm, -MAX_MECH_RPM, MAX_MECH_RPM);
  targetErpmL = rpmL * POLE_PAIRS;
  targetErpmR = rpmR * POLE_PAIRS;
  lastCmdMs = millis();
}

void onEStop(const void *msgin) {
  const std_msgs__msg__Bool *m = (const std_msgs__msg__Bool *)msgin;
  if (m->data) {
    eStop = true;
    applyBrake();
  } else {
    eStop = false;                         // release != resume: targets stay 0
    stopMotorsHard();
  }
}

// ── Ramp + apply (spike backstop only; shaping is velocity_smoother's job) ──
void updateRamps(float dt) {
  if (eStop) { applyBrake(); return; }

  // WATCHDOG — the one line the old robot was missing. 500 ms of command
  // silence is an EMERGENCY, not a smoothing case: hard zero-current stop
  // immediately, no ramp-down (a ramped decay would take ~0.8 s extra from
  // full speed).
  if (millis() - lastCmdMs > WATCHDOG_TIMEOUT_MS) {
    stopMotorsHard();
    return;
  }

  const float maxDelta = MAX_ERPM_DELTA_PER_S * dt;
  float dL = targetErpmL - commandedErpmL;
  commandedErpmL += (fabsf(dL) > maxDelta) ? copysignf(maxDelta, dL) : dL;
  float dR = targetErpmR - commandedErpmR;
  commandedErpmR += (fabsf(dR) > maxDelta) ? copysignf(maxDelta, dR) : dR;

  if (commandedErpmL == 0 && commandedErpmR == 0 &&
      targetErpmL == 0 && targetErpmR == 0) {
    vescLeft.setCurrent(0.0f);             // idle: no torque, no heat
    vescRight.setCurrent(0.0f);
  } else {
    vescLeft.setRPM(commandedErpmL);
    vescRight.setRPM(RIGHT_SIGN * commandedErpmR);
  }
}

// ── Telemetry ────────────────────────────────────────────────────────────────
void publishTelemetry() {
  // Last-good values persist across failed VESC reads: publishing zeros on a
  // read glitch would make cumulative position jump 0-and-back (tripping
  // base_controller's jump guard and silently discarding real travel) and
  // fake a 0 V battery. A failed read repeats the last truth instead.
  static float posL = 0, posR = 0, velL = 0, velR = 0;
  static float volts = 0;
  static bool everGotVolts = false;

  bool gotL = vescLeft.getVescValues();
  bool gotR = vescRight.getVescValues();

  if (gotL) {
    if (!haveTachoL) { startTachoL = vescLeft.data.tachometer; haveTachoL = true; }
    posL  = (vescLeft.data.tachometer - startTachoL) * MM_PER_TACH_TICK / 1000.0f;
    velL  = (vescLeft.data.rpm / (float)POLE_PAIRS) * 2.0f * PI / 60.0f;
    volts = vescLeft.data.inpVoltage;
    everGotVolts = true;
  }
  if (gotR) {
    if (!haveTachoR) { startTachoR = vescRight.data.tachometer; haveTachoR = true; }
    posR = RIGHT_SIGN * (vescRight.data.tachometer - startTachoR) * MM_PER_TACH_TICK / 1000.0f;
    velR = RIGHT_SIGN * (vescRight.data.rpm / (float)POLE_PAIRS) * 2.0f * PI / 60.0f;
    if (!gotL) { volts = vescRight.data.inpVoltage; everGotVolts = true; }
  }

  // Publish telemetry only once at least one VESC has ever answered —
  // otherwise /wheel_telemetry stays SILENT, which is what the connection
  // test treats as "VESC link down" (an honest signal, unlike zeros).
  if (haveTachoL || haveTachoR) {
    msg_joint.position.data[0] = posL;
    msg_joint.position.data[1] = posR;
    msg_joint.velocity.data[0] = velL;
    msg_joint.velocity.data[1] = velR;
    rcl_publish(&pub_joint, &msg_joint, NULL);
  }

  if (everGotVolts && millis() - lastBattMs >= BATTERY_MS) {
    lastBattMs = millis();
    msg_batt.data = volts;
    rcl_publish(&pub_batt, &msg_batt, NULL);
  }
}

// ── Entity lifecycle ─────────────────────────────────────────────────────────
bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp_drive", "", &support));

  RCCHECK(rclc_subscription_init_default(&sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&sub_e_stop, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/e_stop"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_joint, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/wheel_telemetry"));
  RCCHECK(rclc_publisher_init_best_effort(&pub_batt, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/battery_voltage"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
                                         &onCmdVel, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_e_stop, &msg_e_stop,
                                         &onEStop, ON_NEW_DATA));
  lastCmdMs = millis();   // arm the watchdog from "now", not from boot
  return true;
}

void destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rcl_subscription_fini(&sub_e_stop, &node);
  rcl_publisher_fini(&pub_joint, &node);
  rcl_publisher_fini(&pub_batt, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ── Arduino ──────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_PIN, OUTPUT);
  set_microros_transports();              // USB Serial = transport. No prints!

  VescSerial1.begin(115200, SERIAL_8N1, RX1, TX1);
  vescLeft.setSerialPort(&VescSerial1);
  VescSerial2.begin(115200, SERIAL_8N1, RX2, TX2);
  vescRight.setSerialPort(&VescSerial2);

  baseServo.begin();
  elbowServo.begin();
  gripperServo.begin();

  // Static JointState memory: 2 joints, bounded names — micro-ROS needs every
  // sequence preallocated (contract §2 rationale).
  micro_ros_utilities_memory_conf_t conf = {0};
  conf.max_string_capacity = 16;
  conf.max_ros2_type_sequence_capacity = 2;
  conf.max_basic_type_sequence_capacity = 2;
  micro_ros_utilities_create_message_memory(
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      &msg_joint, conf);
  msg_joint.name.size = 2;
  msg_joint.name.data[0] = micro_ros_string_utilities_set(msg_joint.name.data[0], "wheel_left");
  msg_joint.name.data[1] = micro_ros_string_utilities_set(msg_joint.name.data[1], "wheel_right");
  msg_joint.position.size = 2;
  msg_joint.velocity.size = 2;
  msg_joint.effort.size = 0;

  stopMotorsHard();
  agentState = WAITING_AGENT;
}

void loop() {
  // ── Agent connection state machine (canonical micro_ros_arduino pattern) ──
  switch (agentState) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                     ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      // Mid-way create failure -> partially-initialized entities; fini on
      // those is undefined territory. Zero the motors and reboot clean
      // (rare event; reconnects in ~2 s; VESCs sit idle meanwhile).
      if (!createEntities()) {
        stopMotorsHard();
        ESP.restart();
      }
      agentState = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      // Ping every 5 s (was 2 s): failing pings block up to 300 ms and
      // over-eager pinging of a busy serial link can flap the connection.
      // Safety unaffected: the 500 ms /cmd_vel watchdog is the stop
      // mechanism, not the ping.
      EXECUTE_EVERY_N_MS(5000,
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                     ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agentState == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case AGENT_DISCONNECTED:
      // Pi/agent death -> robot stops. Layer 3. BUT: if we were e-stopped,
      // a plain zero-current would RELEASE the brake (freewheel on a slope) —
      // keep braking instead.
      if (eStop) applyBrake(); else stopMotorsHard();
      destroyEntities();
      agentState = WAITING_AGENT;
      break;
  }

  digitalWrite(LED_PIN,
      (agentState == AGENT_CONNECTED) ? HIGH : ((millis() / 400) % 2));

  // ── Drive loop runs regardless of agent state (watchdog needs it) ─────────
  uint32_t now = millis();
  if (now - lastMotorMs >= MOTOR_UPDATE_MS) {
    float dt = (now - lastMotorMs) / 1000.0f;
    if (dt > 0.2f) dt = 0.2f;
    lastMotorMs = now;
    if (agentState == AGENT_CONNECTED) {
      updateRamps(dt);
    } else if (eStop) {
      applyBrake();   // keep the brake asserted even with no agent
    }
    // not connected + not e-stopped: motors were already zeroed; stay so
  }

  if (agentState == AGENT_CONNECTED && now - lastTelemMs >= TELEMETRY_MS) {
    lastTelemMs = now;
    publishTelemetry();
  }

  baseServo.update();
  elbowServo.update();
  gripperServo.update();
  delay(1);
}
