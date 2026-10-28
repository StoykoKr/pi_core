// ============================================================================
// esp_sensors — micro-ROS firmware v1 (connection test) — Re4Life robot
// Board: classic ESP32 DevKitC. Library: micro_ros_arduino, JAZZY release.
//
// ROS interface (contract v4 §4–5):
//   pub  /ultrasonic/left|mid|right|hand  sensor_msgs/Range (best-effort, ~4 Hz)
//        SAFETY-ONLY by contract; currently DEFERRED — no consumers exist.
//   sub  /ui/qr_payload   std_msgs/String (reliable, <=600 B)
//        -> forwarded to the CYD over UART2 as "QR:{json}\n"
//        (the SECURE WIRED channel — the QR never travels over WiFi)
//   pub  /ui/request_qr   std_msgs/Empty (reliable)
//        <- published when the CYD sends "<REQ_QR>" over UART2
//
// V1 NOT included (custom msg, hardware unplugged anyway): /actuator_cmd —
// the AsyncRelay driver is kept compiled and idle; v2 wires it to
// robot_interfaces/ActuatorCommand after the micro-ROS lib rebuild.
//
// IMPORTANT: USB Serial IS the micro-ROS transport — no Serial.print().
// UART2 (pins 4 RX / 27 TX, legacy wiring) talks to the CYD.
// LED: slow blink = waiting for agent, solid = connected.
// ============================================================================

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/empty.h>

#include <Arduino.h>

// ── Pins (legacy wiring kept) ────────────────────────────────────────────────
#define TRIG_PIN       26
#define ECHO_HAND_PIN  16
#define ECHO_MID_PIN   17
#define ECHO_LEFT_PIN  18
#define ECHO_RIGHT_PIN 5

#define RELAY_3 22
#define RELAY_4 21
#define RELAY_5 23
#define RELAY_6 19

#define CYD_RX_PIN 4    // from CYD TX (yellow cable, last checked)
#define CYD_TX_PIN 27   // to CYD RX   (blue cable, last checked)

#define LED_PIN 2

// Contract §2: provisioning_node caps the QR payload at 600 BYTES. The CDR
// string on the wire carries length+null, and micro-ROS rejects a message
// when length+1 > capacity — so the buffer must be 601 or a payload of
// exactly 600 bytes is dropped SILENTLY on both sides. 601, not 600.
#define QR_BUF_SZ 601

// ── Relay actuators (kept, idle in v1 — /actuator_cmd arrives in v2) ─────────
class AsyncRelay {
  int _a, _b, _state, _pending; unsigned long _last; bool _waiting;
  static const int DEADTIME_MS = 50;      // H-bridge: NEVER both HIGH
public:
  AsyncRelay(int a, int b) : _a(a), _b(b), _state(0), _pending(0), _waiting(false) {}
  void begin() { pinMode(_a, OUTPUT); pinMode(_b, OUTPUT);
                 digitalWrite(_a, LOW); digitalWrite(_b, LOW); }
  void set(int action) {
    if (action == _state) return;
    _pending = action;
    digitalWrite(_a, LOW); digitalWrite(_b, LOW);
    _waiting = true; _last = millis();
  }
  void update() {
    if (!_waiting || millis() - _last < DEADTIME_MS) return;
    _waiting = false; _state = _pending;
    digitalWrite(_a, _state == 1 ? HIGH : LOW);
    digitalWrite(_b, _state == 2 ? HIGH : LOW);
  }
};
AsyncRelay actuatorOne(RELAY_3, RELAY_4);
AsyncRelay actuatorTwo(RELAY_5, RELAY_6);

// ── Ultrasonic state (legacy ISR + median-of-3 kept) ─────────────────────────
volatile bool canCollect[4] = {false, false, false, false};
volatile bool canPing[4]    = {true, true, true, true};
volatile unsigned long tStart[4] = {0, 0, 0, 0};
volatile unsigned long tDur[4]   = {0, 0, 0, 0};
volatile bool newData[4]    = {false, false, false, false};
double medianBuf[4][3] = {{0}};
double medianVal[4]    = {0, 0, 0, 0};
// index: 0=left 1=mid 2=right 3=hand
unsigned long lastTrigger = 0;

#define MAKE_ISR(IDX, PIN) \
void IRAM_ATTR isr_##IDX() { \
  if (digitalRead(PIN) == LOW) { \
    if (canCollect[IDX] && !canPing[IDX]) { \
      tDur[IDX] = micros() - tStart[IDX]; canCollect[IDX] = false; newData[IDX] = true; } \
  } else if (canPing[IDX]) { \
    tStart[IDX] = micros(); canPing[IDX] = false; canCollect[IDX] = true; } \
}
MAKE_ISR(0, ECHO_LEFT_PIN)
MAKE_ISR(1, ECHO_MID_PIN)
MAKE_ISR(2, ECHO_RIGHT_PIN)
MAKE_ISR(3, ECHO_HAND_PIN)

static double median3(double a, double b, double c) {
  if (a > b) { double t = a; a = b; b = t; }
  if (b > c) { double t = b; b = c; c = t; }
  if (a > b) { double t = a; a = b; b = t; }
  return b;
}

void serviceUltrasonics() {
  unsigned long now = millis();
  if (now - lastTrigger >= 75) {
    for (int i = 0; i < 4; i++) canPing[i] = true;
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    lastTrigger = now;
  }
  for (int i = 0; i < 4; i++) {
    if (!newData[i]) continue;
    newData[i] = false;
    double cm = (tDur[i] * 0.0343) / 2.0;
    medianBuf[i][0] = medianBuf[i][1];
    medianBuf[i][1] = medianBuf[i][2];
    medianBuf[i][2] = cm;
    medianVal[i] = median3(medianBuf[i][0], medianBuf[i][1], medianBuf[i][2]);
  }
}

// ── CYD UART line reader ─────────────────────────────────────────────────────
static const int CYD_BUF_SZ = 64;
char cydBuf[CYD_BUF_SZ];
int cydIdx = 0;
volatile bool qrRequested = false;

void serviceCydUart() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\n' || c == '\r') {
      if (cydIdx > 0) {
        cydBuf[cydIdx] = 0;
        if (strstr(cydBuf, "REQ_QR") != NULL) qrRequested = true;
        cydIdx = 0;
      }
    } else if (cydIdx < CYD_BUF_SZ - 1) {
      cydBuf[cydIdx++] = c;
    } else {
      cydIdx = 0;  // overflow — reset
    }
  }
}

// ── micro-ROS entities ───────────────────────────────────────────────────────
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t pub_range[4];
rcl_publisher_t pub_req_qr;
rcl_subscription_t sub_qr;
sensor_msgs__msg__Range msg_range[4];
std_msgs__msg__Empty msg_req_qr;
std_msgs__msg__String msg_qr;
static char qrPayloadBuf[QR_BUF_SZ];

const char *RANGE_TOPICS[4] = {"/ultrasonic/left", "/ultrasonic/mid",
                               "/ultrasonic/right", "/ultrasonic/hand"};
const char *RANGE_FRAMES[4] = {"us_left", "us_mid", "us_right", "us_hand"};

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agentState = WAITING_AGENT;
uint32_t lastRangePubMs = 0;

#define RCCHECK(fn)  { if ((fn) != RCL_RET_OK) return false; }
#define EXECUTE_EVERY_N_MS(MS, X) do { \
  static volatile int64_t _t = -1; \
  if (_t == -1) _t = uxr_millis(); \
  if ((int64_t)(uxr_millis() - _t) > (MS)) { X; _t = uxr_millis(); } \
} while (0)

void onQrPayload(const void *msgin) {
  const std_msgs__msg__String *m = (const std_msgs__msg__String *)msgin;
  // Forward over the wire exactly as screen.ino expects: "QR:{json}\n"
  Serial2.print("QR:");
  Serial2.write((const uint8_t *)m->data.data, m->data.size);
  Serial2.print("\n");
}

bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp_sensors", "", &support));

  for (int i = 0; i < 4; i++) {
    RCCHECK(rclc_publisher_init_best_effort(&pub_range[i], &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), RANGE_TOPICS[i]));
  }
  RCCHECK(rclc_publisher_init_default(&pub_req_qr, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty), "/ui/request_qr"));
  RCCHECK(rclc_subscription_init_default(&sub_qr, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/ui/qr_payload"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_qr, &msg_qr,
                                         &onQrPayload, ON_NEW_DATA));
  return true;
}

void destroyEntities() {
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  for (int i = 0; i < 4; i++) rcl_publisher_fini(&pub_range[i], &node);
  rcl_publisher_fini(&pub_req_qr, &node);
  rcl_subscription_fini(&sub_qr, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void publishRanges() {
  for (int i = 0; i < 4; i++) {
    msg_range[i].range = (float)(medianVal[i] / 100.0);  // cm -> m
    rcl_publish(&pub_range[i], &msg_range[i], NULL);
  }
}

// ── Arduino ──────────────────────────────────────────────────────────────────
void setup() {
  pinMode(LED_PIN, OUTPUT);
  set_microros_transports();              // USB Serial = transport. No prints!
  Serial2.begin(115200, SERIAL_8N1, CYD_RX_PIN, CYD_TX_PIN);

  actuatorOne.begin();
  actuatorTwo.begin();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_LEFT_PIN, INPUT);
  pinMode(ECHO_MID_PIN, INPUT);
  pinMode(ECHO_RIGHT_PIN, INPUT);
  pinMode(ECHO_HAND_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_LEFT_PIN),  isr_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_MID_PIN),   isr_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_RIGHT_PIN), isr_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_HAND_PIN),  isr_3, CHANGE);

  // Static Range messages: frame_id set once (contract: frames us_left etc.)
  for (int i = 0; i < 4; i++) {
    msg_range[i].header.frame_id =
        micro_ros_string_utilities_set(msg_range[i].header.frame_id, RANGE_FRAMES[i]);
    msg_range[i].radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    msg_range[i].field_of_view = 0.26f;   // ~15 deg HC-SR04 cone
    msg_range[i].min_range = 0.02f;
    msg_range[i].max_range = 4.0f;
  }

  // Static buffer for the incoming QR payload (contract: 600 B cap)
  msg_qr.data.data = qrPayloadBuf;
  msg_qr.data.size = 0;
  msg_qr.data.capacity = QR_BUF_SZ;

  agentState = WAITING_AGENT;
}

void loop() {
  switch (agentState) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500,
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                     ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      agentState = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (agentState == WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(2000,
        agentState = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                     ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (agentState == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      }
      break;
    case AGENT_DISCONNECTED:
      destroyEntities();
      agentState = WAITING_AGENT;
      break;
  }

  digitalWrite(LED_PIN,
      (agentState == AGENT_CONNECTED) ? HIGH : ((millis() / 400) % 2));

  serviceUltrasonics();
  serviceCydUart();
  actuatorOne.update();
  actuatorTwo.update();

  if (agentState == AGENT_CONNECTED) {
    uint32_t now = millis();
    if (now - lastRangePubMs >= 250) {    // ~4 Hz, legacy cadence
      lastRangePubMs = now;
      publishRanges();
    }
    if (qrRequested) {
      qrRequested = false;
      rcl_publish(&pub_req_qr, &msg_req_qr, NULL);
    }
  } else {
    qrRequested = false;                  // don't queue stale touches
  }

  delay(1);
}
