#include <Arduino.h>
#include <VescUart.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SCH16T.h>  //Library for the murata imu

class AsyncServo {
private:
  Servo _servo;
  int _pin;
  int _currentPos;
  int _targetPos;
  int _minLimit;
  int _maxLimit;

  unsigned long _lastUpdate;
  int _stepInterval;  // Delay in ms between steps (controls speed)

public:
  AsyncServo(int pin, int minLim, int maxLim, int startPos) {
    _pin = pin;
    _minLimit = minLim;
    _maxLimit = maxLim;
    _currentPos = constrain(startPos, _minLimit, _maxLimit);
    _targetPos = _currentPos;
    _stepInterval = 40;  // Default speed
  }

  void begin() {
    _servo.setPeriodHertz(50);
    _servo.attach(_pin, 500, 2500);
    _servo.write(_currentPos);
  }

  // Call this to set a new destination
  void setTarget(int target, int speedDelay = 40) {
    _targetPos = constrain(target, _minLimit, _maxLimit);
    _stepInterval = speedDelay;
    // speedDelay: smaller number = faster, larger = slower
  }

  // Call this inside void loop() every time
  void update() {
    // If we are already at target, do nothing
    if (_currentPos == _targetPos) return;

    // Check if enough time has passed to take a step
    if (millis() - _lastUpdate >= _stepInterval) {
      _lastUpdate = millis();

      if (_currentPos < _targetPos) {
        _currentPos++;
      } else {
        _currentPos--;
      }
      _servo.write(_currentPos);
    }
  }

  int getCurrentPos() {
    return _currentPos;
  }
  bool isMoving() {
    return _currentPos != _targetPos;
  }
};

static const int rx1 = 16;  // VESC1 tx -> ESP32 RX1
static const int tx1 = 17;  // VESC1 RX -> ESP32 tx1  zelenoto na vesc e rx znachi
static const int rx2 = 22;  // VESC2 tx -> ESP32 RX2
static const int tx2 = 21;  // VESC2 RX -> ESP32 tx2


// zelenoto e bqlo za celite na ednoto
/*
#define SPI_OBJECT SPI  // Some platforms have additional SPI intefaces under different names (e.g. SPI1)
#define SPI_SCK_PIN 18
#define SPI_MOSI_PIN 23
#define SPI_MISO_PIN 19  // (Standard is 12, but 12 can cause boot fail if pulled high)
#define CS_PIN 25
#define RESET_PIN -1  // optional, keep -1 if not used
#define DRDY_PIN -1   // optional DRDY interrupt (choose any non-boot pin)

#define FILTER_RATE 68.0f  // Hz, LPF0 Nominal Cut-off Frequency (-3dB).
#define FILTER_ACC12 68.0f
#define FILTER_ACC3 68.0f
#define SENSITIVITY_RATE1 3200.0f  // LSB / dps, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_RATE2 3200.0f
#define SENSITIVITY_ACC1 3200.0f  // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define SENSITIVITY_ACC2 3200.0f
#define SENSITIVITY_ACC3 3200.0f  // LSB / m/s2, DYN1 Nominal Sensitivity for 20 bit data.
#define DECIMATION_RATE 4         // DEC2, Output sample rate decimation. Nominal output rate of 5.9kHz.
#define DECIMATION_ACC 4

SCH16T_K01 imu(SPI_OBJECT, CS_PIN, RESET_PIN);
int init_status = SCH16T_ERR_OTHER;
SCH16T_filter Filter;
SCH16T_sensitivity Sensitivity;
SCH16T_decimation Decimation;
SCH16T_raw_data raw;
SCH16T_result result;

*/
HardwareSerial VescSerial1(1);
HardwareSerial VescSerial2(2);

VescUart vescLeft;
VescUart vescRight;

// Define your pins and limits
// AsyncServo name(PIN, MIN_LIMIT, MAX_LIMIT, START_POS);

AsyncServo baseServo(13, 60, 120, 90);
AsyncServo elbowServo(27, 10, 170, 10);
AsyncServo gripperServo(26, 40, 100, 100);
//AsyncServo raiseServo(25, 60, 120, 90);

static const float WHEEL_DIAMETER_MM = 165.0;  // 6.5 inch standard hoverboard wheel
static const int POLE_PAIRS = 15;
static const float MM_PER_TACH_TICK = (WHEEL_DIAMETER_MM * PI) / (POLE_PAIRS * 6.0);

long startTachoL = 0;
bool hasStartTachoL = false;
long startTachoR = 0;
bool hasStartTachoR = false;


static const unsigned long TELEMETRY_PERIOD_MS = 200;  // send telemetry to Pi every 200 ms
static const unsigned long MOTOR_UPDATE_PERIOD_MS = 65;
static const unsigned long WATCHDOG_TIMEOUT_MS = 3000;  // if no command within this -> stop motors


static const float MAX_RPM_DELTA_PER_SEC = 150.0 * 15;  // maximum rpm change per second
static const float MAX_CURRENT_DELTA_PER_SEC = 20.0;    // amps per second
static const float MAX_DUTY_DELTA_PER_SEC = 2.0;        // duty units per second

unsigned long lastMotorUpdateMillis = 0;
volatile float targetRpmL = 0.0, targetRpmR = 0.0;
volatile float commandedRpmL = 0.0, commandedRpmR = 0.0;

volatile float targetCurrL = 0.0, targetCurrR = 0.0;
volatile float commandedCurrL = 0.0, commandedCurrR = 0.0;

volatile float targetDutyL = 0.0, targetDutyR = 0.0;
volatile float commandedDutyL = 0.0, commandedDutyR = 0.0;

enum ControlMode { MODE_IDLE = 0,
                   MODE_RPM,
                   MODE_CURRENT,
                   MODE_DUTY };
volatile ControlMode activeMode = MODE_IDLE;


unsigned long lastCommandMillis = 0;
unsigned long lastTelemetryMillis = 0;

constexpr float MAX_RPM = 120.0f;

static const int INBUF_SZ = 192;
char inBuf[INBUF_SZ];
int inIdx = 0;

void processLine(const char *line);
void updateRamps(float dt);
void sendTelemetryIfDue();
void stopAllMotorsImmediate();
void ackToPi(const char *s);
static unsigned long lastLoop = millis();
unsigned long currentMillis = millis();


void setup() {
  Serial.begin(115200);
/*
  SPI_OBJECT.begin(SPI_SCK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, CS_PIN);
  delay(50);
  configureIMU();
  init_status = SCH16T_ERR_OTHER;
  while (init_status != SCH16T_OK) {
    init_status = imu.begin(Filter, Sensitivity, Decimation, false);
    Serial.print("<begin() returned: ");
    Serial.print(init_status);
    Serial.println(">");
    if (init_status != SCH16T_OK) {
      delay(300);
    }
  }
  Serial.print("<IMU init OK. Serial: ");
  Serial.print(imu.getSnbr());
  Serial.println(">");
*/
  if (rx1 >= 0 && tx1 >= 0) {
    VescSerial1.begin(115200, SERIAL_8N1, rx1, tx1);
    vescLeft.setSerialPort(&VescSerial1);
  }
  if (rx2 >= 0 && tx2 >= 0) {
    VescSerial2.begin(115200, SERIAL_8N1, rx2, tx2);
    vescRight.setSerialPort(&VescSerial2);
  }

  baseServo.begin();
  elbowServo.begin();
  gripperServo.begin();

  activeMode = MODE_IDLE;
  lastCommandMillis = millis();
  Serial.println("<ESP32 VESC bridge ready.>");
}

void configureIMU() {
  /*
  Filter.Rate12 = FILTER_RATE;
  Filter.Acc12 = FILTER_ACC12;
  Filter.Acc3 = FILTER_ACC3;

  Sensitivity.Rate1 = SENSITIVITY_RATE1;
  Sensitivity.Rate2 = SENSITIVITY_RATE2;
  Sensitivity.Acc1 = SENSITIVITY_ACC1;
  Sensitivity.Acc2 = SENSITIVITY_ACC2;
  Sensitivity.Acc3 = SENSITIVITY_ACC3;

  Decimation.Rate2 = DECIMATION_RATE;
  Decimation.Acc2 = DECIMATION_ACC;*/
}

void sendCombinedTelemetry() {
  // Read IMU
 /* imu.getData(&raw);
  imu.convertData(&raw, &result);

  // Send ONE packet with everything.
  // Format: <M | Gx | Gy | Gz | Ax | Ay | Az>
  Serial.print("<M|");
  Serial.print(result.Rate1[SCH16T_AXIS_X]);
  Serial.print("|");
  Serial.print(result.Rate1[SCH16T_AXIS_Y]);
  Serial.print("|");
  Serial.print(result.Rate1[SCH16T_AXIS_Z]);
  Serial.print("|");

  Serial.print(result.Acc1[SCH16T_AXIS_X]);
  Serial.print("|");
  Serial.print(result.Acc1[SCH16T_AXIS_Y]);
  Serial.print("|");
  Serial.print(result.Acc1[SCH16T_AXIS_Z]);
  Serial.println(">");*/
}

// ------------------ Ramping & apply --------------------
void updateRamps(float dt) {

  // 1. Calculate Ramp Logic
  if (activeMode == MODE_RPM) {
    float maxDelta = MAX_RPM_DELTA_PER_SEC * dt;

    float deltaL = targetRpmL - commandedRpmL;
    if (fabs(deltaL) > maxDelta) commandedRpmL += copysign(maxDelta, deltaL);
    else commandedRpmL = targetRpmL;

    float deltaR = targetRpmR - commandedRpmR;
    if (fabs(deltaR) > maxDelta) commandedRpmR += copysign(maxDelta, deltaR);
    else commandedRpmR = targetRpmR;
    if (rx1 != -1) vescLeft.setRPM(commandedRpmL);
    if (rx2 != -1) vescRight.setRPM(commandedRpmR);

  } else if (activeMode == MODE_CURRENT) {
    float maxDelta = MAX_CURRENT_DELTA_PER_SEC * dt;

    float deltaL = targetCurrL - commandedCurrL;
    if (fabs(deltaL) > maxDelta) commandedCurrL += copysign(maxDelta, deltaL);
    else commandedCurrL = targetCurrL;

    float deltaR = targetCurrR - commandedCurrR;
    if (fabs(deltaR) > maxDelta) commandedCurrR += copysign(maxDelta, deltaR);
    else commandedCurrR = targetCurrR;

    if (rx1 != -1) vescLeft.setCurrent(commandedCurrL);
    if (rx2 != -1) vescRight.setCurrent(commandedCurrR);

  } else if (activeMode == MODE_DUTY) {
    // ... same logic for duty ...
    float maxDelta = MAX_DUTY_DELTA_PER_SEC * dt;
    float deltaL = targetDutyL - commandedDutyL;
    if (fabs(deltaL) > maxDelta) commandedDutyL += copysign(maxDelta, deltaL);
    else commandedDutyL = targetDutyL;

    float deltaR = targetDutyR - commandedDutyR;
    if (fabs(deltaR) > maxDelta) commandedDutyR += copysign(maxDelta, deltaR);
    else commandedDutyR = targetDutyR;

    if (rx1 != -1) vescLeft.setDuty(commandedDutyL);
    if (rx2 != -1) vescRight.setDuty(commandedDutyR);

  } else {
    // IDLE
    commandedRpmL = commandedRpmR = 0;
    commandedCurrL = commandedCurrR = 0;
    commandedDutyL = commandedDutyR = 0;

    // Send 0 current to be safe
    if (rx1 != -1) vescLeft.setCurrent(0.0f);
    if (rx2 != -1) vescRight.setCurrent(0.0f);
  }
}

// -------------- Helpers ----------------------------------
void stopAllMotorsImmediate() {
  if (rx1 != -1) vescLeft.setCurrent(0.0f);
  if (rx2 != -1) vescRight.setCurrent(0.0f);
  targetCurrL = targetCurrR = 0;
  targetRpmL = targetRpmR = 0;
  targetDutyL = targetDutyR = 0;
  commandedCurrL = commandedCurrR = 0;
  commandedRpmL = commandedRpmR = 0;
  commandedDutyL = commandedDutyR = 0;
}


void ackToPi(const char *s) {
  Serial.print("<");
  Serial.print(s);
  Serial.print(">");
  Serial.print("\n");
}
// ----------------- Process one ASCII command line from Pi --------------
// format: M or S (M for motors, S fo servo)|mode|left|right
// mode: 's' = speed (RPM), 'c' = current (A), 'p' = duty (-1..1), 'e' = emergency stop
// if for servo the expected forma is: S|servo_id|angle|speed
void processLine(const char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (line[0] == 0) return;

  char buf[INBUF_SZ];
  strncpy(buf, line, INBUF_SZ - 1);
  buf[INBUF_SZ - 1] = 0;

  char *cmdType = strtok(buf, "|");
  if (!cmdType) return;

  if (strcmp(cmdType, "M") == 0) {
    // Handle VESC logic (rpm, current, etc)
    // Expected: M|mode|left|right
    char *modeChar = strtok(NULL, "|");
    char *leftVal = strtok(NULL, "|");
    char *rightVal = strtok(NULL, "|");

    if (modeChar == NULL) return;
    char mode = modeChar[0];

    if (mode == 'e' || mode == 'E') {
      // emergency stop
      targetRpmL = targetRpmR = 0;
      targetCurrL = targetCurrR = 0;
      targetDutyL = targetDutyR = 0;
      activeMode = MODE_IDLE;
      stopAllMotorsImmediate();
      ackToPi("ACK|E|EMERGENCY_STOP");
      lastCommandMillis = millis();
      return;
    }
    if (leftVal == NULL || rightVal == NULL) {
      Serial.printf("<ERR|bad_format|%s>\n", line);
      return;
    }

    float vLeft = atof(leftVal);
    float vRight = atof(rightVal);

    switch (mode) {
      case 's':
      case 'S':
        // speed in RPM

        targetRpmL = constrain(vLeft * 15, -MAX_RPM * 15, MAX_RPM * 15);
        targetRpmR = constrain(vRight * 15, -MAX_RPM * 15, MAX_RPM * 15);
        activeMode = MODE_RPM;
        lastCommandMillis = millis();
        char ack[64];
        snprintf(ack, sizeof(ack), "ACK|S|%.2f|%.2f", targetRpmL, targetRpmR);
        ackToPi(ack);

        break;

      case 'c':
      case 'C':
        // current in Amps (positive/negative)
        targetCurrL = vLeft;
        targetCurrR = vRight;
        activeMode = MODE_CURRENT;
        lastCommandMillis = millis();
        ackToPi("ACK|C");
        break;

      case 'p':
      case 'P':
        // duty cycle, -1..1 expected. If user sends 0..100, detect and convert
        if (fabs(vLeft) > 1.1 || fabs(vRight) > 1.1) {
          // if they sent percents like 20 or -30, convert
          targetDutyL = constrain(vLeft / 100.0f, -1.0f, 1.0f);
          targetDutyR = constrain(vRight / 100.0f, -1.0f, 1.0f);
        } else {
          targetDutyL = constrain(vLeft, -1.0f, 1.0f);
          targetDutyR = constrain(vRight, -1.0f, 1.0f);
        }
        activeMode = MODE_DUTY;
        lastCommandMillis = millis();
        ackToPi("ACK|P");
        break;

      default:
        Serial.printf("<ERR|unknown_mode|%c>\n", mode);
        return;
    }
  } else if (strcmp(cmdType, "S") == 0) {
    // Handle Servo logic
    // Expected: S|servo_id|angle|speed
    char *idStr = strtok(NULL, "|");
    char *angleStr = strtok(NULL, "|");
    char *speedStr = strtok(NULL, "|");

    if (idStr && angleStr) {
      int id = atoi(idStr);
      int angle = atoi(angleStr);
      int speed = speedStr ? atoi(speedStr) : 40;  // Default speed if not sent

      switch (id) {
        case 0: baseServo.setTarget(angle, speed); break;
        case 1: elbowServo.setTarget(angle, speed); break;
        case 2:
          gripperServo.setTarget(angle, speed);
          break;
          // ...
      }
    }
  }
}
// -------------- telemetry reporter ----------------------
void sendTelemetryIfDue() {
  if ((currentMillis - lastTelemetryMillis) < TELEMETRY_PERIOD_MS) return;
  lastTelemetryMillis = currentMillis;

   //sendCombinedTelemetry();

  // 1. Read Data
  bool gotL = false;
  if (rx1 != -1) gotL = vescLeft.getVescValues();

  bool gotR = false;
  if (rx2 != -1) gotR = vescRight.getVescValues();

  // 2. Prepare Variables
  float distL = 0.0, distR = 0.0;
  float rpmL = 0.0, rpmR = 0.0;
  float ampL = 0.0, ampR = 0.0;
  float voltsL = 0.0, voltsR = 0.0;
  int faultL = 0, faultR = 0;

  // 3. Process Left Motor
  if (gotL) {
    // Tachometer / Distance
    if (!hasStartTachoL) {
      startTachoL = vescLeft.data.tachometer;
      hasStartTachoL = true;
    }
    long ticks = vescLeft.data.tachometer - startTachoL;
    distL = (float)ticks * MM_PER_TACH_TICK;

    // RPM (Mechanical)
    rpmL = vescLeft.data.rpm / 15.0;

    // Amps & Volts & Faults
    ampL = vescLeft.data.avgMotorCurrent;
    voltsL = vescLeft.data.inpVoltage;  // Voltage is usually same for both if shared battery
  }

  // 4. Process Right Motor
  if (gotR) {
    if (!hasStartTachoR) {
      startTachoR = vescRight.data.tachometer;
      hasStartTachoR = true;
    }
    long ticks = vescRight.data.tachometer - startTachoR;
    distR = (float)ticks * MM_PER_TACH_TICK;

    rpmR = vescRight.data.rpm / 15.0;
    ampR = vescRight.data.avgMotorCurrent;
    voltsR = vescRight.data.inpVoltage;  // Voltage is usually same for both if shared battery
  }

  // 5. Send Combined Telemetry
  // Format: T | DistL | DistR | RpmL | RpmR | AmpL | AmpR | Voltage | FaultCodeSum
  // We sum the fault codes (Left + Right) so 0 means healthy, anything else implies issue
  Serial.printf("<T|%.1f|%.1f|%.1f|%.1f|%.1f|%.1f|%.1f|%.1f>\n",
                distL, distR,
                rpmL, rpmR,
                ampL, ampR,
                voltsL, voltsR);
}


void loop() {
  currentMillis = millis();
  // 1) Read incoming characters from Raspberry Pi (USB Serial)
  while (Serial.available()) {
    char c = (char)Serial.read();
    // accept both \n and \r as line terminators
    if (c == '\n' || c == '\r') {
      if (inIdx > 0) {
        inBuf[inIdx] = 0;  // nul-terminate
        processLine(inBuf);
        inIdx = 0;
      }
    } else {
      if (inIdx < INBUF_SZ - 1) {
        inBuf[inIdx++] = c;
      } else {
        // overflow, reset buffer
        inIdx = 0;
      }
    }
  }



  // Watchdog: if no commands within timeout -> bring to idle/stop
  /*if ((currentMillis - lastCommandMillis) > WATCHDOG_TIMEOUT_MS) {
    if (activeMode != MODE_IDLE) {
      activeMode = MODE_IDLE;
      stopAllMotorsImmediate();
      Serial.println("<WARN|watchdog_timeout|motors_stopped>");
    }
  } */
  // Update ramped commanded values towards targets and send to VESC
  if (currentMillis - lastMotorUpdateMillis >= MOTOR_UPDATE_PERIOD_MS) {

    // Calculate time since last update (should be approx 0.05s)
    float dt = (currentMillis - lastMotorUpdateMillis) / 1000.0f;
    lastMotorUpdateMillis = currentMillis;

    // Safety clamp on dt in case loop blocked for a long time
    if (dt > 0.2f) dt = 0.2f;

    updateRamps(dt);
  }

  // These functions check their own timers. If it's not time to move, they return immediately.
  baseServo.update();
  elbowServo.update();
  gripperServo.update();


  sendTelemetryIfDue();
  delay(1);
}
