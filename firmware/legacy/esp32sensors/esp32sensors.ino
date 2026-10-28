#include <SPI.h>
#include <SCH16T.h>  //Library for the murata imu

// --- AsyncRelay Class for H-Bridge Control ---
class AsyncRelay {
private:
  int _pinA, _pinB;
  int _state;  // 0=Stop, 1=Fwd, 2=Back
  int _pendingState;
  unsigned long _lastChange;
  bool _waitingForDeadtime;
  const int DEADTIME_MS = 50;  // Safety pause to prevent short circuits

public:
  AsyncRelay(int pinA, int pinB) {
    _pinA = pinA;
    _pinB = pinB;
    _state = 0;
    _pendingState = 0;
    _waitingForDeadtime = false;
  }

  void begin() {
    pinMode(_pinA, OUTPUT);
    pinMode(_pinB, OUTPUT);
    digitalWrite(_pinA, LOW);
    digitalWrite(_pinB, LOW);
  }

  void set(int action) {
    if (action == _state) return;  // No change

    // If we are moving and want to change direction (or stop),
    // we must stop first, wait, then apply new state.
    _pendingState = action;

    // Immediate stop
    digitalWrite(_pinA, LOW);
    digitalWrite(_pinB, LOW);

    _waitingForDeadtime = true;
    _lastChange = millis();
  }

  void update() {
    if (_waitingForDeadtime) {
      if (millis() - _lastChange >= DEADTIME_MS) {
        _waitingForDeadtime = false;
        _state = _pendingState;

        // Apply new state
        if (_state == 1) {  // Forward
          digitalWrite(_pinA, HIGH);
          digitalWrite(_pinB, LOW);
        } else if (_state == 2) {  // Backward
          digitalWrite(_pinA, LOW);
          digitalWrite(_pinB, HIGH);
        } else {  // Stop
          digitalWrite(_pinA, LOW);
          digitalWrite(_pinB, LOW);
        }
      }
    }
  }
};


#define trigPin 26
#define echoPinHand 16
#define echoMidPin 17
#define echoLeftPin 18
#define echoRightPin 5

#define RelayThree 22
#define RelayFour 21

#define RelayFive 23
#define RelaySix 19



#define DISPLAY_RX_PIN 4   // Connect to Screen TX  // Yellow cable when last checked
#define DISPLAY_TX_PIN 27  // Connect to Screen RX (Pin 22 on CYD)  // Blue cable on last check

volatile bool canCollectDataHand = false;
volatile bool canCollectDataMid = false;
volatile bool canCollectDataLeft = false;
volatile bool canCollectDataRight = false;
volatile bool canPingHand = true;
volatile bool canPingMid = true;
volatile bool canPingRight = true;
volatile bool canPingLeft = true;
volatile unsigned long microSecHand = 0;
volatile unsigned long microSecHandEndDuration = 0;
volatile unsigned long microSecMid = 0;
volatile unsigned long microSecMidEndDuration = 0;
volatile unsigned long microSecLeft = 0;
volatile unsigned long microSecLeftEndDuration = 0;
volatile unsigned long microSecRight = 0;
volatile unsigned long microSecRightEndDuration = 0;
volatile bool newHandData = false;
volatile bool newMidData = false;
volatile bool newLeftData = false;
volatile bool newRightData = false;
unsigned long timeOfLastTrigger = millis();
double distanceMid = 0;
double distanceLeft = 0;
double distanceRight = 0;
double distanceHand = 0;
double midUSArr[] = { 0, 0, 0 };
double leftUSArr[] = { 0, 0, 0 };
double rightUSArr[] = { 0, 0, 0 };
double HandUSArr[] = { 0, 0, 0 };
double _medianMid = 0;
double _medianLeft = 0;
double _medianRight = 0;
double _medianHand = 0;
unsigned long now = millis();

int relayOneAction = 0;  // 0 is stop(default), 1 is forward, 2 is back
int relayTwoAction = 0;  // 0 is stop(default), 1 is forward, 2 is back

AsyncRelay actuatorOne(RelayThree, RelayFour);
AsyncRelay actuatorTwo(RelayFive, RelaySix);

// Helper for Serial Input
const int INBUF_SZ = 512;
char inBuf[INBUF_SZ];
int inIdx = 0;
unsigned long lastTelemetry = 0;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, DISPLAY_RX_PIN, DISPLAY_TX_PIN);

  actuatorOne.begin();
  actuatorTwo.begin();

  if (trigPin >= 0) { pinMode(trigPin, OUTPUT); }
  if (echoMidPin >= 0) {
    pinMode(echoMidPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(echoMidPin), IRS_MidSensor, CHANGE);
  }
  if (echoPinHand >= 0) {
    pinMode(echoPinHand, INPUT);
    attachInterrupt(digitalPinToInterrupt(echoPinHand), IRS_HandSensor, CHANGE);
  }
  if (echoLeftPin >= 0) {
    pinMode(echoLeftPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(echoLeftPin), IRS_LeftSensor, CHANGE);
  }
  if (echoRightPin >= 0) {
    pinMode(echoRightPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(echoRightPin), IRS_RightSensor, CHANGE);
  }
}
void IRS_HandSensor() {
  if (digitalRead(echoPinHand) == 0x0) {
    if (canCollectDataHand && !canPingHand) {
      microSecHandEndDuration = micros() - microSecHand;
      canCollectDataHand = false;
      newHandData = true;
    }
  } else {
    if (canPingHand) {
      microSecHand = micros();
      canPingHand = false;
      canCollectDataHand = true;
    }
  }
}
void IRS_MidSensor() {
  if (digitalRead(echoMidPin) == 0x0) {
    if (canCollectDataMid && !canPingMid) {
      microSecMidEndDuration = micros() - microSecMid;
      canCollectDataMid = false;
      newMidData = true;
    }
  } else {
    if (canPingMid) {
      microSecMid = micros();
      canPingMid = false;
      canCollectDataMid = true;
    }
  }
}
void IRS_LeftSensor() {
  if (digitalRead(echoLeftPin) == 0x0) {
    if (canCollectDataLeft && !canPingLeft) {
      microSecLeftEndDuration = micros() - microSecLeft;
      canCollectDataLeft = false;
      newLeftData = true;
    }
  } else {
    if (canPingLeft) {
      microSecLeft = micros();
      canPingLeft = false;
      canCollectDataLeft = true;
    }
  }
}
void IRS_RightSensor() {
  if (digitalRead(echoRightPin) == 0x0) {
    if (canCollectDataRight && !canPingRight) {
      microSecRightEndDuration = micros() - microSecRight;
      canCollectDataRight = false;
      newRightData = true;
    }
  } else {
    if (canPingRight) {
      microSecRight = micros();
      canPingRight = false;
      canCollectDataRight = true;
    }
  }
}

void GetUltrasoundData() {
  if (now - timeOfLastTrigger >= 75) {
    canPingLeft = true;
    canPingMid = true;
    canPingRight = true;
    canPingHand = true;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    timeOfLastTrigger = millis();
  }
  if (newRightData) {
    newRightData = false;
    distanceRight = (microSecRightEndDuration * 0.0343) / 2;
    rightUSArr[0] = rightUSArr[1];
    rightUSArr[1] = rightUSArr[2];
    rightUSArr[2] = distanceRight;
    if (rightUSArr[2] < rightUSArr[1]) {
      if (rightUSArr[2] < rightUSArr[0]) {
        if (rightUSArr[1] < rightUSArr[0]) {
          _medianRight = rightUSArr[1];
        } else {
          _medianRight = rightUSArr[0];
        }
      } else {
        _medianRight = rightUSArr[2];
      }
    } else {
      if (rightUSArr[2] < rightUSArr[0]) {
        _medianRight = rightUSArr[2];
      } else {
        if (rightUSArr[1] < rightUSArr[0]) {
          _medianRight = rightUSArr[0];
        } else {
          _medianRight = rightUSArr[1];
        }
      }
    }
  }
  if (newHandData) {
    newHandData = false;
    distanceHand = (microSecHandEndDuration * 0.0343) / 2;
    HandUSArr[0] = HandUSArr[1];
    HandUSArr[1] = HandUSArr[2];
    HandUSArr[2] = distanceHand;
    if (HandUSArr[2] < HandUSArr[1]) {
      if (HandUSArr[2] < HandUSArr[0]) {
        if (HandUSArr[1] < HandUSArr[0]) {
          _medianHand = HandUSArr[1];
        } else {
          _medianHand = HandUSArr[0];
        }
      } else {
        _medianHand = HandUSArr[2];
      }
    } else {
      if (HandUSArr[2] < HandUSArr[0]) {
        _medianHand = HandUSArr[2];
      } else {
        if (HandUSArr[1] < HandUSArr[0]) {
          _medianHand = HandUSArr[0];
        } else {
          _medianHand = HandUSArr[1];
        }
      }
    }
  }
  if (newMidData) {
    newMidData = false;
    distanceMid = (microSecMidEndDuration * 0.0343) / 2;
    midUSArr[0] = midUSArr[1];
    midUSArr[1] = midUSArr[2];
    midUSArr[2] = distanceMid;

    if (midUSArr[2] < midUSArr[1]) {
      if (midUSArr[2] < midUSArr[0]) {
        if (midUSArr[1] < midUSArr[0]) {
          _medianMid = midUSArr[1];
        } else {
          _medianMid = midUSArr[0];
        }
      } else {
        _medianMid = midUSArr[2];
      }
    } else {
      if (midUSArr[2] < midUSArr[0]) {
        _medianMid = midUSArr[2];
      } else {
        if (midUSArr[1] < midUSArr[0]) {
          _medianMid = midUSArr[0];
        } else {
          _medianMid = midUSArr[1];
        }
      }
    }
  }
  if (newLeftData) {
    newLeftData = false;
    distanceLeft = (microSecLeftEndDuration * 0.0343) / 2;
    leftUSArr[0] = leftUSArr[1];
    leftUSArr[1] = leftUSArr[2];
    leftUSArr[2] = distanceLeft;
    if (leftUSArr[2] < leftUSArr[1]) {
      if (leftUSArr[2] < leftUSArr[0]) {
        if (leftUSArr[1] < leftUSArr[0]) {
          _medianLeft = leftUSArr[1];
        } else {
          _medianLeft = leftUSArr[0];
        }
      } else {
        _medianLeft = leftUSArr[2];
      }
    } else {
      if (leftUSArr[2] < leftUSArr[0]) {
        _medianLeft = leftUSArr[2];
      } else {
        if (leftUSArr[1] < leftUSArr[0]) {
          _medianLeft = leftUSArr[0];
        } else {
          _medianLeft = leftUSArr[1];
        }
      }
    }
  }
}

void ackToPi(const char *s) {
  Serial.print("<");
  Serial.print(s);
  Serial.print(">");
  Serial.print("\n");
}
void updateRelayOne() {  // !!! H-bridge NEVER HIGH HIGH
  digitalWrite(RelayThree, LOW);
  digitalWrite(RelayFour, LOW);
  delay(25);
  if (relayOneAction == 1) {
    digitalWrite(RelayThree, HIGH);
    digitalWrite(RelayFour, LOW);
  } else if (relayOneAction == 2) {
    digitalWrite(RelayThree, LOW);
    digitalWrite(RelayFour, HIGH);
  }
}
void updateRelayTwo() {  // !!! H-bridge NEVER HIGH HIGH
  digitalWrite(RelayFive, LOW);
  digitalWrite(RelaySix, LOW);
  delay(25);
  if (relayTwoAction == 1) {
    digitalWrite(RelayFive, HIGH);
    digitalWrite(RelaySix, LOW);
  } else if (relayTwoAction == 2) {
    digitalWrite(RelayFive, LOW);
    digitalWrite(RelaySix, HIGH);
  }
}
void StopRelay() {
  relayOneAction = 0;
  relayTwoAction = 0;
  updateRelayOne();
  updateRelayTwo();
}
void sendCombinedTelemetry() {
  // Send ONE packet with everything.
  // Format: <D | US_Left | US_Mid | US_Right | US_Hand | Gx | Gy | Gz | Ax | Ay | Az>
  Serial.print("<D|");
  Serial.print(_medianLeft);
  Serial.print("|");
  Serial.print(_medianMid);
  Serial.print("|");
  Serial.print(_medianRight);
  Serial.print("|");
  Serial.print(_medianHand);
  Serial.print("|");

  //Serial.print(result.Rate1[SCH16T_AXIS_X]);
  Serial.print(0);
  Serial.print("|");
  //Serial.print(result.Rate1[SCH16T_AXIS_Y]);
   Serial.print(0);
  Serial.print("|");
  //Serial.print(result.Rate1[SCH16T_AXIS_Z]);
   Serial.print(0);
  Serial.print("|");

  //Serial.print(result.Acc1[SCH16T_AXIS_X]);
   Serial.print(0);
  Serial.print("|");
  // Serial.print(result.Acc1[SCH16T_AXIS_Y]);
   Serial.print(0);
  Serial.print("|");
  // Serial.print(result.Acc1[SCH16T_AXIS_Z]);
   Serial.print(0);
  Serial.println(">");
}

void processLine(const char *line) {

  if (strncmp(line, "QR|", 3) == 0) {
    Serial2.print("QR:");
    Serial2.print(line + 3);  // Skip "QR|"
    Serial2.print("\n");
    return;
  }

  // If C# sends "STREAM|1" we forward "STREAM:1" to screen
  if (strncmp(line, "STREAM|", 7) == 0) {
    Serial2.print("STREAM:");
    Serial2.print(line + 7);  // Skip "STREAM|"
    Serial2.print("\n");
    return;
  }


  char buf[INBUF_SZ];
  strncpy(buf, line, INBUF_SZ - 1);
  buf[INBUF_SZ - 1] = 0;

  char *cmd = strtok(buf, "|");
  if (!cmd) return;

  // Command: R|id|action (Relay control)
  if (strcmp(cmd, "R") == 0) {
    char *idStr = strtok(NULL, "|");
    char *actStr = strtok(NULL, "|");
    if (idStr && actStr) {
      int id = atoi(idStr);
      int action = atoi(actStr);  // 0=Stop, 1=Fwd, 2=Back

      if (id == 1) actuatorOne.set(action);
      if (id == 2) actuatorTwo.set(action);
    }
  }
}

void loop() {
  now = millis();

  // 1. Read Serial Commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inIdx > 0) {
        inBuf[inIdx] = 0;
        processLine(inBuf);
        inIdx = 0;
      }
    } else if (inIdx < INBUF_SZ - 1) {
      inBuf[inIdx++] = c;
    }
  }

  while (Serial2.available()) {
    char c = (char)Serial2.read();
    // Forward directly to the Pi via USB Serial
    Serial.write(c);
  }


  // 2. Trigger Ultrasonics (Every 75ms)
  GetUltrasoundData();

  // 3. Update Relay State Machines (Non-blocking)
  actuatorOne.update();
  actuatorTwo.update();

  // 4. Send Telemetry (Every 100ms)
  if (now - lastTelemetry > 250) {
    lastTelemetry = now;
    sendCombinedTelemetry();
  }
  delay(1);
}
