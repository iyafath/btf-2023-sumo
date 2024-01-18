/*

Sumo Robot for BTF 2023.

Using Alfa's ESP32:
Choose "DOIT ESP32 DEVKIT V1" for the board.
MAC address: REDACTED

Robot abilities:
- Analog movement
- Actions: dash forward and spin 180 degrees, queueable

*/

// comment out to disable serial output
#define debug TRUE

// comment out to disable motor control
#define motor TRUE

#include <PS4Controller.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

// board and pin definitions
const char ESP_ADDRESS[] = "REDACTED";
const int PIN_R_SPD = 25, PIN_R_FWD = 32, PIN_R_BWD = 33;
const int PIN_L_SPD = 18, PIN_L_FWD = 19, PIN_L_BWD = 21;
const int STICK_RECONNECT_DELAY = 3000; // if ps4 gets disconnected, keep trying to reconnect with this cooldown between attempts
unsigned long stickReconnectTimer = 0;

// controller parameters
struct Controller {
  // x, sq, o, tr -> cross, square, circle, triangle
  int rx; // rotate
  int ly; // forward/backward
  bool x; // "dashF", temp speed boost, prevents rotation, has cooldown
  bool sq; // "dashB"
  bool r1, l1; // fast 180 deg turn, prevents forward/backward, has cooldown
} stick;

// motor control tuning parameters
// necessary for superposition: speed_max + rot_max < stick_max
// rot lead & trail = percentage of rot speed to be applied for fw wheel and bw wheel
const int STICK_ZERO = 10, STICK_FINE = 100, STICK_MAX = 128;
const int SPEED_MIN = 20, SPEED_MID = 70, SPEED_MAX = 100;
const int ROT_MIN = 20, ROT_MID = 60, ROT_MAX = 80;
const float ROT_LEAD = 1, ROT_TRAIL = 1;

// special actions processing
enum Action {Neutral, DashF, DashB, SpinR, SpinL};
const int DASH_DURATION = 300, SPIN_DURATION = 300; // time it takes for an action to finish
const int SPIN_ACTIVE = 100; // how long motor should stay on during spin
const float DASH_SPEED = 1, SPIN_SPEED = 1; // normalized speed
const int ACTION_DEBOUNCE = 200; // cooldown time for same-action queueing
Action currentAction, nextAction;
unsigned long lastActionStart = -100000;
#ifdef debug
const char* getActionName(Action act) {
  switch(act) {
    case Neutral: return "Neutral";
    case DashF: return "DashF";
    case DashB: return "DashB";
    case SpinR: return "SpinR";
    case SpinL: return "SpinR";
  }
  return "ActionNameUndefined";
}
#endif

// ======================= UTILITY FUNCTIONS =========================

const float EPSILON = 0.001;

// map float to float
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  // check for zero div just in case
  if (abs(in_max - in_min) < EPSILON) return in_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// true if past debounce
bool checkDebounce (unsigned long deb = ACTION_DEBOUNCE) {
  return (millis() - lastActionStart > deb);
}

// ======================= CONTROLLER CHECKS =========================

// from ps4 example
void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisconnect() {
  Serial.println("Disconnected!");
}

// ======================== LOW-LEVEL CONTROL ============================

#ifdef motor

// normalized speed = -1 to 1. first parameter = left wheel, second = right wheel
void setSpeed(float l, float r) {
  // verify speeds
  if (r < -1) r = -1; else if (r > 1) r = 1;
  if (l < -1) l = -1; else if (l > 1) l = 1;

  if (r < 0) {
    digitalWrite(PIN_R_BWD, 1);
    digitalWrite(PIN_R_FWD, 0);
    r = -r;
  }
  else {
    digitalWrite(PIN_R_FWD, 1);
    digitalWrite(PIN_R_BWD, 0);
  }
  if (l < 0) {
    digitalWrite(PIN_L_BWD, 1);
    digitalWrite(PIN_L_FWD, 0);
    l = -l;
  }
  else {
    digitalWrite(PIN_L_FWD, 1);
    digitalWrite(PIN_L_BWD, 0);
  }

  analogWrite(PIN_R_SPD, (int)(r * 255));
  analogWrite(PIN_L_SPD, (int)(l * 255));
}

#else

void setSpeed(float l, float r) {
  // verify speeds
  if (r < -1) r = -1; else if (r > 1) r = 1;
  if (l < -1) l = -1; else if (l > 1) l = 1;

  Serial.print("MOTORPWM\t");

  if (r < 0) {
    Serial.printf("R%d\t", (int)(r * 255));
  }
  else {
    Serial.printf("R+%d\t", (int)(r * 255));
  }
  if (l < 0) {
    Serial.printf("L%d\t", (int)(l * 255));
  }
  else {
    Serial.printf("L+%d\t", (int)(l * 255));
  }
  Serial.println();
}

#endif

// ===================== PROCESS ANALOG CTRL =======================

// tunes analog stick values for better motor speed control
float mapSpeedStick(int val) {
  int sign = (val < 0) ? -1 : 1;
  val = abs(val);
  if (val <= STICK_ZERO) return 0;
  if (val <= STICK_FINE) return sign * fmap(val, STICK_ZERO, STICK_FINE, SPEED_MIN, SPEED_MID);
  return sign * fmap(val, STICK_FINE, STICK_MAX, SPEED_MID, SPEED_MAX);
}

float mapRotStick(int val) {
  int sign = (val < 0) ? -1 : 1;
  val = abs(val);
  if (val <= STICK_ZERO) return 0;
  if (val <= STICK_FINE) return sign * fmap(val, STICK_ZERO, STICK_FINE, ROT_MIN, ROT_MID);
  return sign * fmap(val, STICK_FINE, STICK_MAX, ROT_MID, ROT_MAX);
}

// run motors given values of speed and rot sticks
// assume pure rotation rotates in place
void move(int speedStick, int rotStick) {
  // speed component
  float speed = mapSpeedStick(speedStick) / STICK_MAX;
  // rot component. + = right, - = left
  float rot = mapRotStick(rotStick) / STICK_MAX;
  float rotR = -rot, rotL = rot;
  if (rotR > 0) {
    rotR *= ROT_LEAD; rotL *= ROT_TRAIL;
  }
  else {
    rotR *= ROT_TRAIL; rotL *= ROT_LEAD;
  }

#ifdef debug
  Serial.print("SPEEDOUT\t");
  Serial.print("Speed: "); Serial.print(speed);
  Serial.print("\tRot: "); Serial.print(rot);
  Serial.print("\tNet R: "); Serial.print(speed + rotR);
  Serial.print("\tNet L: "); Serial.print(speed + rotL);
  Serial.println();
#endif
  float l = speed + rotL, r = speed + rotR;
  if (max(abs(l), abs(r)) > 1) {
    float mx = max(abs(l), abs(r));
    l /= mx; r /= mx;
  }
  setSpeed(l, r);
}

// ======================== ACTIONS ============================

void dashF()
{
  setSpeed(DASH_SPEED, DASH_SPEED);
}

void dashB()
{
  setSpeed(-DASH_SPEED, -DASH_SPEED);
}

void spinR()
{
  // active motor phase
  if (!checkDebounce(SPIN_ACTIVE)) setSpeed(SPIN_SPEED * ROT_LEAD, -SPIN_SPEED * ROT_TRAIL);
  else setSpeed(0, 0);
}

void spinL()
{
  if (!checkDebounce(SPIN_ACTIVE)) setSpeed(-SPIN_SPEED * ROT_TRAIL, SPIN_SPEED * ROT_LEAD);
  else setSpeed(0, 0);
}

void reset()
{
  setSpeed(0, 0);
}

// ======================== PROCESS ACTIONS ============================

unsigned long getCooldown (Action act) {
  if (act == DashF || act == DashB) return DASH_DURATION;
  else if (act != Neutral) return SPIN_DURATION;
  return 0;
}

// true if past cooldown
bool checkCooldown (Action act) {
  return checkDebounce(getCooldown(act));
}

// return 0 if currently not doing action (dash/spin)
// return 1 if currently doing action, and there is no further action queued
// return 2 if currently doing action, and there is another action queued
// updates currentAction, nextAction and lastActionStart if new action is being taken
int processActionQueue()
{
  if (currentAction != Neutral) {
    if (checkCooldown(currentAction)) {
      if (currentAction == SpinR || currentAction == SpinL) reset();
      currentAction = nextAction;
      nextAction = Neutral;
      if (currentAction != Neutral) lastActionStart = millis();
    }
  }
  else if (nextAction != Neutral) {
    currentAction = nextAction;
    nextAction = Neutral;
    lastActionStart = millis();
  }

#ifdef debug
  Serial.print("ACTIONS\t");
  Serial.print("currentAction: "); Serial.print(getActionName(currentAction));
  Serial.print("\tnextAction: "); Serial.print(getActionName(nextAction));
  if (currentAction) {
    Serial.print("\t| Cooldown: "); Serial.print(getCooldown(currentAction));
  }
  Serial.println();
#endif

  // "!!" converts nonzero integer to 1. Neutral = 0
  return (!!currentAction + !!nextAction);
}

void executeAction()
{
  if (currentAction == DashF) dashF();
  else if(currentAction == DashB) dashB();
  else if (currentAction == SpinR) spinR();
  else if (currentAction == SpinL) spinL();
}

void processController(int actionQueueStatus)
{
  // spin
  stick.r1 = PS4.R1();
  stick.l1 = PS4.L1();
  // dashF
  stick.x = PS4.Cross();
  // dashB
  stick.sq = PS4.Square();
  // analog sticks
  stick.ly = PS4.LStickY();
  stick.rx = PS4.RStickX();

  // if queue not full, queue next action
  if (actionQueueStatus != 2) {
    // check action validity, prioritize dash
    if (stick.x) {
      if (currentAction != DashF || checkDebounce()) nextAction = DashF;
    }
    else if(stick.sq) {
        if (currentAction != DashB || checkDebounce()) nextAction = DashB;
    }
    else if (stick.r1) {
      if (currentAction != SpinR || checkDebounce()) nextAction = SpinR;
    }
    else if (stick.l1) {
      if (currentAction != SpinL || checkDebounce()) nextAction = SpinL;
    }
  }

  // if not doing action, process analog movement
  if (actionQueueStatus == 0) move(stick.ly, stick.rx);

#ifdef debug
  if (stick.r1 || stick.l1 || stick.x || stick.sq || stick.ly || stick.rx) {
    Serial.print("READINGS\t");
    if (stick.x) Serial.print("X ");
    if (stick.sq) Serial.print("Square ");
    if (stick.r1) Serial.print("R1 ");
    if (stick.l1) Serial.print("L1 ");
    if (stick.ly) {
      Serial.print("LY:"); Serial.print(stick.ly); Serial.print(" ");
    }
    if (stick.rx) {
      Serial.print("RX:"); Serial.print(stick.rx); Serial.print(" ");
    }
  }
  Serial.println();
#endif
}

void setup() {
  pinMode(PIN_R_SPD, OUTPUT);
  pinMode(PIN_R_FWD, OUTPUT);
  pinMode(PIN_R_BWD, OUTPUT);
  pinMode(PIN_L_SPD, OUTPUT);
  pinMode(PIN_L_FWD, OUTPUT);
  pinMode(PIN_L_BWD, OUTPUT);

  PS4.begin(ESP_ADDRESS);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisconnect);   

  Serial.begin(115200);
  Serial.println();

  currentAction = nextAction = Neutral;
  reset();
}

void loop() {
  if (PS4.isConnected()) {
    // process action queue and execute if any
    int actionQueueStatus = processActionQueue();
    executeAction();
    
    processController(actionQueueStatus);
    delay(50);
#ifndef motor
    delay(100);
#endif
  }
  else {
    reset();
    if (millis() - stickReconnectTimer > STICK_RECONNECT_DELAY) {
      removePairedDevices();
      PS4.end();
      PS4.begin(ESP_ADDRESS);
      stickReconnectTimer = millis();
    }
  }
}
