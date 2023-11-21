/*

Sumo Robot for BTF 2023.

Using Alfa's ESP32:
Choose "DOIT ESP32 DEVKIT V1" for the board.
MAC address: 0C:B8:15:D8:7D:F0

Robot abilities:
- Analog movement, rotation and movement able to be combined.
- Actions: dash forward and spin 180 degrees, queueable

Action queueing = ability to register next action while previous action is still being executed

*/

// set to FALSE to disable serial output
#define debug FALSE

#include <PS4Controller.h>

// board and pin definitions
const char ESP_ADDRESS[] = "CENSORED";
const int PIN_R_SPD = 25, PIN_R_FWD = 32, PIN_R_BWD = 33;
const int PIN_L_SPD = 0, PIN_L_FWD = 0, PIN_L_BWD = 0;

// controller parameters
struct Controller {
  // x, sq, o, tr -> cross, square, circle, triangle
  int rx; // rotate
  int ly; // forward/backward
  bool x; // "dash", temp speed boost, prevents rotation, has cooldown
  bool r1, l1; // fast 180 deg turn, prevents forward/backward, has cooldown
} stick;

// motor control tuning parameters
// necessary for superposition: speed_max + rot_max < stick_max
// rot lead & trail = percentage of rot speed to be applied
// lead rotates forward, trail rotates backward. must be tuned so net rotation is in place
const int STICK_ZERO = 10, STICK_FINE = 100, STICK_MAX = 128;
const int SPEED_MIN = 15, SPEED_MID = 23, SPEED_MAX = 70;
const int ROT_MIN = 15, ROT_MID = 20, ROT_MAX = 30;
const float ROT_LEAD = 1, ROT_TRAIL = 1;

// special actions processing
enum Action {Neutral, Dash, SpinR, SpinL};
const int DASH_DURATION = 300, SPIN_DURATION = 300; // time it takes for an action to finish
const int ACTION_DEBOUNCE = 200; // cooldown time for same-action queueing
Action currentAction, nextAction;
unsigned long lastActionStart = -100000;
#ifdef debug
const char* getActionName(Action act) {
  switch(act) {
    case Neutral: return "Neutral";
    case Dash: return "Dash";
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

// ======================== LOW-LEVEL CONTROL ============================

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

  setSpeed(speed + rotL, speed + rotR);
}

// ======================== ACTIONS ============================

void dash()
{
  setSpeed(1, 1);
}

void spinR()
{
  setSpeed(1, -1);
}

void spinL()
{
  setSpeed(-1, 1);
}

void reset()
{
  setSpeed(0, 0);
  delay(100);
}

// ======================== PROCESS ACTIONS ============================

unsigned long getCooldown (Action act) {
  if (act == Dash) return DASH_DURATION;
  else if (act != Neutral) return SPIN_DURATION;
  return 0;
}

// true if past cooldown
bool checkCooldown (Action act) {
  return (millis() - lastActionStart > getCooldown(act));
}

// true if past debounce
bool checkDebounce () {
  return (millis() - lastActionStart > ACTION_DEBOUNCE);
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

  #if defined debug
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
  if (currentAction == Dash) dash();
  else if (currentAction == SpinR) spinR();
  else if (currentAction == SpinL) spinL();
}

void processController(int actionQueueStatus)
{
  // spin
  stick.r1 = PS4.R1();
  stick.l1 = PS4.L1();
  // dash
  stick.x = PS4.Cross();
  // analog sticks (L-Y, R-X)
  stick.ly = PS4.LStickY();
  stick.rx = PS4.RStickX();

  // if queue not full, queue next action
  if (actionQueueStatus != 2) {
    // check action validity, prioritize dash
    if (stick.x) {
      if (currentAction != Dash || checkDebounce()) nextAction = Dash;
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
  if (stick.r1 || stick.l1 || stick.x || stick.ly || stick.rx) {
    Serial.print("READINGS\t");
    if (stick.x) Serial.print(" X");
    if (stick.r1) Serial.print(" R1");
    if (stick.l1) Serial.print(" L1");
    if (stick.ly) {
      Serial.print(" LY:"); Serial.print(stick.ly);
    }
    if (stick.rx) {
      Serial.print(" RX:"); Serial.print(stick.ly);
    }
    Serial.println();
  }
  #endif
}

void setup() {
  #ifdef debug
  Serial.begin(115200);
  Serial.println();
  #endif

  PS4.begin(ESP_ADDRESS);
  pinMode(PIN_R_SPD, OUTPUT);
  pinMode(PIN_R_FWD, OUTPUT);
  pinMode(PIN_R_BWD, OUTPUT);
  pinMode(PIN_L_SPD, OUTPUT);
  pinMode(PIN_L_FWD, OUTPUT);
  pinMode(PIN_L_BWD, OUTPUT);

  currentAction = nextAction = Neutral;
  reset();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (PS4.isConnected()) {
    // process action queue and execute if any
    int actionQueueStatus = processActionQueue();
    if (actionQueueStatus) executeAction();
    
    processController(actionQueueStatus);
    #ifdef debug
    delay(50);
    #endif
  }
  else {
    reset();
  }
}
