/*
 * Hexapod_ESP32_v2.ino  -  18-servo hexapod: tripod / wave gait, gripper, wifi, RViz twin,
 *                         Nav2 safety mode (nav gait, stands), drift correction
 *
 * Tripod and wave walking gaits for an 18-servo hexapod, driven by
 * direct joint angles (no IK), plus a gripper on its own GPIO.
 *
 * Directions: forward, backward, turn left/right in place.
 * Gaits:      tripod (3 legs up at a time, fast)
 *             wave   (1 leg up at a time, 5 feet down, slow and stable)
 *
 * Digital twin: while a TCP client is connected, the board reports
 * every joint angle it commands (~25 times a second while moving),
 * so the RViz model can copy the real robot.
 *
 * Hardware:
 *   ESP32 on wifi, I2C SDA=21, SCL=22
 *   2x PCA9685 @ 0x40 and 0x41, 50 Hz
 *   18x MG995, 6V from XL4005 buck converters
 *   1x gripper servo on GPIO 13, its own 5-6V supply
 *
 * ------------------------------------------------------------
 * LAYOUT (confirmed)
 * ------------------------------------------------------------
 *   0x40             0x41
 *   leg 0  front R   leg 3  front L
 *   leg 1  middle R  leg 4  middle L
 *   leg 2  back R    leg 5  back L
 *
 *   Channels within each board, per leg:
 *     femur = (leg%3)*3 + 0
 *     tibia = (leg%3)*3 + 1
 *     coxa  = (leg%3)*3 + 2
 *
 *   Left legs (3,4,5) are mechanically mirrored from right legs
 *   (0,1,2) on all three joints, so their direction is inverted.
 *
 *              front
 *          0 ----- 3
 *          |       |
 *          1       4
 *          |       |
 *          2 ----- 5
 *              back
 *
 *   Tripod A = 0, 4, 2   (front-R, middle-L, back-R)
 *   Tripod B = 3, 1, 5   (front-L, middle-R, back-L)
 *
 *   Wave order, forward / turns = 2, 1, 0, 5, 4, 3
 *     (back-R, middle-R, front-R, back-L, middle-L, front-L)
 *   Wave order, backward        = 0, 1, 2, 3, 4, 5
 *     (front-R, middle-R, back-R, front-L, middle-L, back-L)
 *
 *   Gripper: calibrated open 150 deg, closed 100 deg.
 *
 * ------------------------------------------------------------
 * COMMANDS (Serial Monitor, 115200, line ending = Newline)
 * ------------------------------------------------------------
 *   w                walk forward
 *   s                walk backward
 *   a                turn left  (in place)
 *   d                turn right (in place)
 *   x                stop, settle into stance
 *
 *   tripod           use the tripod gait (default)
 *   wave             use the wave gait (one leg at a time)
 *   gait tripod / gait wave   same as above
 *
 *   g / grab         close the gripper
 *   r / release      open the gripper
 *   grip             toggle the gripper
 *
 *   1                one gait cycle (current direction) then stop
 *   1 d              one gait cycle in a given direction (w/s/a/d)
 *   h                stance pose (all 90)
 *   c                center everything to 90
 *   p                print current pose
 *   t                test each leg in turn (lift and lower)
 *
 *   40 5 120         set board 0x40 ch5 to 120
 *   41 2 +10         nudge board 0x41 ch2 by +10
 *
 *   dir 0 femur -1   flip a joint's direction, live
 *   dump             print the LEGS table with current dir values
 *
 *   swing 12 / turn 20 / lift 20 / tuck 12 / stance 0 / speed 180
 *   wlift 4          wave only: extra femur lift (degrees)
 *   wpause 200       wave only: rest on six feet between legs (ms)
 *   wswing 8         wave only: coxa stride forward/backward
 *   wturn 12         wave only: coxa travel when turning
 *   twin on / off    joint reports for the RViz twin (on by default)
 *   navgait wave     navigation gait: wave (default) or tripod   (saved)
 *   navsteps 0       navigation, wave: cycles between rest stands (0 = off)   (saved)
 *   navtsteps 3      navigation, tripod: cycles between rest stands (0 = off) (saved)
 *   navhold 1000     navigation: how long each stand lasts (ms)
 *   fdrift 0         straight-line fix forward:  + = more right, - = more left (saved)
 *   bdrift 0         straight-line fix backward: same rule                     (saved)
 *   ?                help
 *
 * ------------------------------------------------------------
 * NAVIGATION SAFETY MODE (commands nw / ns / na / nd)
 * ------------------------------------------------------------
 *   Sent by the ROS cmd_vel_bridge when Nav2 is driving (never typed by
 *   hand). Same directions as w / s / a / d, but always:
 *     - the navigation gait: `navgait wave` (default) or `navgait tripod`
 *       (saved on the ESP32)
 *     - a full stand before every change of direction (both gaits)
 *     - tripod: a rest stand every NAV_TSTEPS complete cycles (default 3)
 *     - wave:   a rest stand every NAV_STEPS complete cycles (default 0 = off)
 *       (one cycle = every leg stepped once)
 *   A stand = every leg back to centre one at a time, lifted, then the
 *   stance pose, held for NAV_HOLD_MS. Manual w / s / a / d / x / h ...
 *   leave navigation mode and behave as before.
 *
 * ------------------------------------------------------------
 * TWIN REPORT (TCP only, never on USB serial)
 * ------------------------------------------------------------
 *   J c0 f0 t0 c1 f1 t1 ... c5 f5 t5 g
 *     18 logical joint angles in degrees (90 = neutral, same
 *     meaning as the gait code), legs 0..5, then the gripper
 *     servo angle (150 open .. 100 closed).
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_system.h>

Adafruit_PWMServoDriver boards[2] = {
  Adafruit_PWMServoDriver(0x40),
  Adafruit_PWMServoDriver(0x41)
};

const uint8_t BOARD_ADDR[2] = {0x40, 0x41};

#define SERVOMIN     110
#define SERVOMAX     490
#define SERVO_FREQ   50

#define NUM_LEGS     6
#define COXA         0
#define FEMUR        1
#define TIBIA        2

#define ANGLE_MIN    25      // hard clamp; protects the linkages
#define ANGLE_MAX    155

#define NEUTRAL      90
#define INTERP_MS    20


// ============================================================
// GRIPPER
// ============================================================
//
// Separate servo, not on the PCA9685 boards. Driven directly from
// an ESP32 GPIO through ESP32Servo, so it is independent of the
// gait and can be commanded mid-walk.

#define GRIPPER_PIN        13
#define GRIPPER_PULSE_MIN  500
#define GRIPPER_PULSE_MAX  2500

const int GRIPPER_OPEN  = 150;   // calibrated release angle
const int GRIPPER_CLOSE = 100;   // calibrated grab angle

Servo gripperServo;
bool  gripHolding = false;
int   gripAngle   = GRIPPER_OPEN;   // last angle written, for the twin



// ============================================================
// WIFI LINK
// ============================================================
//
// The ESP32 joins the same network as the Raspberry Pi and listens
// on TCP_PORT. Commands arriving on that socket are fed into the
// same handleLine() the serial console uses, so the vocabulary is
// identical over both transports. USB serial stays live for
// bring-up and flashing.

const char *WIFI_SSID = "HONOR X9c";
const char *WIFI_PASS = "@12345678";

#define TCP_PORT      8888
#define MDNS_NAME     "hexapod-esp"
#define WIFI_TIMEOUT  15000

WiFiServer server(TCP_PORT);
WiFiClient client;
String netLine = "";
bool netActive = false;

class Tee : public Print {
  char buf[160];
  size_t n = 0;
public:
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (n < sizeof(buf)) buf[n++] = (char)c;
    if (c == '\n' || n == sizeof(buf)) push();
    return 1;
  }
  void push() {
    if (n > 0 && client && client.connected())
      client.write((const uint8_t *)buf, n);
    n = 0;
  }
};

Tee Out;


// ============================================================
// LEG TABLE
// ============================================================
//
// dir: +1 or -1, chosen so a POSITIVE logical angle means
//        coxa  -> leg swings toward the FRONT of the robot
//        femur -> foot lifts UP
//        tibia -> foot curls IN toward the body
//
// Right legs +1, left legs -1 (mirrored build, all three joints).
// If a joint turns out backwards, fix it live with the `dir`
// command, then `dump` and paste the result back here.
//
// trim: degrees added at write time for horns that could not be
//       seated exactly at neutral.

struct Joint {
  uint8_t board;
  uint8_t ch;
  int8_t  dir;
  int8_t  trim;
};

struct Leg {
  Joint j[3];        // indexed COXA, FEMUR, TIBIA
  const char *name;
};

Leg LEGS[NUM_LEGS] = {
  //     COXA             FEMUR            TIBIA
  { { {0, 2, +1, 0}, {0, 0, +1, 0}, {0, 1, +1, -8} }, "0 front-R"  },
  { { {0, 5, +1, 0}, {0, 3, +1, 0}, {0, 4, +1, 0} }, "1 middle-R" },
  { { {0, 8, +1, 0}, {0, 6, +1, 0}, {0, 7, +1, 0} }, "2 back-R"   },
  { { {1, 2, -1, 0}, {1, 0, -1, 0}, {1, 1, -1, 0} }, "3 front-L"  },
  { { {1, 5, -1, 0}, {1, 3, -1, 0}, {1, 4, -1, 0} }, "4 middle-L" },
  { { {1, 8, -1, 0}, {1, 6, -1, 0}, {1, 7, -1, 0} }, "5 back-L"   }
};

const uint8_t TRIPOD_A[3] = {0, 4, 2};
const uint8_t TRIPOD_B[3] = {3, 1, 5};

// Wave gait: one leg at a time, from the trailing leg to the leading
// leg on each side.
//   forward / turns: back to front  (right side, then left side)
//   backward:        front to back  (the front legs are trailing)
const uint8_t WAVE_ORDER[NUM_LEGS]      = {2, 1, 0, 5, 4, 3};
const uint8_t WAVE_ORDER_BACK[NUM_LEGS] = {0, 1, 2, 3, 4, 5};

inline bool isRightLeg(int leg) { return leg < 3; }


// ============================================================
// GAIT PARAMETERS (tunable live)
// ============================================================

int COXA_SWING   = 12;   // stride length forward/back, degrees of coxa
int TURN_SWING   = 16;   // coxa travel when turning in place
int FEMUR_LIFT   = 20;   // how high a swinging leg raises
int TIBIA_TUCK   = 12;   // tibia curl during the lift
int STANCE_DROP  = 0;    // 0 = stand at all-90, no crouch
int PHASE_MS     = 200;  // duration of one gait phase

// Wave gait only (tripod ignores these)
int WAVE_EXTRA_LIFT = 4;    // extra femur lift on top of FEMUR_LIFT
int WAVE_PAUSE_MS   = 200;  // rest on all six feet before the next leg lifts
int WAVE_SWING      = 8;    // coxa stride for forward/backward (tripod uses COXA_SWING)
int WAVE_TURN       = 12;   // coxa travel when turning (tripod uses TURN_SWING)

// Straight-line correction (both gaits). The robot doesn't walk exactly
// straight, so one side takes a slightly longer stride than the other.
//   fdrift = walking forward,  bdrift = walking backward
//   + value -> robot goes more to ITS RIGHT,  - value -> more to its left
// So if it drifts LEFT, give a + value; if it drifts RIGHT, a - value.
// Saved on the ESP32 (survives a restart). Range -10..10, start with 1-2.
int FWD_DRIFT  = 0;
int BACK_DRIFT = 0;
#define DRIFT_MAX 10

Preferences prefs;


// ============================================================
// DIRECTION AND GAIT
// ============================================================
//
// Plain ints, not an enum: the Arduino IDE hoists a prototype for
// every function to the top of the .ino, above any type declared
// here, so a custom type in a function signature fails to compile.

#define MODE_FORWARD     0
#define MODE_BACKWARD    1
#define MODE_TURN_LEFT   2
#define MODE_TURN_RIGHT  3

const char *MODE_NAMES[] = {
  "forward", "backward", "turn left", "turn right"
};

int mode = MODE_FORWARD;

#define GAIT_TRIPOD      0
#define GAIT_WAVE        1

const char *GAIT_NAMES[] = { "tripod", "wave" };

int gait = GAIT_TRIPOD;


// ============================================================
// STATE
// ============================================================

int  current[NUM_LEGS][3];    // logical angles, 90 = neutral
int  target [NUM_LEGS][3];

// per-leg gait state; target[] is built from these
int  legSwing[NUM_LEGS];      // coxa offset from neutral
bool legUp   [NUM_LEGS];      // lifted?

// wave gait state
bool wavePrimed = false;      // legs placed in the staggered wave pattern?
int  wavePrimedMode = -1;     // direction the pattern was built for
int  waveStep = 0;            // index into the wave order of the next leg to swing

int  rawAngle[2][9];          // for the manual board/channel commands

// navigation safety mode (nw / ns / na / nd)
bool navMode     = false;
int  navNextMode = MODE_FORWARD;   // direction to take after the next stand
int  navCycles   = 0;              // complete cycles since the last stand
int  NAV_GAIT    = GAIT_WAVE;      // gait used while navigating (navgait, saved)
int  NAV_STEPS   = 0;              // wave:   cycles between rest stands (0 = off)
int  NAV_TSTEPS  = 3;              // tripod: cycles between rest stands (0 = off)
int  NAV_HOLD_MS = 1000;           // how long each stand is held
bool walking = false;
char pendingAction = 0;       // motion requested from the console
String inputLine = "";

void pollSerial();


// ============================================================
// DIGITAL TWIN REPORT
// ============================================================
//
// shown[][] is the logical angle last sent to each servo, including
// the in-between steps of every interpolated move, so the RViz model
// follows the real motion and not just the end points. Servos have
// no position feedback, so this is the COMMANDED pose.

#define TWIN_MS        40     // at most 25 reports a second while moving
#define TWIN_IDLE_MS   500    // and 2 a second while standing still

int  shown[NUM_LEGS][3];
bool twinOn = true;
bool twinDirty = true;
unsigned long lastTwin = 0;

void sendTwin(bool force) {
  if (!twinOn || !client || !client.connected()) return;

  unsigned long now = millis();
  if (!force) {
    unsigned long gap = twinDirty ? TWIN_MS : TWIN_IDLE_MS;
    if (now - lastTwin < gap) return;
  }

  char buf[112];
  int n = snprintf(buf, sizeof(buf), "J");
  for (int l = 0; l < NUM_LEGS; l++)
    for (int j = 0; j < 3; j++)
      n += snprintf(buf + n, sizeof(buf) - n, " %d", shown[l][j]);
  n += snprintf(buf + n, sizeof(buf) - n, " %d\n", gripAngle);

  Out.push();                       // never split a half-written console line
  client.write((const uint8_t *)buf, n);
  lastTwin = now;
  twinDirty = false;
}


// ============================================================
// GRIPPER CONTROL
// ============================================================
//
// Instant writes, no interpolation: these are safe to call from
// inside pollSerial() while a gait phase is interpolating, because
// the gripper servo is not part of the leg tables.

void gripGrab() {
  gripperServo.write(GRIPPER_CLOSE);
  gripAngle = GRIPPER_CLOSE;
  gripHolding = true;
  Out.println("gripper: grabbed");
  sendTwin(true);
}

void gripRelease() {
  gripperServo.write(GRIPPER_OPEN);
  gripAngle = GRIPPER_OPEN;
  gripHolding = false;
  Out.println("gripper: released");
  sendTwin(true);
}

void gripToggle() {
  if (gripHolding) gripRelease();
  else             gripGrab();
}


// ============================================================
// LOW LEVEL
// ============================================================

int clampAngle(int a) {
  if (a < ANGLE_MIN) return ANGLE_MIN;
  if (a > ANGLE_MAX) return ANGLE_MAX;
  return a;
}

int angleToPulse(int a) {
  return map(a, 0, 180, SERVOMIN, SERVOMAX);
}

void writeJoint(int leg, int joint, int logicalAngle) {
  Joint &jt = LEGS[leg].j[joint];
  int physical = clampAngle(NEUTRAL + jt.dir * (logicalAngle - NEUTRAL) + jt.trim);
  boards[jt.board].setPWM(jt.ch, 0, angleToPulse(physical));

  // what the servo actually got, back in logical terms (after the clamp)
  int shownAngle = NEUTRAL + jt.dir * (physical - jt.trim - NEUTRAL);
  if (shown[leg][joint] != shownAngle) {
    shown[leg][joint] = shownAngle;
    twinDirty = true;
  }
}

void writeRaw(int b, int ch, int a) {
  a = clampAngle(a);
  boards[b].setPWM(ch, 0, angleToPulse(a));

  // keep the twin in step with manual channel commands too
  for (int l = 0; l < NUM_LEGS; l++)
    for (int j = 0; j < 3; j++) {
      Joint &jt = LEGS[l].j[j];
      if (jt.board == b && jt.ch == ch) {
        shown[l][j] = NEUTRAL + jt.dir * (a - jt.trim - NEUTRAL);
        twinDirty = true;
      }
    }
  sendTwin(true);
}

bool boardPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}


// ============================================================
// MOTION
// ============================================================

void moveToTarget(int durationMs) {
  int steps = durationMs / INTERP_MS;
  if (steps < 1) steps = 1;

  int start[NUM_LEGS][3];
  for (int l = 0; l < NUM_LEGS; l++)
    for (int j = 0; j < 3; j++)
      start[l][j] = current[l][j];

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;

    for (int l = 0; l < NUM_LEGS; l++)
      for (int j = 0; j < 3; j++)
        writeJoint(l, j, start[l][j] + (int)((target[l][j] - start[l][j]) * t));

    sendTwin(false);
    delay(INTERP_MS);
    pollSerial();   // only records commands; never starts leg motion itself
  }

  for (int l = 0; l < NUM_LEGS; l++)
    for (int j = 0; j < 3; j++)
      current[l][j] = target[l][j];

  sendTwin(true);   // always report where the move ended
}

// Hold still for ms, but keep reading commands.
void pauseMs(int ms) {
  for (int t = 0; t < ms; t += INTERP_MS) {
    delay(INTERP_MS);
    pollSerial();
  }
}

// Build one leg's target angles from its gait state.
void applyLeg(int l) {
  target[l][COXA]  = NEUTRAL + legSwing[l];
  int lift = FEMUR_LIFT + (gait == GAIT_WAVE ? WAVE_EXTRA_LIFT : 0);
  target[l][FEMUR] = NEUTRAL + STANCE_DROP + (legUp[l] ? lift : 0);
  target[l][TIBIA] = NEUTRAL + (legUp[l] ? TIBIA_TUCK : 0);
}

void applyAll() {
  for (int l = 0; l < NUM_LEGS; l++) applyLeg(l);
}

void resetLegState() {
  for (int l = 0; l < NUM_LEGS; l++) {
    legSwing[l] = 0;
    legUp[l]    = false;
  }
  wavePrimed = false;
}

bool legsOffNeutral() {
  for (int l = 0; l < NUM_LEGS; l++)
    if (legSwing[l] != 0) return true;
  return false;
}

// Coxa offset for a PLANTED leg at the end of its push, for the
// current direction. The airborne leg swings to the opposite offset.
//
//   forward      all legs push back                     (coxa -)
//   backward     all legs push forward                  (coxa +)
//   turn right   right legs push forward, left push back
//   turn left    right legs push back,   left push forward
int pushOffset(int leg) {
  bool right = isRightLeg(leg);

  // Wave gait takes shorter steps than tripod.
  int sw = (gait == GAIT_WAVE) ? WAVE_SWING : COXA_SWING;
  int tw = (gait == GAIT_WAVE) ? WAVE_TURN  : TURN_SWING;

  switch (mode) {
    // drift: right legs push a little less / more than the left legs
    case MODE_FORWARD:    return -sw + (right ? FWD_DRIFT  : -FWD_DRIFT);
    case MODE_BACKWARD:   return +sw + (right ? -BACK_DRIFT : BACK_DRIFT);
    case MODE_TURN_RIGHT: return right ? +tw : -tw;
    case MODE_TURN_LEFT:  return right ? -tw : +tw;
  }
  return 0;
}


// ============================================================
// POSES
// ============================================================

void poseCenter() {
  resetLegState();
  for (int l = 0; l < NUM_LEGS; l++)
    for (int j = 0; j < 3; j++)
      target[l][j] = NEUTRAL;

  moveToTarget(800);
  Out.println("centered");
}

// With STANCE_DROP at 0 this is identical to the centered pose:
// every joint at 90, body held at its natural standing height.
void poseStance() {
  resetLegState();
  applyAll();
  moveToTarget(600);
  Out.println("stance");
}


// ============================================================
// TRIPOD GAIT
// ============================================================
//
//   1. lift tripod A
//   2. A swings forward while B, planted, pushes
//   3. lower A
//   4. lift tripod B
//   5. B swings while A, planted, pushes
//   6. lower B
//
// The planted group pushing is what moves the body. The direction
// only changes which way each leg pushes (see pushOffset).

void liftGroup(const uint8_t *group) {
  for (int i = 0; i < 3; i++) legUp[group[i]] = true;
  applyAll();
  moveToTarget(PHASE_MS);
}

void lowerGroup(const uint8_t *group) {
  for (int i = 0; i < 3; i++) legUp[group[i]] = false;
  applyAll();
  moveToTarget(PHASE_MS);
}

void strideGroups(const uint8_t *swingGroup, const uint8_t *pushGroup) {
  for (int i = 0; i < 3; i++) {
    int sl = swingGroup[i];               // airborne: reaches the opposite way
    legSwing[sl] = -pushOffset(sl);

    int pl = pushGroup[i];                // planted: pushes
    legSwing[pl] =  pushOffset(pl);
  }

  applyAll();
  moveToTarget(PHASE_MS * 2);
}

bool gaitCycle() {
  wavePrimed = false;   // tripod moves the legs out of any wave pattern

  liftGroup(TRIPOD_A);
  strideGroups(TRIPOD_A, TRIPOD_B);
  lowerGroup(TRIPOD_A);

  liftGroup(TRIPOD_B);
  strideGroups(TRIPOD_B, TRIPOD_A);
  lowerGroup(TRIPOD_B);
  return true;
}

// Bring a tripod back to neutral by lifting it, so the feet
// don't drag across the floor.
void settleGroup(const uint8_t *group) {
  bool needed = false;
  for (int i = 0; i < 3; i++)
    if (legSwing[group[i]] != 0) needed = true;
  if (!needed) return;

  wavePrimed = false;

  liftGroup(group);
  for (int i = 0; i < 3; i++) legSwing[group[i]] = 0;
  applyAll();
  moveToTarget(PHASE_MS);
  lowerGroup(group);
}

void settleToStance() {
  settleGroup(TRIPOD_A);
  settleGroup(TRIPOD_B);
  poseStance();
}


// ============================================================
// WAVE GAIT
// ============================================================
//
// One leg in the air at a time, five feet always down.
//
// Each leg swings through the full stride in one phase (from the
// end of its push, P, to touchdown, -P), then pushes back over the
// next five phases, 2P/5 per phase, while the other legs take
// their turn. P comes from pushOffset(), so forward, backward and
// both turns all work with the same code.
//
// To keep the body moving smoothly the legs have to be staggered
// through the stride, not all at neutral. "Priming" places each leg
// at its staggered position, one leg at a time, lifted, so nothing
// drags. That happens on the first wave cycle, after a stance or
// tripod walk, and whenever the direction changes.
//
// Each wave phase:
//   1. lift the swing leg
//   2. swing it forward to touchdown while the five planted legs
//      each push one step (this is what moves the body)
//   3. lower the swing leg

// i-th leg in the wave sequence. Uses the direction the pattern was
// primed for, so the order can't change halfway through a phase.
int waveLeg(int i) {
  return (wavePrimedMode == MODE_BACKWARD) ? WAVE_ORDER_BACK[i]
                                           : WAVE_ORDER[i];
}

// Coxa offset of a leg that still has `remaining` push steps
// (0..5) before its next swing.
//   remaining 0 -> P  (end of push, about to lift)
//   remaining 5 -> -P (just touched down)
int waveOffset(int leg, int remaining) {
  int P = pushOffset(leg);
  return P - (2 * P * remaining) / 5;
}

// Lift one leg, move its coxa to `swing`, put it down.
void repositionLeg(int l, int swing) {
  legUp[l] = true;
  applyAll();
  moveToTarget(PHASE_MS);

  legSwing[l] = swing;
  applyAll();
  moveToTarget(PHASE_MS);

  legUp[l] = false;
  applyAll();
  moveToTarget(PHASE_MS);
  pauseMs(WAVE_PAUSE_MS);
}

// Place every leg at its staggered starting position for the
// current direction. Leg waveLeg(i) has i push steps left,
// so waveLeg(0) is ready to swing first.
void wavePrime() {
  wavePrimed = false;
  wavePrimedMode = mode;
  waveStep = 0;

  for (int i = 0; i < NUM_LEGS; i++) {
    if (pendingAction || mode != wavePrimedMode) return;   // interrupted

    int l    = waveLeg(i);
    int want = waveOffset(l, i);
    if (legSwing[l] != want || legUp[l]) repositionLeg(l, want);
  }

  wavePrimed = true;
}

// One wave phase: swing waveLeg(waveStep), push the others.
void wavePhase() {
  int sl = waveLeg(waveStep);

  // 1. lift
  legUp[sl] = true;
  applyAll();
  moveToTarget(PHASE_MS);

  // 2. swing leg to touchdown, planted legs push one step.
  //    Leg waveLeg(i) has this many push steps left afterwards:
  //    the swing leg gets 5, the next one in line gets 0.
  for (int i = 0; i < NUM_LEGS; i++) {
    int l = waveLeg(i);
    int remaining = ((i - waveStep - 1) % NUM_LEGS + NUM_LEGS) % NUM_LEGS;
    legSwing[l] = waveOffset(l, remaining);
  }
  applyAll();
  moveToTarget(PHASE_MS * 2);

  // 3. lower
  legUp[sl] = false;
  applyAll();
  moveToTarget(PHASE_MS);

  // 4. rest on all six feet so the body settles before the next leg
  pauseMs(WAVE_PAUSE_MS);

  waveStep = (waveStep + 1) % NUM_LEGS;
}

// Six phases = every leg swings once. Stops early, with all six
// feet down, if a stop/other command or a new direction arrives.
// Returns true only if all six legs stepped.
bool waveCycle() {
  if (!wavePrimed || wavePrimedMode != mode) {
    wavePrime();
    if (!wavePrimed) return false;
  }

  for (int n = 0; n < NUM_LEGS; n++) {
    if (pendingAction || mode != wavePrimedMode) return false;
    wavePhase();
  }
  return true;
}

// Bring every leg back to neutral, one at a time, lifted.
void settleWave() {
  for (int i = 0; i < NUM_LEGS; i++) {
    int l = waveLeg(i);
    if (legSwing[l] != 0 || legUp[l]) repositionLeg(l, 0);
  }
  wavePrimed = false;
}

void settleWaveToStance() {
  settleWave();
  poseStance();
}


// ============================================================
// GAIT SELECTION
// ============================================================

bool walkCycle() {
  if (gait == GAIT_WAVE) return waveCycle();
  return gaitCycle();
}


// ============================================================
// NAVIGATION SAFETY MODE
// ============================================================

// A nav direction command (nw/ns/na/nd). Never changes direction
// directly: a new direction always goes through a full stand first.
void navCommand(int m) {
  bool wasNav = navMode;
  navMode = true;

  // same direction, same gait, already walking, nothing queued: carry on
  // (the gait itself is switched during the stand, see runAction 'S')
  if (wasNav && walking && m == mode && gait == NAV_GAIT && !pendingAction) return;

  navNextMode = m;
  walking = true;                     // resumes after the stand
  pendingAction = 'S';
  Out.print("nav: stand, then ");
  Out.println(MODE_NAMES[m]);
}

// Full stand: every leg back to centre one at a time (lifted, no
// dragging), stance pose, hold. Works from wave or tripod positions.
void navStand() {
  settleWave();
  settleGroup(TRIPOD_A);              // anything left over from a tripod walk
  settleGroup(TRIPOD_B);
  poseStance();
  pauseMs(NAV_HOLD_MS);
}

void setGait(int g) {
  if (g == gait) {
    Out.print("gait already ");
    Out.println(GAIT_NAMES[gait]);
    return;
  }

  gait = g;
  wavePrimed = false;

  // Coming out of the staggered wave pattern into tripod: bring the
  // legs back to neutral first (lifted, no dragging). Walking, if
  // on, carries on afterwards. Going into wave needs nothing here,
  // priming handles any starting position.
  if (gait == GAIT_TRIPOD && legsOffNeutral() && !pendingAction)
    pendingAction = 'n';

  Out.print("gait: ");
  Out.println(GAIT_NAMES[gait]);
}


// ============================================================
// LEG TEST
// ============================================================

// Lifts and lowers each leg in turn so you can confirm the numbering
// and the femur/tibia directions before walking.
void testLegs() {
  poseStance();

  for (int l = 0; l < NUM_LEGS; l++) {
    Out.print("  lifting leg ");
    Out.println(LEGS[l].name);

    legUp[l] = true;
    applyAll();
    moveToTarget(400);
    delay(600);

    legUp[l] = false;
    applyAll();
    moveToTarget(400);
    delay(400);
  }

  Out.println("leg test done");
}


// ============================================================
// CONSOLE
// ============================================================

void printPose() {
  Out.println();
  Out.println("  leg           coxa  femur  tibia");
  for (int l = 0; l < NUM_LEGS; l++) {
    Out.print("  ");
    Out.print(LEGS[l].name);
    Out.print("    ");
    for (int j = 0; j < 3; j++) {
      int a = current[l][j];
      if (a < 100) Out.print(" ");
      Out.print(a);
      Out.print("    ");
    }
    Out.println();
  }
  Out.print("  direction: ");
  Out.print(MODE_NAMES[mode]);
  Out.println(walking ? "  (walking)" : "  (stopped)");
  Out.print("  gait:      ");
  Out.println(GAIT_NAMES[gait]);
  Out.print("  gripper:   ");
  Out.print(gripHolding ? "closed" : "open");
  Out.print(" (");
  Out.print(gripHolding ? GRIPPER_CLOSE : GRIPPER_OPEN);
  Out.println(" deg)");
  Out.print("  nav mode:  ");
  Out.print(navMode ? "on" : "off");
  Out.print("   nav gait: ");
  Out.println(GAIT_NAMES[NAV_GAIT]);
  Out.print("  twin:      ");
  Out.println(twinOn ? "on" : "off");
  Out.println();
}

void printParams() {
  Out.print("gait=");     Out.print(GAIT_NAMES[gait]);
  Out.print("  swing=");  Out.print(COXA_SWING);
  Out.print("  turn=");   Out.print(TURN_SWING);
  Out.print("  lift=");   Out.print(FEMUR_LIFT);
  Out.print("  tuck=");   Out.print(TIBIA_TUCK);
  Out.print("  stance="); Out.print(STANCE_DROP);
  Out.print("  speed=");  Out.print(PHASE_MS);
  Out.print("ms  wlift="); Out.print(WAVE_EXTRA_LIFT);
  Out.print("  wpause="); Out.print(WAVE_PAUSE_MS);
  Out.print("ms  wswing="); Out.print(WAVE_SWING);
  Out.print("  wturn=");  Out.print(WAVE_TURN);
  Out.print("  navgait="); Out.print(GAIT_NAMES[NAV_GAIT]);
  Out.print("  navsteps="); Out.print(NAV_STEPS);
  Out.print("  navtsteps="); Out.print(NAV_TSTEPS);
  Out.print("  navhold="); Out.print(NAV_HOLD_MS);
  Out.print("ms  fdrift="); Out.print(FWD_DRIFT);
  Out.print("  bdrift="); Out.println(BACK_DRIFT);
}

void dumpTable() {
  Out.println();
  Out.println("Leg LEGS[NUM_LEGS] = {");
  for (int l = 0; l < NUM_LEGS; l++) {
    Out.print("  { { ");
    for (int j = 0; j < 3; j++) {
      Joint &jt = LEGS[l].j[j];
      Out.print("{");
      Out.print(jt.board); Out.print(", ");
      Out.print(jt.ch);    Out.print(", ");
      if (jt.dir > 0) Out.print("+");
      Out.print(jt.dir);   Out.print(", ");
      Out.print(jt.trim);  Out.print("}");
      if (j < 2) Out.print(", ");
    }
    Out.print(" }, \"");
    Out.print(LEGS[l].name);
    Out.println(l < NUM_LEGS - 1 ? "\" }," : "\" }");
  }
  Out.println("};");
  Out.println();
}

void printHelp() {
  Out.println();
  Out.println("  w   forward     s   backward    x   stop");
  Out.println("  a   turn left   d   turn right");
  Out.println("  tripod / wave     choose the gait");
  Out.println("  g   grab        r   release     grip  toggle");
  Out.println("  1   one cycle   1 d  one cycle turning right, etc.");
  Out.println("  h   stance      c   center      p   print pose");
  Out.println("  t   test each leg in turn");
  Out.println("  40 5 120     set board 0x40 ch5 to 120");
  Out.println("  41 2 +10     nudge board 0x41 ch2 by +10");
  Out.println("  dir 0 femur -1    flip a joint direction");
  Out.println("  dump              print the LEGS table");
  Out.println("  swing / turn / lift / tuck / stance / speed  <value>");
  Out.println("  wlift / wpause  <value>   wave-only extra lift, rest between legs");
  Out.println("  wswing / wturn  <value>   wave-only stride, turn step");
  Out.println("  twin on / off     joint reports for the RViz twin");
  Out.println("  navgait wave / tripod      gait Nav2 uses (saved)");
  Out.println("  navsteps / navtsteps <n>   nav rest stand every n cycles, wave / tripod (0 = off)");
  Out.println("  navhold <ms>               how long each nav stand lasts");
  Out.println("  fdrift / bdrift <value>  straight-line fix, forward / backward");
  Out.println("                    + = goes more right, - = more left (-10..10, saved)");
  Out.println();
  printParams();
}

int parseBoard(String tok) {
  tok.toLowerCase();
  if (tok == "40" || tok == "0x40" || tok == "0") return 0;
  if (tok == "41" || tok == "0x41" || tok == "1") return 1;
  return -1;
}

int parseJoint(String tok) {
  tok.toLowerCase();
  if (tok == "coxa"  || tok == "c") return COXA;
  if (tok == "femur" || tok == "f") return FEMUR;
  if (tok == "tibia" || tok == "t") return TIBIA;
  return -1;
}

int parseGait(String tok) {
  tok.toLowerCase();
  if (tok == "tripod") return GAIT_TRIPOD;
  if (tok == "wave")   return GAIT_WAVE;
  return -1;
}

// Returns true and sets m if c is a direction key.
bool parseModeKey(char c, int &m) {
  switch (c) {
    case 'w': case 'W': m = MODE_FORWARD;    return true;
    case 's': case 'S': m = MODE_BACKWARD;   return true;
    case 'a': case 'A': m = MODE_TURN_LEFT;  return true;
    case 'd': case 'D': m = MODE_TURN_RIGHT; return true;
  }
  return false;
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.length() == 1) {
    char c = line.charAt(0);
    int m;

    if (parseModeKey(c, m)) {
      navMode = false;                // manual driving: normal behaviour
      mode = m;
      walking = true;
      pendingAction = 0;
      Out.print("walking ");
      Out.print(MODE_NAMES[mode]);
      Out.print(" (");
      Out.print(GAIT_NAMES[gait]);
      Out.println(")");
      return;
    }

    // Gripper acts immediately, even mid-stride: it is a separate
    // servo and does not disturb the leg interpolation.
    if (c == 'g' || c == 'G') { gripGrab();    return; }
    if (c == 'r' || c == 'R') { gripRelease(); return; }

    // Leg motion commands are queued and run from loop(), after any
    // gait cycle (or wave phase) in progress has finished with all
    // feet down.
    if (c == 'x' || c == 'X') { walking = false; navMode = false; pendingAction = 'x'; Out.println("stopping"); return; }
    if (c == '1')             { walking = false; navMode = false; pendingAction = '1'; return; }
    if (c == 'h' || c == 'H') { walking = false; navMode = false; pendingAction = 'h'; return; }
    if (c == 'c' || c == 'C') { walking = false; navMode = false; pendingAction = 'c'; return; }
    if (c == 't' || c == 'T') { walking = false; navMode = false; pendingAction = 't'; return; }

    if (c == 'p' || c == 'P') { printPose(); return; }
    if (c == '?')             { printHelp(); return; }
  }

  String lower = line;
  lower.toLowerCase();
  if (lower == "dump")    { dumpTable();   return; }
  if (lower == "grab")    { gripGrab();    return; }
  if (lower == "release") { gripRelease(); return; }
  if (lower == "grip")    { gripToggle();  return; }
  if (lower == "tripod")  { setGait(GAIT_TRIPOD); return; }
  if (lower == "wave")    { setGait(GAIT_WAVE);   return; }
  if (lower == "twin on")  { twinOn = true;  Out.println("twin: on");  sendTwin(true); return; }
  if (lower == "twin off") { twinOn = false; Out.println("twin: off"); return; }

  // navigation moves from the ROS cmd_vel_bridge
  if (lower == "nw" || lower == "ns" || lower == "na" || lower == "nd") {
    int m;
    parseModeKey(lower.charAt(1), m);
    navCommand(m);
    return;
  }

  // tokenize into up to 4 pieces
  String tok[4];
  int n = 0, i = 0;
  while (i < (int)line.length() && n < 4) {
    while (i < (int)line.length() && line.charAt(i) == ' ') i++;
    int start = i;
    while (i < (int)line.length() && line.charAt(i) != ' ') i++;
    if (i > start) tok[n++] = line.substring(start, i);
  }

  // dir <leg> <joint> <1|-1>
  if (n == 4) {
    String cmd = tok[0]; cmd.toLowerCase();
    if (cmd == "dir") {
      int leg = tok[1].toInt();
      int jn  = parseJoint(tok[2]);
      int d   = tok[3].toInt();

      if (leg < 0 || leg >= NUM_LEGS) { Out.println("? leg must be 0-5"); return; }
      if (jn < 0)                     { Out.println("? joint must be coxa/femur/tibia"); return; }
      if (d != 1 && d != -1)          { Out.println("? dir must be 1 or -1"); return; }

      LEGS[leg].j[jn].dir = d;
      Out.print("leg ");
      Out.print(LEGS[leg].name);
      Out.print("  ");
      Out.print(tok[2]);
      Out.print(" dir = ");
      Out.println(d);
      return;
    }
    Out.println("? unknown - type ? for help");
    return;
  }

  if (n == 2) {
    String k = tok[0]; k.toLowerCase();

    // gait <tripod|wave>
    if (k == "gait") {
      int g = parseGait(tok[1]);
      if (g < 0) { Out.println("? gait must be tripod or wave"); return; }
      setGait(g);
      return;
    }

    // navgait <tripod|wave>  -> gait used while Nav2 drives
    if (k == "navgait") {
      int g = parseGait(tok[1]);
      if (g < 0) { Out.println("? navgait must be tripod or wave"); return; }
      NAV_GAIT = g;
      prefs.putInt("navgait", g);
      Out.print("nav gait: ");
      Out.println(GAIT_NAMES[g]);
      // already navigating: stand, switch gait, carry on in the same direction
      if (navMode && walking && gait != g && !pendingAction) {
        navNextMode = mode;
        pendingAction = 'S';
      }
      return;
    }

    // 1 <direction>  -> one cycle in that direction
    if (k == "1" && tok[1].length() == 1) {
      int m;
      if (parseModeKey(tok[1].charAt(0), m)) {
        mode = m;
        walking = false;
        pendingAction = '1';
        return;
      }
      Out.println("? direction must be w/s/a/d");
      return;
    }

    // parameter commands
    int v = tok[1].toInt();

    if (k == "swing")  { COXA_SWING  = v; printParams(); return; }
    if (k == "turn")   { TURN_SWING  = v; printParams(); return; }
    if (k == "lift")   { FEMUR_LIFT  = v; printParams(); return; }
    if (k == "tuck")   { TIBIA_TUCK  = v; printParams(); return; }
    if (k == "stance") { STANCE_DROP = v; printParams(); return; }
    if (k == "speed")  { PHASE_MS    = (v < 40 ? 40 : v); printParams(); return; }
    if (k == "wlift")  { WAVE_EXTRA_LIFT = v; printParams(); return; }
    if (k == "wpause") { WAVE_PAUSE_MS   = (v < 0 ? 0 : v); printParams(); return; }
    if (k == "wswing") { WAVE_SWING = v; wavePrimed = false; printParams(); return; }
    if (k == "wturn")  { WAVE_TURN  = v; wavePrimed = false; printParams(); return; }
    if (k == "navsteps")  { NAV_STEPS  = (v < 0 ? 0 : v); prefs.putInt("navsteps", NAV_STEPS);   printParams(); return; }
    if (k == "navtsteps") { NAV_TSTEPS = (v < 0 ? 0 : v); prefs.putInt("navtsteps", NAV_TSTEPS); printParams(); return; }
    if (k == "navhold")  { NAV_HOLD_MS = (v < 0 ? 0 : v); printParams(); return; }
    if (k == "fdrift" || k == "bdrift") {
      v = constrain(v, -DRIFT_MAX, DRIFT_MAX);
      if (k == "fdrift") { FWD_DRIFT = v;  prefs.putInt("fdrift", v); }
      else               { BACK_DRIFT = v; prefs.putInt("bdrift", v); }
      wavePrimed = false;               // re-stagger the wave legs for the new strides
      printParams();
      return;
    }

    Out.println("? unknown - type ? for help");
    return;
  }

  // manual board/channel/angle
  if (n == 3) {
    int b = parseBoard(tok[0]);
    if (b < 0) { Out.println("? board must be 40 or 41"); return; }

    int ch = tok[1].toInt();
    if (ch < 0 || ch > 8) { Out.println("? channel must be 0-8"); return; }

    walking = false;

    bool relative = (tok[2].charAt(0) == '+' || tok[2].charAt(0) == '-');
    int value = tok[2].toInt();
    int a = clampAngle(relative ? rawAngle[b][ch] + value : value);

    rawAngle[b][ch] = a;
    writeRaw(b, ch, a);

    Out.print("0x"); Out.print(BOARD_ADDR[b], HEX);
    Out.print(" ch"); Out.print(ch);
    Out.print(" -> "); Out.print(a);
    Out.println(" deg");
    return;
  }

  Out.println("? unknown - type ? for help");
}

void pollNetwork() {
  if (!client || !client.connected()) {
    WiFiClient incoming = server.available();
    if (incoming) {
      if (client) client.stop();
      client = incoming;
      client.setNoDelay(true);
      netActive = true;
      netLine = "";
      Out.println("client connected");
      Out.println("type ? for help");
      sendTwin(true);           // the new client gets the pose straight away
    }
  }

  while (client && client.available() > 0) {
    char c = client.read();

    if (c == '\n' || c == '\r') {
      if (netLine.length() > 0) {
        handleLine(netLine);
        netLine = "";
      }
    } else {
      netLine += c;
      if (netLine.length() > 48) netLine = "";
    }
  }
}

void pollSerial() {
  pollNetwork();

  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        handleLine(inputLine);
        inputLine = "";
      }
    } else {
      inputLine += c;
      if (inputLine.length() > 48) inputLine = "";
    }
  }
}

void runAction(char a) {
  switch (a) {
    case 'x':
      if (gait == GAIT_WAVE) settleWaveToStance();
      else                   settleToStance();
      break;
    case 'S':                   // navigation: stand, then go
      navStand();
      if (gait != NAV_GAIT) {     // switch to the navigation gait while standing
        gait = NAV_GAIT;
        wavePrimed = false;
        Out.print("nav: ");
        Out.print(GAIT_NAMES[gait]);
        Out.println(" gait");
      }
      if (pendingAction) break;   // something newer arrived during the stand
      mode = navNextMode;
      navCycles = 0;
      walking = navMode;
      break;
    case 'h': poseStance();     break;
    case 'c': poseCenter();     break;
    case 't': testLegs();       break;
    case 'n':                   // switching wave -> tripod
      settleGroup(TRIPOD_A);
      settleGroup(TRIPOD_B);
      Out.println("legs settled for tripod");
      break;
    case '1':
      Out.print("one cycle ");
      Out.print(MODE_NAMES[mode]);
      Out.print(" (");
      Out.print(GAIT_NAMES[gait]);
      Out.println(")");
      walkCycle();
      Out.println("cycle done");
      break;
  }
}


// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  // Low power: 80 MHz is plenty for the gait and keeps the current lower
  // (the board is powered from the Raspberry Pi's USB).
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  delay(200);
  Out.println();
  Out.println("=== hexapod tripod/wave gait + gripper ===");

  // why did it restart last time? (BROWNOUT = the supply voltage dipped)
  esp_reset_reason_t why = esp_reset_reason();
  Out.print("last reset: ");
  Out.println(why == ESP_RST_BROWNOUT ? "BROWNOUT (power dip - supply too weak)" :
              why == ESP_RST_POWERON  ? "power on" :
              why == ESP_RST_SW       ? "software" :
              why == ESP_RST_PANIC    ? "crash" :
              why == ESP_RST_INT_WDT || why == ESP_RST_TASK_WDT || why == ESP_RST_WDT ? "watchdog" :
              "other");

  // saved straight-line correction
  prefs.begin("hexapod", false);
  FWD_DRIFT  = constrain(prefs.getInt("fdrift", 0), -DRIFT_MAX, DRIFT_MAX);
  BACK_DRIFT = constrain(prefs.getInt("bdrift", 0), -DRIFT_MAX, DRIFT_MAX);
  NAV_GAIT   = (prefs.getInt("navgait", GAIT_WAVE) == GAIT_TRIPOD) ? GAIT_TRIPOD : GAIT_WAVE;
  NAV_STEPS  = max(0, (int)prefs.getInt("navsteps", 0));
  NAV_TSTEPS = max(0, (int)prefs.getInt("navtsteps", 3));
  Out.print("nav gait: "); Out.println(GAIT_NAMES[NAV_GAIT]);
  Out.print("drift: forward "); Out.print(FWD_DRIFT);
  Out.print("  backward "); Out.println(BACK_DRIFT);

  Wire.begin(21, 22);

  for (int b = 0; b < 2; b++) {
    boards[b].begin();
    boards[b].setPWMFreq(SERVO_FREQ);
    Out.print("board 0x");
    Out.print(BOARD_ADDR[b], HEX);
    Out.println(boardPresent(BOARD_ADDR[b]) ? "  detected" : "  NOT DETECTED");
  }

  // Gripper on its own GPIO, boots open so nothing is pinched.
  ESP32PWM::allocateTimer(0);
  gripperServo.setPeriodHertz(50);
  gripperServo.attach(GRIPPER_PIN, GRIPPER_PULSE_MIN, GRIPPER_PULSE_MAX);
  gripperServo.write(GRIPPER_OPEN);
  gripAngle = GRIPPER_OPEN;
  gripHolding = false;
  Out.print("gripper on GPIO ");
  Out.print(GRIPPER_PIN);
  Out.print("  open=");
  Out.print(GRIPPER_OPEN);
  Out.print(" close=");
  Out.println(GRIPPER_CLOSE);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  // Lower transmit power: smaller current spikes on every Wi-Fi send,
  // which is what pulls the USB supply down. Fine within a few metres.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    server.begin();
    server.setNoDelay(true);
    MDNS.begin(MDNS_NAME);
    MDNS.addService("hexapod", "tcp", TCP_PORT);

    Serial.print("wifi ");
    Serial.print(WiFi.localIP());
    Serial.print(" port ");
    Serial.print(TCP_PORT);
    Serial.print("  (");
    Serial.print(MDNS_NAME);
    Serial.println(".local)");
  } else {
    Serial.println("wifi failed, serial only");
  }

  delay(500);

  for (int l = 0; l < NUM_LEGS; l++)
    for (int j = 0; j < 3; j++) {
      current[l][j] = NEUTRAL;
      shown[l][j]   = NEUTRAL;
    }

  for (int b = 0; b < 2; b++)
    for (int ch = 0; ch < 9; ch++)
      rawAngle[b][ch] = NEUTRAL;

  resetLegState();
  poseCenter();

  printHelp();
  Out.println("Prop the body up, then type t before walking.");
}

void loop() {
  pollSerial();
  sendTwin(false);    // keeps the twin fresh while standing still

  // Safety: if the link drops while the robot is walking, stop and
  // settle onto all six feet rather than marching on unattended.
  if (walking && netActive &&
      (WiFi.status() != WL_CONNECTED || !client || !client.connected())) {
    walking = false;
    pendingAction = 'x';
    Serial.println("link lost, stopping");
  }

  if (pendingAction) {
    char a = pendingAction;
    pendingAction = 0;
    runAction(a);
  } else if (walking) {
    bool full = walkCycle();

    // navigation: rest stand every NAV_TSTEPS (tripod) / NAV_STEPS (wave) cycles, 0 = off
    int every = (gait == GAIT_TRIPOD) ? NAV_TSTEPS : NAV_STEPS;
    if (navMode && full && every > 0 && ++navCycles >= every && !pendingAction) {
      navCycles = 0;
      navNextMode = mode;
      pendingAction = 'S';
      Out.println("nav: rest stand");
    }
  }
}
