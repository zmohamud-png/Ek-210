/*
  Shake / Twist / Whack: handheld reaction game
  Board: Arduino Uno. Every pin number below is taken from the project schematic.

  Libraries (Arduino IDE: Tools > Manage Libraries):
    - "U8g2" by oliver (the OLED)
    - "Encoder" by Paul Stoffregen
  The GY-521 (MPU-6050) is read directly over I2C with Wire, so it needs no extra library.
  Readings are converted to m/s² with the same factor Adafruit's MPU6050 library uses.

  Serial Monitor at 9600 baud shows the key events. Set DEBUG_SENSORS to true to graph the
  sensors in the Serial Plotter instead.
*/

#include <Wire.h>
#include <U8g2lib.h>
#include <Encoder.h>

// ======================================================================
//  PINS (from the schematic)
// ======================================================================

// Rotary encoder, Bourns PEL12S: CHANNEL A -> D3, CHANNEL B -> D2, COMMON -> GND.
// D2 and D3 are the Uno's only external-interrupt pins. The A/B order only decides
// which direction counts up. TWIST uses |count|, so either direction works.
const uint8_t ENC_PIN_A    = 3;
const uint8_t ENC_PIN_B    = 2;
const uint8_t KNOB_LED_PIN = 4;   // D4 -> R1 (220 ohm) -> encoder LED+, LED- -> GND

// LED pushbuttons, as parallel lists in the same color order: green, blue, red, yellow, clear.
// Button: Terminal A -> pin, Terminal B -> GND   (INPUT_PULLUP, so a press reads LOW)
// LED:    LED+ -> pin,       LED- -> GND         (HIGH = on, resistor built into the button)
// D13 also drives the Uno's onboard LED, so the red LED blinks briefly at reset.
const uint8_t BUTTON_PINS[] = { 10, 12, A0,  8,  6 };
const uint8_t LED_PINS[]    = {  9, 11, 13,  7,  5 };
const char* const BUTTON_NAMES[] = { "GREEN", "BLUE", "RED", "YELLOW", "CLEAR" };
const uint8_t NUM_BUTTONS = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);
const uint8_t CLEAR_INDEX = 4;    // position of the clear button in the lists above

static_assert(sizeof(LED_PINS) == sizeof(BUTTON_PINS), "BUTTON_PINS and LED_PINS must be the same length");
static_assert(sizeof(BUTTON_NAMES) / sizeof(BUTTON_NAMES[0]) == NUM_BUTTONS, "BUTTON_NAMES must match BUTTON_PINS");
static_assert(CLEAR_INDEX < NUM_BUTTONS, "CLEAR_INDEX must point at a button in the lists");

// I2C on A4 (SDA) and A5 (SCL), shared by the OLED and the GY-521.
const uint8_t  OLED_ADDRESS    = 0x3C;    // the address your SSD1309 test sketches used
const uint8_t  MPU_ADDRESS     = 0x68;    // GY-521 with AD0 tied to GND, as on the schematic
const uint32_t WIRE_TIMEOUT_US = 25000;   // a stuck I2C bus (loose wire) resets instead of freezing

// Wired on the schematic but not used in this version (the sensor is polled, and no audio yet):
const uint8_t ACCEL_INT_PIN = A3;   // <- GY-521 INT
const uint8_t SOUND_RX_PIN  = A1;   // <- Audio FX TX
const uint8_t SOUND_TX_PIN  = A2;   // -> Audio FX RX, held HIGH (serial idle) so the board reads no noise
// To add sound later: SoftwareSerial sfx(SOUND_RX_PIN, SOUND_TX_PIN) with Adafruit's
// Soundboard library (UG is tied to GND, so the board starts in serial mode), then play
// clips from onSuccess(), onFailure(), onLevelComplete() and onWin().

const uint8_t SCREEN_WIDTH = 128;   // OLED width in pixels

// ======================================================================
//  GAME RULES
// ======================================================================
const uint16_t TASK_TIME_MS[] = { 3000, 2000, 1000 };   // time limit for levels 1, 2, 3
const uint8_t  MAX_LEVEL      = sizeof(TASK_TIME_MS) / sizeof(TASK_TIME_MS[0]);  // = 3
const uint8_t  LEVEL_TARGET   = 10;                     // correct tasks to clear a level

// ======================================================================
//  SHAKE (GY-521 / MPU-6050), in m/s²
// ======================================================================
const uint8_t  ACCEL_CONFIG_8G      = 0x10;     // ACCEL_CONFIG register value for ±8 g
const float    ACCEL_LSB_PER_G      = 4096.0;   // sensor counts per g at ±8 g
const float    GRAVITY_MS2          = 9.80665;  // m/s² per g
// On-chip low-pass filter (CONFIG register): 0 = 260 Hz (chip default), 4 = 21 Hz.
// Adafruit's MPU6050 examples use 21 Hz, and the ~0.02 m/s² resting noise you measured
// points to 21 Hz too. Peak sizes depend on this, so match whatever your test sketch used.
const uint8_t  ACCEL_DLPF_CFG       = 4;
const float    SHAKE_THRESHOLD      = 13.0;     // a peak counts when deviation rises above this
const float    SHAKE_REARM          = 6.5;      // the next peak counts only after dropping below this
const uint8_t  SHAKE_PEAKS_REQUIRED = 2;        // this many peaks...
const uint16_t SHAKE_WINDOW_MS      = 600;      // ...within this window = SHAKE
const uint8_t  CAL_SAMPLES          = 100;      // readings averaged for the resting baseline
const uint16_t CAL_SETTLE_MS        = 1000;     // time to set the device down after "Hold still"
const uint8_t  CAL_SAMPLE_MS        = 5;        // gap between calibration readings
const float    CAL_MAX_SPREAD       = 1.0;      // start over if the readings vary more than this
const uint16_t ACCEL_LOST_MS        = 1000;     // no good reading for this long = sensor unplugged

// ======================================================================
//  TWIST (PEL12S: 24 pulses per revolution, no detents)
// ======================================================================
const long COUNTS_PER_REV        = 96;   // the Encoder library counts 4 edges per pulse: 24 x 4
const long TWIST_DEGREES         = 60;
const long TWIST_COUNTS_REQUIRED = COUNTS_PER_REV * TWIST_DEGREES / 360;   // = 16 counts

// ======================================================================
//  TIMING (ms)
// ======================================================================
const uint8_t  DEBOUNCE_MS         = 20;    // a press must be steady this long to count
const uint8_t  RELEASE_DEBOUNCE_MS = 150;   // a release must last this long before the same
                                            // button can count again (absorbs shaky re-presses)
const uint16_t ROUND_PAUSE_MS      = 1000;  // pause after a success before the next prompt
const uint16_t IDLE_TIME_MS        = 300;   // inputs must be still this long before a prompt
const long     IDLE_KNOB_SLACK     = 1;     // knob jitter (counts) still treated as "not moved",
                                            // for a knob resting on a contact edge. 0 = none at all.
const uint16_t LEVEL_INTRO_MS      = 2000;  // "Level N / Get ready!" screen
const uint16_t LEVEL_COMPLETE_MS   = 2000;  // "Level Complete" screen
const uint16_t LEVEL_FLASH_MS      = 500;   // the single all-LED flash on level complete
const uint16_t WIN_BLINK_MS        = 250;   // win blink: time on, then time off
const uint8_t  WIN_BLINKS          = 6;     // number of win blinks
const uint16_t FAILED_SCREEN_MS    = 1500;  // "Level Failed" screen
const uint16_t GAME_OVER_SCREEN_MS = 2000;  // "Game Over" screen
const uint16_t SENSOR_RETRY_MS     = 1000;  // how often to look for a missing accelerometer

// ======================================================================
//  SERIAL / TUNING
// ======================================================================
const unsigned long SERIAL_BAUD = 9600;
// true = print sensor values for Tools > Serial Plotter while the game runs:
//   dev    = highest shake deviation since the last line (so short peaks aren't missed)
//   thresh, rearm = the two shake thresholds, drawn as flat reference lines
//   peaks  = shake peaks inside the current SHAKE_WINDOW_MS
//   enc    = knob count since the current prompt started
//   total  = knob count that never resets. To check COUNTS_PER_REV, turn the knob exactly
//            one revolution and watch total change by about 96 (easiest to read in the
//            Serial Monitor). If it doesn't, set COUNTS_PER_REV to what you see;
//            TWIST_COUNTS_REQUIRED follows automatically.
const bool     DEBUG_SENSORS  = false;
const uint16_t DEBUG_PRINT_MS = 100;  // keep >= 100 ms at 9600 baud so printing never stalls play
// Key events (prompt, button, success, failure, level) go to the Serial Monitor. They're off in
// tuning mode because lines of text confuse the Serial Plotter.
const bool     LOG_EVENTS     = !DEBUG_SENSORS;

// MPU-6050 registers (InvenSense register map)
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

// ======================================================================
//  OBJECTS AND STATE
// ======================================================================
// OLED: AITRIP 2.42" module, SSD1309 driver, 128x64, 4-pin I2C; its RES pin isn't connected.
// U8g2 draws it one 128-byte page at a time instead of keeping a 1 KB copy of the screen,
// which leaves RAM free for sound later. If the picture ever looks shifted 2 pixels sideways,
// use U8G2_SSD1309_128X64_NONAME2_1_HW_I2C instead.
U8G2_SSD1309_128X64_NONAME0_1_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Encoder knob(ENC_PIN_A, ENC_PIN_B);   // sets up D3/D2 with pull-ups and interrupts

enum GameState : uint8_t {
  INIT, CALIBRATE, NEW_GAME, SHOW_PROMPT, WAIT_FOR_INPUT, WHACK_LIGHT, WHACK_WAIT,
  SUCCESS, LEVEL_COMPLETE, WIN, FAILURE, GAME_OVER, RESTART_PROMPT, END
};
enum PromptType : uint8_t { PROMPT_SHAKE, PROMPT_TWIST, PROMPT_WHACK };

GameState     state        = INIT;
bool          stateIsNew   = true;   // true until the new state's entry actions have run
unsigned long stateStartMs = 0;

// Game
uint8_t       level        = 1;
uint16_t      score        = 0;
uint8_t       taskCount    = 0;
uint16_t      taskTimeMs   = 0;
PromptType    prompt       = PROMPT_SHAKE;
unsigned long timerStartMs = 0;      // countdown start for the prompt or the lit button
uint16_t      pauseBeforePromptMs = 0;
const __FlashStringHelper* failReason = nullptr;
bool          restartShown = false;

// WHACK
bool    whackHit[NUM_BUTTONS];
uint8_t whackHits = 0;
uint8_t litButton = 0;

// Buttons
struct ButtonState {
  bool down;                  // debounced: held right now
  bool rawDown;               // last raw reading
  bool pressed;               // true for one loop pass when a new press is confirmed
  unsigned long rawChangeMs;  // when the raw reading last changed
};
ButtonState buttons[NUM_BUTTONS];

// Accelerometer and shake detector
bool  accelReady        = false;         // true once calibrated
bool  accelLost         = false;         // set when the sensor stops answering
unsigned long lastAccelReadMs = 0;
float baseline          = GRAVITY_MS2;   // set by calibration
float deviation         = 0;             // latest |magnitude - baseline|
float debugMaxDeviation = 0;
bool  shakeArmed        = false;
bool  shakeDetected     = false;
unsigned long peakTimes[SHAKE_PEAKS_REQUIRED];
uint8_t peakCount = 0;
uint8_t peakHead  = 0;

// Calibration
uint8_t       calCount   = 0;
float         calSum     = 0, calMin = 0, calMax = 0;
unsigned long calTimerMs = 0;
uint16_t      calWaitMs  = 0;
uint32_t      randomMix  = 0;

// Knob and idle check
long          encoderTotal = 0;   // only used by the tuning printout
long          idleRefCount = 0;
unsigned long idleSinceMs  = 0;

// ======================================================================
//  SERIAL LOG
// ======================================================================
void logLine(const __FlashStringHelper* text) {
  if (LOG_EVENTS) Serial.println(text);
}

void logButton(uint8_t i) {
  if (LOG_EVENTS) {
    Serial.print(F("BUTTON  "));
    Serial.println(BUTTON_NAMES[i]);
  }
}

const __FlashStringHelper* promptName(PromptType p) {
  switch (p) {
    case PROMPT_SHAKE: return F("SHAKE");
    case PROMPT_TWIST: return F("TWIST");
    default:           return F("WHACK");
  }
}

// ======================================================================
//  LEDS
// ======================================================================
void setButtonLeds(bool on) {
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) digitalWrite(LED_PINS[i], on ? HIGH : LOW);
}

void setKnobLed(bool on) {
  digitalWrite(KNOB_LED_PIN, on ? HIGH : LOW);
}

void allLedsOff() {
  setButtonLeds(false);
  setKnobLed(false);
}

// ======================================================================
//  BUTTONS
// ======================================================================
// Debounces every button and flags new presses. A press counts once, on the press itself, so
// holding a button never counts twice or carries into the next round. A release only counts
// after RELEASE_DEBOUNCE_MS, so a shaky finger bouncing on the button still makes one press.
void updateButtons(unsigned long now) {
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    ButtonState& b = buttons[i];
    bool raw = digitalRead(BUTTON_PINS[i]) == LOW;
    b.pressed = false;
    if (raw != b.rawDown) {
      b.rawDown = raw;
      b.rawChangeMs = now;
    }
    unsigned long steadyMs = now - b.rawChangeMs;
    if (raw && !b.down && steadyMs >= DEBOUNCE_MS) {
      b.down = true;
      b.pressed = true;
    } else if (!raw && b.down && steadyMs >= RELEASE_DEBOUNCE_MS) {
      b.down = false;
    }
  }
}

// Index of a button pressed during this loop pass, or -1.
int8_t pressedButton() {
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    if (buttons[i].pressed) return i;
  }
  return -1;
}

bool anyButtonDown() {
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    if (buttons[i].down) return true;
  }
  return false;
}

// ======================================================================
//  ACCELEROMETER (GY-521 / MPU-6050)
// ======================================================================
bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMpuRegisters(uint8_t reg, uint8_t* data, uint8_t count) {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDRESS, count) != count) return false;
  for (uint8_t i = 0; i < count; i++) data[i] = Wire.read();
  return true;
}

// Resets the sensor, wakes it, and sets ±8 g and the filter. False if nothing answers.
bool initAccelerometer() {
  Wire.beginTransmission(MPU_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  writeMpuRegister(REG_PWR_MGMT_1, 0x80);   // reset to power-on defaults
  delay(100);                               // startup only, never during play
  bool ok = writeMpuRegister(REG_PWR_MGMT_1, 0x01)     // wake up, clock from the X gyro
         && writeMpuRegister(REG_SMPLRT_DIV, 0)         // 1 kHz sample rate
         && writeMpuRegister(REG_CONFIG, ACCEL_DLPF_CFG)
         && writeMpuRegister(REG_ACCEL_CONFIG, ACCEL_CONFIG_8G);

  uint8_t id = 0;
  readMpuRegisters(REG_WHO_AM_I, &id, 1);
  if (LOG_EVENTS) {
    Serial.print(F("Accelerometer found, WHO_AM_I = 0x"));   // 0x68 on a genuine MPU-6050
    Serial.println(id, HEX);
  }
  return ok;
}

// Reads the three axes as raw sensor counts.
bool readAccel(int16_t raw[3]) {
  uint8_t data[6];
  if (!readMpuRegisters(REG_ACCEL_XOUT_H, data, 6)) return false;
  for (uint8_t i = 0; i < 3; i++) {
    raw[i] = (int16_t)(((uint16_t)data[2 * i] << 8) | data[2 * i + 1]);
  }
  // All zeros means the sensor lost power and restarted asleep. Treat it as a missed reading.
  return raw[0] != 0 || raw[1] != 0 || raw[2] != 0;
}

// Total acceleration, sqrt(x² + y² + z²), in m/s². Tilting doesn't change it; shaking does.
float magnitudeMs2(const int16_t raw[3]) {
  float x = raw[0], y = raw[1], z = raw[2];
  return sqrt(x * x + y * y + z * z) * (GRAVITY_MS2 / ACCEL_LSB_PER_G);
}

// ======================================================================
//  SHAKE DETECTOR
// ======================================================================
// Forgets earlier peaks. Called at the start of every prompt.
void resetShake() {
  peakCount = 0;
  peakHead = 0;
  shakeDetected = false;
}

// Reads the accelerometer on every loop pass and looks for shake peaks. There's no smoothing,
// so peaks keep their full size.
void updateShake(unsigned long now) {
  if (!accelReady) return;   // not calibrated yet
  int16_t raw[3];
  if (!readAccel(raw)) {
    deviation = 0;           // a missed reading counts as calm
    if (now - lastAccelReadMs > ACCEL_LOST_MS) {
      accelReady = false;
      accelLost = true;
    }
    return;
  }
  lastAccelReadMs = now;

  deviation = fabs(magnitudeMs2(raw) - baseline);
  if (deviation > debugMaxDeviation) debugMaxDeviation = deviation;

  if (!shakeArmed) {
    if (deviation < SHAKE_REARM) shakeArmed = true;   // back near rest: the next peak can count
    return;
  }
  if (deviation <= SHAKE_THRESHOLD) return;

  // A new peak. It counts once, until the deviation drops back below SHAKE_REARM.
  shakeArmed = false;
  peakTimes[peakHead] = now;
  peakHead = (peakHead + 1) % SHAKE_PEAKS_REQUIRED;
  if (peakCount < SHAKE_PEAKS_REQUIRED) peakCount++;
  // Once the list is full, peakHead points at the oldest of the last SHAKE_PEAKS_REQUIRED peaks.
  if (peakCount == SHAKE_PEAKS_REQUIRED && now - peakTimes[peakHead] <= SHAKE_WINDOW_MS) {
    shakeDetected = true;
  }
}

// Peaks within the last SHAKE_WINDOW_MS (for the tuning printout).
uint8_t recentPeaks(unsigned long now) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < peakCount; i++) {
    if (now - peakTimes[i] <= SHAKE_WINDOW_MS) n++;
  }
  return n;
}

// ======================================================================
//  KNOB (rotary encoder)
// ======================================================================
// Sets the count back to 0 at the start of a prompt. The running total is only for tuning.
void resetKnob() {
  encoderTotal += knob.read();
  knob.write(0);
}

// TWIST: the knob is at least TWIST_COUNTS_REQUIRED counts (60°) from where it was when the
// prompt started, in either direction. Wobbling back and forth cancels out.
bool knobTwisted() {
  return labs(knob.read()) >= TWIST_COUNTS_REQUIRED;
}

// ======================================================================
//  IDLE CHECK
// ======================================================================
void resetIdle(unsigned long now) {
  idleSinceMs = now;
  idleRefCount = knob.read();
}

// True once nothing has happened for IDLE_TIME_MS: the device is calm, the knob hasn't moved,
// and no button is held. Call resetIdle() when a new wait starts.
bool inputsIdle(unsigned long now) {
  long count = knob.read();
  if (deviation >= SHAKE_REARM || labs(count - idleRefCount) > IDLE_KNOB_SLACK || anyButtonDown()) {
    idleSinceMs = now;
    idleRefCount = count;
    return false;
  }
  return now - idleSinceMs >= IDLE_TIME_MS;
}

// ======================================================================
//  SCREEN (2.42" 128x64 SSD1309 OLED via U8g2)
// ======================================================================
// U8g2 draws a screen one strip at a time, so each show...() function runs its drawing inside a
// firstPage() / nextPage() loop. Text y positions are baselines: where the bottom of the
// capital letters sits.
const uint8_t* const FONT_BIG    = u8g2_font_logisoso22_tr;   // commands, 22 px capitals
const uint8_t* const FONT_MEDIUM = u8g2_font_helvB14_tr;      // messages
const uint8_t* const FONT_SMALL  = u8g2_font_6x10_tr;         // details

void drawCentered(const char* text, uint8_t y) {
  display.drawStr((SCREEN_WIDTH - display.getStrWidth(text)) / 2, y, text);
}

// U8g2 measures text in RAM, so text kept in flash with F("...") is copied over first.
void drawCentered(const __FlashStringHelper* text, uint8_t y) {
  char buf[24];
  strncpy_P(buf, (const char*)text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  drawCentered(buf, y);
}

void drawLevel(uint8_t y) {
  char text[10];
  snprintf_P(text, sizeof(text), PSTR("Level %u"), (unsigned)level);
  drawCentered(text, y);
}

void drawScore(uint8_t y) {
  char text[12];
  snprintf_P(text, sizeof(text), PSTR("Score %u"), (unsigned)score);
  drawCentered(text, y);
}

// Top line during prompts: level on the left, score on the right, and a rule under them.
void drawStatusBar() {
  char text[12];
  display.setFont(FONT_SMALL);
  snprintf_P(text, sizeof(text), PSTR("Level %u"), (unsigned)level);
  display.drawStr(0, 8, text);
  snprintf_P(text, sizeof(text), PSTR("Score %u"), (unsigned)score);
  display.drawStr(SCREEN_WIDTH - display.getStrWidth(text), 8, text);
  display.drawHLine(0, 11, SCREEN_WIDTH);
}

void showHoldStill(bool again) {
  display.firstPage();
  do {
    display.setFont(FONT_MEDIUM);
    drawCentered(F("Hold still"), 26);
    display.setFont(FONT_SMALL);
    drawCentered(again ? F("Set it down, please") : F("Calibrating..."), 48);
  } while (display.nextPage());
}

void showSensorError() {
  display.firstPage();
  do {
    display.setFont(FONT_SMALL);
    drawCentered(F("Accelerometer"), 16);
    drawCentered(F("not found."), 28);
    drawCentered(F("Check the wiring."), 50);
  } while (display.nextPage());
}

void showLevelIntro() {
  display.firstPage();
  do {
    display.setFont(FONT_BIG);
    drawLevel(30);
    display.setFont(FONT_MEDIUM);
    drawCentered(F("Get ready!"), 58);
  } while (display.nextPage());
}

// The current prompt. For WHACK it also names the lit button.
void showPrompt() {
  display.firstPage();
  do {
    drawStatusBar();
    display.setFont(FONT_BIG);
    if (prompt == PROMPT_WHACK) {
      drawCentered(F("WHACK!"), 38);
      display.setFont(FONT_MEDIUM);
      drawCentered(BUTTON_NAMES[litButton], 60);
    } else {
      drawCentered(prompt == PROMPT_SHAKE ? F("SHAKE!") : F("TWIST!"), 49);
    }
  } while (display.nextPage());
}

void showSuccess() {
  const __FlashStringHelper* praise;   // chosen before drawing, since the drawing repeats per strip
  switch (random(3)) {
    case 0:  praise = F("GREAT!"); break;
    case 1:  praise = F("NICE!");  break;
    default: praise = F("YES!");   break;
  }
  display.firstPage();
  do {
    display.setFont(FONT_BIG);
    drawCentered(praise, 32);
    display.setFont(FONT_MEDIUM);
    drawScore(58);
  } while (display.nextPage());
}

void showLevelComplete() {
  display.firstPage();
  do {
    display.setFont(FONT_MEDIUM);
    drawLevel(18);
    drawCentered(F("Complete!"), 38);
    display.setFont(FONT_SMALL);
    drawScore(58);
  } while (display.nextPage());
}

void showWin() {
  display.firstPage();
  do {
    display.setFont(FONT_BIG);
    drawCentered(F("YOU WIN!"), 34);
    display.setFont(FONT_MEDIUM);
    drawScore(60);
  } while (display.nextPage());
}

void showLevelFailed() {
  display.firstPage();
  do {
    display.setFont(FONT_MEDIUM);
    drawCentered(F("Level Failed"), 26);
    display.setFont(FONT_SMALL);
    if (failReason) drawCentered(failReason, 50);
  } while (display.nextPage());
}

void showGameOver() {
  display.firstPage();
  do {
    display.setFont(FONT_MEDIUM);
    drawCentered(F("Game Over"), 26);
    drawScore(54);
  } while (display.nextPage());
}

void showPlayAgain() {
  display.firstPage();
  do {
    display.setFont(FONT_MEDIUM);
    drawCentered(F("Play again?"), 22);
    display.setFont(FONT_SMALL);
    drawCentered(F("Press CLEAR = yes"), 44);
    drawCentered(F("Twist knob = no"), 58);
  } while (display.nextPage());
}

void showGoodbye() {
  display.firstPage();
  do {
    display.setFont(FONT_BIG);
    drawCentered(F("BYE!"), 30);
    display.setFont(FONT_SMALL);
    drawCentered(F("Press RESET"), 46);
    drawCentered(F("to play again"), 58);
  } while (display.nextPage());
}

// ======================================================================
//  GAME EVENTS (each one is the place to add a sound later)
// ======================================================================
void onSuccess() {
  score++;
  taskCount++;
  allLedsOff();
  if (LOG_EVENTS) {
    Serial.print(F("SUCCESS  score "));
    Serial.print(score);
    Serial.print(F(", level "));
    Serial.print(level);
    Serial.print(F(" task "));
    Serial.print(taskCount);
    Serial.print('/');
    Serial.println(LEVEL_TARGET);
  }
  // Sound: play the "correct" clip here.
}

void onFailure() {
  allLedsOff();   // the button LEDs and the knob LED
  if (LOG_EVENTS) {
    Serial.print(F("FAILURE  "));
    Serial.println(failReason);
  }
  showLevelFailed();
  // Sound: play the "wrong" clip here.
}

void onLevelComplete() {
  setKnobLed(false);
  setButtonLeds(true);   // the single flash; the LEVEL_COMPLETE state turns them off again
  showLevelComplete();
  if (LOG_EVENTS) {
    Serial.print(F("LEVEL "));
    Serial.print(level);
    Serial.println(F(" COMPLETE"));
  }
  // Sound: play the "level complete" clip here.
}

void onWin() {
  showWin();
  if (LOG_EVENTS) {
    Serial.print(F("YOU WIN!  final score "));
    Serial.println(score);
  }
  // Sound: play the "win" clip here.
}

// ======================================================================
//  STATE MACHINE HELPERS
// ======================================================================
void changeState(GameState next) {
  state = next;
  stateIsNew = true;
  stateStartMs = millis();
}

unsigned long timeInState() {
  return millis() - stateStartMs;
}

void fail(const __FlashStringHelper* reason) {
  failReason = reason;
  changeState(FAILURE);
}

// ======================================================================
//  CALIBRATION
// ======================================================================
// Shows "Hold still" and starts the averaging over, after CAL_SETTLE_MS to let the device settle.
void startCalibration(bool again) {
  showHoldStill(again);
  calCount = 0;
  calSum = 0;
  calMin = 1000;
  calMax = 0;
  calTimerMs = millis();
  calWaitMs = CAL_SETTLE_MS;
}

// Averages CAL_SAMPLES readings of the total acceleration (about 9.8 m/s² at rest). If the
// device moved during the readings, it starts over, so a bump can't skew the baseline.
void runCalibration(bool entering) {
  if (entering) {
    logLine(F("CALIBRATING  hold still"));
    startCalibration(false);
  }
  if (millis() - calTimerMs < calWaitMs) return;
  calTimerMs = millis();
  calWaitMs = CAL_SAMPLE_MS;

  int16_t raw[3];
  if (!readAccel(raw)) {   // the sensor went away
    changeState(INIT);
    return;
  }
  // Every analog pin is wired, so there's no floating pin for randomSeed(analogRead(...)).
  // The sensor's noise seeds the random prompts instead.
  randomMix = randomMix * 1664525UL + 1013904223UL + (uint16_t)raw[0]
            + ((uint32_t)(uint16_t)raw[1] << 8) + ((uint32_t)(uint16_t)raw[2] << 16);
  float m = magnitudeMs2(raw);
  calSum += m;
  if (m < calMin) calMin = m;
  if (m > calMax) calMax = m;
  if (++calCount < CAL_SAMPLES) return;

  if (calMax - calMin > CAL_MAX_SPREAD) {
    logLine(F("CALIBRATING  moved, starting over"));
    startCalibration(true);
    return;
  }
  baseline = calSum / calCount;
  randomSeed(randomMix ^ micros());
  deviation = 0;
  shakeArmed = true;
  accelReady = true;
  lastAccelReadMs = millis();
  if (LOG_EVENTS) {
    Serial.print(F("CALIBRATED  resting magnitude "));
    Serial.print(baseline, 2);
    Serial.println(F(" m/s^2"));
  }
  changeState(NEW_GAME);
}

// ======================================================================
//  ROUNDS
// ======================================================================
// New level: task count back to 0, the level's time limit, and the "Level N" screen.
// The first prompt follows after the pause and the idle check.
void startLevel() {
  taskCount = 0;
  taskTimeMs = TASK_TIME_MS[level - 1];
  if (LOG_EVENTS) {
    Serial.print(F("LEVEL "));
    Serial.print(level);
    Serial.print(F("  time limit "));
    Serial.print(taskTimeMs);
    Serial.println(F(" ms"));
  }
  showLevelIntro();
  pauseBeforePromptMs = LEVEL_INTRO_MS;
  changeState(SHOW_PROMPT);
}

// Picks SHAKE, TWIST or WHACK at random and starts its countdown.
void startPrompt() {
  prompt = (PromptType)random(3);
  if (LOG_EVENTS) {
    Serial.print(F("PROMPT  "));
    Serial.println(promptName(prompt));
  }
  if (prompt == PROMPT_WHACK) {
    for (uint8_t i = 0; i < NUM_BUTTONS; i++) whackHit[i] = false;
    whackHits = 0;
    changeState(WHACK_LIGHT);
    return;
  }
  setButtonLeds(false);
  setKnobLed(prompt == PROMPT_TWIST);   // the knob lights up only for TWIST
  showPrompt();
  resetKnob();                          // count from 0 for this prompt
  resetShake();
  timerStartMs = millis();
  changeState(WAIT_FOR_INPUT);
}

// WHACK: all button LEDs off, then light a random button not yet hit in this task.
void lightNextWhackButton() {
  uint8_t pick = random(NUM_BUTTONS - whackHits);
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    if (whackHit[i]) continue;
    if (pick == 0) {
      litButton = i;
      break;
    }
    pick--;
  }
  setButtonLeds(false);
  setKnobLed(false);
  digitalWrite(LED_PINS[litButton], HIGH);
  showPrompt();
  if (LOG_EVENTS) {
    Serial.print(F("WHACK  light "));
    Serial.println(BUTTON_NAMES[litButton]);
  }
  timerStartMs = millis();   // each lit button gets the full time limit
}

// ======================================================================
//  TUNING PRINTOUT
// ======================================================================
void printSensorData(unsigned long now) {
  static unsigned long lastPrintMs = 0;
  if (now - lastPrintMs < DEBUG_PRINT_MS) return;
  lastPrintMs = now;
  long count = knob.read();
  Serial.print(F("dev:"));      Serial.print(debugMaxDeviation, 1);
  Serial.print(F(",thresh:"));  Serial.print(SHAKE_THRESHOLD, 1);
  Serial.print(F(",rearm:"));   Serial.print(SHAKE_REARM, 1);
  Serial.print(F(",peaks:"));   Serial.print(recentPeaks(now));
  Serial.print(F(",enc:"));     Serial.print(count);
  Serial.print(F(",total:"));   Serial.println(encoderTotal + count);
  debugMaxDeviation = 0;
}

// ======================================================================
//  SETUP AND LOOP
// ======================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    pinMode(LED_PINS[i], OUTPUT);
  }
  pinMode(KNOB_LED_PIN, OUTPUT);
  allLedsOff();
  digitalWrite(SOUND_TX_PIN, HIGH);   // serial idle level for the sound board's RX
  pinMode(SOUND_TX_PIN, OUTPUT);

  Wire.begin();
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);

  display.setI2CAddress(OLED_ADDRESS * 2);   // U8g2 takes the address shifted left one bit
  display.begin();

  logLine(F("Shake / Twist / Whack"));
  changeState(INIT);
}

void loop() {
  unsigned long now = millis();
  updateButtons(now);
  updateShake(now);
  if (DEBUG_SENSORS) printSensorData(now);

  if (accelLost) {   // a loose accelerometer wire: show the error screen until it's back
    accelLost = false;
    logLine(F("Accelerometer stopped answering"));
    if (state != END) changeState(INIT);
  }

  bool entering = stateIsNew;
  stateIsNew = false;

  switch (state) {
    case INIT:
      // Find the accelerometer. If it's missing, show an error and look again every second.
      if (entering) allLedsOff();
      if (entering || millis() - timerStartMs >= SENSOR_RETRY_MS) {
        timerStartMs = millis();
        if (initAccelerometer()) {
          changeState(CALIBRATE);
        } else {
          if (entering) showSensorError();
          logLine(F("Accelerometer not found at I2C address 0x68. Check the wiring."));
        }
      }
      break;

    case CALIBRATE:
      runCalibration(entering);
      break;

    case NEW_GAME:
      // Level 1, score 0. startLevel() sets the task count to 0 and the level 1 time limit.
      allLedsOff();
      level = 1;
      score = 0;
      logLine(F("NEW GAME"));
      startLevel();
      break;

    case SHOW_PROMPT:
      // Pause between rounds, and wait until nothing has moved for IDLE_TIME_MS.
      if (entering) resetIdle(now);
      if (inputsIdle(now) && timeInState() >= pauseBeforePromptMs) startPrompt();
      break;

    case WAIT_FOR_INPUT: {
      // SHAKE or TWIST: the right action succeeds; any other action fails.
      bool shaken  = shakeDetected;
      bool twisted = knobTwisted();
      int8_t button = pressedButton();
      if (prompt == PROMPT_SHAKE ? shaken : twisted) {
        changeState(SUCCESS);
      } else if (twisted) {          // during SHAKE
        fail(F("Oops! You twisted."));
      } else if (shaken) {           // during TWIST
        fail(F("Oops! You shook it."));
      } else if (button >= 0) {
        logButton(button);
        fail(F("Oops! No buttons."));
      } else if (millis() - timerStartMs >= taskTimeMs) {
        fail(F("Too slow!"));
      }
      break;
    }

    case WHACK_LIGHT:
      lightNextWhackButton();
      changeState(WHACK_WAIT);
      break;

    case WHACK_WAIT: {
      // Only the buttons count. Shaking and the knob are ignored, since whacking jostles both.
      int8_t button = pressedButton();
      if (buttons[litButton].pressed) {
        logButton(litButton);
        digitalWrite(LED_PINS[litButton], LOW);
        whackHit[litButton] = true;
        whackHits++;
        changeState(whackHits >= NUM_BUTTONS ? SUCCESS : WHACK_LIGHT);
      } else if (button >= 0) {
        logButton(button);
        fail(F("Oops! Wrong button."));
      } else if (millis() - timerStartMs >= taskTimeMs) {
        fail(F("Too slow!"));
      }
      break;
    }

    case SUCCESS:
      onSuccess();
      if (taskCount >= LEVEL_TARGET) {
        changeState(LEVEL_COMPLETE);
      } else {
        showSuccess();
        pauseBeforePromptMs = ROUND_PAUSE_MS;
        changeState(SHOW_PROMPT);
      }
      break;

    case LEVEL_COMPLETE:
      if (entering) onLevelComplete();
      if (timeInState() >= LEVEL_FLASH_MS) setButtonLeds(false);   // end of the single flash
      if (timeInState() >= LEVEL_COMPLETE_MS) {
        if (level < MAX_LEVEL) {
          level++;
          startLevel();   // task count 0, the new time limit, then the idle check
        } else {
          changeState(WIN);
        }
      }
      break;

    case WIN: {
      if (entering) onWin();
      unsigned long phase = timeInState() / WIN_BLINK_MS;   // even = on, odd = off
      if (phase < 2UL * WIN_BLINKS) {
        setButtonLeds(phase % 2 == 0);
      } else {
        setButtonLeds(false);
        changeState(RESTART_PROMPT);
      }
      break;
    }

    case FAILURE:
      if (entering) onFailure();
      if (timeInState() >= FAILED_SCREEN_MS) changeState(GAME_OVER);
      break;

    case GAME_OVER:
      if (entering) {
        showGameOver();
        if (LOG_EVENTS) {
          Serial.print(F("GAME OVER  final score "));
          Serial.println(score);
        }
      }
      if (timeInState() >= GAME_OVER_SCREEN_MS) changeState(RESTART_PROMPT);
      break;

    case RESTART_PROMPT:
      // Wait until nothing is moving first, so a twist still in progress can't answer "no".
      if (entering) {
        restartShown = false;
        resetIdle(now);
      }
      if (!restartShown) {
        if (inputsIdle(now)) {
          showPlayAgain();
          digitalWrite(LED_PINS[CLEAR_INDEX], HIGH);
          setKnobLed(true);
          resetKnob();
          restartShown = true;
          logLine(F("PLAY AGAIN?  CLEAR = yes, twist = no"));
        }
      } else if (buttons[CLEAR_INDEX].pressed) {
        logLine(F("RESTART"));
        allLedsOff();
        changeState(NEW_GAME);
      } else if (knobTwisted()) {
        logLine(F("QUIT"));
        changeState(END);
      }
      break;

    case END:
      if (entering) {
        allLedsOff();
        showGoodbye();
        logLine(F("STOPPED  press RESET to play again"));
      }
      break;
  }
}