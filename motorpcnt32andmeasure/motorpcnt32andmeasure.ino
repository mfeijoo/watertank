#include <Arduino.h>
#include "driver/pcnt.h"
#include <math.h>
#include <SPI.h>

// =================== STEPPER PINS ===================
static const int X_STEP = 19;
static const int X_DIR  = 20;

static const int Y_STEP = 47;
static const int Y_DIR  = 14;

static const int Z_STEP = 10;
static const int Z_DIR  = 3;

// =================== ENCODER PINS (MAX3094E outputs) ===================
static const int X_ENC_A = 4;
static const int X_ENC_B = 5;

static const int Y_ENC_A = 6;
static const int Y_ENC_B = 7;

static const int Z_ENC_A = 15;
static const int Z_ENC_B = 16;

//===========================================================================
// STEPPING SETTINGS
//===========================================================================
static const uint32_t STEP_PULSE_US = 800; // STEP high time
static const uint32_t STEP_GAP_US   = 800; // STEP low time

//===============================================================================
// PCNT 32-bit extensions settings
//===============================================================================
static const int16_t PCNT_LIMIT = 30000;

//==========================================================
// PCNT 32-bit helper struct (software extension)
//==========================================================
struct Pcnt32 {
  pcnt_unit_t unit;
  pcnt_channel_t ch;
  int pinA;
  int pinB;
  volatile int32_t base; // software extension
};

// One unit per axis
static Pcnt32 pcX {PCNT_UNIT_0, PCNT_CHANNEL_0, X_ENC_A, X_ENC_B};
static Pcnt32 pcY {PCNT_UNIT_1, PCNT_CHANNEL_0, Y_ENC_A, Y_ENC_B};
static Pcnt32 pcZ {PCNT_UNIT_2, PCNT_CHANNEL_0, Z_ENC_A, Z_ENC_B};

//============================================================
// Z logical coordinate offset (so Z comp moves don't change "coord Z")
//============================================================
static volatile int32_t z_offset = 0;

// =================== KINEMATICS / CALIBRATION ===================
static const double STEPS_PER_REV      = 200.0;   // full steps, no microstep
static const double ENC_COUNTS_PER_REV = 2000.0;  // 1000 PPR * 2 (your PCNT mode)

static const double WHEEL_DIAMETER_MM = 12.0;
static const double YZ_MM_PER_REV     = WHEEL_DIAMETER_MM * 3.141592653589793;
static const double X_MM_PER_REV      = 1.0;

static const double X_MM_PER_COUNT = X_MM_PER_REV  / ENC_COUNTS_PER_REV;
static const double Y_MM_PER_COUNT = YZ_MM_PER_REV / ENC_COUNTS_PER_REV;
static const double Z_MM_PER_COUNT = YZ_MM_PER_REV / ENC_COUNTS_PER_REV;

static const double COUNTS_PER_STEP = ENC_COUNTS_PER_REV / STEPS_PER_REV;

//===============================================================================
// =================== DETECTOR / SPI SETTINGS ===================
// YOU MUST EDIT THESE PINS TO MATCH YOUR WIRING ON ESP32-S3
//===============================================================================

// SPI pins (set to your wiring)
static const int DET_SCK  = 12;
static const int DET_MISO = 13;
static const int DET_MOSI = 11;

// Chip selects (set to your wiring)
static const int CS_ADQ = 37;

// Integrator control pins (set to your wiring)
static const int RST_PIN  = 40;
static const int HOLD_PIN = 41;

// Timing
static volatile uint32_t integraltimemicros = 700; // default 700 us, can set to 200 us
static const uint32_t resettimemicros = 10;

// SPI settings from your old code
static SPISettings detSPI(66670000, MSBFIRST, SPI_MODE0);

// Raw detector readings (2 channels like your old code)
static volatile uint16_t det_ch0 = 0;
static volatile uint16_t det_ch1 = 0;

// Measurement buffer
struct Sample {
  uint32_t dt_us;
  uint16_t ch0;
  uint16_t ch1;
};

static const uint32_t MEAS_MAX_SAMPLES     = 5000;  // RAM use ~ (5000 * 8) = 40 KB
static const uint32_t MEAS_DEFAULT_SAMPLES = 1000;  // predefined "period" = samples * integraltimemicros
static Sample measBuf[MEAS_MAX_SAMPLES];

//======================================================================
// PCNT ISR: extend counter when it hits high/low limit
//======================================================================
static void IRAM_ATTR pcnt_isr(void *arg) {
  Pcnt32 *p = (Pcnt32*)arg;

  uint32_t st = 0;
  pcnt_get_event_status(p->unit, &st);

  if (st & PCNT_EVT_H_LIM) {
    p->base += PCNT_LIMIT;
    pcnt_counter_clear(p->unit);
  }
  if (st & PCNT_EVT_L_LIM) {
    p->base -= PCNT_LIMIT;
    pcnt_counter_clear(p->unit);
  }
}

//=========================================================================
// PCNT setup for quadrature (A/B)
//=========================================================================
static void pcntSetup(Pcnt32 &p) {
  pcnt_config_t c = {};
  c.pulse_gpio_num = p.pinA;
  c.ctrl_gpio_num  = p.pinB;
  c.unit           = p.unit;
  c.channel        = p.ch;

  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;

  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;

  c.counter_h_lim = PCNT_LIMIT;
  c.counter_l_lim = -PCNT_LIMIT;

  pcnt_unit_config(&c);

  pcnt_set_filter_value(p.unit, 100);
  pcnt_filter_enable(p.unit);

  pcnt_event_enable(p.unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(p.unit, PCNT_EVT_L_LIM);

  pcnt_counter_pause(p.unit);
  pcnt_counter_clear(p.unit);

  static bool isrInstalled = false;
  if (!isrInstalled) {
    pcnt_isr_service_install(0);
    isrInstalled = true;
  }

  pcnt_isr_handler_add(p.unit, pcnt_isr, (void*)&p);

  p.base = 0;
  pcnt_counter_resume(p.unit);
}

static int32_t pcntRead32(const Pcnt32 &p) {
  int16_t v = 0;
  pcnt_get_counter_value(p.unit, &v);
  return p.base + (int32_t)v;
}

static void pcntZero(Pcnt32 &p) {
  pcnt_counter_pause(p.unit);
  pcnt_counter_clear(p.unit);
  p.base = 0;
  pcnt_counter_resume(p.unit);
}

// Logical Z coordinate (raw Z + software offset)
static int32_t zCoord() {
  return pcntRead32(pcZ) + z_offset;
}

//============================================================
// Stepper helpers
//============================================================
static void stepPulse(int stepPin) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_GAP_US);
}

static void stepPulse2(int stepPin1, int stepPin2) {
  digitalWrite(stepPin1, HIGH);
  digitalWrite(stepPin2, HIGH);
  delayMicroseconds(STEP_PULSE_US);
  digitalWrite(stepPin1, LOW);
  digitalWrite(stepPin2, LOW);
  delayMicroseconds(STEP_GAP_US);
}

static void moveAxisSteps(char axis, int32_t steps) {
  int stepPin = -1, dirPin = -1;

  switch (axis){
    case 'x': stepPin = X_STEP; dirPin = X_DIR; break;
    case 'y': stepPin = Y_STEP; dirPin = Y_DIR; break;
    case 'z': stepPin = Z_STEP; dirPin = Z_DIR; break;
    default: return;
  }

  bool dir = (steps >= 0);
  uint32_t n = (steps >= 0) ? (uint32_t)steps : (uint32_t)(-steps);

  digitalWrite(dirPin, dir ? HIGH : LOW);
  delayMicroseconds(2);

  for (uint32_t i = 0; i < n; i++) stepPulse(stepPin);
}

// True coupled Y+Z: one Y step + one Z step at the same time
static void moveYZCoupledSteps(int32_t steps) {
  bool dir = (steps >= 0);
  uint32_t n = (steps >= 0) ? (uint32_t)steps : (uint32_t)(-steps);

  digitalWrite(Y_DIR, dir ? HIGH : LOW);
  digitalWrite(Z_DIR, dir ? HIGH : LOW);
  delayMicroseconds(2);

  for (uint32_t i = 0; i < n; i++) {
    stepPulse2(Y_STEP, Z_STEP);
  }
}

//============================================================
// MM helpers
//============================================================
static double xMM() { return (double)pcntRead32(pcX) * X_MM_PER_COUNT; }
static double yMM() { return (double)pcntRead32(pcY) * Y_MM_PER_COUNT; }
static double zMM() { return (double)zCoord()         * Z_MM_PER_COUNT; } // logical Z

static void printCoords() {
  int32_t x = pcntRead32(pcX);
  int32_t y = pcntRead32(pcY);
  int32_t z = zCoord();

  Serial.print("X coord: "); Serial.print(x);
  Serial.print(", Y coord: "); Serial.print(y);
  Serial.print(", Z coord: "); Serial.println(z);
}

static void printCoordsMM() {
  Serial.print("X mm: "); Serial.print(xMM(), 3);
  Serial.print(", Y mm: "); Serial.print(yMM(), 3);
  Serial.print(", Z mm: "); Serial.println(zMM(), 3);
}

static int32_t llround_i32(double v) { return (int32_t)llround(v); }
static int32_t mmToTargetCountsX(double mm) { return llround_i32(mm / X_MM_PER_COUNT); }
static int32_t mmToTargetCountsY(double mm) { return llround_i32(mm / Y_MM_PER_COUNT); }
static int32_t mmToTargetCountsZ(double mm) { return llround_i32(mm / Z_MM_PER_COUNT); }
static int32_t deltaCountsToSteps(int32_t dcounts) {
  return llround_i32((double)dcounts / COUNTS_PER_STEP);
}

static bool parse3DoublesComma(const char *s, double &a, double &b, double &c) {
  char *end = nullptr;
  a = strtod(s, &end);
  if (end == s || *end != ',') return false;
  b = strtod(end + 1, &end);
  if (*end != ',') return false;
  c = strtod(end + 1, &end);
  return true;
}

//============================================================
// Serial helpers
//============================================================
static bool readCmd(char *buf, size_t maxlen) {
  static size_t idx = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') continue;

    if (c == ';') {
      buf[idx] = 0;
      idx = 0;
      return true;
    }

    if (idx < maxlen - 1) buf[idx++] = c;
    else idx = 0; // overflow -> reset
  }
  return false;
}

static void sendCoordsBinary() {
  int32_t x = pcntRead32(pcX);
  int32_t y = pcntRead32(pcY);
  int32_t z = zCoord();

  Serial.write(0xAA);
  Serial.write(0x55);
  Serial.write((uint8_t*)&x, 4);
  Serial.write((uint8_t*)&y, 4);
  Serial.write((uint8_t*)&z, 4);
}

//============================================================
// DETECTOR: SPI read (based on your old model10board approach)
//============================================================
static void detReadChannels() {
  SPI.beginTransaction(detSPI);

  digitalWrite(CS_ADQ0, LOW);
  uint16_t v0 = SPI.transfer16(0b1101000000010000);
  SPI.transfer16(0);
  digitalWrite(CS_ADQ0, HIGH);

  digitalWrite(CS_ADQ1, LOW);
  uint16_t v1 = SPI.transfer16(0b1101000000010000);
  SPI.transfer16(0);
  digitalWrite(CS_ADQ1, HIGH);

  SPI.endTransaction();

  det_ch0 = v0;
  det_ch1 = v1;
}

// One integration sample: HOLD high -> read -> reset -> HOLD low
static void detReadOnce() {
  digitalWrite(HOLD_PIN, HIGH);
  detReadChannels();

  digitalWrite(RST_PIN, LOW);
  delayMicroseconds(resettimemicros);
  digitalWrite(RST_PIN, HIGH);

  delayMicroseconds(10);
  digitalWrite(HOLD_PIN, LOW);
}

// Measure N samples, store them, then send as one binary block
static void detMeasureAndSend(uint32_t N) {
  if (N == 0) N = 1;
  if (N > MEAS_MAX_SAMPLES) N = MEAS_MAX_SAMPLES;

  Serial.print("MEAS samples: ");
  Serial.print(N);
  Serial.print("  integ_us: ");
  Serial.println((uint32_t)integraltimemicros);

  uint32_t t0 = micros();

  // Ensure we start in a known state
  digitalWrite(RST_PIN, HIGH);
  digitalWrite(HOLD_PIN, LOW);

  // First reset cycle to start integration cleanly
  detReadOnce();
  uint32_t last = micros();

  for (uint32_t i = 0; i < N; i++) {
    // wait until integration time elapsed since last sample reset
    while ((uint32_t)(micros() - last) < (uint32_t)integraltimemicros) { /* busy wait */ }

    detReadOnce();          // reads the integrated value, then resets integrator
    last = micros();        // new integration starts now

    measBuf[i].dt_us = (uint32_t)(last - t0);
    measBuf[i].ch0   = det_ch0;
    measBuf[i].ch1   = det_ch1;
  }

  // Send everything together
  Serial.write(0xAB);
  Serial.write(0xCD);

  Serial.write((uint8_t*)&N, 4);

  uint32_t integ = (uint32_t)integraltimemicros;
  Serial.write((uint8_t*)&integ, 4);

  // payload
  Serial.write((uint8_t*)measBuf, N * sizeof(Sample));

  Serial.println("MEAS Done");
}

//============================================================
// Arduino setup/loop
//============================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT);
  pinMode(Z_STEP, OUTPUT); pinMode(Z_DIR, OUTPUT);

  digitalWrite(X_STEP, LOW);
  digitalWrite(Y_STEP, LOW);
  digitalWrite(Z_STEP, LOW);

  pcntSetup(pcX);
  pcntSetup(pcY);
  pcntSetup(pcZ);

  z_offset = 0;

  // Detector pins
  pinMode(CS_ADQ0, OUTPUT);
  pinMode(CS_ADQ1, OUTPUT);
  digitalWrite(CS_ADQ0, HIGH);
  digitalWrite(CS_ADQ1, HIGH);

  pinMode(RST_PIN, OUTPUT);
  pinMode(HOLD_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  digitalWrite(HOLD_PIN, LOW);

  // SPI
  SPI.begin(DET_SCK, DET_MISO, DET_MOSI);

  Serial.println("READY");
  Serial.println("Commands:");
  Serial.println(" xN; yN; zN;      (move motor axis by N motor steps, independent troubleshooting)");
  Serial.println(" YN;              (move Y and compensate Z STEP-BY-STEP, keep Z coord constant)");
  Serial.println(" p;               (print coords in counts)");
  Serial.println(" P;               (print coords in mm)");
  Serial.println(" b;               (send coords bytes: AA 55 + 3*int32 counts)");
  Serial.println(" Mx,y,z;           (move to absolute coords in mm, example: M10,25.5,-3;)");
  Serial.println(" z;               (zero coords)");
  Serial.println(" iN;              (set integration time in us, e.g. i700; or i200;)");
  Serial.println(" m; or mN;         (measure N samples, store, then send one binary block)");
}

void loop() {
  char cmd[48];

  if (!readCmd(cmd, sizeof(cmd))) return;
  if (cmd[0] == 0) return;

  //-------zero------
  if (cmd[0] == 'z' && cmd[1] == 0) {
    pcntZero(pcX);
    pcntZero(pcY);
    pcntZero(pcZ);
    z_offset = 0;
    Serial.println("OK Zerod");
    return;
  }

  //-----print coords in counts
  if (cmd[0] == 'p' && cmd[1] == 0) {
    printCoords();
    return;
  }

  //-----print coords in mm
  if (cmd[0] == 'P' && cmd[1] == 0) {
    printCoordsMM();
    return;
  }

  //-----binary coords (counts)
  if (cmd[0] == 'b' && cmd[1] == 0) {
    sendCoordsBinary();
    return;
  }

  //-----set integration time: i700;
  if (cmd[0] == 'i') {
    uint32_t v = (uint32_t)strtoul(cmd + 1, nullptr, 10);
    if (v < 50) v = 50;          // simple guard
    if (v > 50000) v = 50000;    // simple guard
    integraltimemicros = v;
    Serial.print("OK integ_us=");
    Serial.println((uint32_t)integraltimemicros);
    return;
  }

  //-----measure: m; or m2000;
  if (cmd[0] == 'm') {
    uint32_t N = MEAS_DEFAULT_SAMPLES;
    if (cmd[1] != 0) {
      N = (uint32_t)strtoul(cmd + 1, nullptr, 10);
    }
    detMeasureAndSend(N);
    return;
  }

  //============================================================
  // Move to absolute mm:  Mx,y,z;
  //============================================================
  if (cmd[0] == 'M') {
    double tx_mm, ty_mm, tz_mm;
    if (!parse3DoublesComma(cmd + 1, tx_mm, ty_mm, tz_mm)) {
      Serial.println("ERR M format. Use: Mx,y,z;");
      return;
    }

    int32_t cx = pcntRead32(pcX);
    int32_t cy = pcntRead32(pcY);
    int32_t cz = zCoord();

    int32_t tx = mmToTargetCountsX(tx_mm);
    int32_t ty = mmToTargetCountsY(ty_mm);
    int32_t tz = mmToTargetCountsZ(tz_mm);

    int32_t dx = tx - cx;
    int32_t dy = ty - cy;
    int32_t dz = tz - cz;

    int32_t sx = deltaCountsToSteps(dx);
    int32_t sy = deltaCountsToSteps(dy);
    int32_t sz = deltaCountsToSteps(dz);

    Serial.print("MOV M to mm: ");
    Serial.print(tx_mm, 3); Serial.print(", ");
    Serial.print(ty_mm, 3); Serial.print(", ");
    Serial.println(tz_mm, 3);

    if (sx != 0) moveAxisSteps('x', sx);

    if (sy != 0) {
      int32_t z_before = pcntRead32(pcZ);
      moveYZCoupledSteps(sy);
      int32_t z_after  = pcntRead32(pcZ);
      z_offset -= (z_after - z_before); // keep logical Z fixed
    }

    if (sz != 0) moveAxisSteps('z', sz);

    Serial.println("Done");
    printCoordsMM();
    return;
  }

  //============================================================
  // Coupled move: "Y200" or "Y-50"
  // TRUE step-by-step coupling Y+Z
  //============================================================
  if (cmd[0] == 'Y') {
    int32_t n = (int32_t)strtol(cmd + 1, nullptr, 10);

    Serial.print("MOV Y (coupled) ");
    Serial.println(n);

    int32_t z_before = pcntRead32(pcZ);
    moveYZCoupledSteps(n);
    int32_t z_after = pcntRead32(pcZ);

    z_offset -= (z_after - z_before);

    Serial.println("Done");
    printCoords();
    return;
  }

  //============================================================
  // Independent axis move: "x200" "y-50" "z1000"
  //============================================================
  char a = (char)tolower(cmd[0]);
  if (a == 'x' || a == 'y' || a == 'z') {
    int32_t n = (int32_t)strtol(cmd + 1, nullptr, 10);

    Serial.print("MOV ");
    Serial.print(a);
    Serial.print(" ");
    Serial.println(n);

    moveAxisSteps(a, n);

    Serial.println("Done");
    printCoords();
    return;
  }

  Serial.println("ERR");
}
