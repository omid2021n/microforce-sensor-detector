#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
const uint8_t FMA_ADDR = 0x28;

// --- SET THIS FROM YOUR SENSOR'S PART NUMBER (015 or 025) ---
constexpr float FORCE_RANGE_N = 25.0f;   // <-- CONFIRM against the sticker

constexpr uint16_t FULL_SCALE_COUNTS = 16384;          // 2^14
constexpr float    COUNTS_PER_NEWTON =
    (0.60f * FULL_SCALE_COUNTS) / FORCE_RANGE_N;       // slope from 20-80% transfer fn

// Deadband applied at the Arduino output level only. Set to 0.0f for raw.
constexpr float ZERO_DEADBAND_N = 0.02f;

// Sample period in milliseconds. 20 ms = 50 Hz.
constexpr uint32_t SAMPLE_PERIOD_MS = 20;

// If the loop falls more than this many periods behind (for example, the
// serial port blocks), restart the schedule instead of sending a burst of
// fast catch-up samples.
constexpr uint32_t MAX_LAG_PERIODS = 5;

// Consecutive-fault counter (note: this logic currently prints no warning)
constexpr uint8_t FAULT_WARN_THRESHOLD = 10;

// Longest accepted command, in characters
constexpr uint8_t CMD_MAX_LEN = 16;

// ---------------------------------------------------------------------------
// Output protocol (one line per sample)
//   <millis>,<force>   e.g. 8557,0.000
//   <millis>,-1        sensor status fault / stale
//   <millis>,-2        I2C read failed
//   # ...              banner / info, ignore
//
// Commands received (each one followed by a newline)
//   TARE  (or r)       re-zero the sensor - it must be unloaded
//   RESET              reboot the board; USB disconnects and reconnects
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
int32_t  zeroOffsetCounts  = 0;
uint8_t  consecutiveFaults = 0;
uint32_t lastSampleMs      = 0;   // scheduled time of the most recent sample

char     cmdBuf[CMD_MAX_LEN + 1];
uint8_t  cmdLen            = 0;
bool     cmdOverflow       = false;

// ---------------------------------------------------------------------------
// Low-level read
// ---------------------------------------------------------------------------
bool readRawCounts(uint16_t &counts, uint8_t &status) {
  uint8_t n = Wire.requestFrom((uint8_t)FMA_ADDR, (uint8_t)2);
  if (n != 2) return false;

  uint32_t t0 = micros();
  while (Wire.available() < 2 && (micros() - t0) < 2000) { /* spin */ }
  if (Wire.available() < 2) return false;

  uint8_t byte1 = Wire.read();
  uint8_t byte2 = Wire.read();

  status = (byte1 >> 6) & 0x03;
  counts = ((uint16_t)(byte1 & 0x3F) << 8) | byte2;
  return true;
}

// ---------------------------------------------------------------------------
// Tare
// ---------------------------------------------------------------------------
void performTare() {
  Serial.println(F("# Taring... do not touch sensor."));
  int64_t sum = 0;
  constexpr uint8_t tareSamples = 32;
  uint8_t got = 0;

  { uint16_t c; uint8_t st; readRawCounts(c, st); }   // discard first sample

  for (uint8_t i = 0; i < tareSamples; i++) {
    uint16_t c; uint8_t st;
    if (readRawCounts(c, st) && st == 0) { sum += c; got++; }
    delay(5);
  }

  if (got == 0) {
    Serial.println(F("# Tare failed - no valid samples. Keeping previous offset."));
    return;
  }

  zeroOffsetCounts = sum / got;
  Serial.print(F("# Zero offset counts: "));
  Serial.println(zeroOffsetCounts);
}

// ---------------------------------------------------------------------------
// Board reset
// ---------------------------------------------------------------------------
void resetBoard() {
  Serial.println(F("# Resetting board..."));
  Serial.flush();          // let the message leave before USB drops
  delay(100);
  NVIC_SystemReset();      // ARM Cortex-M software reset (SAMD21 on the Nano 33 IoT)
}

// ---------------------------------------------------------------------------
// Serial commands
// ---------------------------------------------------------------------------
void handleCommand(const char *cmd) {
  if (strcmp(cmd, "TARE") == 0 || strcmp(cmd, "r") == 0) {
    performTare();
    lastSampleMs = millis();   // tare takes ~200 ms; restart the schedule
  } else if (strcmp(cmd, "RESET") == 0) {
    resetBoard();
  } else if (cmd[0] != '\0') {
    Serial.print(F("# Unknown command: "));
    Serial.println(cmd);
  }
}

void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') continue;                  // accept both \n and \r\n

    if (c == '\n') {
      if (cmdOverflow) {
        Serial.println(F("# Command too long - ignored"));
      } else {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
      }
      cmdLen = 0;
      cmdOverflow = false;
    } else if (cmdLen < CMD_MAX_LEN) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = true;                     // discard until the newline
    }
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Wire.begin();

#ifdef WIRE_HAS_TIMEOUT
  Wire.setTimeOut(50);
#endif
  Wire.setClock(100000);

  Serial.println(F("# FMA Series I2C Force Sensor Reader (no filter)"));
  Serial.println(F("# fw=v2 (RESET and TARE commands)"));
  Serial.println(F("# Commands: TARE (or r), RESET - end each one with a newline."));
  Serial.print(F("# period_ms="));
  Serial.println(SAMPLE_PERIOD_MS);

  delay(50);
  performTare();

  lastSampleMs = millis();   // start the schedule after the tare
}

// ---------------------------------------------------------------------------
// Loop - fixed-rate schedule, one timestamped value per line
// ---------------------------------------------------------------------------
void loop() {
  pollSerialCommands();

  uint32_t now = millis();

  // Not time yet: return immediately and check again on the next loop.
  // Unsigned subtraction stays correct when millis() wraps after ~49.7 days.
  if (now - lastSampleMs < SAMPLE_PERIOD_MS) return;

  if (now - lastSampleMs > MAX_LAG_PERIODS * SAMPLE_PERIOD_MS) {
    lastSampleMs = now;                 // fell far behind: resynchronise
  } else {
    lastSampleMs += SAMPLE_PERIOD_MS;   // stay on the fixed 20 ms grid
  }

  uint16_t rawCounts;
  uint8_t  status;
  bool ok = readRawCounts(rawCounts, status);

  // Timestamp = the real millis() when this sample slot started (measured,
  // not the scheduled value, so any timing problem stays visible).
  Serial.print(now);
  Serial.print(',');

  if (!ok) {
    Serial.println(F("-2"));
    return;
  }

  if (status != 0) {
    Serial.println(F("-1"));
    if (++consecutiveFaults >= FAULT_WARN_THRESHOLD) {
      consecutiveFaults = 0;
    }
    return;
  }
  consecutiveFaults = 0;

  float force = (rawCounts - zeroOffsetCounts) / COUNTS_PER_NEWTON;

  if (fabsf(force) < ZERO_DEADBAND_N) force = 0.0f;

  Serial.println(force, 3);
}
