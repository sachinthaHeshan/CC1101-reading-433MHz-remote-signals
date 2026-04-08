/*
 * CC1101 Raw 433MHz Signal Capture — Arduino Uno
 * Captures raw pulse timings from garage remotes and other 433MHz devices.
 *
 * Wiring (Arduino Uno -> CC1101):
 *   Pin 13 (SCK)  -> CC1101 SCK
 *   Pin 11 (MOSI) -> CC1101 MOSI (SI)
 *   Pin 12 (MISO) -> CC1101 MISO (SO)
 *   Pin 10        -> CC1101 CSN  (CS)
 *   Pin  2        -> CC1101 GDO0  (INT0)
 *   3.3V          -> CC1101 VCC   *** NOT 5V! ***
 *   GND           -> CC1101 GND
 *
 * IMPORTANT: CC1101 is a 3.3V chip. The Uno SPI pins are 5V.
 *   Most CC1101 breakout boards tolerate 5V on SPI lines, but
 *   for safety use a level shifter or voltage divider on
 *   SCK, MOSI, and CSN lines (5V -> 3.3V).
 *   MISO (3.3V out) can connect directly to Uno pin 12.
 *   Power CC1101 from the Uno 3.3V pin.
 */

#include <SPI.h>

// ─── Pin Definitions ───────────────────────────────────────────
#define CC1101_CS    10
#define CC1101_GDO0  2   // Must be pin 2 (INT0) or 3 (INT1) on Uno

// ─── CC1101 Register Addresses ─────────────────────────────────
#define CC1101_IOCFG2   0x00
#define CC1101_IOCFG0   0x02
#define CC1101_FIFOTHR  0x03
#define CC1101_PKTCTRL0 0x08
#define CC1101_FSCTRL1  0x0B
#define CC1101_FREQ2    0x0D
#define CC1101_FREQ1    0x0E
#define CC1101_FREQ0    0x0F
#define CC1101_MDMCFG4  0x10
#define CC1101_MDMCFG3  0x11
#define CC1101_MDMCFG2  0x12
#define CC1101_MDMCFG1  0x13
#define CC1101_DEVIATN  0x15
#define CC1101_MCSM0    0x18
#define CC1101_FOCCFG   0x19
#define CC1101_AGCCTRL2 0x1B
#define CC1101_AGCCTRL1 0x1C
#define CC1101_AGCCTRL0 0x1D
#define CC1101_FREND1   0x21
#define CC1101_FSCAL3   0x23
#define CC1101_FSCAL2   0x24
#define CC1101_FSCAL1   0x25
#define CC1101_FSCAL0   0x26
#define CC1101_TEST2    0x2C
#define CC1101_TEST1    0x2D
#define CC1101_TEST0    0x2E

// ─── CC1101 Strobe Commands ────────────────────────────────────
#define CC1101_SRES    0x30
#define CC1101_SRX     0x34
#define CC1101_SIDLE   0x36
#define CC1101_SFRX    0x3A

// ─── Capture Settings ──────────────────────────────────────────
// Uno has only 2KB RAM — keep buffer small
#define MAX_PULSES       200
#define MIN_PULSE_US     80
#define MAX_PULSE_US     65000

// ─── Pulse Buffer ──────────────────────────────────────────────
volatile uint16_t pulseTimings[MAX_PULSES];  // uint16 to save RAM
volatile uint16_t pulseCount = 0;
volatile unsigned long lastEdgeTime = 0;
volatile bool capturing = false;
volatile bool captureReady = false;

// ─── ISR: Record pulse timings on each edge ────────────────────
void handleGDO0Interrupt() {
  unsigned long now = micros();

  if (!capturing) return;

  if (lastEdgeTime > 0) {
    unsigned long duration = now - lastEdgeTime;

    if (duration >= MIN_PULSE_US && duration <= MAX_PULSE_US) {
      if (pulseCount < MAX_PULSES) {
        // Clamp to uint16 max
        pulseTimings[pulseCount++] = (duration > 65535) ? 65535 : (uint16_t)duration;
      }
    }
  }

  lastEdgeTime = now;
}

// ─── SPI Communication ────────────────────────────────────────
void cc1101_writeReg(uint8_t addr, uint8_t value) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(12));  // Wait MISO low (chip ready)
  SPI.transfer(addr);
  SPI.transfer(value);
  digitalWrite(CC1101_CS, HIGH);
}

uint8_t cc1101_readReg(uint8_t addr) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(12));
  SPI.transfer(addr | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CC1101_CS, HIGH);
  return val;
}

void cc1101_strobe(uint8_t strobe) {
  digitalWrite(CC1101_CS, LOW);
  while (digitalRead(12));
  SPI.transfer(strobe);
  digitalWrite(CC1101_CS, HIGH);
}

// ─── CC1101 Reset ──────────────────────────────────────────────
void cc1101_reset() {
  digitalWrite(CC1101_CS, HIGH);
  delayMicroseconds(30);
  digitalWrite(CC1101_CS, LOW);
  delayMicroseconds(30);
  digitalWrite(CC1101_CS, HIGH);
  delayMicroseconds(45);

  cc1101_strobe(CC1101_SRES);
  delay(10);
}

// ─── Configure CC1101 for 433.92 MHz ASK/OOK RX ───────────────
void cc1101_initRaw433() {
  cc1101_reset();

  // GDO2 & GDO0: async serial data output
  cc1101_writeReg(CC1101_IOCFG2,   0x0D);
  cc1101_writeReg(CC1101_IOCFG0,   0x0D);

  // Packet handling OFF — raw/async mode
  cc1101_writeReg(CC1101_PKTCTRL0, 0x32);

  // Frequency: 433.92 MHz
  cc1101_writeReg(CC1101_FREQ2,    0x10);
  cc1101_writeReg(CC1101_FREQ1,    0xB0);
  cc1101_writeReg(CC1101_FREQ0,    0x71);

  // IF frequency
  cc1101_writeReg(CC1101_FSCTRL1,  0x06);

  // Modem: ASK/OOK, wide RX BW ~203 kHz, no sync
  cc1101_writeReg(CC1101_MDMCFG4,  0x87);
  cc1101_writeReg(CC1101_MDMCFG3,  0x32);
  cc1101_writeReg(CC1101_MDMCFG2,  0x30);
  cc1101_writeReg(CC1101_MDMCFG1,  0x22);

  cc1101_writeReg(CC1101_DEVIATN,  0x00);
  cc1101_writeReg(CC1101_MCSM0,    0x18);
  cc1101_writeReg(CC1101_FOCCFG,   0x16);

  // AGC tuned for OOK
  cc1101_writeReg(CC1101_AGCCTRL2, 0x04);
  cc1101_writeReg(CC1101_AGCCTRL1, 0x00);
  cc1101_writeReg(CC1101_AGCCTRL0, 0x92);

  cc1101_writeReg(CC1101_FREND1,   0x56);

  // Freq synth cal
  cc1101_writeReg(CC1101_FSCAL3,   0xE9);
  cc1101_writeReg(CC1101_FSCAL2,   0x2A);
  cc1101_writeReg(CC1101_FSCAL1,   0x00);
  cc1101_writeReg(CC1101_FSCAL0,   0x1F);

  // Test registers for ASK
  cc1101_writeReg(CC1101_TEST2,    0x81);
  cc1101_writeReg(CC1101_TEST1,    0x35);
  cc1101_writeReg(CC1101_TEST0,    0x09);

  // Enter RX
  cc1101_strobe(CC1101_SIDLE);
  cc1101_strobe(CC1101_SFRX);
  cc1101_strobe(CC1101_SRX);
}

// ─── Print captured pulses ─────────────────────────────────────
void printCapture() {
  Serial.println(F("\n=== SIGNAL CAPTURED ==="));
  Serial.print(F("Pulses: "));
  Serial.println(pulseCount);

  // Raw timings table
  Serial.println(F("\nIdx | us    | Level"));
  Serial.println(F("----|-------|------"));
  for (uint16_t i = 0; i < pulseCount; i++) {
    if (i < 10)  Serial.print(' ');
    if (i < 100) Serial.print(' ');
    Serial.print(i);
    Serial.print(F(" | "));
    if (pulseTimings[i] < 10000) Serial.print(' ');
    if (pulseTimings[i] < 1000)  Serial.print(' ');
    if (pulseTimings[i] < 100)   Serial.print(' ');
    Serial.print(pulseTimings[i]);
    Serial.print(F(" | "));
    Serial.println((i % 2 == 0) ? F("HIGH") : F("LOW"));
  }

  // Compact CSV format
  Serial.println(F("\n-- CSV (copy-paste) --"));
  for (uint16_t i = 0; i < pulseCount; i++) {
    Serial.print(pulseTimings[i]);
    if (i < pulseCount - 1) Serial.print(',');
    if ((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();

  // Basic stats
  uint16_t minP = 65535, maxP = 0;
  unsigned long total = 0;
  uint16_t shortC = 0, medC = 0, longC = 0;

  for (uint16_t i = 0; i < pulseCount; i++) {
    if (pulseTimings[i] < minP) minP = pulseTimings[i];
    if (pulseTimings[i] > maxP) maxP = pulseTimings[i];
    total += pulseTimings[i];
    if (pulseTimings[i] < 500)       shortC++;
    else if (pulseTimings[i] < 1500) medC++;
    else                              longC++;
  }

  Serial.println(F("\n-- Analysis --"));
  Serial.print(F("Min pulse : ")); Serial.print(minP); Serial.println(F(" us"));
  Serial.print(F("Max pulse : ")); Serial.print(maxP); Serial.println(F(" us"));
  Serial.print(F("Total     : ")); Serial.print(total / 1000); Serial.println(F(" ms"));
  Serial.print(F("Short <500: ")); Serial.println(shortC);
  Serial.print(F("Med 500-1500: ")); Serial.println(medC);
  Serial.print(F("Long >1500: ")); Serial.println(longC);

  // Protocol guess
  Serial.println(F("\n-- Protocol Hints --"));
  if (minP > 100 && minP < 500 && maxP > 800 && maxP < 2000) {
    float ratio = (float)maxP / minP;
    Serial.print(F("Ratio long/short: "));
    Serial.println(ratio, 1);
    if (ratio > 2.5 && ratio < 3.5) {
      Serial.println(F("-> Likely PT2262/EV1527 (3:1)"));
    } else if (ratio > 1.8 && ratio < 2.2) {
      Serial.println(F("-> Possible Manchester (2:1)"));
    }
  }

  if (pulseCount > 40 && pulseCount < 60) {
    Serial.println(F("-> ~24-bit code (PT2262/EV1527)"));
  } else if (pulseCount > 120 && pulseCount < 140) {
    Serial.println(F("-> ~66-bit (possible KeeLoq rolling)"));
  }

  Serial.println(F("==========================="));
}

// ─── Reset capture state ───────────────────────────────────────
void resetCapture() {
  pulseCount = 0;
  lastEdgeTime = 0;
  captureReady = false;

  cc1101_strobe(CC1101_SIDLE);
  cc1101_strobe(CC1101_SFRX);
  cc1101_strobe(CC1101_SRX);

  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleGDO0Interrupt, CHANGE);
  capturing = true;
}

// ─── Setup ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  Serial.println(F("\n=== CC1101 433MHz Raw Capture ==="));
  Serial.println(F("=== Arduino Uno Edition       ===\n"));

  // Init SPI
  pinMode(CC1101_CS, OUTPUT);
  digitalWrite(CC1101_CS, HIGH);
  SPI.begin();

  // Init GDO0
  pinMode(CC1101_GDO0, INPUT);

  // Init CC1101
  Serial.print(F("Init CC1101... "));
  cc1101_initRaw433();

  // Verify chip
  uint8_t partnum = cc1101_readReg(0xF0);
  uint8_t version = cc1101_readReg(0xF1);

  if (version == 0x00 || version == 0xFF) {
    Serial.println(F("FAILED!"));
    Serial.print(F("PARTNUM=0x")); Serial.print(partnum, HEX);
    Serial.print(F(" VERSION=0x")); Serial.println(version, HEX);
    Serial.println(F("Check wiring! VCC must be 3.3V."));
    while (1) delay(1000);
  }

  Serial.println(F("OK"));
  Serial.print(F("PARTNUM=0x")); Serial.print(partnum, HEX);
  Serial.print(F(" VERSION=0x")); Serial.println(version, HEX);
  Serial.println(F("\nListening... Press your remote!\n"));

  // Start capturing
  attachInterrupt(digitalPinToInterrupt(CC1101_GDO0), handleGDO0Interrupt, CHANGE);
  capturing = true;
}

// ─── Main Loop ─────────────────────────────────────────────────
void loop() {
  static unsigned long lastActivity = 0;
  static uint16_t lastPulseCount = 0;
  static bool signalDetected = false;

  // Detect new pulses arriving
  if (pulseCount > 0 && pulseCount != lastPulseCount) {
    lastActivity = millis();
    lastPulseCount = pulseCount;

    if (!signalDetected) {
      Serial.println(F("[>] Signal detected..."));
      signalDetected = true;
    }
  }

  // Signal complete when no new edges for 50ms
  if (signalDetected && (millis() - lastActivity > 50)) {
    capturing = false;
    detachInterrupt(digitalPinToInterrupt(CC1101_GDO0));

    if (pulseCount > 10) {
      printCapture();
    } else {
      Serial.println(F("[!] Noise — too few pulses."));
    }

    // Reset
    lastPulseCount = 0;
    signalDetected = false;
    delay(500);

    resetCapture();
    Serial.println(F("\nReady. Press remote again...\n"));
  }

  delay(1);
}
