// =============================================================================
// Sensorized Robot Training Glove — ESP32-S3-N8R8 Firmware
// =============================================================================
// Hardware:
//   IMU:  Adafruit BNO085 at I2C address 0x4A (SDA=GPIO8, SCL=GPIO9)
//   Analog sensors: 5x flex sensors + 3x pressure/FSR sensors through CD4051BE
//         Z=GPIO4 ADC, A=GPIO5, B=GPIO6, C=GPIO7, channels Y0..Y7
//
// Serial output at 115200 baud, three machine-readable lines per frame at 100ms:
//   Q:w,x,y,z   — quaternion for 3D orientation (parsed by visualizer/recorder)
//   F:t,ui,li,um,lm — normalized flex values 0.0(open)..1.0(closed)
//   P:p1,p2,p3 — normalized pressure values 0.0(no pressure)..1.0(firm press)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// -----------------------------------------------------------------------------
// CD4051BE multiplexer pin assignments.
// The mux output Z feeds one ADC pin. Select lines A/B/C choose Y0..Y7.
// -----------------------------------------------------------------------------
#define MUX_ADC_PIN 4   // ADC1 CH3 — CD4051BE Z/common output
#define MUX_SEL_A   5   // CD4051BE A / select bit 0
#define MUX_SEL_B   6   // CD4051BE B / select bit 1
#define MUX_SEL_C   7   // CD4051BE C / select bit 2

// Flex sensor mux channel assignments, in serial output order:
//   F:t,ui,li,um,lm
// Y3 and Y4 are intentionally swapped here to match the glove wiring:
// upper middle is wired to Y4, lower middle is wired to Y3.
const uint8_t FLEX_MUX_CHANNEL[5] = {0, 1, 2, 4, 3};

// Pressure/FSR mux channel assignments, in serial output order:
//   P:p1,p2,p3
// Wiring diagram places pressure sensors on CD4051BE Y5, Y6, and Y7.
const uint8_t PRESSURE_MUX_CHANNEL[3] = {5, 6, 7};

// Discard the first ADC read after switching mux channels, then average a few
// samples so the ESP32 ADC input and CD4051 output have time to settle.
const int ADC_SAMPLES_PER_CHANNEL = 4;

// -----------------------------------------------------------------------------
// Flex sensor normalization constants
//
// Each flex sensor sits in a voltage divider:
//   3.3V ──[10kΩ]── ADC_PIN ──[flex sensor]── GND
//
// Flat (low resistance ~10-25kΩ) → lower ADC reading
// Bent (high resistance ~50-100kΩ) → higher ADC reading
//
// Tune FLEX_MIN (finger open) and FLEX_MAX (finger fully closed) per sensor
// by reading raw values in the serial monitor at both extremes.
// normalizeFlex() maps [MIN..MAX] → [0.0..1.0] and clamps outside values.
// -----------------------------------------------------------------------------
const int FLEX_MIN[5] = {2800, 3273, 2972, 3642, 3271};  // flat/extended
const int FLEX_MAX[5] = {3700, 3700, 3584, 3700, 3682};  // fully curled

// -----------------------------------------------------------------------------
// Pressure/FSR normalization constants
//
// Each pressure sensor sits in the same divider orientation:
//   3.3V ──[10kΩ]── ADC_PIN ──[FSR]── GND
//
// No pressure (very high FSR resistance) → higher ADC reading
// Firm pressure (lower FSR resistance)   → lower ADC reading
//
// The normalization is inverted relative to flex:
//   PRESSURE_IDLE  maps to 0.0
//   PRESSURE_PRESS maps to 1.0
//
// These are safe starting points. Tune with the serial monitor once the FSRs are
// mechanically mounted by noting raw values at no touch and firm intended press.
// -----------------------------------------------------------------------------
const int PRESSURE_IDLE[3]  = {4095, 4095, 4095};  // no pressure
const int PRESSURE_PRESS[3] = {1800, 1800, 1800};  // firm pressure

const uint32_t SERIAL_WAIT_TIMEOUT_MS = 3000;
const uint32_t IMU_RETRY_INTERVAL_MS  = 2000;

// BNO085 driver and sensor event container
Adafruit_BNO08x  bno;
sh2_SensorValue_t sensorValue;

// Latest quaternion components (updated each time a rotation event arrives)
float   qw = 1, qx = 0, qy = 0, qz = 0;
uint8_t quatAccuracy = 0;  // 0=unreliable, 1=low, 2=medium, 3=high
bool    imuOk = false;
uint8_t imuAddress = 0;
uint32_t lastImuRetryMs = 0;

const uint8_t BNO085_I2C_ADDRS[] = {0x4A, 0x4B};

// -----------------------------------------------------------------------------
// normalizeFlex — maps raw 12-bit ADC value to 0.0..1.0 finger curl
// idx selects the per-sensor calibration constants from FLEX_MIN/FLEX_MAX
// -----------------------------------------------------------------------------
float normalizeFlex(int raw, int idx) {
  float n = (float)(raw - FLEX_MIN[idx]) / (FLEX_MAX[idx] - FLEX_MIN[idx]);
  return constrain(n, 0.0f, 1.0f);
}

// -----------------------------------------------------------------------------
// normalizePressure — maps raw 12-bit ADC value to 0.0..1.0 pressure
// idx selects the per-sensor calibration constants from PRESSURE_IDLE/PRESS.
// This mapping is inverted because pressing an FSR lowers its divider voltage.
// -----------------------------------------------------------------------------
float normalizePressure(int raw, int idx) {
  float n = (float)(PRESSURE_IDLE[idx] - raw) /
            (PRESSURE_IDLE[idx] - PRESSURE_PRESS[idx]);
  return constrain(n, 0.0f, 1.0f);
}

void selectMuxChannel(uint8_t channel) {
  digitalWrite(MUX_SEL_A, (channel & 0x01) ? HIGH : LOW);
  digitalWrite(MUX_SEL_B, (channel & 0x02) ? HIGH : LOW);
  digitalWrite(MUX_SEL_C, (channel & 0x04) ? HIGH : LOW);
}

int readMuxAdc(uint8_t channel) {
  selectMuxChannel(channel);
  delayMicroseconds(100);

  analogRead(MUX_ADC_PIN);  // discard stale sample after mux switching

  int sum = 0;
  for (int i = 0; i < ADC_SAMPLES_PER_CHANNEL; i++) {
    delayMicroseconds(50);
    sum += analogRead(MUX_ADC_PIN);
  }
  return sum / ADC_SAMPLES_PER_CHANNEL;
}

void bootLog(const char *msg) {
  Serial.print("[boot] ");
  Serial.print(millis());
  Serial.print("ms  ");
  Serial.println(msg);
}

void logI2cScan() {
  bool foundAny = false;

  bootLog("scanning I2C bus");
  for (uint8_t addr = 1; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      foundAny = true;
      Serial.print("[i2c] found device at 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  if (!foundAny) {
    bootLog("no I2C devices found");
  }
}

bool initImuAtAddress(uint8_t address) {
  if (!bno.begin_I2C(address, &Wire)) {
    quatAccuracy = 0;
    qw = 1; qx = 0; qy = 0; qz = 0;
    return false;
  }

  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);  // 10000 µs = 10ms
  imuAddress = address;
  return true;
}

bool initImu() {
  imuAddress = 0;

  for (uint8_t i = 0; i < sizeof(BNO085_I2C_ADDRS); i++) {
    uint8_t address = BNO085_I2C_ADDRS[i];
    if (initImuAtAddress(address)) {
      return true;
    }
  }

  quatAccuracy = 0;
  qw = 1; qx = 0; qy = 0; qz = 0;
  return false;
}

void setup() {
  // USB CDC serial — wait briefly for a host, but do not block forever.
  Serial.begin(115200);
  delay(200);
  uint32_t serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart) < SERIAL_WAIT_TIMEOUT_MS) {
    delay(10);
  }
  delay(50);

  Serial.println();
  bootLog("setup start");
  if (Serial) {
    bootLog("USB CDC connected");
  } else {
    bootLog("USB CDC timeout - continuing anyway");
  }

  // 12-bit ADC resolution gives 0–4095 range (vs default 10-bit 0–1023)
  analogReadResolution(12);
  bootLog("ADC configured for 12-bit reads");

  pinMode(MUX_ADC_PIN, INPUT);
  pinMode(MUX_SEL_A, OUTPUT);
  pinMode(MUX_SEL_B, OUTPUT);
  pinMode(MUX_SEL_C, OUTPUT);
  selectMuxChannel(0);
  bootLog("mux select pins configured");

  // I2C bus — custom pins for this board layout
  Wire.begin(8, 9);  // SDA=GPIO8, SCL=GPIO9
  bootLog("I2C started on SDA=GPIO8 SCL=GPIO9");
  logI2cScan();

  imuOk = initImu();
  if (imuOk) {
    Serial.print("[boot] ");
    Serial.print(millis());
    Serial.print("ms  BNO085 detected at 0x");
    if (imuAddress < 0x10) Serial.print("0");
    Serial.println(imuAddress, HEX);
    bootLog("Game Rotation Vector enabled");
  } else {
    bootLog("BNO085 not detected at 0x4A or 0x4B - continuing without IMU");
  }
  bootLog("firmware ready");
}

void loop() {
  // If the IMU was absent at boot, keep retrying so the board still streams
  // flex/pressure data and recovers automatically once the IMU responds.
  if (!imuOk && (millis() - lastImuRetryMs) >= IMU_RETRY_INTERVAL_MS) {
    lastImuRetryMs = millis();
    bootLog("retrying BNO085 init");
    logI2cScan();
    imuOk = initImu();
    if (imuOk) {
      Serial.print("[boot] ");
      Serial.print(millis());
      Serial.print("ms  BNO085 recovered at 0x");
      if (imuAddress < 0x10) Serial.print("0");
      Serial.println(imuAddress, HEX);
      bootLog("Game Rotation Vector enabled");
    }
  }

  // --------------------------------------------------------------------------
  // IMU: drain all pending sensor events from the BNO085 FIFO.
  // The BNO085 queues events internally; calling getSensorEvent() in a while
  // loop ensures we consume all of them and always use the freshest reading.
  // --------------------------------------------------------------------------
  if (imuOk) {
    while (bno.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
        // Quaternion components: real (w) + imaginary vector (i,j,k = x,y,z)
        // Represents the rotation needed to go from the sensor's initial pose
        // to its current pose. The Python visualizer converts this to a 3x3
        // rotation matrix and applies it to the hand mesh vertices.
        qw = sensorValue.un.gameRotationVector.real;
        qx = sensorValue.un.gameRotationVector.i;
        qy = sensorValue.un.gameRotationVector.j;
        qz = sensorValue.un.gameRotationVector.k;
        quatAccuracy = sensorValue.status & 0x03;
      }
    }
  }

  // --------------------------------------------------------------------------
  // Flex sensors: select each CD4051BE channel and read the shared ADC pin.
  // The ESP32-S3 ADC is not perfectly linear but is stable enough for gesture
  // classification after per-sensor normalization.
  // --------------------------------------------------------------------------
  int raw[5];
  for (int i = 0; i < 5; i++) {
    raw[i] = readMuxAdc(FLEX_MUX_CHANNEL[i]);
  }

  float flex[5];
  for (int i = 0; i < 5; i++) flex[i] = normalizeFlex(raw[i], i);

  // --------------------------------------------------------------------------
  // Pressure sensors: remaining mux channels Y5-Y7.
  // With the current divider orientation, raw values decrease as pressure rises.
  // --------------------------------------------------------------------------
  int pressureRaw[3];
  for (int i = 0; i < 3; i++) {
    pressureRaw[i] = readMuxAdc(PRESSURE_MUX_CHANNEL[i]);
  }

  float pressure[3];
  for (int i = 0; i < 3; i++) {
    pressure[i] = normalizePressure(pressureRaw[i], i);
  }

  // --------------------------------------------------------------------------
  // Machine-readable output — parsed by visualize_imu.py and record_session.py
  // Q: quaternion (w,x,y,z) — drives 3D hand rotation in the visualizer
  // F: normalized flex (0=open, 1=closed) — drives finger curl animation
  // P: normalized pressure (0=no pressure, 1=firm press)
  // --------------------------------------------------------------------------
  Serial.print("Q:");
  Serial.print(qw, 4); Serial.print(",");
  Serial.print(qx, 4); Serial.print(",");
  Serial.print(qy, 4); Serial.print(",");
  Serial.println(qz, 4);

  Serial.print("F:");
  for (int i = 0; i < 5; i++) {
    Serial.print(flex[i], 3);
    if (i < 4) Serial.print(",");
  }
  Serial.println();

  Serial.print("P:");
  for (int i = 0; i < 3; i++) {
    Serial.print(pressure[i], 3);
    if (i < 2) Serial.print(",");
  }
  Serial.println();

  // --------------------------------------------------------------------------
  // Human-readable output — for serial monitor debugging
  // Shows both raw ADC and normalized value side-by-side for calibration
  // --------------------------------------------------------------------------
  Serial.print("Quat   | W:"); Serial.print(qw, 3);
  Serial.print(" X:"); Serial.print(qx, 3);
  Serial.print(" Y:"); Serial.print(qy, 3);
  Serial.print(" Z:"); Serial.print(qz, 3);
  Serial.print("  Acc:"); Serial.println(quatAccuracy);
  if (!imuOk) {
    Serial.println("IMU    | not detected - retrying init in background");
  }

  Serial.print("Flex   | T:");  Serial.print(raw[0]);
  Serial.print("("); Serial.print(flex[0], 2); Serial.print(")");
  Serial.print("  UI:"); Serial.print(raw[1]);
  Serial.print("("); Serial.print(flex[1], 2); Serial.print(")");
  Serial.print("  LI:"); Serial.print(raw[2]);
  Serial.print("("); Serial.print(flex[2], 2); Serial.print(")");
  Serial.print("  UM:"); Serial.print(raw[3]);
  Serial.print("("); Serial.print(flex[3], 2); Serial.print(")");
  Serial.print("  LM:"); Serial.print(raw[4]);
  Serial.print("("); Serial.print(flex[4], 2); Serial.println(")");

  Serial.print("Press  | P1:"); Serial.print(pressureRaw[0]);
  Serial.print("("); Serial.print(pressure[0], 2); Serial.print(")");
  Serial.print("  P2:"); Serial.print(pressureRaw[1]);
  Serial.print("("); Serial.print(pressure[1], 2); Serial.print(")");
  Serial.print("  P3:"); Serial.print(pressureRaw[2]);
  Serial.print("("); Serial.print(pressure[2], 2); Serial.println(")");

  Serial.println("---");
  delay(100);
}
