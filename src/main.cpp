// =============================================================================
// Sensorized Robot Training Glove — ESP32-S3-N8R8 Firmware
// =============================================================================
// Hardware:
//   IMU:  Adafruit BNO085 at I2C address 0x4A (SDA=GPIO8, SCL=GPIO9)
//   Flex: 5x resistive flex sensors on ADC1 (GPIO1,2,4,5,6)
//
// Serial output at 115200 baud, two machine-readable lines per frame at 100ms:
//   Q:w,x,y,z   — quaternion for 3D orientation (parsed by visualizer/recorder)
//   F:t,ui,li,um,lm — normalized flex values 0.0(open)..1.0(closed)
// =============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// -----------------------------------------------------------------------------
// Flex sensor pin assignments (ADC1 only — ADC2 is reserved for WiFi on ESP32)
// GPIO3 is skipped: it's a strapping pin that causes boot issues if pulled low.
// -----------------------------------------------------------------------------
#define FLEX_THUMB        1   // ADC1 CH0 — thumb (currently broken sensor)
#define FLEX_UPPER_INDEX  2   // ADC1 CH1 — upper index finger segment
#define FLEX_LOWER_INDEX  4   // ADC1 CH3 — lower index finger segment
#define FLEX_UPPER_MIDDLE 5   // ADC1 CH4 — upper middle finger segment
#define FLEX_LOWER_MIDDLE 6   // ADC1 CH5 — lower middle finger segment

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
  const int FLEX_MIN[5] = {2800, 2800, 2800, 2800, 2800};  // flat/extended
  const int FLEX_MAX[5] = {3700, 3700, 3700, 3700, 3700};  // fully curled

// BNO085 driver and sensor event container
Adafruit_BNO08x  bno;
sh2_SensorValue_t sensorValue;

// Latest quaternion components (updated each time a rotation event arrives)
float   qw = 0, qx = 0, qy = 0, qz = 0;
uint8_t quatAccuracy = 0;  // 0=unreliable, 1=low, 2=medium, 3=high

// -----------------------------------------------------------------------------
// normalizeFlex — maps raw 12-bit ADC value to 0.0..1.0 finger curl
// idx selects the per-sensor calibration constants from FLEX_MIN/FLEX_MAX
// -----------------------------------------------------------------------------
float normalizeFlex(int raw, int idx) {
  float n = (float)(raw - FLEX_MIN[idx]) / (FLEX_MAX[idx] - FLEX_MIN[idx]);
  return constrain(n, 0.0f, 1.0f);
}

void setup() {
  // USB CDC serial — delay allows the port to enumerate before we send data
  Serial.begin(115200);
  delay(500);
  while (!Serial);

  // 12-bit ADC resolution gives 0–4095 range (vs default 10-bit 0–1023)
  analogReadResolution(12);

  // I2C bus — custom pins for this board layout
  Wire.begin(8, 9);  // SDA=GPIO8, SCL=GPIO9

  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not detected. Check wiring!");
    while (1);
  }

  // Enable Game Rotation Vector at 10ms intervals (100Hz).
  //
  // WHY Game Rotation Vector instead of Rotation Vector?
  //   - Rotation Vector fuses gyro + accel + magnetometer → gives absolute
  //     heading relative to magnetic north, but is sensitive to nearby metal,
  //     motors, and other magnetic interference common in robot environments.
  //   - Game Rotation Vector fuses only gyro + accel → gives stable relative
  //     orientation unaffected by magnetic disturbances. Heading drifts slowly
  //     over minutes, but for hand-pose capture (seconds to tens of seconds)
  //     this is negligible and the stability is far more useful.
  bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);  // 10000 µs = 10ms

  Serial.println("BNO085 + flex sensors ready.");
}

void loop() {
  // --------------------------------------------------------------------------
  // IMU: drain all pending sensor events from the BNO085 FIFO.
  // The BNO085 queues events internally; calling getSensorEvent() in a while
  // loop ensures we consume all of them and always use the freshest reading.
  // --------------------------------------------------------------------------
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

  // --------------------------------------------------------------------------
  // Flex sensors: single analogRead() per pin.
  // The ESP32-S3 ADC is not perfectly linear but is stable enough for gesture
  // classification after per-sensor normalization.
  // --------------------------------------------------------------------------
  int raw[5] = {
    analogRead(FLEX_THUMB),
    analogRead(FLEX_UPPER_INDEX),
    analogRead(FLEX_LOWER_INDEX),
    analogRead(FLEX_UPPER_MIDDLE),
    analogRead(FLEX_LOWER_MIDDLE),
  };

  float flex[5];
  for (int i = 0; i < 5; i++) flex[i] = normalizeFlex(raw[i], i);

  // --------------------------------------------------------------------------
  // Machine-readable output — parsed by visualize_imu.py and record_session.py
  // Q: quaternion (w,x,y,z) — drives 3D hand rotation in the visualizer
  // F: normalized flex (0=open, 1=closed) — drives finger curl animation
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

  // --------------------------------------------------------------------------
  // Human-readable output — for serial monitor debugging
  // Shows both raw ADC and normalized value side-by-side for calibration
  // --------------------------------------------------------------------------
  Serial.print("Quat   | W:"); Serial.print(qw, 3);
  Serial.print(" X:"); Serial.print(qx, 3);
  Serial.print(" Y:"); Serial.print(qy, 3);
  Serial.print(" Z:"); Serial.print(qz, 3);
  Serial.print("  Acc:"); Serial.println(quatAccuracy);

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

  Serial.println("---");
  delay(100);
}
