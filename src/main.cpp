#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

float qw = 0, qx = 0, qy = 0, qz = 0;
uint8_t quatAccuracy = 0;
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

void setReports() {
  bno.enableReport(SH2_ROTATION_VECTOR, 10000);
  bno.enableReport(SH2_ACCELEROMETER, 10000);
  bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  while (!Serial);

  Wire.begin(8, 9);

  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO085 not detected. Check wiring!");
    while (1);
  }

  setReports();
  Serial.println("BNO085 connected!");
}

void loop() {
  // Drain all pending sensor events
  while (bno.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        qw = sensorValue.un.rotationVector.real;
        qx = sensorValue.un.rotationVector.i;
        qy = sensorValue.un.rotationVector.j;
        qz = sensorValue.un.rotationVector.k;
        quatAccuracy = sensorValue.status & 0x03;
        break;
      case SH2_ACCELEROMETER:
        ax = sensorValue.un.accelerometer.x;
        ay = sensorValue.un.accelerometer.y;
        az = sensorValue.un.accelerometer.z;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        gx = sensorValue.un.gyroscope.x;
        gy = sensorValue.un.gyroscope.y;
        gz = sensorValue.un.gyroscope.z;
        break;
    }
  }

  // Machine-readable quaternion line for Python parser
  Serial.print("Q:");
  Serial.print(qw, 4); Serial.print(",");
  Serial.print(qx, 4); Serial.print(",");
  Serial.print(qy, 4); Serial.print(",");
  Serial.println(qz, 4);

  // Human-readable quaternion
  Serial.print("Quat   | W: "); Serial.print(qw, 4);
  Serial.print("  X: ");        Serial.print(qx, 4);
  Serial.print("  Y: ");        Serial.print(qy, 4);
  Serial.print("  Z: ");        Serial.print(qz, 4);
  Serial.print("  Accuracy: "); Serial.println(quatAccuracy);

  // Accelerometer (m/s²)
  Serial.print("Accel  | X: "); Serial.print(ax, 2);
  Serial.print("  Y: ");        Serial.print(ay, 2);
  Serial.print("  Z: ");        Serial.println(az, 2);

  // Gyroscope (rad/s)
  Serial.print("Gyro   | X: "); Serial.print(gx, 3);
  Serial.print("  Y: ");        Serial.print(gy, 3);
  Serial.print("  Z: ");        Serial.println(gz, 3);

  Serial.println("---");
  delay(10);
}
