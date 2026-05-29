#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // 0x28 confirmed

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("❌ ไม่พบ BNO055!");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("Ready");
  Serial.println("Yaw | Pitch(+90) | Roll | Gx | Gy | Gz");
}

void loop() {
  sensors_event_t orientEvent;
  bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);
  float yaw   = orientEvent.orientation.x;
  float pitch = orientEvent.orientation.y + 90.0;
  float roll  = orientEvent.orientation.z;

  sensors_event_t gyroEvent;
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.printf("Yaw: %6.1f°  Pitch: %6.1f°  Roll: %6.1f°  ||  Gx: %5.2f  Gy: %5.2f  Gz: %5.2f °/s\n",
                yaw, pitch, roll,
                gyroEvent.gyro.x, gyroEvent.gyro.y, gyroEvent.gyro.z);
  delay(100);
}