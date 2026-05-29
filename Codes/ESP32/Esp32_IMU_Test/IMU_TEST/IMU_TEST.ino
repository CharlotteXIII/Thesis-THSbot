#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("❌ ไม่พบ BNO055!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Serial.println("✅ พร้อมใช้งาน");
}

void loop() {
  sensors_event_t orientEvent;
  bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);
  float roll = orientEvent.orientation.z;

  int zone = -1; // ค่า default ถ้าไม่อยู่ใน zone ไหนเลย

  if      (roll > -45  && roll <= 45)   zone = 0; // วางหงาย
  else if (roll > 45   && roll <= 135)  zone = 1; // ก้มหน้า ~90°
  else if (roll > 135  || roll < -135)  zone = 2; // คว่ำหน้า ~180°
  else if (roll >= -135 && roll <= -45) zone = 3; // หงายหลัง ~-90°

  Serial.println(zone);
  delay(100);
}