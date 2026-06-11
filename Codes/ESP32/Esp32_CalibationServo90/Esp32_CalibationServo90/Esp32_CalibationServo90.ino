#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// เลือกบอร์ด pca2 (0x41) สำหรับเทสต์ MG996R
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40); 

const int testChannel = 0; // เสียบเซอร์โวที่จะเทสต์ไว้ที่ช่อง 0

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pca.begin();
  pca.setPWMFreq(50); // ความถี่ 50Hz เหมือนในหุ่นจริง
  
  Serial.println("=== Servo Calibration Tool ===");
  Serial.println("Type a number between 50 - 600 and press Enter.");
}

void loop() {
  if (Serial.available() > 0) {
    int pwmValue = Serial.parseInt(); // รับค่าตัวเลขจากคีย์บอร์ด
    
    if (pwmValue > 10) { 
      pca.setPWM(testChannel, 0, pwmValue); // สั่งมอเตอร์
      Serial.print("Sending PWM Step: ");
      Serial.println(pwmValue);
    }
  }
}