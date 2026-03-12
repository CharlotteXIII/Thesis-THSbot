////////////////////////// FindESP32MAC_Address /////////////////////////////////////////
// #include <Arduino.h>
// #include <WiFi.h> // ใช้ไลบรารี WiFi เพื่อดึงค่า MAC Address ของชิป Bluetooth/WiFi

// void setup() {
//   Serial.begin(115200); // เปิดการสื่อสารผ่านสาย USB
  
//   // โค้ดสำหรับดึงที่อยู่ MAC Address มาแสดงผล
//   WiFi.mode(WIFI_MODE_STA);
//   Serial.println(" ");
//   Serial.print("ESP32 MAC Address is: ");
//   Serial.println(WiFi.macAddress());
// }

// void loop() {
// }

/////////////////////////////////////////////////////////////////////////////////////////////

#include <Arduino.h>
#include <Bluepad32.h>

ControllerPtr myController = nullptr;

// ฟังก์ชันเมื่อจอยเชื่อมต่อสำเร็จ
void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Serial.println("SUCCESS: DualShock 4 Connected!");
        myController = ctl;
        // สั่งให้ไฟจอยสติ๊กเป็นสีน้ำเงิน
        myController->setColorLED(0, 0, 255); 
    }
}

// ฟังก์ชันเมื่อจอยหลุดการเชื่อมต่อ
void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.println("WARNING: DualShock 4 Disconnected!");
        myController = nullptr;
    }
}

// ฟังก์ชันตรวจสอบการกดปุ่ม
void processControllers() {
    // เช็คว่ามีจอยเชื่อมต่ออยู่จริงๆ ก่อนทำงาน
    if (myController && myController->isConnected()) {
        
        // ดึงสถานะปัจจุบันของปุ่ม D-Pad ออกมา
        uint8_t dpad = myController->dpad();

        // --- 1. เช็คปุ่ม D-Pad (ทิศทาง) ---
        // ใช้การเทียบ Bitmask ตามมาตรฐาน Bluepad32 v3.1.0
        if (dpad & 0x01) {
            Serial.println("Sent: F"); // เดินหน้า (Up)
        } else if (dpad & 0x08) {
            Serial.println("Sent: L"); // ซ้าย (Left)
        } else if (dpad & 0x04) {
            Serial.println("Sent: R"); // ขวา (Right)
        }

        // --- 2. เช็คปุ่ม Action (แปลงร่าง) ---
        if (myController->b()) { // ปุ่ม O (Circle)
            Serial.println("Sent: C"); // แปลงร่างเป็น Spherical
        }
        if (myController->a()) { // ปุ่ม X (Cross)
            Serial.println("Sent: H"); // แปลงร่างเป็น Humanoid
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    // เริ่มต้นระบบ Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    
    // ลบความจำ Bluetooth เก่าๆ ทิ้ง เพื่อบังคับให้เริ่มจับคู่ใหม่
    BP32.forgetBluetoothKeys(); 

    Serial.println("ESP32 Ready!");
    Serial.println("PRESS and HOLD [SHARE] + [PS] button to Pair...");
}

void loop() {
    // อัปเดตข้อมูลจากจอยสติ๊ก (เรียกใช้ตรงๆ โดยไม่ต้องรอรับค่า bool)
    BP32.update();
    
    // เรียกฟังก์ชันตรวจสอบและปริ้นท์ปุ่ม
    processControllers();
    
    delay(50); // หน่วงเวลาเล็กน้อย
}