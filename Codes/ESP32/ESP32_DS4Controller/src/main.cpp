#include <Arduino.h>
#include <WiFi.h> // ใช้ไลบรารี WiFi เพื่อดึงค่า MAC Address ของชิป Bluetooth/WiFi

void setup() {
  Serial.begin(115200); // เปิดการสื่อสารผ่านสาย USB
  
  // โค้ดสำหรับดึงที่อยู่ MAC Address มาแสดงผล
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(" ");
  Serial.print("ESP32 MAC Address is: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
}