#include <esp_now.h>
#include <WiFi.h>

#define RXD2 16 // RX - GPIO16 Rx0
#define TXD2 17 // TX  GPIO17 Tx0

typedef struct struct_message {
  int command;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  Serial.print("Received: ");
  Serial.println(myData.command);


  Serial2.println(myData.command); 
}

void setup() {
  Serial.begin(115200); 
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); 

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  Serial.println("Robot Receiver Ready...");
}

void loop() {
}