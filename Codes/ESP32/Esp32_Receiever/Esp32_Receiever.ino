#include <esp_now.h>
#include <WiFi.h>

typedef struct struct_message {
  int command;
} struct_message;

struct_message myData;

//Display 
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  
  Serial.print("Received Command: ");
  Serial.println(myData.command);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  //Register and Type Cast
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  Serial.println("Robot Receiver Ready...");
}

void loop() {

}