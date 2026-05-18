#include <esp_now.h>

#include <WiFi.h>



//StopRightThere

int lastCommandSent = -1;



//MAC Address

uint8_t broadcastAddress[] = {0xA4, 0xF0, 0x0F, 0x6F, 0x8C, 0xC8};



#define PIN_UP    25

#define PIN_LEFT  26

#define PIN_RIGHT 27

#define PIN_X     32

#define PIN_O     33



typedef struct struct_message {

  int command;

} struct_message;



struct_message myData;

esp_now_peer_info_t peerInfo;



// XandO 1 tap

bool lastStateX = false;

bool lastStateO = false;





void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

}



void setup() {

  Serial.begin(115200);





  pinMode(PIN_UP, INPUT);

  pinMode(PIN_LEFT, INPUT);

  pinMode(PIN_RIGHT, INPUT);

  pinMode(PIN_X, INPUT);

  pinMode(PIN_O, INPUT);



  WiFi.mode(WIFI_STA);



  if (esp_now_init() != ESP_OK) {

    Serial.println("Error initializing ESP-NOW");

    return;

  }





  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);



  memcpy(peerInfo.peer_addr, broadcastAddress, 6);

  peerInfo.channel = 0;  

  peerInfo.encrypt = false;



  if (esp_now_add_peer(&peerInfo) != ESP_OK){

    Serial.println("Failed to add peer");

    return;

  }

}



void loop() {



  bool currentStateUp = digitalRead(PIN_UP) == HIGH;

  bool currentStateLeft = digitalRead(PIN_LEFT) == HIGH;

  bool currentStateRight = digitalRead(PIN_RIGHT) == HIGH;

  bool currentStateX = digitalRead(PIN_X) == HIGH;

  bool currentStateO = digitalRead(PIN_O) == HIGH;



  int commandToSend = 0; //Default start

  bool triggerSingleShot = false;



  if (currentStateX == true && lastStateX == false) {

    commandToSend = 6;

    triggerSingleShot = true;

  }

  else if (currentStateO == true && lastStateO == false) {

    commandToSend = 7;

    triggerSingleShot = true;

  }



  if (triggerSingleShot == false) {

    if (currentStateUp) {

      commandToSend = 1;

    } else if (currentStateLeft) {

      commandToSend = 2;

    } else if (currentStateRight) {

      commandToSend = 3;

    }

  }



  lastStateX = currentStateX;

  lastStateO = currentStateO;

 

  if (commandToSend != 0 || lastCommandSent != 0) {

    myData.command = commandToSend;

    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

   

    Serial.print("Sending Command: ");

    Serial.println(commandToSend);



    lastCommandSent = commandToSend;

  }





  delay(50);

}