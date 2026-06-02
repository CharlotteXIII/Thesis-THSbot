// ================================================================
//  Core 0 : Receive ESP-NOW put into Queue
//  Core 1 : Receive Queue and command servo through PCA9685 (X2)
//
//  PCA9685#1 (0x40) - MG90s  ch 0-11
//  PCA9685#2 (0x41) - MG996R ch 0-9
//
//  Commands:
//    1 = UP    (Press and hold)
//    2 = LEFT  (Press and hold)
//    3 = RIGHT (Press and hold)
//    6 = X     (Press once)
//    7 = O     (Press once)
// ================================================================
//  BNO055: Detect Rolling Angles
// ================================================================
//  CharlotteXIII

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40); //MG90S
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41); //MG996R
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define SERVO_MIN  150
#define SERVO_MAX  600
#define PWM_FREQ   50

QueueHandle_t commandQueue; //Create Queue
volatile int stateLock = 0;
volatile int zone = -1;

float cur_pca1[12] = {0.0f};
float cur_pca2[10] = {0.0f};

//Create a box that stored int command from Controller
typedef struct struct_message {
  int command;
} struct_message;

//=============================================
// Convert Angle to PWM
void pca1_SetAngle(uint8_t ch, float angle) {
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  uint16_t pulse = SERVO_MIN + (uint16_t)((angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
  pca1.setPWM(ch, 0, pulse);
  cur_pca1[ch] = angle;
}

void pca2_SetAngle(uint8_t ch, float angle) {
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  uint16_t pulse = SERVO_MIN + (uint16_t)((angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
  pca2.setPWM(ch, 0, pulse);
  cur_pca2[ch] = angle;
}
//=============================================

//===========Smooth Sweep======================
void smoothSweepPair(Adafruit_PWMServoDriver &pca, uint8_t ch1, uint8_t ch2, float startAngle, float targetAngle, int stepDelay_ms) {
  if (startAngle < targetAngle) {
    for (float a = startAngle; a <= targetAngle; a += 1.0f) {
      if (ch1 != 255) { if(&pca == &pca1) pca1_SetAngle(ch1, a); else pca2_SetAngle(ch1, a); }
      if (ch2 != 255) { if(&pca == &pca1) pca1_SetAngle(ch2, a); else pca2_SetAngle(ch2, a); }
      vTaskDelay(pdMS_TO_TICKS(stepDelay_ms));
    }
  } else {
    for (float a = startAngle; a >= targetAngle; a -= 1.0f) {
      if (ch1 != 255) { if(&pca == &pca1) pca1_SetAngle(ch1, a); else pca2_SetAngle(ch1, a); }
      if (ch2 != 255) { if(&pca == &pca1) pca1_SetAngle(ch2, a); else pca2_SetAngle(ch2, a); }
      vTaskDelay(pdMS_TO_TICKS(stepDelay_ms));
    }
  }
}
//=============================================

//===============ACTIONs=======================

void action_default() {
  //Legs
  smoothSweepPair(pca2, 4, 255, cur_pca2[4], 90.0f, 10);
  smoothSweepPair(pca2, 5, 255, cur_pca2[5], 90.0f, 10);
  smoothSweepPair(pca2, 6, 255, cur_pca2[6], 90.0f, 10);
  smoothSweepPair(pca2, 7, 255, cur_pca2[7], 90.0f, 10);
  smoothSweepPair(pca2, 8, 255, cur_pca2[8], 90.0f, 10);
  smoothSweepPair(pca2, 9, 255, cur_pca2[9], 90.0f, 10);

  //Arms
  smoothSweepPair(pca2, 0, 255, cur_pca2[0], 150.0f, 10);
  smoothSweepPair(pca2, 2, 255, cur_pca2[2], 60.0f, 10);
  smoothSweepPair(pca2, 1, 255, cur_pca2[1]0.0f, 120.0f, 10);
  smoothSweepPair(pca2, 3, 255, cur_pca2[3], 30.0f, 15);

  //Shells
  for (float a = 0.0f; a <= 90.0f; a += 2.0f) {
    for(int i=0; i<12; i++) {
      if (a >= cur_pca1[i]) pca1_SetAngle(i, a);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void action_up() {
  if (stateLock == 1 && zone == 0)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 1)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 2)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 3)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
}

void action_left() {
  if (stateLock == 1 && zone == 0)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 1)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 2)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 3)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
}

void action_right() {
  if (stateLock == 1 && zone == 0)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 1)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 2)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
  else if (stateLock == 1 && zone == 3)
  {
    // [Shells] //////////////////////////////
    pca1_SetAngle( 0, 90.0f);
    pca1_SetAngle( 1, 90.0f);
  }
}

void action_X() {
  // Not Create stand up sequence yet
  if (stateLock == 1){
  // [LEGS]
    smoothSweepPair(pca2, 6, 9, 0.0f, 90.0f, 10);
    smoothSweepPair(pca2, 5, 8, 0.0f, 90.0f, 10);
    smoothSweepPair(pca2, 4, 7, 0.0f, 90.0f, 10);
  // [ARMS]
    smoothSweepPair(pca2, 0, 2, 0.0f, 30.0f, 10);
    smoothSweepPair(pca2, 1, 3, 0.0f, 30.0f, 10);

  // [Shells] //////////////////////////////
    for (float a = 0.0f; a <= 90.0f; a += 2.0f) {
      for(int i=0; i<12; i++) { pca1_SetAngle(i, a); }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    stateLock = 0;
  }
}

void action_O() {
  if (stateLock == 0)
  {  
    stateLock = 1; 
    // [LEGS] //////////////////////////////
    smoothSweepPair(pca2, 4, 255, 90.0f, 0.0f, 15);
    smoothSweepPair(pca2, 7, 255, 90.0f, 180.0f, 15);
    vTaskDelay(pdMS_TO_TICKS(300));

    smoothSweepPair(pca2, 5, 255, 90.0f, 30.0f, 15);
    smoothSweepPair(pca2, 4, 255, 90.0f, 120.0f, 15);
    vTaskDelay(pdMS_TO_TICKS(500));

    smoothSweepPair(pca2, 6, 255, 90.0f, 60.0f, 15);
    smoothSweepPair(pca2, 9, 255, 90.0f, 150.0f, 15);
    vTaskDelay(pdMS_TO_TICKS(300));

    // [ARMS] //////////////////////////////
    smoothSweepPair(pca2, 0, 255, 90.0f, 180.0f, 15);
    smoothSweepPair(pca2, 2, 255, 90.0f, 0.0f, 15);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    smoothSweepPair(pca2, 1, 255, 90.0f, 150.0f, 15);
    smoothSweepPair(pca2, 3, 255, 90.0f, 60.0f, 15);
    vTaskDelay(pdMS_TO_TICKS(500));

    // [Shells] //////////////////////////////
    for (float a = 90.0f; a >= 0.0f; a -= 2.0f) {
      for(int i=0; i<12; i++) { pca1_SetAngle(i, a); }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}
//=============================================

//===============BNO055=======================
void bnoTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(500));
  for (;;) {
    sensors_event_t orientEvent;
    bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);
    float roll = orientEvent.orientation.z;

    if      (roll > -30  && roll <= 30)   zone = 0;
    else if (roll > 30   && roll <= 135)  zone = 1;
    else if (roll > 135  || roll < -135)  zone = 2;
    else if (roll >= -135 && roll <= -30) zone = 3;

    // Serial.printf("[BNO] Roll: %.1f → Zone: %d\n", roll, zone);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ================================================================
// CORE 0 — ESP-NOW callback
// ================================================================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  struct_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(commandQueue, &msg.command, &xHigherPriorityTaskWoken);
}

// ================================================================
// CORE 1 — Servo Task
// ================================================================
void servoTask(void *pvParameters) {
  int cmd;
  vTaskDelay(pdMS_TO_TICKS(200));
  action_default();

  for (;;) {
    if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      
      switch (cmd) {
        case 1: action_up();    break;
        case 2: action_left();  break;
        case 3: action_right(); break;
        case 6: action_X();     break;
        case 7: action_O();     break;
        default: break;
      }
    }
  }
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  pca1.begin(); pca1.setPWMFreq(PWM_FREQ);
  pca2.begin(); pca2.setPWMFreq(PWM_FREQ);
  
  if (!bno.begin()) {
    Serial.println("BNO055 not found");
    while (1);
  }
  bno.setExtCrystalUse(true);

  commandQueue = xQueueCreate(5, sizeof(int));
  if (commandQueue == NULL) {
    Serial.println("Queue create failed");
    while (1);
  }

  xTaskCreatePinnedToCore(servoTask, "ServoTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bnoTask,   "BNOTask",   4096, NULL, 1, NULL, 0);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
  Serial.println("Receiver ready.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
}