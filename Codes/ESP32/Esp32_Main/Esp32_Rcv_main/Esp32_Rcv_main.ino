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

#define PCA1_MIN  100
#define PCA1_MAX  570

#define PCA2_MIN  160
#define PCA2_MAX  565

#define PWM_FREQ   50

QueueHandle_t commandQueue; //Create Queue
volatile int stateLock = 0;
volatile int zone = -1;

// float cur_pca1[12] = {0.0f};
float cur_pca2[10] = {0.0f};

//Create a box that stored int command from Controller
typedef struct struct_message {
  int command;
} struct_message;

//=============================================
// Convert Angle to PWM
// void pca1_SetAngle(uint8_t ch, float angle) {
//   if (angle < 0.0f)   angle = 0.0f;
//   if (angle > 180.0f) angle = 180.0f;
//   uint16_t pulse = PCA1_MIN + (uint16_t)((angle / 180.0f) * (PCA1_MAX - PCA1_MIN));
//   pca1.setPWM(ch, 0, pulse);
//   cur_pca1[ch] = angle;
// }

void pca2_SetAngle(uint8_t ch, float angle) {
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  uint16_t pulse = PCA2_MIN + (uint16_t)((angle / 180.0f) * (PCA2_MAX - PCA2_MIN));
  pca2.setPWM(ch, 0, pulse);
  cur_pca2[ch] = angle;
}
//=============================================

//===========Smooth Sweep======================
// void smoothSweepPair(Adafruit_PWMServoDriver &pca, uint8_t ch1, uint8_t ch2, float startAngle, float targetAngle, int stepDelay_ms) {
//   if (startAngle < targetAngle) {
//     for (float a = startAngle; a <= targetAngle; a += 1.0f) {
//       if (ch1 != 255) { if(&pca == &pca1) pca1_SetAngle(ch1, a); else pca2_SetAngle(ch1, a); }
//       if (ch2 != 255) { if(&pca == &pca1) pca1_SetAngle(ch2, a); else pca2_SetAngle(ch2, a); }
//       vTaskDelay(pdMS_TO_TICKS(stepDelay_ms));
//     }
//   } else {
//     for (float a = startAngle; a >= targetAngle; a -= 1.0f) {
//       if (ch1 != 255) { if(&pca == &pca1) pca1_SetAngle(ch1, a); else pca2_SetAngle(ch1, a); }
//       if (ch2 != 255) { if(&pca == &pca1) pca1_SetAngle(ch2, a); else pca2_SetAngle(ch2, a); }
//       vTaskDelay(pdMS_TO_TICKS(stepDelay_ms));
//     }
//   }
// }
//=============================================

//===========Smooth Sweep Dual======================
void smoothSweepDual(Adafruit_PWMServoDriver &pca,
                     uint8_t chA, float startA, float targetA,
                     uint8_t chB, float startB, float targetB,
                     int stepDelay_ms) {
  float a = startA;
  float b = startB;

  // เปลี่ยนจาก != เป็น fabs เพื่อป้องกัน float ไม่เท่ากันพอดี
  while (fabs(a - targetA) > 0.5f || fabs(b - targetB) > 0.5f) {
    if (a > targetA + 0.5f)      a -= 1.0f;
    else if (a < targetA - 0.5f) a += 1.0f;
    else                          a = targetA;  // snap ถึงที่หมาย

    if (b > targetB + 0.5f)      b -= 1.0f;
    else if (b < targetB - 0.5f) b += 1.0f;
    else                          b = targetB;

    // if (&pca == &pca1) { pca1_SetAngle(chA, a); pca1_SetAngle(chB, b); }
    // else               { pca2_SetAngle(chA, a); pca2_SetAngle(chB, b); }
    pca2_SetAngle(chA, a); pca2_SetAngle(chB, b); //delete, use in 'else'
    vTaskDelay(pdMS_TO_TICKS(stepDelay_ms));
  }
}
//=============================================

//===============ACTIONs=======================

void action_default() {
  vTaskDelay(pdMS_TO_TICKS(1000));
  // Legs

  smoothSweepDual(pca2, 6, cur_pca2[6], 90.0f,9, cur_pca2[9], 90.0f, 10);
  vTaskDelay(pdMS_TO_TICKS(300));
  // smoothSweepDual(pca1, 9, cur_pca1[9], 120.0f,11, cur_pca1[11], 30.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));

  smoothSweepDual(pca2, 5, cur_pca2[5], 90.0f,8, cur_pca2[8], 90.0f, 10);
  vTaskDelay(pdMS_TO_TICKS(300));
  // smoothSweepDual(pca1, 8, cur_pca1[8], 120.0f,10, cur_pca1[10], 30.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));

  smoothSweepDual(pca2, 4, cur_pca2[4], 90.0f,7, cur_pca2[7], 90.0f, 10);
  vTaskDelay(pdMS_TO_TICKS(300));

  //Arms
  smoothSweepDual(pca2, 0, cur_pca2[0], 120.0f,2, cur_pca2[2], 60.0f, 10);
  vTaskDelay(pdMS_TO_TICKS(300));
  // smoothSweepDual(pca1, 0, cur_pca1[0], 180.0f,2, cur_pca1[2], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  
  smoothSweepDual(pca2, 1, cur_pca2[1], 150.0f,3, cur_pca2[3], 30.0f, 10);
  vTaskDelay(pdMS_TO_TICKS(300));
  // smoothSweepDual(pca1, 1, cur_pca1[1], 150.0f,3, cur_pca1[3], 30.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));

  //Shells
  
//   smoothSweepDual(pca1, 4, cur_pca1[4], 90.0f,5, cur_pca1[5], 90.0f, 10);
//   vTaskDelay(pdMS_TO_TICKS(30));
//   smoothSweepDual(pca1, 6, cur_pca1[6], 90.0f,7, cur_pca1[7], 90.0f, 10);
//   vTaskDelay(pdMS_TO_TICKS(30));
}

void action_up() {
  // if (stateLock == 1) {
  //   check_BNO_Zone(); 

  //   if (zone == 0) {
  //     smoothSweepDual(pca1, 8, cur_pca1[8], 150.0f,10, cur_pca1[10], 30.0f, 10);
  //     vTaskDelay(pdMS_TO_TICKS(30));
  //   }
  //   else if (zone == 1) {
  //     smoothSweepDual(pca1, 9, cur_pca1[9], 150.0f,11, cur_pca1[11], 30.0f, 10);
  //     vTaskDelay(pdMS_TO_TICKS(100));
  //     smoothSweepDual(pca1, 1, cur_pca1[1], 150.0f,3, cur_pca1[3], 30.0f, 10);
  //     vTaskDelay(pdMS_TO_TICKS(30));
  //   }
  //   else if (zone == 2) {
  //     smoothSweepDual(pca1, 0, cur_pca1[0], 150.0f,2, cur_pca1[2], 30.0f, 10);
  //     vTaskDelay(pdMS_TO_TICKS(100));
  //     smoothSweepDual(pca1, 4, cur_pca1[4], 150.0f,5, cur_pca1[5], 30.0f, 10);
  //     vTaskDelay(pdMS_TO_TICKS(30));
  //   }
  //   else if (zone == 3) {
  //     smoothSweepDual(pca1, 6, cur_pca1[6], 150.0f,7, cur_pca1[7], 30.0f, 10);
  //     vTaskDelay(pdMS_TO_TICKS(100));
  //   } 
  // }
}

void action_left() {
  // if (stateLock == 1 && zone == 0)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // else if (stateLock == 1 && zone == 1)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // else if (stateLock == 1 && zone == 2)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // else if (stateLock == 1 && zone == 3)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // smoothSweepDual(pca1, 0, cur_pca1[0], 90.0f,2, cur_pca1[2], 90.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 1, cur_pca1[1], 90.0f,3, cur_pca1[3], 90.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 4, cur_pca1[4], 90.0f,5, cur_pca1[5], 90.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 6, cur_pca1[6], 90.0f,7, cur_pca1[7], 90.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 8, cur_pca1[8], 90.0f,10, cur_pca1[10], 90.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 9, cur_pca1[9], 90.0f,11, cur_pca1[11], 90.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  
    // smoothSweepDual(pca1, 0, cur_pca1[0], 90.0f,2, cur_pca1[2], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 1, cur_pca1[1], 90.0f,3, cur_pca1[3], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 4, cur_pca1[4], 90.0f,5, cur_pca1[5], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 6, cur_pca1[6], 90.0f,7, cur_pca1[7], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 8, cur_pca1[8], 90.0f,10, cur_pca1[10], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 9, cur_pca1[9], 90.0f,11, cur_pca1[11], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));

    // smoothSweepPair(pca1, 2, 255, cur_pca2[2], 90.0f, 20);
}

void action_right() {
  // if (stateLock == 1 && zone == 0)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // else if (stateLock == 1 && zone == 1)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // else if (stateLock == 1 && zone == 2)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // else if (stateLock == 1 && zone == 3)
  // {
  //   // [Shells] //////////////////////////////
  //   pca1_SetAngle( 0, 90.0f);
  //   pca1_SetAngle( 1, 90.0f);
  // }
  // smoothSweepDual(pca1, 0, cur_pca1[0], 180.0f,2, cur_pca1[2], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 1, cur_pca1[1], 180.0f,3, cur_pca1[3], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 4, cur_pca1[4], 180.0f,5, cur_pca1[5], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 6, cur_pca1[6], 180.0f,7, cur_pca1[7], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 8, cur_pca1[8], 180.0f,10, cur_pca1[10], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
  // smoothSweepDual(pca1, 9, cur_pca1[9], 180.0f,11, cur_pca1[11], 0.0f, 10);
  // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 0, cur_pca1[0], 180.0f,2, cur_pca1[2], 0.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 1, cur_pca1[1], 180.0f,3, cur_pca1[3], 0.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 4, cur_pca1[4], 180.0f,5, cur_pca1[5], 0.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 6, cur_pca1[6], 180.0f,7, cur_pca1[7], 0.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 8, cur_pca1[8], 180.0f,10, cur_pca1[10], 0.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 9, cur_pca1[9], 180.0f,11, cur_pca1[11], 0.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
}

void action_X() {
  // Not Create stand up sequence yet
  if (stateLock == 1){
    Serial.println("[ACTION X] Abandon Transformation.");
    stateLock = 0;
    // action_default();
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Legs

    smoothSweepDual(pca2, 0, cur_pca2[0], 120.0f,2, cur_pca2[2], 60.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));

    smoothSweepDual(pca2, 4, cur_pca2[4], 90.0f,7, cur_pca2[7], 90.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));

    smoothSweepDual(pca2, 5, cur_pca2[5], 90.0f,8, cur_pca2[8], 90.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));

    smoothSweepDual(pca2, 6, cur_pca2[6], 90.0f,9, cur_pca2[9], 90.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    smoothSweepDual(pca2, 1, cur_pca2[1], 150.0f,3, cur_pca2[3], 30.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));
    
  }
}

void action_O() {
  if (stateLock == 0)
  {  
    Serial.println("[ACTION O] Start Transformation.");

    // smoothSweepDual(pca2, 0, cur_pca2[0], 90.0f,2, cur_pca2[2], 90.0f, 10);
    pca2_SetAngle(0,90.0f);
    vTaskDelay(pdMS_TO_TICKS(500));
    // smoothSweepDual(pca2, 1, cur_pca2[1], 90.0f,3, cur_pca2[3], 90.0f, 10);
    pca2_SetAngle(2,90.0f);
    vTaskDelay(pdMS_TO_TICKS(500));

    // [LEGS] //////////////////////////////
    smoothSweepDual(pca2, 6, cur_pca2[6], 30.0f,9, cur_pca2[9], 150.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));

    smoothSweepDual(pca2, 5, cur_pca2[5], 30.0f,8, cur_pca2[8], 150.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(500));

    smoothSweepDual(pca2, 4, cur_pca2[4], 0.0f,7, cur_pca2[7], 180.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(300));
 

    // [ARMS] //////////////////////////////
    smoothSweepDual(pca2, 0, cur_pca2[0], 180.0f,2, cur_pca2[2], 0.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(500));
    smoothSweepDual(pca2, 1, cur_pca2[1], 150.0f,3, cur_pca2[3], 30.0f, 10);
    vTaskDelay(pdMS_TO_TICKS(500));

    // [Shells] //////////////////////////////
    // smoothSweepDual(pca1, 0, cur_pca1[0], 90.0f,2, cur_pca1[2], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 1, cur_pca1[1], 90.0f,3, cur_pca1[3], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 4, cur_pca1[4], 90.0f,5, cur_pca1[5], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 6, cur_pca1[6], 90.0f,7, cur_pca1[7], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 8, cur_pca1[8], 90.0f,10, cur_pca1[10], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    // smoothSweepDual(pca1, 9, cur_pca1[9], 90.0f,11, cur_pca1[11], 90.0f, 10);
    // vTaskDelay(pdMS_TO_TICKS(30));
    stateLock = 1; 
  }
}
void action_sleep() {
  Serial.println("[ACTION SLEEP] Initiating Safe Shutdown...");
  
  // for (int i = 0; i < 12; i++) {
  //   pca1.setPWM(i, 0, 0); 
  // }
  
  for (int i = 0; i < 10; i++) {
    pca2.setPWM(i, 0, 0); 
  }
  
  stateLock = 0; 
  
  Serial.println("[ACTION SLEEP] Servos offline. Safe to unplug LiPo.");
}
//=============================================

//===============BNO055=======================
void check_BNO_Zone() {
  sensors_event_t orientEvent;
  bno.getEvent(&orientEvent, Adafruit_BNO055::VECTOR_EULER);
  float roll = orientEvent.orientation.z;
  
  if      (roll > -45   && roll <= 45)   zone = 0;
  else if (roll > 30    && roll <= 135)  zone = 1;
  else if (roll > 135   || roll < -135)  zone = 2;
  else if (roll >= -135 && roll <= -45)  zone = 3;
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
        case 8: action_sleep(); break;
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
  Wire.setTimeOut(20);
  // pca1.begin(); pca1.setPWMFreq(PWM_FREQ);
  pca2.begin(); pca2.setPWMFreq(PWM_FREQ);

// cur_pca1[ 0] = 90.0f;
// cur_pca1[ 1] = 90.0f;
// cur_pca1[ 2] = 90.0f;
// cur_pca1[ 3] = 90.0f;
// cur_pca1[ 4] = 90.0f;
// cur_pca1[ 5] = 90.0f;
// cur_pca1[ 6] = 90.0f;
// cur_pca1[ 7] = 90.0f;
// cur_pca1[ 8] = 90.0f;
// cur_pca1[ 9] = 90.0f;
// cur_pca1[10] = 90.0f;
// cur_pca1[11] = 90.0f;

cur_pca2[0] = 150.0f;
cur_pca2[1] = 150.0f;
cur_pca2[2] =  30.0f;
cur_pca2[3] =  30.0f;
cur_pca2[4] = 90.0f;
cur_pca2[5] = 90.0f;
cur_pca2[6] = 90.0f;
cur_pca2[7] = 90.0f;
cur_pca2[8] = 90.0f;
cur_pca2[9] = 90.0f;

// for(int i = 0; i < 12; i++) pca1_SetAngle(i, cur_pca1[i]);
for(int i = 0; i < 10; i++) pca2_SetAngle(i, cur_pca2[i]);

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