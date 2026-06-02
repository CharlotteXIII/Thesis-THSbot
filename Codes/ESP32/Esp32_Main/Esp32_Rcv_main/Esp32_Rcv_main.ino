// ================================================================
//  Core 0 : Receive ESP-NOW put into Queue
//  Core 1 : Receive Queue and command servo through PCA9685 (X2)
//
//  PCA9685#1 (0x40) - MG90s  ch 0-15
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

Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x41);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

#define SERVO_MIN  150
#define SERVO_MAX  600
#define PWM_FREQ   50

QueueHandle_t commandQueue; //Create Queue

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
}

void pca2_SetAngle(uint8_t ch, float angle) {
  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;
  uint16_t pulse = SERVO_MIN + (uint16_t)((angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
  pca2.setPWM(ch, 0, pulse);
}
//=============================================

//===============ACTIONs=======================
void action_up() {
  Serial.printf("[CMD 1] UP — state %d\n", state_cmd1);
  if (state_cmd1 == 0) {
    pca1_SetAngle(0,   0.0f);
    pca2_SetAngle(0,   0.0f);
    state_cmd1 = 1;
  }
  else if (state_cmd1 == 1) {
    pca1_SetAngle(0,  90.0f);
    pca2_SetAngle(0,  90.0f);
    state_cmd1 = 2;
  }
  else if (state_cmd1 == 2) {
    pca1_SetAngle(0, 180.0f);
    pca2_SetAngle(0, 180.0f);
    state_cmd1 = 0;
  }
}

void action_left() {
  Serial.printf("[CMD 2] LEFT — state %d\n", state_cmd2);
  if (state_cmd2 == 0) {
    pca1_SetAngle(0,  0.0f);
    pca2_SetAngle(0,  0.0f);
    state_cmd2 = 1;
  }
  else if (state_cmd2 == 1) {
    pca1_SetAngle(0, 90.0f);
    pca2_SetAngle(0, 90.0f);
    state_cmd2 = 0;
  }
}

void action_right() {
  Serial.printf("[CMD 3] RIGHT — state %d\n", state_cmd3);
  if (state_cmd3 == 0) {
    pca1_SetAngle(0,  0.0f);
    pca2_SetAngle(0,  0.0f);
    state_cmd3 = 1;
  }
  else if (state_cmd3 == 1) {
    pca1_SetAngle(0, 90.0f);
    pca2_SetAngle(0, 90.0f);
    state_cmd3 = 0;
  }
}

void action_X() {
  Serial.printf("[CMD 6] X — state %d\n", state_cmd6);
  if (state_cmd6 == 0) {
    pca1_SetAngle(0,   0.0f);
    state_cmd6 = 1;
  }
  else if (state_cmd6 == 1) {
    pca1_SetAngle(0,  90.0f);
    state_cmd6 = 2;
  }
  else if (state_cmd6 == 2) {
    pca1_SetAngle(0, 180.0f);
    state_cmd6 = 0;
  }
}

void action_O() {
  Serial.printf("[CMD 7] O — state %d\n", state_cmd7);
  if (state_cmd7 == 0) {
    pca2_SetAngle(0,   0.0f);
    state_cmd7 = 1;
  }
  else if (state_cmd7 == 1) {
    pca2_SetAngle(0,  90.0f);
    state_cmd7 = 2;
  }
  else if (state_cmd7 == 2) {
    pca2_SetAngle(0, 180.0f);
    state_cmd7 = 0;
  }
}
//=============================================


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
  action_idle();

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

  commandQueue = xQueueCreate(5, sizeof(int));
  if (commandQueue == NULL) {
    Serial.println("Queue create failed!");
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
  vTaskDelay(pdMS_TO_TICKS(1000));
}