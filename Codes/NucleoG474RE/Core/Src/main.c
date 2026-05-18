/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c4_rx;
DMA_HandleTypeDef hdma_i2c4_tx;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_buffer[1];
int currentCommand = 0;

// Flag variable for communication between Interrupt and the main loop
volatile int process_command = -1;

//Var for State Machine and Live Expressions
uint8_t servo0_state = 0;       // Press states
float live_angle_ch0 = 0.0f;    // Angel on Live Expression
uint16_t live_pulse_ch0 = 0;    // Pulse on Live Expression

//PCA9685
#define PCA9685_ADDR 0x40 << 1
#define PCA_MODE1    0x00
#define PCA_PRESCALE 0xFE
#define PCA_LED0_ON_L 0x06
#define SERVO_MIN 150  // min pulse (0 deg)
#define SERVO_MAX 600  // max pulse (180 deg)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PCA9685_WriteReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    HAL_I2C_Master_Transmit(&hi2c4, PCA9685_ADDR, data, 2, 100);
}

uint8_t PCA9685_ReadReg(uint8_t reg) {
    uint8_t value;
    HAL_I2C_Master_Transmit(&hi2c4, PCA9685_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c4, PCA9685_ADDR, &value, 1, 100);
    return value;
}

void PCA9685_Init(void) {
    PCA9685_WriteReg(PCA_MODE1, 0x00);
    float freq = 50.0f;
    float prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;
    uint8_t prescale = round(prescaleval);

    uint8_t oldmode = PCA9685_ReadReg(PCA_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;
    PCA9685_WriteReg(PCA_MODE1, newmode);
    PCA9685_WriteReg(PCA_PRESCALE, prescale);
    PCA9685_WriteReg(PCA_MODE1, oldmode);
    HAL_Delay(5);
    PCA9685_WriteReg(PCA_MODE1, oldmode | 0xA0);
}

void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off) {
    uint8_t data[5];
    data[0] = PCA_LED0_ON_L + 4 * channel;
    data[1] = on & 0xFF;
    data[2] = on >> 8;
    data[3] = off & 0xFF;
    data[4] = off >> 8;
    HAL_I2C_Master_Transmit(&hi2c4, PCA9685_ADDR, data, 5, 100);
}

void PCA9685_SetAngle(uint8_t channel, float angle) {
    if(angle < 0.0f) angle = 0.0f;
    if(angle > 180.0f) angle = 180.0f;

    uint16_t pulse = SERVO_MIN + (uint16_t)((angle / 180.0f) * (SERVO_MAX - SERVO_MIN));
    PCA9685_SetPWM(channel, 0, pulse);

    if(channel == 0) {
        live_angle_ch0 = angle;
        live_pulse_ch0 = pulse;
    }
}
// ========================================================
//                DYNAMIXEL PROTOCOL 2.0
// =======================================================
// 1. Func Check CRC16
uint16_t update_crc(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size) {
    uint16_t i, j;
    for(j = 0; j < data_blk_size; j++) {
        crc_accum ^= ((uint16_t)data_blk_ptr[j] << 8);
        for(i = 0; i < 8; i++) {
            if(crc_accum & 0x8000) crc_accum = (crc_accum << 1) ^ 0x8005;
            else crc_accum = (crc_accum << 1);
        }
    }
    return crc_accum;
}
// 2. Func control 74HC126 auto
void Dynamixel_Transmit(uint8_t *packet, uint16_t length) {
    // open transmit gate
    HAL_GPIO_WritePin(DIR_PORT_GPIO_Port, DIR_PORT_Pin, GPIO_PIN_SET);
    // transmit gate
    HAL_UART_Transmit(&huart3, packet, length, 100);
    // close transmit gate and prepare to recieve data
    HAL_GPIO_WritePin(DIR_PORT_GPIO_Port, DIR_PORT_Pin, GPIO_PIN_RESET);
}
// 3. Torque Enable
void Dynamixel_TorqueEnable(uint8_t id, uint8_t enable) {
    uint8_t packet[13];
    packet[0] = 0xFF; // Header 1
    packet[1] = 0xFF; // Header 2
    packet[2] = 0xFD; // Header 3
    packet[3] = 0x00; // Reserved
    packet[4] = id;   // Packet id
    packet[5] = 0x06; // Length Low
    packet[6] = 0x00; // Length High
    packet[7] = 0x03; // Instruction: WRITE DATA
    packet[8] = 64;   // Address Low Byte (0x40) 64 = enable torque
    packet[9] = 0x00; // Address High Byte (0x00)
    packet[10] = enable; // Header 1

    uint16_t crc = update_crc(0, packet, 11);
    packet[11] = crc & 0xFF;			// CRC Low Byte
    packet[12] = (crc >> 8) & 0xFF;		// CRC High Byte
    Dynamixel_Transmit(packet, 13);
}
// 4. Func control motor angles | Raw Packet | Step 0-4095
void Dynamixel_SetGoalPosition(uint8_t id, uint32_t position) {
    uint8_t packet[16];
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = id;
    packet[5] = 0x09;
    packet[6] = 0x00;
    packet[7] = 0x03;
    // --- Address 116 Goal Pos ---
    packet[8] = 116;
    packet[9] = 0x00;
    // --- Data 4 Bytes (Little-Endian) for 1 Angle ---
    packet[10] = position & 0xFF;
    packet[11] = (position >> 8) & 0xFF;
    packet[12] = (position >> 16) & 0xFF;
    packet[13] = (position >> 24) & 0xFF;

    uint16_t crc = update_crc(0, packet, 14);
    packet[14] = crc & 0xFF;
    packet[15] = (crc >> 8) & 0xFF;
    Dynamixel_Transmit(packet, 16);
}
// 5. Func Convert Deg to Step
void Dynamixel_SetGoalPosition_Degree(uint8_t id, float degree) {
    // Safety Limit
    if (degree < 0.0f) degree = 0.0f;
    if (degree > 360.0f) degree = 360.0f;

    // Convert Degree to Step (0-4095)
    uint32_t step_value = (uint32_t)((degree / 360.0f) * 4095.0f);

    // Send Step Packet
    Dynamixel_SetGoalPosition(id, step_value);
}
// 6. Main Func Read
uint8_t Dynamixel_ReadData(uint8_t id, uint16_t address, uint16_t length, uint8_t *out_data) {
    uint8_t tx_packet[14];
    tx_packet[0] = 0xFF;
    tx_packet[1] = 0xFF;
    tx_packet[2] = 0xFD;
    tx_packet[3] = 0x00;
    tx_packet[4] = id;
    tx_packet[5] = 0x07;
    tx_packet[6] = 0x00;
    tx_packet[7] = 0x02; // READ (0x02)

    // Read Address Little-Endian
    tx_packet[8] = address & 0xFF;
    tx_packet[9] = (address >> 8) & 0xFF;

    // Input Length of read
    tx_packet[10] = length & 0xFF;
    tx_packet[11] = (length >> 8) & 0xFF;

    uint16_t crc = update_crc(0, tx_packet, 12);
    tx_packet[12] = crc & 0xFF;
    tx_packet[13] = (crc >> 8) & 0xFF;

    // Clear RX
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_FLUSH_DRREGISTER(&huart3);

    uint8_t dummy;
    while(HAL_UART_Receive(&huart3, &dummy, 1, 0) == HAL_OK) {}

    // Transmit Read command
    Dynamixel_Transmit(tx_packet, 14);

    // Calculate expected read bytes
    uint16_t expected_rx_length = 11 + length;

    uint8_t rx_packet[30];

    // Wait for 50ms
    if (HAL_UART_Receive(&huart3, rx_packet, expected_rx_length, 50) == HAL_OK) {
        // Check if data is from Protocol 2.0 and is the same ID
        if (rx_packet[0] == 0xFF && rx_packet[1] == 0xFF && rx_packet[2] == 0xFD && rx_packet[4] == id) {

            // read Error Code at byte num 8
            uint8_t error_code = rx_packet[8];
            if(error_code != 0) {
                // Print error to serial
                char msg[60];
                sprintf(msg, ">> [ERROR] Motor %d Hardware Error Code: 0x%02X\r\n", id, error_code);
                HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
            }

            // out_data
            for(int i = 0; i < length; i++) {
                out_data[i] = rx_packet[9 + i];
            }
            return 1; // return 1 = success
        }
    }
    return 0; // return 0 = failed
}
// 7.Func Read current Pos (4 bytes; Address 132)
float Dynamixel_GetPresentPosition_Degree(uint8_t id) {
    uint8_t data[4];
    if (Dynamixel_ReadData(id, 132, 4, data)) {
        // Assembly 4 bytes to 32 int
        int32_t step_pos = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        // Convert Step to Deg
        float degree = ((float)step_pos / 4095.0f) * 360.0f;
        return degree;
    }
    return -1.0f; // If fail return -1
}
// 8.Func Read Temperature
uint8_t Dynamixel_GetTemperature(uint8_t id) {
    uint8_t data[1];
    if (Dynamixel_ReadData(id, 146, 1, data)) {
        return data[0]; // °C
    }
    return 0; // If fail return 0
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init();
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    char scan_msg[64];

    sprintf(scan_msg, "\r\n--- Starting I2C Scanner ---\r\n");
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);

    uint8_t device_count = 0;
    for(uint8_t i = 1; i < 128; i++) {

        HAL_StatusTypeDef result = HAL_I2C_IsDeviceReady(&hi2c4, (uint16_t)(i << 1), 2, 10);

        if (result == HAL_OK) {
            sprintf(scan_msg, "-> Found I2C Device at address: 0x%02X\r\n", i);
            HAL_UART_Transmit(&hlpuart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
            device_count++;
        }
    }

    if (device_count == 0) {
        sprintf(scan_msg, "-> No I2C devices found. Check Wiring!\r\n");
    } else {
        sprintf(scan_msg, "-> I2C Scan Complete. Found %d device(s).\r\n", device_count);
    }
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);
    sprintf(scan_msg, "----------------------------\r\n");
    HAL_UART_Transmit(&hlpuart1, (uint8_t*)scan_msg, strlen(scan_msg), 100);

    // PCA9685_Init();
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (process_command != -1) {

		  switch(process_command) {
			  case 6:
				  if (servo0_state == 0) {
					  PCA9685_SetAngle(0, 0.0f);
					  servo0_state = 1;
				  }
				  else if (servo0_state == 1) {
					  PCA9685_SetAngle(0, 90.0f);
					  servo0_state = 2;
				  }
				  else if (servo0_state == 2) {
					  PCA9685_SetAngle(0, 180.0f);
					  servo0_state = 0;
				  }
				  break;

			  case 7:
				  {
					  static uint8_t toggle_pos = 0;
					  uint8_t id_to_test = 7;

					  // Enable torque
					  Dynamixel_TorqueEnable(id_to_test, 1);
					  HAL_Delay(10);

					  // Move to the angle
					  if(toggle_pos == 0) {
						  Dynamixel_SetGoalPosition_Degree(id_to_test, 180.0f);
						  toggle_pos = 1;
						  char msg[] = "Plz Moving to 180 Deg\r\n";
						  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
					  }
					  else {
						  Dynamixel_SetGoalPosition_Degree(id_to_test, 90.0f);
						  toggle_pos = 0;
						  char msg[] = "Plz Moving to 90 Deg\r\n";
						  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
					  }

					  HAL_Delay(50); //Delay for motor to prepare

					  // Read Current Pos and Temp
					  float current_deg = Dynamixel_GetPresentPosition_Degree(id_to_test);
					  uint8_t current_temp = Dynamixel_GetTemperature(id_to_test);

					  char msg[100];
					  if(current_deg != -1.0f) {
						  sprintf(msg, "ID:%d | Target Sent. Current Pos: %.2f Deg | Temp: %d C\r\n",id_to_test, current_deg, current_temp);
					  }

					  else {
						  sprintf(msg, "ID:%d | Target Sent, but FAILED to read status!\r\n", id_to_test);
					  }
					  HAL_UART_Transmit(&hlpuart1, (uint8_t*)msg, strlen(msg), 100);
				  }
				  break;
		  }
		  process_command = -1;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10802D9B;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIR_PORT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_PORT_Pin */
  GPIO_InitStruct.Pin = DIR_PORT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DIR_PORT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        if (rx_buffer[0] >= '0' && rx_buffer[0] <= '9') {
            currentCommand = rx_buffer[0] - '0';
            process_command = currentCommand;
        }

        HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
