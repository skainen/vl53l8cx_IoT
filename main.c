/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "vl53l8cx_api.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TCA9548A_ADDR 0x70

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

VL53L8CX_Configuration Dev;
VL53L8CX_ResultsData Results;
/* USER CODE BEGIN PV */

VL53L8CX_Configuration Dev_Sensor1;  // Channel 0
VL53L8CX_Configuration Dev_Sensor2;  // Channel 1
VL53L8CX_ResultsData Results_Sensor1;
VL53L8CX_ResultsData Results_Sensor2;

/* USER CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function to select multiplexer channel
void SelectMuxChannel(uint8_t channel)
{
    uint8_t channel_byte = (1 << channel);  // Convert to bit mask
    HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &channel_byte, 1, HAL_MAX_DELAY);
    HAL_Delay(10);  // Give mux time to switch
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
   BspCOMInit.BaudRate   = 115200;
   BspCOMInit.WordLength = COM_WORDLENGTH_8B;
   BspCOMInit.StopBits   = COM_STOPBITS_1;
   BspCOMInit.Parity     = COM_PARITY_NONE;
   BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
   if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
   {
     Error_Handler();
   }

  printf("Starting VL53L8CX initialization...\r\n");
  printf("Starting VL53L8CX init...\r\n");

  // Proper boot sequence

  // Enable power regulators first
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(10);  // Wait for regulators to stabilize

  HAL_GPIO_WritePin(VL53L8_LPn_GPIO_Port, VL53L8_LPn_Pin, GPIO_PIN_RESET);  // Start LOW
  HAL_Delay(100);
  HAL_GPIO_WritePin(VL53L8_LPn_GPIO_Port, VL53L8_LPn_Pin, GPIO_PIN_SET);    // Set HIGH
  HAL_Delay(100);  // Wait for sensor to boot


  // TEST: Check if multiplexer responds
  printf("Testing multiplexer at 0x70...\r\n");
  HAL_StatusTypeDef mux_result = HAL_I2C_IsDeviceReady(&hi2c1, 0x70 << 1, 3, 1000);
  printf("Multiplexer result: %d\r\n", mux_result);

  if(mux_result != HAL_OK)
  {
      printf("ERROR: Multiplexer not detected!\r\n");
      Error_Handler();
  }

  printf("--- Initializing Sensor 1 (Channel 0) ---\r\n");

  SelectMuxChannel(0);  // Select channel 0

  Dev_Sensor1.platform.address = 0x52;

  // Test I2C
  HAL_StatusTypeDef i2c_result;
  i2c_result = HAL_I2C_IsDeviceReady(&hi2c1, 0x52, 3, 1000);
  printf("Sensor 1 I2C result: %d\r\n", i2c_result);

  if(i2c_result != HAL_OK)
  {
      printf("ERROR: Sensor 1 not detected!\r\n");
      Error_Handler();
  }

  // Check if alive
  uint8_t isAlive, status;
  status = vl53l8cx_is_alive(&Dev_Sensor1, &isAlive);
  printf("Sensor 1 is_alive: status=%d, alive=%d\r\n", status, isAlive);

  if(status != 0 || !isAlive)
  {
      printf("ERROR: Sensor 1 not alive!\r\n");
      Error_Handler();
  }

  // Initialize sensor 1
  printf("Initializing Sensor 1 (takes ~5 seconds)...\r\n");
  status = vl53l8cx_init(&Dev_Sensor1);
  printf("Sensor 1 init status: %d\r\n", status);

  if(status != 0)
  {
      printf("ERROR: Sensor 1 init failed!\r\n");
      Error_Handler();
  }

  // Configure sensor 1
  vl53l8cx_set_resolution(&Dev_Sensor1, VL53L8CX_RESOLUTION_8X8);
  vl53l8cx_set_ranging_frequency_hz(&Dev_Sensor1, 15);
  vl53l8cx_start_ranging(&Dev_Sensor1);

  printf("✓ Sensor 1 initialized and ranging!\r\n\r\n");

  // ========================================
  // Initialize Sensor 2 (Channel 1)
  // ========================================
  printf("--- Initializing Sensor 2 (Channel 1) ---\r\n");

  SelectMuxChannel(1);  // Select channel 1

  Dev_Sensor2.platform.address = 0x52;

  // Test I2C
  i2c_result = HAL_I2C_IsDeviceReady(&hi2c1, 0x52, 3, 1000);
  printf("Sensor 2 I2C result: %d\r\n", i2c_result);

  if(i2c_result != HAL_OK)
  {
      printf("ERROR: Sensor 2 not detected!\r\n");
      Error_Handler();
  }

  // Check if alive
  status = vl53l8cx_is_alive(&Dev_Sensor2, &isAlive);
  printf("Sensor 2 is_alive: status=%d, alive=%d\r\n", status, isAlive);

  if(status != 0 || !isAlive)
  {
      printf("ERROR: Sensor 2 not alive!\r\n");
      Error_Handler();
  }

  // Initialize sensor 2
  printf("Initializing Sensor 2 (takes ~5 seconds)...\r\n");
  status = vl53l8cx_init(&Dev_Sensor2);
  printf("Sensor 2 init status: %d\r\n", status);

  if(status != 0)
  {
      printf("ERROR: Sensor 2 init failed!\r\n");
      Error_Handler();
  }

  // Configure sensor 2
  vl53l8cx_set_resolution(&Dev_Sensor2, VL53L8CX_RESOLUTION_8X8);
  vl53l8cx_set_ranging_frequency_hz(&Dev_Sensor2, 15);
  vl53l8cx_start_ranging(&Dev_Sensor2);

  printf("✓ Sensor 2 initialized and ranging!\r\n\r\n");

  printf("=== Both Sensors Ready! ===\r\n\r\n");

  /* USER CODE END 2 */
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);


  /* Infinite loop */
 /* USER CODE BEGIN WHILE */

  printf("\033[2J\033[H");  // Clear screen

  while (1)
  {
      uint8_t isReady1, isReady2;

      // Check Sensor 1 (Channel 0)
      SelectMuxChannel(0);
      vl53l8cx_check_data_ready(&Dev_Sensor1, &isReady1);

      if(isReady1)
      {
          vl53l8cx_get_ranging_data(&Dev_Sensor1, &Results_Sensor1);
      }

      // Check Sensor 2 (Channel 1)
      SelectMuxChannel(1);
      vl53l8cx_check_data_ready(&Dev_Sensor2, &isReady2);

      if(isReady2)
      {
          vl53l8cx_get_ranging_data(&Dev_Sensor2, &Results_Sensor2);
      }

      // Display Both Sensors
      if(isReady1 || isReady2)
      {
          printf("\033[H");  // Move cursor to top

          // Title
          printf("╔════════════════════════════════════════════════════════════════════╗\r\n");
          printf("║            VL53L8CX Dual Sensor System - 8x8 Mode                ║\r\n");
          printf("╚════════════════════════════════════════════════════════════════════╝\r\n\r\n");

          // ========================================
          // SENSOR 1 Display
          // ========================================
          printf("┌─────────────────── SENSOR 1 (Channel 0) ────────────────────┐\r\n");
          printf("│ Distance Map (mm):                                          │\r\n");
          printf("├─────────────────────────────────────────────────────────────┤\r\n");

          for(int row = 0; row < 8; row++)
          {
              printf("│ ");
              for(int col = 0; col < 8; col++)
              {
                  int zone = row * 8 + col;

                  if(Results_Sensor1.target_status[zone] == 5 ||
                     Results_Sensor1.target_status[zone] == 9)
                  {
                      int dist = Results_Sensor1.distance_mm[zone];

                      // Color coding
                      if(dist < 300)
                          printf("\033[41m %4d \033[0m", dist);      // Red background (very close)
                      else if(dist < 600)
                          printf("\033[31m %4d \033[0m", dist);      // Red text (close)
                      else if(dist < 1000)
                          printf("\033[33m %4d \033[0m", dist);      // Yellow (medium)
                      else if(dist < 2000)
                          printf("\033[32m %4d \033[0m", dist);      // Green (far)
                      else
                          printf("\033[36m %4d \033[0m", dist);      // Cyan (very far)
                  }
                  else
                  {
                      printf(" \033[90m---\033[0m ");  // Gray for invalid
                  }
              }
              printf(" │\r\n");
          }

          // Sensor 1 stats
          uint16_t min1 = 65535, max1 = 0, avg1 = 0;
          int valid1 = 0;

          for(int i = 0; i < 64; i++)
          {
              if(Results_Sensor1.target_status[i] == 5)
              {
                  int dist = Results_Sensor1.distance_mm[i];
                  valid1++;
                  avg1 += dist;
                  if(dist < min1) min1 = dist;
                  if(dist > max1) max1 = dist;
              }
          }
          if(valid1 > 0) avg1 /= valid1;

          printf("├─────────────────────────────────────────────────────────────┤\r\n");
          printf("│ Min: \033[32m%4d mm\033[0m  Max: \033[36m%4d mm\033[0m  Avg: %4d mm  Valid: %2d/64 │\r\n",
                 min1, max1, avg1, valid1);
          printf("│ Temperature: %3d°C                                           │\r\n",
                 Results_Sensor1.silicon_temp_degc);
          printf("└─────────────────────────────────────────────────────────────┘\r\n\r\n");

          // ========================================
          // SENSOR 2 Display
          // ========================================
          printf("┌─────────────────── SENSOR 2 (Channel 1) ────────────────────┐\r\n");
          printf("│ Distance Map (mm):                                          │\r\n");
          printf("├─────────────────────────────────────────────────────────────┤\r\n");

          for(int row = 0; row < 8; row++)
          {
              printf("│ ");
              for(int col = 0; col < 8; col++)
              {
                  int zone = row * 8 + col;

                  if(Results_Sensor2.target_status[zone] == 5 ||
                     Results_Sensor2.target_status[zone] == 9)
                  {
                      int dist = Results_Sensor2.distance_mm[zone];

                      // Color coding
                      if(dist < 300)
                          printf("\033[41m %4d \033[0m", dist);      // Red background
                      else if(dist < 600)
                          printf("\033[31m %4d \033[0m", dist);      // Red text
                      else if(dist < 1000)
                          printf("\033[33m %4d \033[0m", dist);      // Yellow
                      else if(dist < 2000)
                          printf("\033[32m %4d \033[0m", dist);      // Green
                      else
                          printf("\033[36m %4d \033[0m", dist);      // Cyan
                  }
                  else
                  {
                      printf(" \033[90m---\033[0m ");  // Gray
                  }
              }
              printf(" │\r\n");
          }

          // Sensor 2 stats
          uint16_t min2 = 65535, max2 = 0, avg2 = 0;
          int valid2 = 0;

          for(int i = 0; i < 64; i++)
          {
              if(Results_Sensor2.target_status[i] == 5)
              {
                  int dist = Results_Sensor2.distance_mm[i];
                  valid2++;
                  avg2 += dist;
                  if(dist < min2) min2 = dist;
                  if(dist > max2) max2 = dist;
              }
          }
          if(valid2 > 0) avg2 /= valid2;

          printf("├─────────────────────────────────────────────────────────────┤\r\n");
          printf("│ Min: \033[32m%4d mm\033[0m  Max: \033[36m%4d mm\033[0m  Avg: %4d mm  Valid: %2d/64 │\r\n",
                 min2, max2, avg2, valid2);
          printf("│ Temperature: %3d°C                                           │\r\n",
                 Results_Sensor2.silicon_temp_degc);
          printf("└─────────────────────────────────────────────────────────────┘\r\n\r\n");

          // Legend
          printf("Legend: ");
          printf("\033[41m VERY CLOSE \033[0m ");
          printf("\033[31m CLOSE \033[0m ");
          printf("\033[33m MEDIUM \033[0m ");
          printf("\033[32m FAR \033[0m ");
          printf("\033[36m VERY FAR \033[0m ");
          printf("\033[90m INVALID \033[0m\r\n");

          printf("                                                                    ");
      }

      HAL_Delay(50);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L8_LPn_Pin|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VL53L8_LPn_Pin PC3 */
  GPIO_InitStruct.Pin = VL53L8_LPn_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_EN_Pin */
  GPIO_InitStruct.Pin = PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
