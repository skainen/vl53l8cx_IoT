/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - 6 Sensor Configuration
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
#define NUM_SENSORS 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

VL53L8CX_Configuration Dev_Sensors[NUM_SENSORS];
VL53L8CX_ResultsData Results_Sensors[NUM_SENSORS];

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
    uint8_t channel_byte = (1 << channel);
    HAL_I2C_Master_Transmit(&hi2c1, TCA9548A_ADDR << 1, &channel_byte, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
}

// Function to initialize a single sensor
uint8_t InitializeSensor(uint8_t sensor_num, uint8_t channel)
{
    printf("--- Initializing Sensor %d (Channel %d) ---\r\n", sensor_num + 1, channel);

    SelectMuxChannel(channel);

    Dev_Sensors[sensor_num].platform.address = 0x52;

    // Test I2C
    HAL_StatusTypeDef i2c_result = HAL_I2C_IsDeviceReady(&hi2c1, 0x52, 3, 1000);
    printf("Sensor %d I2C result: %d\r\n", sensor_num + 1, i2c_result);

    if(i2c_result != HAL_OK)
    {
        printf("ERROR: Sensor %d not detected!\r\n", sensor_num + 1);
        return 1;
    }

    // Check if alive
    uint8_t isAlive, status;
    status = vl53l8cx_is_alive(&Dev_Sensors[sensor_num], &isAlive);
    printf("Sensor %d is_alive: status=%d, alive=%d\r\n", sensor_num + 1, status, isAlive);

    if(status != 0 || !isAlive)
    {
        printf("ERROR: Sensor %d not alive!\r\n", sensor_num + 1);
        return 1;
    }

    // Initialize sensor
    printf("Initializing Sensor %d (takes ~5 seconds)...\r\n", sensor_num + 1);
    status = vl53l8cx_init(&Dev_Sensors[sensor_num]);
    printf("Sensor %d init status: %d\r\n", sensor_num + 1, status);

    if(status != 0)
    {
        printf("ERROR: Sensor %d init failed!\r\n", sensor_num + 1);
        return 1;
    }

    // Configure sensor
    vl53l8cx_set_resolution(&Dev_Sensors[sensor_num], VL53L8CX_RESOLUTION_8X8);
    vl53l8cx_set_ranging_frequency_hz(&Dev_Sensors[sensor_num], 15);
    vl53l8cx_start_ranging(&Dev_Sensors[sensor_num]);

    printf("OK - Sensor %d initialized and ranging!\r\n\r\n", sensor_num + 1);

    return 0;
}

// Calculate sensor statistics
void CalculateSensorStats(uint8_t sensor_num, uint16_t *min, uint16_t *max, uint16_t *avg, uint8_t *valid)
{
    *min = 65535;
    *max = 0;
    *avg = 0;
    *valid = 0;

    for(int i = 0; i < 64; i++)
    {
        if(Results_Sensors[sensor_num].target_status[i] == 5 ||
           Results_Sensors[sensor_num].target_status[i] == 9)
        {
            uint16_t dist = Results_Sensors[sensor_num].distance_mm[i];
            (*valid)++;
            *avg += dist;
            if(dist < *min) *min = dist;
            if(dist > *max) *max = dist;
        }
    }

    if(*valid > 0)
        *avg /= *valid;
    else
        *min = 0;
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

  /* Initialize COM1 port */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  printf("\r\n\r\n");
  printf("========================================\r\n");
  printf("  VL53L8CX 6-Sensor System Startup\r\n");
  printf("========================================\r\n\r\n");

  // Power up sequence
  HAL_GPIO_WritePin(PWR_EN_GPIO_Port, PWR_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  HAL_GPIO_WritePin(VL53L8_LPn_GPIO_Port, VL53L8_LPn_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(VL53L8_LPn_GPIO_Port, VL53L8_LPn_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Test multiplexer
  printf("Testing multiplexer at 0x70...\r\n");
  HAL_StatusTypeDef mux_result = HAL_I2C_IsDeviceReady(&hi2c1, TCA9548A_ADDR << 1, 3, 1000);
  printf("Multiplexer result: %d\r\n\r\n", mux_result);

  if(mux_result != HAL_OK)
  {
      printf("ERROR: Multiplexer not detected!\r\n");
      Error_Handler();
  }

  // Initialize all 6 sensors
  for(int i = 0; i < NUM_SENSORS; i++)
  {
      if(InitializeSensor(i, i) != 0)
      {
          printf("FATAL: Sensor %d initialization failed!\r\n", i + 1);
          Error_Handler();
      }
  }

  printf("========================================\r\n");
  printf("  All Sensors Ready - Starting Ranging\r\n");
  printf("========================================\r\n\r\n");

  HAL_Delay(1000);

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t frame_count = 0;

  while (1)
  {
      uint8_t data_ready[NUM_SENSORS] = {0};

      // Check all sensors for new data
      for(int i = 0; i < NUM_SENSORS; i++)
      {
          SelectMuxChannel(i);
          vl53l8cx_check_data_ready(&Dev_Sensors[i], &data_ready[i]);

          if(data_ready[i])
          {
              vl53l8cx_get_ranging_data(&Dev_Sensors[i], &Results_Sensors[i]);
          }
      }

      // Display if any sensor has new data
      uint8_t any_ready = 0;
      for(int i = 0; i < NUM_SENSORS; i++)
      {
          if(data_ready[i]) any_ready = 1;
      }

      if(any_ready)
      {
          frame_count++;

          printf("\033[2J\033[H");  // Clear screen and home cursor

          printf("================================================================================\r\n");
          printf("  VL53L8CX 6-Sensor System | Frame: %lu\r\n", frame_count);
          printf("================================================================================\r\n\r\n");

          printf("Sensor | Min(mm) | Max(mm) | Avg(mm) | Valid Zones | Temp(C) | Status\r\n");
          printf("-------+---------+---------+---------+-------------+---------+--------\r\n");

          for(int i = 0; i < NUM_SENSORS; i++)
          {
              uint16_t min, max, avg;
              uint8_t valid;

              CalculateSensorStats(i, &min, &max, &avg, &valid);

              printf("   %d   |  %5d  |  %5d  |  %5d  |    %2d/64    |   %3d   | %s\r\n",
                     i + 1,
                     min,
                     max,
                     avg,
                     valid,
                     Results_Sensors[i].silicon_temp_degc,
                     data_ready[i] ? "OK" : "--");
          }

          printf("\r\n");
          printf("Detailed View (minimum distance per sensor):\r\n");
          printf("-----------------------------------------------------------------------\r\n");

          for(int i = 0; i < NUM_SENSORS; i++)
          {
              uint16_t min, max, avg;
              uint8_t valid;

              CalculateSensorStats(i, &min, &max, &avg, &valid);

              // Visual bar representation
              printf("S%d: [", i + 1);

              int bar_length = (min < 4000) ? (40 - (min / 100)) : 0;
              if(bar_length < 0) bar_length = 0;
              if(bar_length > 40) bar_length = 40;

              for(int j = 0; j < bar_length; j++)
              {
                  printf("=");
              }
              for(int j = bar_length; j < 40; j++)
              {
                  printf(" ");
              }

              printf("] %4dmm\r\n", min);
          }

          printf("\r\n");
      }

      HAL_Delay(50);

    /* USER CODE END WHILE */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
