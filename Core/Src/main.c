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
#include <math.h>
#include <stdio.h>

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t cont_500ms = 0;
volatile uint32_t cont_2000ms = 0;
volatile uint8_t sensor_sample_pending = 0;
volatile uint8_t digital_alarm = 0;
volatile uint8_t weather_warning = 0;
volatile uint8_t pressure_warning = 0;
float temp_dht = NAN;
float hum_dht = NAN;
float pres_bmp = NAN;
float temp_bmp = NAN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void DHT11_Read(float *temperatura, float *humedad);
void BMP280_Read(float *presion, float *temperatura);
static void DelayUs(uint32_t microseconds);
static uint8_t DHT11_WaitForLevel(GPIO_PinState level, uint32_t timeout_us);
static HAL_StatusTypeDef BMP280_ReadReg(uint8_t address, uint8_t reg,
                                        uint8_t *data, uint16_t size);
static HAL_StatusTypeDef BMP280_WriteReg(uint8_t address, uint8_t reg,
                                         uint8_t value);
static void FormatFloat1(float value, char *buffer, size_t buffer_size);
static void UART_SendReadings(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

    static uint8_t buzzer_active = 0U;
    static uint32_t buzzer_last_toggle_cycles = 0U;
    uint8_t sample_pending;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    sample_pending = sensor_sample_pending;
    sensor_sample_pending = 0U;
    __set_PRIMASK(primask);

    if (sample_pending != 0U)
    {
      DHT11_Read(&temp_dht, &hum_dht);
      BMP280_Read(&pres_bmp, &temp_bmp);
      pressure_warning = (isfinite(pres_bmp) &&
                          (pres_bmp >= 300.0f) && (pres_bmp <= 1100.0f) &&
                          (pres_bmp < 990.0f)) ? 1U : 0U;
      weather_warning = ((isfinite(temp_dht) && isfinite(hum_dht) &&
                          (temp_dht >= 0.0f) && (temp_dht <= 50.0f) &&
                          (hum_dht >= 0.0f) && (hum_dht <= 100.0f) &&
                          (temp_dht > 40.0f)) ||
                         (pressure_warning != 0U)) ? 1U : 0U;
      UART_SendReadings();
    }

    if ((digital_alarm != 0U) || (weather_warning != 0U))
    {
      uint32_t now = DWT->CYCCNT;
      uint32_t half_period_cycles = SystemCoreClock / 4000U;

      if (buzzer_active == 0U)
      {
        HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
        buzzer_active = 1U;
        buzzer_last_toggle_cycles = now;
      }
      else if ((now - buzzer_last_toggle_cycles) >= half_period_cycles)
      {
        HAL_GPIO_TogglePin(BUZ_GPIO_Port, BUZ_Pin);
        buzzer_last_toggle_cycles = now;
      }
    }
    else if (buzzer_active != 0U)
    {
      HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);
      buzzer_active = 0U;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT_Pin|BUZ_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GAS_Pin LLU_Pin */
  GPIO_InitStruct.Pin = GAS_Pin|LLU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT_Pin BUZ_Pin LED1_Pin LED2_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = DHT_Pin|BUZ_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void FormatFloat1(float value, char *buffer, size_t buffer_size)
{
  int32_t scaled = (int32_t)((value * 10.0f) +
                             ((value >= 0.0f) ? 0.5f : -0.5f));
  uint32_t magnitude = (scaled < 0) ? (uint32_t)(-scaled) :
                                      (uint32_t)scaled;

  (void)snprintf(buffer, buffer_size, "%s%lu.%lu",
                 (scaled < 0) ? "-" : "",
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
}

static void UART_SendReadings(void)
{
  char temperature[16];
  char humidity[16];
  char pressure[16];
  char message[100];
  uint8_t dht_ok = (isfinite(temp_dht) && isfinite(hum_dht) &&
                    (temp_dht >= 0.0f) && (temp_dht <= 50.0f) &&
                    (hum_dht >= 0.0f) && (hum_dht <= 100.0f)) ? 1U : 0U;
  uint8_t bmp_ok = (isfinite(pres_bmp) &&
                    (pres_bmp >= 300.0f) && (pres_bmp <= 1100.0f)) ? 1U : 0U;
  int message_length;

  if (dht_ok != 0U)
  {
    FormatFloat1(temp_dht, temperature, sizeof(temperature));
    FormatFloat1(hum_dht, humidity, sizeof(humidity));
  }
  else
  {
    (void)snprintf(temperature, sizeof(temperature), "N/A");
    (void)snprintf(humidity, sizeof(humidity), "N/A");
  }

  if (bmp_ok != 0U)
  {
    FormatFloat1(pres_bmp, pressure, sizeof(pressure));
  }
  else
  {
    (void)snprintf(pressure, sizeof(pressure), "N/A");
  }

  message_length = snprintf(message, sizeof(message),
                            "DHT11=%s T=%s C H=%s %% | BMP280=%s P=%s hPa\r\n",
                            (dht_ok != 0U) ? "OK" : "ERROR",
                            temperature, humidity,
                            (bmp_ok != 0U) ? "OK" : "ERROR", pressure);
  if ((message_length < 0) || ((size_t)message_length >= sizeof(message)))
  {
    Error_Handler();
  }

  if (HAL_UART_Transmit(&huart1, (uint8_t *)message,
                        (uint16_t)message_length, HAL_MAX_DELAY) != HAL_OK)
  {
    Error_Handler();
  }
}

static void DelayUs(uint32_t microseconds)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = (SystemCoreClock / 1000000U) * microseconds;

  while ((DWT->CYCCNT - start) < cycles)
  {
  }
}

static uint8_t DHT11_WaitForLevel(GPIO_PinState level, uint32_t timeout_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t timeout_cycles = (SystemCoreClock / 1000000U) * timeout_us;

  while (HAL_GPIO_ReadPin(DHT_GPIO_Port, DHT_Pin) != level)
  {
    if ((DWT->CYCCNT - start) >= timeout_cycles)
    {
      return 0U;
    }
  }

  return 1U;
}

void DHT11_Read(float *temperatura, float *humedad)
{
  uint8_t data[5] = {0};
  uint8_t success = 1U;
  uint32_t primask;
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if ((temperatura == NULL) || (humedad == NULL))
  {
    return;
  }

  *temperatura = NAN;
  *humedad = NAN;

  GPIO_InitStruct.Pin = DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_RESET);
  DelayUs(18000U);
  HAL_GPIO_WritePin(DHT_GPIO_Port, DHT_Pin, GPIO_PIN_SET);
  DelayUs(30U);

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

  if ((DHT11_WaitForLevel(GPIO_PIN_RESET, 100U) == 0U) ||
      (DHT11_WaitForLevel(GPIO_PIN_SET, 100U) == 0U) ||
      (DHT11_WaitForLevel(GPIO_PIN_RESET, 100U) == 0U))
  {
    success = 0U;
  }
  else
  {
    primask = __get_PRIMASK();
    __disable_irq();

    for (uint8_t bit = 0U; bit < 40U; bit++)
    {
      uint32_t pulse_start;
      uint32_t pulse_cycles;

      if ((DHT11_WaitForLevel(GPIO_PIN_SET, 100U) == 0U))
      {
        success = 0U;
        break;
      }

      pulse_start = DWT->CYCCNT;
      if (DHT11_WaitForLevel(GPIO_PIN_RESET, 100U) == 0U)
      {
        success = 0U;
        break;
      }

      pulse_cycles = DWT->CYCCNT - pulse_start;
      data[bit / 8U] <<= 1;
      if (pulse_cycles > ((SystemCoreClock / 1000000U) * 40U))
      {
        data[bit / 8U] |= 1U;
      }
    }

    __set_PRIMASK(primask);
  }

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT_GPIO_Port, &GPIO_InitStruct);

  if ((success == 0U) ||
      (data[4] != (uint8_t)(data[0] + data[1] + data[2] + data[3])))
  {
    return;
  }

  *humedad = (float)data[0] + ((float)data[1] / 10.0f);
  *temperatura = (float)(data[2] & 0x7FU) + ((float)data[3] / 10.0f);
  if ((data[2] & 0x80U) != 0U)
  {
    *temperatura = -*temperatura;
  }
}

static HAL_StatusTypeDef BMP280_ReadReg(uint8_t address, uint8_t reg,
                                        uint8_t *data, uint16_t size)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(address << 1), reg,
                          I2C_MEMADD_SIZE_8BIT, data, size, 20U);
}

static HAL_StatusTypeDef BMP280_WriteReg(uint8_t address, uint8_t reg,
                                         uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(address << 1), reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U, 20U);
}

void BMP280_Read(float *presion, float *temperatura)
{
  uint8_t address = 0x76U;
  uint8_t chip_id;
  uint8_t calibration[24];
  uint8_t raw[6];
  int32_t adc_pressure;
  int32_t adc_temperature;
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
  float var1;
  float var2;
  float t_fine;
  float pressure;

  if ((presion == NULL) || (temperatura == NULL))
  {
    return;
  }

  *presion = NAN;
  *temperatura = NAN;

  if ((BMP280_ReadReg(address, 0xD0U, &chip_id, 1U) != HAL_OK) ||
      (chip_id != 0x58U))
  {
    address = 0x77U;
    if ((BMP280_ReadReg(address, 0xD0U, &chip_id, 1U) != HAL_OK) ||
        (chip_id != 0x58U))
    {
      return;
    }
  }
  if ((BMP280_ReadReg(address, 0x88U, calibration, sizeof(calibration)) != HAL_OK) ||
      (BMP280_WriteReg(address, 0xF4U, 0x25U) != HAL_OK))
  {
    return;
  }

  DelayUs(10000U);
  if (BMP280_ReadReg(address, 0xF7U, raw, sizeof(raw)) != HAL_OK)
  {
    return;
  }

  adc_pressure = ((int32_t)raw[0] << 12) |
                 ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
  adc_temperature = ((int32_t)raw[3] << 12) |
                    ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

  dig_T1 = (uint16_t)(calibration[1] << 8) | calibration[0];
  dig_T2 = (int16_t)((uint16_t)(calibration[3] << 8) | calibration[2]);
  dig_T3 = (int16_t)((uint16_t)(calibration[5] << 8) | calibration[4]);
  dig_P1 = (uint16_t)(calibration[7] << 8) | calibration[6];
  dig_P2 = (int16_t)((uint16_t)(calibration[9] << 8) | calibration[8]);
  dig_P3 = (int16_t)((uint16_t)(calibration[11] << 8) | calibration[10]);
  dig_P4 = (int16_t)((uint16_t)(calibration[13] << 8) | calibration[12]);
  dig_P5 = (int16_t)((uint16_t)(calibration[15] << 8) | calibration[14]);
  dig_P6 = (int16_t)((uint16_t)(calibration[17] << 8) | calibration[16]);
  dig_P7 = (int16_t)((uint16_t)(calibration[19] << 8) | calibration[18]);
  dig_P8 = (int16_t)((uint16_t)(calibration[21] << 8) | calibration[20]);
  dig_P9 = (int16_t)((uint16_t)(calibration[23] << 8) | calibration[22]);

  var1 = (((float)adc_temperature / 16384.0f) -
          ((float)dig_T1 / 1024.0f)) * (float)dig_T2;
  var2 = (((float)adc_temperature / 131072.0f) -
          ((float)dig_T1 / 8192.0f));
  var2 = var2 * var2 * (float)dig_T3;
  t_fine = var1 + var2;
  *temperatura = t_fine / 5120.0f;

  var1 = (t_fine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * ((float)dig_P6 / 32768.0f);
  var2 += var1 * (float)dig_P5 * 2.0f;
  var2 = (var2 / 4.0f) + ((float)dig_P4 * 65536.0f);
  var1 = (((float)dig_P3 * var1 * var1 / 524288.0f) +
          ((float)dig_P2 * var1)) / 524288.0f;
  var1 = (1.0f + (var1 / 32768.0f)) * (float)dig_P1;

  if (var1 == 0.0f)
  {
    *temperatura = NAN;
    return;
  }

  pressure = 1048576.0f - (float)adc_pressure;
  pressure = ((pressure - (var2 / 4096.0f)) * 6250.0f) / var1;
  var1 = (float)dig_P9 * pressure * pressure / 2147483648.0f;
  var2 = pressure * (float)dig_P8 / 32768.0f;
  pressure += (var1 + var2 + (float)dig_P7) / 16.0f;
  *presion = pressure / 100.0f;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    GPIO_PinState gas_state = HAL_GPIO_ReadPin(GAS_GPIO_Port, GAS_Pin);
    GPIO_PinState rain_state = HAL_GPIO_ReadPin(LLU_GPIO_Port, LLU_Pin);

    digital_alarm = ((gas_state == GPIO_PIN_RESET) ||
                     (rain_state == GPIO_PIN_RESET)) ? 1U : 0U;

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                      (pressure_warning != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if ((digital_alarm != 0U) || (weather_warning != 0U))
    {
      cont_500ms = 0U;
      HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
    else
    {
      cont_500ms++;
      if (cont_500ms >= 5U)
      {
        cont_500ms = 0U;
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
      }
    }

    cont_2000ms++;
    if (cont_2000ms >= 20U)
    {
      cont_2000ms = 0U;
      HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
      sensor_sample_pending = 1U;
    }
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
