/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
//    float buffer[FILTER_ORDER];
	int32_t buffer[FILTER_ORDER];
    int index;
} CircularBuffer;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
CircularBuffer cb;
uint16_t adc_val;
uint8_t filter_mode = 2;  // 0: RAW, 1: FIR, 2: IIR
float emg_signal_value;
float iir_prev_output = 0.0f;
float fir_buffer[BUFFER_SIZE] = {0};
char buffer[128];
const float fir_coefficients[FILTER_ORDER] = {
	0.000000, 0.000001, 0.000002, 0.000005, 0.000011, 0.000020, 0.000033, 0.000051, 0.000073, 0.000100,
	0.000130, 0.000162, 0.000192, 0.000218, 0.000236, 0.005187, 0.005801, 0.007612, 0.010539, 0.014451,
	0.019175, 0.024503, 0.030201, 0.036021, 0.041709, 0.047016, 0.051714, 0.055598, 0.058499, 0.060292,
	0.060898, 0.060292, 0.058499, 0.055598, 0.051714, 0.047016, 0.041709, 0.036021, 0.030201, 0.024503,
	0.019175, 0.014451, 0.010539, 0.007612, 0.005801, 0.005187, 0.000236, 0.000218, 0.000192, 0.000162,
	0.000130, 0.000100, 0.000073, 0.000051, 0.000033, 0.000020, 0.000011, 0.000005, 0.000002, 0.000001,
	0.000000
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UART_Transmit(UART_HandleTypeDef *huart, char *data) {
	HAL_UART_Transmit(huart, (uint8_t*) data, strlen(data), HAL_MAX_DELAY);
}

// Example IIR filter (Single-pole low-pass filter)
float IIR_Filter(float input, float *prev_output, float alpha) {
    *prev_output = alpha * input + (1.0f - alpha) * (*prev_output);
    return *prev_output;
}

void CircularBuffer_Init(CircularBuffer* cb) {
    memset(cb->buffer, 0, sizeof(cb->buffer));
    cb->index = 0;
}

float FIR_Filter(CircularBuffer* cb, float input) {
    float sum = 0.0f;

    // Update buffer with new input
    cb->buffer[cb->index] = input;

    int idx = cb->index;
    for (int i = 0; i < FILTER_ORDER; i++) {
        sum += cb->buffer[idx] * fir_coefficients[i];
        idx = (idx == 0) ? (FILTER_ORDER - 1) : (idx - 1);
    }

    cb->index = (cb->index + 1) % FILTER_ORDER;

    return sum * EMG_SIGNAL_MAX_VOLTAGE;
}

/* Function to Update LED Indicator */
void Update_LED_Indicator(void)
{
	HAL_GPIO_WritePin(GPIOD, LD4_Green_Pin|LD3_Orange_Pin|LD5_Red_Pin|LD6_Blue_Pin, GPIO_PIN_RESET);

	// Set LED based on the active filter mode
	if (filter_mode == 0) {
		HAL_GPIO_WritePin(GPIOD, LD6_Blue_Pin, GPIO_PIN_SET);   // Blue LED for RAW mode
	} else if (filter_mode == 1) {
		HAL_GPIO_WritePin(GPIOD, LD5_Red_Pin, GPIO_PIN_SET);  // Green LED for FIR mode
	} else if (filter_mode == 2) {
		HAL_GPIO_WritePin(GPIOD, LD3_Orange_Pin, GPIO_PIN_SET); // Orange LED for IIR mode
	}
}

void Poll_Button(void)
{
	if (HAL_GPIO_ReadPin(GPIO_BUTTON_GPIO_Port, GPIO_BUTTON_Pin) == GPIO_PIN_RESET) {
		HAL_Delay(DEBOUNCE_DELAY); // Debounce delay
		if (HAL_GPIO_ReadPin(GPIO_BUTTON_GPIO_Port, GPIO_BUTTON_Pin) == GPIO_PIN_RESET) {
			filter_mode = (filter_mode + 1) % 3;  // Cycle through 0 (RAW), 1 (FIR), 2 (IIR)
			Update_LED_Indicator(); // Update LEDs based on the current mode
			while (HAL_GPIO_ReadPin(GPIO_BUTTON_GPIO_Port, GPIO_BUTTON_Pin) == GPIO_PIN_RESET); // Wait for button release
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  CircularBuffer_Init(&cb);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
	  Error_Handler();

  if(HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  	  	  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Poll_Button(); // Poll button state and update filter flag

	  // Perform other periodic tasks here if needed
	  // Example: You might want to add some other functionalities or delays
	  HAL_Delay(10); // Small delay to avoid rapid polling
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 33600;
  htim2.Init.Period = 56000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Green_Pin|LD3_Orange_Pin|LD5_Red_Pin|LD6_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GPIO_BUTTON_Pin */
  GPIO_InitStruct.Pin = GPIO_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Green_Pin LD3_Orange_Pin LD5_Red_Pin LD6_Blue_Pin */
  GPIO_InitStruct.Pin = LD4_Green_Pin|LD3_Orange_Pin|LD5_Red_Pin|LD6_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // Get the ADC value
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
    float voltage = REF_VOLTAGE * adc_val / ADC_MAX_VAL - DC_BIAS;

    if (filter_mode == 0) {
    	// RAW signal, no filtering just voltage scaling
    	emg_signal_value = voltage * EMG_SIGNAL_MAX_VOLTAGE;
    } else if (filter_mode == 1) {
    	// FIR filtering
    	emg_signal_value = FIR_Filter(&cb, voltage);
    } else if (filter_mode == 2) {
    	// IIR filtering
    	emg_signal_value = IIR_Filter(voltage, &emg_signal_value, ALPHA);
    }

    // Send the processed signal value over UART
   //  sprintf(buffer, "%.6f\r\n", emg_signal_value);
    snprintf(buffer, sizeof(buffer), "%.6f\r\n", emg_signal_value);
    UART_Transmit(&huart2, buffer);

    // Toggle the Green LED to indicate ADC activity
    HAL_GPIO_TogglePin(GPIOD, LD4_Green_Pin);
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
