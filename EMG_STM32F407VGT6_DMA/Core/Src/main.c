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
#include "arm_math.h"
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define N 32
#define halfN 16
#define NUM_TAPS 32
#define BUFFER_SIZE 128  // Adjust this size as necessary
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef struct {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} CircularBuffer;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint32_t adc_buffer[N];
uint32_t dac_buffer[N];
/* Buffers store values as integers, therefore casting is needed before sending
   values to floating point FIR filter */
float32_t filt_in[N];
float32_t filt_out[N];
float32_t *filt_in_ptr = &filt_in[0];
float32_t *filt_out_ptr = &filt_out[0];
volatile bool transmissionInProgress = false;

arm_fir_instance_f32 filter1;
CircularBuffer uartTxBuffer;
float32_t State[halfN + NUM_TAPS - 1];
float32_t Coeffs[NUM_TAPS] = {
	0.0000010668, 0.0000023755, 0.0000043788, 0.0000074511, 0.0000118578,
	0.0000177189, 0.0000249661, 0.0000332940, 0.0000421089, 0.0000504867,
	0.0000571499, 0.0000604758, 0.0000585464, 0.0000492457, 0.0000304027,
	0.0050037879, 0.0055890127, 0.0073898940, 0.0103248816, 0.0142630770,
	0.0190300660, 0.0244156217, 0.0301829344, 0.0360789651, 0.0418454728,
	0.0472302419, 0.0519980307, 0.0559407718, 0.0588865910, 0.0607072568,
	0.0613237390, 0.0607072568
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, N);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, dac_buffer, N, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim8);
  arm_fir_init_f32(&filter1, NUM_TAPS, Coeffs, State, halfN);  // Puca na ovoj inicijalizaciji, stack frame ukazuje na ConvHalfCplCallback
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 56000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pins : LD4_Green_Pin LD3_Orange_Pin LD5_Red_Pin LD6_Blue_Pin */
  GPIO_InitStruct.Pin = LD4_Green_Pin|LD3_Orange_Pin|LD5_Red_Pin|LD6_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void BlinkLED(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t duration, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // Turn on LED
        HAL_Delay(duration); // Delay for the specified duration
        HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET); // Turn off LED
        HAL_Delay(duration); // Delay for the specified duration
    }
}

void EnqueueToUartBuffer(uint8_t* data, size_t size) {
    for (size_t i = 0; i < size; i++) {
        // Wait if buffer is full
        while (((uartTxBuffer.head + 1) % BUFFER_SIZE) == uartTxBuffer.tail);
        uartTxBuffer.buffer[uartTxBuffer.head] = data[i];
        uartTxBuffer.head = (uartTxBuffer.head + 1) % BUFFER_SIZE;
    }

    // Start transmission if not already ongoing
	if (!transmissionInProgress) {
		transmissionInProgress = true;
		HAL_UART_Transmit_DMA(&huart2, &uartTxBuffer.buffer[uartTxBuffer.tail], 1);
	}
}

void SendDACData(uint32_t* data, size_t size) {
    char buffer[128];
    for (size_t i = 0; i < size; i++) {
        int len = snprintf(buffer, sizeof(buffer), "DAC[%d]: %lu\r\n", i, data[i]);
        EnqueueToUartBuffer((uint8_t*)buffer, len);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {  // Ensure it's the correct UART
        // Move the tail forward in the circular buffer
        if (uartTxBuffer.tail != uartTxBuffer.head) {
            uartTxBuffer.tail = (uartTxBuffer.tail + 1) % BUFFER_SIZE;
            HAL_UART_Transmit_DMA(&huart2, &uartTxBuffer.buffer[uartTxBuffer.tail], 1);
        } else {
        	transmissionInProgress = false; // Transmission completed
        	// Blink blue LED when transmission is complete
        	BlinkLED(GPIOD, LD6_Blue_Pin, 1000, 1); // Adjust GPIO port and pin number accordingly
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
	for (int n=0; n<halfN; n++){
		filt_in[n] = (float32_t) adc_buffer[n];
	}
	arm_fir_f32(&filter1, filt_in_ptr, filt_out_ptr, halfN);
//	arm_fir_f32(&filter1, filt_in, filt_out, halfN);
	for (int n=0; n<halfN; n++){
		dac_buffer[n] = (uint32_t) filt_out[n];
	}
	// Transmit DAC data via UART
	SendDACData(dac_buffer, halfN);

	BlinkLED(GPIOD, LD4_Green_Pin, 100, 1); // Adjust GPIO port and pin number accordingly
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	for( int n=halfN; n<N; n++){ filt_in[n] = (float32_t) adc_buffer[n];}
	arm_fir_f32(&filter1, filt_in_ptr + halfN, filt_out_ptr + halfN, halfN);
	for( int n=halfN; n<N; n++){ dac_buffer[n] = (uint32_t) filt_out[n];}

	// Transmit DAC data via UART
	SendDACData(dac_buffer + halfN, halfN);

	BlinkLED(GPIOD, LD5_Red_Pin, 100, 1); // Adjust GPIO port and pin number accordingly
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
