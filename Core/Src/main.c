/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  *                               GENERAL INFORMATION
  * 
  * @brief      ADC input works at 10kHz, DAC and PWM outputs work at 100 Hz
  *             Average value goes to UART DMA every 1 second.
  * @author     Malenkov K.S.
  * @version    1.01
  * @date       29.07.2020
  * @warning    This program is made only with educational goal.
  * 
  ****************************************************************************
  *
  *																		THE TASK
  *
  * Необходимо реализовать чтение значений со входа АЦП с использованием DMA с дискретизацией 10 КГц 
  * и установки полученных усреднённых значений на выходе ЦАП и PWM раз в 10млСек. 
  * А так же выдаче усреднённых за 1 секунду значений по USART DMA*. 
  * Полученные значения выводить в мили вольтах, формат представления в ASCII кодах с символом перевода строки на конце 
  * (пример: «1500 mV\r»). 
  * Дублировать вывод информации по SWD интерфейсу в терминал IDE Keil**.
  *
	*	*Вывод в USB (Virtual COM PORT) не реализован из-за отсутствия данной функции в ПО STM32CubeMX и соответственно МК.
	*	 Заменено на USART DMA.
	*	*�?з-за показанной ошибки в IDE Keil "Target HW not present" сделан вывод, что так же данная функция невозможна
	*		на данной отладочной плате.
	*
	*	
	*		
	*
	******************************************************************************
	*	@brief	
	*																	HARDWARE INFO
	*
	* Developmentboard 					: STM32F0DISCOVERY
	* Microcontroller						:	STM32F051R8T6	(64KB Flash,8 KB RAM, 48 MHz max)
	*	FrequencyCLK							: F = 8 MHz
	*	DataSheetDevBoard					: https://static.chipdip.ru/lib/735/DOC000735976.pdf
	*	DataSheetMicrocontroller	: https://ru.mouser.com/datasheet/2/389/dm00039193-1797631.pdf
	*
	*																FILES STRUCTURE
	*
	*	- STM32_ADC_DAC_PWM_UART.ioc										(STM32ClubMX project)
	*	Core\
	*			src\ 
	*					- main.c																(Main program)						
	*					-	stm32f0xx_hal_msp.h
	*					- stm32f0xx_it.h
	*					- system_stm32f0xx.h
	*	MDK-ARM\
	*					- STM32_ADC_DAC_PWM_UART.uvprojx				(Keil v5 Project)
	* DataSheets\
	*					- STM32F051x4_DataSheet.pdf
	*					- STM32F0DISCOVERY_DataSheet.pdf
	*
	*																USED PINS

	*		Description		|			PIN				|						Additional information				|
	*===========================================================================|
	*				ADC				|			PA0				|							ADC_IN0 (Vref = 3V)***			|
	*				DAC				|			PA4				|									DAC1_OUT1								|
	*		PWM_OUTPUT		|			PA5				|									TIM2_CH1								|
	*		UART_RX				|			PA10			|USART1_RX Baud rate:115200, parity:None  |
	*		UART_TX				|			PA9				|									USART1_TX								|
	*
	* *** ! Warning: ADC volatage can be maximum 3V !
	*
	*																TIMER PARAMETERS
	*
	*		Timer name		|									Timer Parameters												|
	*=============================================================================
	*			TIM1				|			Prescaler 0, Counter period 799											|
	*			TIM2				|			Prescaler	0, Counter period	79999										|
	*
	*
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES_NUMBER	100																									///< Value for ADC	Fs = 10 kHz (0.1ms), 100 Samples = 10ms				
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
char trans_str[63];																													///< Variable for Text(string) to UART
uint16_t	ADC_to_memory [2*SAMPLES_NUMBER];																	///< Variable for storing ADC data (to DMA) 
/// States from ADC(DMA) conversion 
typedef enum {
		NO_INTERRUPT,				///< There is no interrupt
		HALF_INTERRUPT,			///< Interrupt half samples number
		FULL_INTERRUPT			///< Interrupt all samples number
} 
adc_conversion_state_t;			///< enum Variable

adc_conversion_state_t adc_state; 																						///< Variable for interrupts flags after half Data to DMA and full data to DMA
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc ADC handle
  * @retval None
  * @description When ADC completed conversion of all data , then send interrupt to while block in main
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
			adc_state = FULL_INTERRUPT;
}
/**
  * @brief  Conversion complete callback in non blocking mode 
  * @param  hadc ADC handle
  * @retval None
  * @description When ADC completed conversing of half data , then send interrupt to while block in main
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
			adc_state = HALF_INTERRUPT;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		uint32_t ADC_avg_to_UART		[SAMPLES_NUMBER] = {0,};									///< local massive for calculate every data every 10msec
		uint32_t ADC_avg_value = 0;																						///< local variable for calculate average ADC data every 10msec
		uint32_t ADC_Res_avg_to_UART = 0;																			///< local variable for calculate average ADC data every 1 sec for UART
		int j = 0;																														///< general counter
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
  MX_ADC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_DAC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc);																			///< Start Colibration ADC before working with this one
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADC_to_memory, 2*SAMPLES_NUMBER);	///< Start ADC working in DMA mode with Channel 1 ADC and Samples = 100
	HAL_TIM_Base_Start(&htim1);																							///< Start TIMER 1 (Frequency for ADC1 = 10khz)
	HAL_TIM_Base_Start(&htim2);																							///< Start TIMER 2 (Frequency for DAC,PWM = 100 Hz)
	//============================================== TIMER PWM OUTPUT ================================================================/
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);																///< TIMER 2 CHANNEL 1 Set as PWM generated output
																							
	//============================================== DAC SETTINGS ====================================================================/
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);																		///< Start DAC1 CHANNEL1 working
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
		if (adc_state)
		{
			if (adc_state == HALF_INTERRUPT)																		///< If half convert completed calculating sum value
			{
				for (int i = 0; i < SAMPLES_NUMBER; i++)
					ADC_avg_value += ADC_to_memory[i];
			}	
			else if (adc_state == FULL_INTERRUPT)																///< If full convert complete calculating sum value
			{
				for (int i = SAMPLES_NUMBER; i < 2*SAMPLES_NUMBER; i++)
					ADC_avg_value += ADC_to_memory[i];
			}
			adc_state = NO_INTERRUPT;																						///< After each calculation reset our interrupt
			ADC_avg_value /= SAMPLES_NUMBER;																		///< Calculate average value
	
			ADC_avg_to_UART[j] = ADC_avg_value; 																///< Send average data to massive (which will be prepared for sending average/sec)		
			if (j == SAMPLES_NUMBER - 1)																				///< If our massive is filled, then begin to calculate average value of this one
			{
				for (int i = 0; i < SAMPLES_NUMBER; i++)
				{
				  ADC_Res_avg_to_UART += ADC_avg_to_UART[i];
				}
				ADC_Res_avg_to_UART /= SAMPLES_NUMBER;
				snprintf(trans_str, 63, "%d mV\r\n", ADC_Res_avg_to_UART * 732 / 1000);
				huart1.gState = HAL_UART_STATE_READY;
				HAL_UART_Transmit_DMA(&huart1, (uint8_t *)trans_str, strlen(trans_str));
				j = 0;																	
				ADC_Res_avg_to_UART = 0;
			}
			j++;
			HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1, DAC_ALIGN_12B_R, ADC_avg_value);	///< Send average data to DAC
			TIM2->CCR1 = ADC_avg_value*80000/4095;															///< send average data to PWM channel	(REGISTER)										
			ADC_avg_value = 0;
		}	
			
	}	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Period = 79999;
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
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.Mode = UART_MODE_TX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
