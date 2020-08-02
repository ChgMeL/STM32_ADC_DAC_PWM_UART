/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : <h2><center>Основная программа</center></h2>
  ******************************************************************************
  * 
  *
  ******************************************************************************
  ***************************************************************************
  *                  <h3><center>Общая информация</center></h3>
  * 
  * @brief      Работа с АЦП в режиме DMA, ЦАП, PWM сигналом и UART в режиме DMA.
  *  
  * @author     Маленков К.С.
  * @version    1.02
  * @date       30.07.2020
  * @warning    Эта программа используется в образовательных целях.
  * 
  ****************************************************************************
  ****************************************************************************
  *
  *													<h2><center>Условие задачи</center></h2>
  *
  * Необходимо реализовать чтение значений со входа АЦП с использованием DMA с дискретизацией 10 КГц 
  * и установки полученных усреднённых значений на выходе ЦАП и PWM раз в 10млСек. 
  * А так же выдаче усреднённых за 1 секунду значений по USART DMA(1)). 
  * Полученные значения выводить в мили вольтах, формат представления в ASCII кодах с символом перевода строки на конце 
  * (пример: «1500 mV\r»). 
  * Дублировать вывод информации по SWD интерфейсу в терминал IDE Keil(2).
  *
	*	1.Вывод в USB (Virtual COM PORT) не реализован из-за отсутствия данной функции в ПО STM32CubeMX и соответственно МК.
	*	 Заменено на USART DMA.
	*	2.Отладка по SWD интерфейсу невозможна из-за того, что компания ST заблокировала её на аппаратном уровне в программе Keil.
	*	****************************************************************************
	*****************************************************************************
	*
  *		
	*											
  *|		Устройство		|		  Название			
	*|  :-------------: | :-----------: 	| 
	*|Отладочная плата	|	STM32F0DISCOVERY|	
	*|Микроконтроллер		|	STM32F051R8T6		|
  *
	******************************************************************************
  ******************************************************************************
  *
	*													<h2><center>Структура проекта</center></h2>
	*|		Путь											|		  Название файла 						|		Описание файла															|					
	*|  :------------- 							| :----------- 									| :--------------------													|
	*|															|	STM32_ADC_DAC_PWM_UART.ioc		|	Проект STM32ClubMX														|
	*|	Core\src										|	main.c 												|	Основная программа														|
	*|	MDK-ARM											|	STM32_ADC_DAC_PWM_UART.uvprojx|	Проект Keil v5																|
	*|	DataSheets									|	STM32F051x4_DataSheet.pdf			|	Справочный материал по микроконтроллеру				|
	*|	^														|	STM32F0DISCOVERY_DataSheet.pdf|	Справочный материал по отладочной плате				|
	*|	Documentation								|	index.html										|	Документация Doxygen в html	формате						|
	*	 																		
	*	
	*																										
	* 
	*	
	*	
	* 
	*					
	*					
	*******************************************************************************
  *******************************************************************************
	*											<h2><center>Используемые контакты</center></h2>
  *
	*|		Назначение		|	Номер контакта|					Дополнительная информация				|
	*|  :-------------: | :-----------: | :------------------------------------:  | 
	*|				ADC				|			PA0				|							ADC_IN0 (Vref = 3V)***			|
	*|				DAC				|			PA4				|									DAC1_OUT1								|
	*|		PWM_OUTPUT		|			PA5				|									TIM2_CH1								|
	*|		UART_RX				|			PA10			| Baud rate:115200, parity:None  					|
	*|		UART_TX				|			PA9				|						^															|
	*|		OSC_IN				|			PF0				|		 	Внешний источник частоты						|
	*
	* @warning *** ! : Максимальное значение АЦП может быть 3В !
	******************************************************************************
  ******************************************************************************
	*																<h2><center>Параметры счетчиков</center></h2>
	*
	*|Обозначение	таймера	|									Параметры счетчика											|
	*|  :-------------: 	| :----------------------------------------------------:  |
	*|			TIM1					|			Prescaler 0, Counter period 799											|
	*|			TIM2					|			Prescaler	0, Counter period	79999										|
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
#define SAMPLES_NUMBER	100																									///< Количество преобразований АЦП (частота дискретизации 10кГц, 100 отсчетов = 0.1мс)				
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;																											///< Переменная для работы с АЦП
DMA_HandleTypeDef hdma_adc;																									///< Переменная для работы с АЦП в режиме DMA

DAC_HandleTypeDef hdac1;																										///< Переменная для работы с ЦАП

TIM_HandleTypeDef htim1;																										///< Переменная для работы с счетчиком #1																										
TIM_HandleTypeDef htim2;																										///< Переменная для работы с счетчиком #2							

UART_HandleTypeDef huart1;																									///< Переменная для работы с UART
DMA_HandleTypeDef hdma_usart1_tx;																						///< Переменная для работы с UART в режиме DMA (в режиме только передача)

/* USER CODE BEGIN PV */
char trans_str[63];																													///< Переменная для хранения текса в формате string для отправки по UART
uint16_t	ADC_to_memory [2*SAMPLES_NUMBER];																	///< Переменная для хранения данных преобразований АЦП 
/// Состояние АЦП в режиме DMA
typedef enum {
		NO_INTERRUPT,				///< АЦП в процессе выполнения преобразований
		HALF_INTERRUPT,			///< АЦП выполнил половину преобразований
		FULL_INTERRUPT			///< АЦП выполнил полный цикл преобразований
} 
adc_conversion_state_t;			///< Переменная enum

adc_conversion_state_t adc_state; 																						///< Переменная , которая указывает о прерывании АЦП на разных этапах преобразований в режиме DMA
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
	* @brief  АЦП выполнил все преобразования и ставит флаг(неблокирующий) об этом действии  
	* Когда АЦП выполнил все преобрзования, тогда отправляет об этом окончании в основную программу
	* @param  hadc Обращение к АЦП
  * @retval void
  *  
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
			adc_state = FULL_INTERRUPT;
}
/**
  * @brief  АЦП выполнил половину преобразования и ставит флаг(неблокирующий) об этом действии  
  * огда АЦП выполнил половину преобрзований, тогда отправляет об этом окончании в основную программу
  * @param  hadc Обращение к АЦП
  * @retval void
  *  
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
			adc_state = HALF_INTERRUPT;
}
/* USER CODE END 0 */

/**
* @brief  Основная программа:
*					Получает данные о 100 преобразований с АЦП, вычисляет среднее значение  и выдает каждые 10 мс в ЦАП 
*					и PWM сигнал, а так же вычисляет среднее значение за 1 c и отправляет данные по UART  порту в режиме DMA
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t ADC_avg_to_UART		[SAMPLES_NUMBER] = {0,};										//  Массив для хранения усредненный данных за 10 мс.
	uint32_t ADC_avg_value = 0;																							//  Переменная усреденных данных АЦП за 10мс
	uint32_t ADC_Res_avg_to_UART = 0;																				//  Переменная хранения усредненный данных АЦП за 1 секунду для отправки в UART
	int j = 0;																															// 	Общий счетчик для усреденных значний за 10 мс
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
	HAL_ADCEx_Calibration_Start(&hadc);																			// Калибровка АЦП
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)&ADC_to_memory, 2*SAMPLES_NUMBER);	// АЦП в режиме DMA
	HAL_TIM_Base_Start(&htim1);																							// Счетчик #1 (Частота 10кГц)
	HAL_TIM_Base_Start(&htim2);																							// Счетчик #2 (Для ЦАП и PWM сигнала с частотой = 100 Гц)
	//============================================== TIMER PWM OUTPUT ================================================================/
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);																// Счетчик #1 канал 1 как выход PWM
																							
	//============================================== DAC SETTINGS ====================================================================/
	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);																		// ЦАП канал 1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	{
		if (adc_state)
		{
			if (adc_state == HALF_INTERRUPT)																		// Если пришло прерывания с половины преобразований АЦП, вычисляем сумму этих данных
			{
				for (int i = 0; i < SAMPLES_NUMBER; i++)
					ADC_avg_value += ADC_to_memory[i];
			}	
			else if (adc_state == FULL_INTERRUPT)																// Если пришло прерывания со всех преобразований АЦП, вычисляем сумму этих данных
			{
				for (int i = SAMPLES_NUMBER; i < 2*SAMPLES_NUMBER; i++)
					ADC_avg_value += ADC_to_memory[i];
			}
			adc_state = NO_INTERRUPT;																						// Сбрасываем прерывания АЦП
			ADC_avg_value /= SAMPLES_NUMBER;																		// Высчитываем среднее значение
	
			ADC_avg_to_UART[j] = ADC_avg_value; 																// Добавляем среднее значение в массив для будущей отправки в UART за 1 секунду
			if (j == SAMPLES_NUMBER - 1)																				// Если массив для отправки в UART заполнен данными - высчитываем сумму
			{
				for (int i = 0; i < SAMPLES_NUMBER; i++)
				{
				  ADC_Res_avg_to_UART += ADC_avg_to_UART[i];
				}
				ADC_Res_avg_to_UART /= SAMPLES_NUMBER;														//	Высчитываем среднее значение для отправки в UART
				snprintf(trans_str, 63, "%d mV\r\n", ADC_Res_avg_to_UART * 732 / 1000);		//Преобразуем это значение в строковую переменную, проводя данные к новой шкале (максимум 3000 мВ)
				huart1.gState = HAL_UART_STATE_READY;															// Так как UART  работае в DMA  режиме и находится в режиме ожидания, выставляем режим готовности к отправки.					
				HAL_UART_Transmit_DMA(&huart1, (uint8_t *)trans_str, strlen(trans_str));		//Отправляем данные по UART  в режиме DMA
				j = 0;																														// 	Обнуляем счетчик
				ADC_Res_avg_to_UART = 0;																					// Обнуляем среднее значение для UART
			}
			j++;																																
			HAL_DAC_SetValue(&hdac1,DAC_CHANNEL_1, DAC_ALIGN_12B_R, ADC_avg_value);	// Записываем в ЦАП среднее значение за 10мс
			TIM2->CCR1 = ADC_avg_value*80000/4095;															// Записываем в регистр PWM сигнала за 10 мс										
			ADC_avg_value = 0;																									// Обнуляем среднее значение за 10мс.
		}	
			
	}	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  //}
  /* USER CODE END 3 */
}

/**
* @brief Функция для настройки генератора частоты , PLL и т.д.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
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
