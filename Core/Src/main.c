
/* USER CODE BEGIN Header */
/**
   ******************************************************************************
  * FILE           : main.c
  * PROJECT        : thermistor
  * PROGRAMMERS     :Chalumuri Saikishore
  * FIRST VERSION  : 2021-01-11
  * DESCRIPTION    : Code for embedded system implementing the multi-access-point
  *                  thermometer by using the STM32L432 Nucleo-32 board.This
  *                  multi-access point thermometer can be used in hospitals to
  *                  monitor patients’temperatures from different location.This
  *                  embedded software determines the temperature of patient using
  *                  Mcp9700 thermistor and converted it in to degree Celsius by
  *                  using the voltagetotempconv functionThe temperature values are displayed on
  *                  OLED ssd1331  and putty. The RGB led light indicates different
  *                  colors when the temperature are in different region. When the
  *                  temperature is not in the desired region an appropriate message
  *                  is shown in the putty screen.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1331.h"// header file for oled display inbuilt it contins oled functions
#include "fonts.h"
#include<stdio.h>
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void temploop(float temp );// all the above are function declrations of functions used in the code
float cntToVoltage(void);
float VoltageToTemp(float voltage);
void ledOn(float temp);
			;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
float temp;
float voltage;
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ssd1331_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, SET);
	    	  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, SET);
    	  	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
	        voltage = cntToVoltage(); //this is a function which converts into voltage by using thermistor and ON BOARD adc
	        temp = VoltageToTemp(voltage);// this is a function which converts the output voltage of adc into temprature
	        temploop(temp );// this is function to display the particular temprature on the board
	        ledOn(temp);// based on the temprature we need to indicate a particular led so this function turns on an led based on a situation

	        HAL_Delay(3000);


	// ssd1331_display_string(4, 8, " PATIENT ", FONT_1608, GREEN);
	 //ssd1331_display_string(7, 8, "TEMP=", FONT_1608, GREEN);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|SSD1331_CS_Pin|SSD1331_DC_Pin|SSD1331_RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin SSD1331_CS_Pin SSD1331_DC_Pin SSD1331_RES_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|SSD1331_CS_Pin|SSD1331_DC_Pin|SSD1331_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// this is a function which is used to get the voltage from the


float cntToVoltage(void){
	uint32_t adcValue;
	float vDetected ;
	printf(" step 1:Checking Serial Connection \r\n");
	printf("________________________________\r\n");
	HAL_ADC_Start(&hadc1);// this used to enable the adc pheripheral
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);// enable the conversion of adc
	adcValue = HAL_ADC_GetValue(&hadc1);// to get the digital value of adc pheripheral we use this statement
	vDetected =  (3.3*adcValue)/4096;// converting the digital value into voltage
	printf("step 2: Voltage detected is %.2f\r\n",vDetected);
	printf("________________________________\r\n");
	return vDetected;
}
// By using the function convert the voltage to temprature
float VoltageToTemp(float voltage){

	float temp;
	printf("________________________________\r\n");
	printf("step 3:the voltage is  %f\r\n",voltage);
	temp = 100*(voltage-0.5);// formulae to convert voltage to temprature
	printf("step 4:Temperature detected is %.2f\r\n",temp);
	printf("________________________________\r\n");

	return temp;

}
// THIS FUNCTION IS USED TO SWITCH ON LED
void ledOn(float temp){
	// printf("end of %s\r\n",__func__);
	if(temp >= -40 && temp < 20.5)            //Blue  PA10
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET);
	else if (temp >= 20.5 && temp < 37.3)  //Green PA9
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
	else if (temp >= 39.5 && temp < 150)  //Red on PA8
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
	else if (temp >= 37.3 && temp < 39.5)  //Orange
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, RESET);
	}

}

void temploop(float temp )// THIS FUNCTION USED TO DISPLAY TEMPRATURE ON TO THE LED SCREEN
{

					                             char stemp[20];//
					                                ssd1331_display_string(2, 8, "WELCOME ", FONT_1608, GREEN);
										 	 	    HAL_Delay(50);
										  	        ssd1331_clear_screen(BLACK ) ;
										 	       ssd1331_display_string(2, 8, "HEAD NURSE  ", FONT_1608, GREEN);
										 		   HAL_Delay(50);
										 		   ssd1331_clear_screen(BLACK ) ;

										           ssd1331_display_string(2, 8, "THE ", FONT_1608, GREEN);
										          HAL_Delay(50);
										          ssd1331_clear_screen(BLACK ) ;
										          ssd1331_display_string(2, 8, "PATIENT ", FONT_1608, GREEN);
										          ssd1331_clear_screen(BLACK ) ;
										          HAL_Delay(50);
										          ssd1331_display_string(2, 8, "TEMPRATURE IS ", FONT_1608, GREEN);
										          ssd1331_clear_screen(BLACK );
										          HAL_Delay(100);
					                              sprintf(stemp,"%.2f",temp);     //Converting float to string and storing it in stemp variable.
					                                ssd1331_display_string(2, 8, stemp , FONT_1608,GREEN);// code to display the stemp on to the screen

		        // ssd1331_display_num(2, 8, temp, 2, FONT_1608,GREEN);
			      // ssd1331_clear_screen(BLACK ) ;


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
