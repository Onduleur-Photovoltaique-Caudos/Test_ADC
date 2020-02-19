/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_ll_adc.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "Command.h"
#include "Loop.h"
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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define ADC_STEPS 4096
#define ADC_FULL_MEASURE_MV (VREFINT_CAL_VREF * 1.0f)
#define RESISTOR_400V 4422
#define RESISTOR_200V 4222
#define RESISTOR_3V 22


unsigned short * pRefInt = (((uint16_t*)((uint32_t)0x1FFFF7BAU)));
unsigned short sRefInt;
float mvCorrectionFactor = 1.0;
const float mvFactor0 = (float)VREFINT_CAL_VREF / ADC_STEPS;
const float mvFactor1 = (float)VREFINT_CAL_VREF / ADC_STEPS * RESISTOR_400V / RESISTOR_3V;
const float mvFactor2 = (float)VREFINT_CAL_VREF / ADC_STEPS * RESISTOR_200V / RESISTOR_3V;

//const float iOffset = 2344;  // experimentaly calibrated (temperature dependend!)
const float iOffset = 2500;// / (float)VREFINT_CAL_VREF * ADC_STEPS * 6.8f / 9;

const float iDivider = 9.0f / 6.8f; // values of resistor bridge for current
const float iFactor1 = 1.0f /100.0f;   // for 20A model 100mV/A
const float iFactor2 = 1.0f / 185.0f;   // for 5A model 185mv/A    



volatile float fM_VIN, fM_V225, fM_IHFL, fM_VOUT1, fM_VOUT2, fM_Temp, fM_Vref;
volatile float fM_V175, fM_IOUT, fM_IH1, fM_IH2, fM_IIN, fM_I175, fM_I225;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us_DWT(int uSec)
{
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*uSec;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}

#define ADC_BUFFERM_LENGTH 7
uint16_t g_ADCBufferM[ADC_BUFFERM_LENGTH * 2];
uint16_t * pM_VIN  =  &g_ADCBufferM[0];
uint16_t * pM_V175  = &g_ADCBufferM[1];
uint16_t * pM_V225  = &g_ADCBufferM[2];
uint16_t * pM_IOUT  = &g_ADCBufferM[3];
uint16_t * pM_IHFL  = &g_ADCBufferM[4];
uint16_t * pM_IH1   = &g_ADCBufferM[5];
uint16_t * pM_VOUT1 = &g_ADCBufferM[6];
uint16_t * pM_IH2   = &g_ADCBufferM[7];
uint16_t * pM_VOUT2 = &g_ADCBufferM[8];
uint16_t * pM_IIN   = &g_ADCBufferM[9];
uint16_t * pM_Temp  = &g_ADCBufferM[10];
uint16_t * pM_I175  = &g_ADCBufferM[11];
uint16_t * pM_Vref  = &g_ADCBufferM[12];
uint16_t * pM_I225  = &g_ADCBufferM[13];
// store previous values
unsigned short g_ADCOld[14];
uint16_t * oM_VIN = &g_ADCOld[0];
uint16_t * oM_V175 = &g_ADCOld[1];
uint16_t * oM_V225 = &g_ADCOld[2];
uint16_t * oM_IOUT = &g_ADCOld[3];
uint16_t * oM_IHFL = &g_ADCOld[4];
uint16_t * oM_IH1 = &g_ADCOld[5];
uint16_t * oM_VOUT1 = &g_ADCOld[6];
uint16_t * oM_IH2 = &g_ADCOld[7];
uint16_t * oM_VOUT2 = &g_ADCOld[8];
uint16_t * oM_IIN = &g_ADCOld[9];
uint16_t * oM_Temp = &g_ADCOld[10];
uint16_t * oM_I175 = &g_ADCOld[11];
uint16_t * oM_Vref = &g_ADCOld[12];
uint16_t * oM_I225 = &g_ADCOld[13];
 
//#define VALIDATE(newValue,oldValue) ((newValue) != 0 ? (oldValue) = ((newValue) > 0 && (newValue) > (oldValue)*9/10 && (newValue) < (oldValue)*11/10? (newValue):(oldValue)):(oldValue))
#define VALIDATE(newValue,oldValue) ((oldValue) = (newValue))


int g_MeasurementNumber=0;
int g_DMACount = 0;
bool bErrorADC;
bool bErrorDMA;

void   HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) // data overrun
{
	if (hadc->ErrorCode == HAL_ADC_ERROR_DMA) {
		bErrorDMA = true;
	} else if (hadc->ErrorCode == HAL_ADC_ERROR_OVR) {
		bErrorADC = 1;
	} else {
		bErrorADC = 2;
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* adcHandle)
{
	HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET); 
}
volatile static bool doneADC;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle){// end of DMA
	HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_SET);   // Pin 5 of serial connector
	g_MeasurementNumber++;

	VALIDATE(*pM_VIN, *oM_VIN);
	fM_VIN = (*oM_VIN) *mvFactor1  * mvCorrectionFactor;  		// test at P21 -- ok pas de bruit
	VALIDATE(*pM_V225, *oM_V225);
	fM_V225 = (*oM_V225) *mvFactor2  * mvCorrectionFactor;  	// test at P3  -- ok pas de bruit
	VALIDATE(*pM_IHFL, *oM_IHFL);
	fM_IHFL = ((*oM_IHFL) - iOffset) *iDivider * mvCorrectionFactor;   	// test at U11 pin 1 -- error lit sur VOUT1
	VALIDATE(*pM_VOUT1, *oM_VOUT1);
	fM_VOUT1 = (*oM_VOUT1) *mvFactor1 * mvCorrectionFactor;  			// ADC1 IN11 test at P20 -- OK
	VALIDATE(*pM_VOUT2, *oM_VOUT2);
	fM_VOUT2 = (*oM_VOUT2) *mvFactor1 * mvCorrectionFactor;  			// test at P18 -- OK
	VALIDATE(*pM_Temp, *oM_Temp);
	fM_Temp = 1.0f * ((*TEMPSENSOR_CAL1_ADDR) - (*oM_Temp)) * (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP) / ((*TEMPSENSOR_CAL1_ADDR) - (*TEMPSENSOR_CAL2_ADDR)) + TEMPSENSOR_CAL1_TEMP; 
	; 
	VALIDATE(*pM_Vref, *oM_Vref);
	fM_Vref = (*oM_Vref) * mvFactor0;
	sRefInt = *pRefInt;
	mvCorrectionFactor =  0.01f*(* VREFINT_CAL_ADDR) / (*oM_Vref) + 0.99f * mvCorrectionFactor;
	// ADC2
	VALIDATE(*pM_V175, *oM_V175);
	fM_V175 = (*oM_V175) *mvFactor2  * mvCorrectionFactor;  			// test at P4  -- ok 
	VALIDATE(*pM_IOUT, *oM_IOUT);
	fM_IOUT = ((*oM_IOUT)*mvFactor0  * mvCorrectionFactor * iDivider - iOffset) * iFactor1;     	// test at P13 pin2 (range 5V) -- bruite error, pas connecte essayer R43 R44
	VALIDATE(*pM_IH1, *oM_IH1);
	fM_IH1 = ((*oM_IH1) - iOffset) *iDivider* mvCorrectionFactor;   		// test at U20 pin1 -- error lit sur IOUT
	VALIDATE(*pM_IH2, *oM_IH2);
	fM_IH2 = ((*oM_IH2) - iOffset) *iDivider* mvCorrectionFactor;   		// test at U17 pin1 -- OK
	VALIDATE(*pM_IIN, *oM_IIN);
	fM_IIN = ((*oM_IIN)*mvFactor0  * mvCorrectionFactor * iDivider - iOffset) * iFactor1;   	// test at P16 pin2 range 5V --OK
	VALIDATE(*pM_I175, *oM_I175);
	fM_I175 = ((*oM_I175)*mvFactor0  * mvCorrectionFactor * iDivider - iOffset) * iFactor2;    	// test at P10 pin2 range 5V --OK
	VALIDATE(*pM_I225, *oM_I225);
	fM_I225 = ((*oM_I225)*mvFactor0  * mvCorrectionFactor * iDivider - iOffset) * iFactor2;    	// test at P9 pin2 range 5V -- OK

	HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_RESET);
	doneADC = true;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {
		if (doneADC) {
			doneADC = false;
			HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_SET);
			delay_us_DWT(0);
			HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_RESET);
			return;
		}
		HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_RESET);
	} else if (htim == &htim3){
		HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_SET);
		//delay_us_DWT(1);
		HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_RESET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2) {
		HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_SET);
		delay_us_DWT(0);
		HAL_GPIO_WritePin(Sync_GPIO_Port, Sync_Pin, GPIO_PIN_RESET);
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
	DWT->CYCCNT = 0;
  // Enable hi resolution counter 
	DWT->CTRL &= ~0x00000001;
	DWT->CTRL |= 0x00000001;
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
  MX_ADC2_Init();
  MX_CRC_Init();
  MX_HRTIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  HAL_Delay(2); // must wait for UART to initialize
  /* USER CODE BEGIN 2 */
  
	initializeCommand();
	HAL_Delay(2);	// wait for ADV voltage regulator to settle before calibrating (10us)
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_Delay(2);
	// tim3
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	
	// tim2 for ADC DMA
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	//HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3);      // sync pulse on the P8 (PA9) (repurposed) connector

	setTempThreshold(45);
	HAL_ADC_Start(&hadc2);
	HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*) g_ADCBufferM, ADC_BUFFERM_LENGTH);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  doLoop();
	  //HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	  //HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET); 
	  HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  while(1) 
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
