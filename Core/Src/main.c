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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bsp_dht21.h"
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
//#define ADC_CHANNEL_CNT 10 	//采样通道数
//#define ADC_CHANNEL_FRE 1	//单个通道采样次数，用来取平均值
//uint16_t adc1_val_buf[10]; //传递给DMA存放多通道采样值的数组
//uint16_t i,j;
//uint32_t adc1_aver_val[ADC_CHANNEL_CNT] = {0}; //保存多通道的平均采样值的数组
//uint32_t dma_cnt1 = 0;

uint32_t ADC_Value[1000];
uint8_t ADC_Convert_Value[20];
uint16_t i;
uint32_t ad1,ad2;
uint32_t  ad3,ad4;
uint32_t ad10,ad5,ad6,ad7,ad8,ad9;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch, FILE *f){
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart1, temp, 1, 2); 
	return ch;
}

#define Log 1
#define USER_MAIN_DEBUG 1

#ifdef USER_MAIN_DEBUG
#define user_main_printf(format, ...) printf( format "\r\n", ##__VA_ARGS__)
#define user_main_info(format, ...) printf("[\tmain]info:" format "\r\n", ##__VA_ARGS__)
#define user_main_debug(format, ...) printf("[\tmain]debug:" format "\r\n", ##__VA_ARGS__)
#define user_main_error(format, ...) printf("[\tmain]error:" format "\r\n",##__VA_ARGS__)
#else
#define user_main_printf(format, ...)
#define user_main_info(format, ...)
#define user_main_debug(format, ...)
#define user_main_error(format, ...)
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	uint16_t AD_Value,AD_Value_get;
	

 
//	uint8_t ADC_Convert_array[2];
//	uint8_t AM_OnlineFlag;

 
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//			HAL_Delay(500);
 
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
//		HAL_Delay(500);
//		for(i = 0,ad1 =0,ad2=0,ad3=0,ad4=0; i < 1000;){
//		ad1 += ADC_Value[i++];
//		ad2 += ADC_Value[i++];
//		ad3 += ADC_Value[i++];
//		ad4 += ADC_Value[i++];
//		}
//		ad1 /= 250;
//		ad2 /= 250;
//		ad3 /= 250;
//		ad4 /= 250;
//		printf("\r\n********ADC-DMA-Example********\r\n");
//		printf("[\tmain]info:AD1_value=%1.3fV\r\n", ad1*3.3f/4095);
//		printf("[\tmain]info:AD2_value=%1.3fV\r\n", ad2*3.3f/4095);
//		printf("[\tmain]info:AD3_value=%1.3fV\r\n", ad3*3.3f/4095);
//		printf("[\tmain]info:AD4_value=%1.3fV\r\n\n", ad4*3.3f/4095);
//						
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
			printf("\r\n********ADC-DMA-Example********\r\n");

//	HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &adc1_val_buf, (ADC_CHANNEL_CNT*ADC_CHANNEL_FRE));
//	HAL_Delay(100);
// 
	 HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_Delay(500);
		
		for(i = 0,ad1 =0,ad2=0,ad3=0,ad4=0; i < 1000;){
		ad1 += ADC_Value[i++];
		ad2 += ADC_Value[i++];
		ad3 += ADC_Value[i++];
		ad4 += ADC_Value[i++];
		ad5 += ADC_Value[i++];
		ad6 += ADC_Value[i++];
		ad7 += ADC_Value[i++];
		ad8 += ADC_Value[i++];
		ad9 += ADC_Value[i++];
		ad10 += ADC_Value[i++];
		}
		ad1 /= 100;
		ad2 /= 100;
		ad3 /= 100;
		ad4 /= 100;
		ad5 /= 100;
		ad6 /= 100;
		ad7 /= 100;
		ad8 /= 100;
		ad9 /= 100;
		ad10 /= 100;
// uint8_t ADC_Convert_Value[10];
		
		ADC_Convert_Value[1]=(ad1>>8) & 0x0f;//high
  	ADC_Convert_Value[0]=ad1&0xff; //low
		
		ADC_Convert_Value[3]=(ad2>>8) & 0x0f;//high
  	ADC_Convert_Value[2]=ad2&0xff; //low
		
		ADC_Convert_Value[5]=(ad3>>8) & 0x0f;//high
  	ADC_Convert_Value[4]=ad3&0xff; //low
		
		ADC_Convert_Value[7]=(ad4>>8) & 0x0f;//high
  	ADC_Convert_Value[6]=ad4&0xff; //low

		ADC_Convert_Value[9]=(ad5>>8) & 0x0f;//high
  	ADC_Convert_Value[8]=ad5&0xff; //low

		ADC_Convert_Value[11]=(ad6>>8) & 0x0f;//high
  	ADC_Convert_Value[10]=ad6&0xff; //low
		
		ADC_Convert_Value[13]=(ad7>>8) & 0x0f;//high
  	ADC_Convert_Value[12]=ad7&0xff; //low
		
		ADC_Convert_Value[15]=(ad8>>8) & 0x0f;//high
  	ADC_Convert_Value[14]=ad8&0xff; //low
		
		ADC_Convert_Value[17]=(ad9>>8) & 0x0f;//high
  	ADC_Convert_Value[16]=ad9&0xff; //low
		
		ADC_Convert_Value[19]=(ad10>>8) & 0x0f;//high
  	ADC_Convert_Value[18]=ad10&0xff; //low
		
		 HAL_SPI_Transmit_DMA (&hspi1, ADC_Convert_Value, 1);
//		printf("\r\n********ADC-DMA-Example********\r\n");
//		printf("[\tmain]info:AD1_value=%1.3fV\r\n", ad1*3.3f/4095);
//		printf("[\tmain]info:AD2_value=%1.3fV\r\n", ad2*3.3f/4095);
//		printf("[\tmain]info:AD3_value=%1.3fV\r\n", ad3*3.3f/4095);
//		printf("[\tmain]info:AD4_value=%1.3fV\r\n", ad4*3.3f/4095);
//		printf("[\tmain]info:AD5_value=%1.3fV\r\n", ad5*3.3f/4095);
//		printf("[\tmain]info:AD6_value=%1.3fV\r\n", ad6*3.3f/4095);
//		printf("[\tmain]info:AD7_value=%1.3fV\r\n", ad7*3.3f/4095);
//		printf("[\tmain]info:AD8_value=%1.3fV\r\n", ad8*3.3f/4095);
//		printf("[\tmain]info:AD9_value=%1.3fV\r\n", ad9*3.3f/4095);
//		printf("[\tmain]info:AD10_value=%1.3fV\r\n\n", ad10*3.3f/4095);										
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	if(hadc==(&hadc1))
//	{
//		dma_cnt1++;
//	}   
//}
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
