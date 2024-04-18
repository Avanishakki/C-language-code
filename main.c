/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
uint8_t Return[2];
uint8_t TxBuf[2];
uint8_t Rx_x[2];
uint8_t Rx_y[2];
uint8_t Rx_z[2];
uint8_t out[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
      TxBuf[0]=0x20;//Address of Register
      TxBuf[1]=0x37;//Data for Register
      HAL_UART_Transmit(&huart4, TxBuf, 2,50);
      //HAL_SPI_Transmit(&hspi1, TxBuf, 2, 50);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High  0

      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
      TxBuf[0]=0x20 | 0x80;//Address of Register
      HAL_UART_Transmit(&huart4, TxBuf, 1,50);
      //HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
      HAL_UART_Receive(&huart4, Return, 1, 50);
      //HAL_SPI_Receive(&hspi1,Return,1,50);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
	  TxBuf[0]=0x29|0x80;  //address of outx_h
	  HAL_UART_Transmit(&huart4, TxBuf, 1,50);
	  HAL_UART_Receive(&huart4, Rx_x, 1, 50);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High
	  //HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
	  //HAL_SPI_Receive(&hspi1, Rx_x, 1, 50);

	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
	  TxBuf[0]=0x2B|0x80;  //address of outy_h
	  HAL_UART_Transmit(&huart4, TxBuf, 1,50);
	  HAL_UART_Receive(&huart4, Rx_y, 1, 50);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High
	  //HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
	//  HAL_SPI_Receive(&hspi1, Rx_y, 1, 50);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
	 TxBuf[0]=0x2D|0x80;  //address of outy_h
	 HAL_UART_Transmit(&huart4, TxBuf, 1,50);
	 HAL_UART_Receive(&huart4, Rx_z, 1, 50);
	 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High

	 sprintf(out,"Rx_x=%d\t ,Rx_y=%d\t ,Rx_z=%d\r\n",Rx_x,Rx_y,Rx_z);

	  if(Rx_x[0]==0 || Rx_y[0]==0 || Rx_y==-1){
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);
	  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,1);

	}

	  else if((-65<Rx_x[0]<-1) || (0<Rx_x[0]<65)){
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
		HAL_Delay(50);
		HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
		HAL_Delay(50);
	}
	else if((Rx_y[0]>3) || (Rx_y[0]<-3)){
	 HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);
	 HAL_Delay(50);
	 HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_14);
	 HAL_Delay(50);
	 HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
	 HAL_Delay(50);
	 HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
	 HAL_Delay(50);
	}



	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);//CS Pin Low
	TxBuf[0]=0x2D|0x80;  //address of outz_h
	//HAL_SPI_Transmit(&hspi1, TxBuf, 1, 50);
	HAL_UART_Transmit(&huart4, TxBuf, 1,50);
	HAL_UART_Receive(&huart4, Rx_z, 1, 50);
	//HAL_SPI_Receive(&hspi1, Rx_z, 1, 50);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);//CS Pin High
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
