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
#include "can.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "i2c-lcd.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//    BOARD 5    //

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailBox;
char CanMsg[8] ={"Mathias\0"};
char comMsg[50];
uint16_t OwnID = 0x124, RemoteID = 0x123;
uint32_t startTime = 0, stopTime = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void serialMsg(char msg[]);
void serialClear(void);
void HAL_CAN_RxFifo0FullCallBack(CAN_HandleTypeDef *hcan);
void CAN_filterConfig(void);
void CAN_Tx(char msg[]);
void CAN_Rx(void);
void delay(uint16_t delay);
void startMsg(void);
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_I2C3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //setup the CAN interface
  CAN_filterConfig();
  HAL_CAN_Start(&hcan1);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* USER CODE END 2 */

<<<<<<< HEAD
  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
=======
>>>>>>> d9629ff5028a8f59c560b620084442843eb5aa73
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  startMsg();
  while (1)
  {

		  HAL_Delay(1000);
		  CAN_Rx();



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

<<<<<<< HEAD
  /** Initializes the CPU, AHB and APB busses clocks
=======
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
>>>>>>> d9629ff5028a8f59c560b620084442843eb5aa73
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
<<<<<<< HEAD
  /** Initializes the CPU, AHB and APB busses clocks
=======
  /** Initializes the CPU, AHB and APB buses clocks
>>>>>>> d9629ff5028a8f59c560b620084442843eb5aa73
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
<<<<<<< HEAD
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
=======
>>>>>>> d9629ff5028a8f59c560b620084442843eb5aa73
}

/* USER CODE BEGIN 4 */
void startMsg(void){

	  serialClear();
	  lcd_init();
	  lcd_put_cur(0,0);
	  serialMsg("***Init!***\r\n");
	  lcd_send_string("***Init!***");
	  HAL_Delay(1000);
	  serialClear();
	  lcd_clear();
}

void serialMsg(char msg[]){
	char msgEnd[] = "\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*)msgEnd, strlen(msgEnd), HAL_MAX_DELAY);

}

void serialClear(void){
	serialMsg("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
}

void HAL_CAN_RxFifo0FullCallBack(CAN_HandleTypeDef *hcan){
	if(hcan) return;
}

void CAN_Tx(char msg[]){

	if(sizeof(*msg)<=8){
		TxHeader.DLC = 8;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.StdId = OwnID;


		if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, (uint8_t*)msg, &TxMailBox) != HAL_OK) Error_Handler();

		while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailBox));
		//serialMsg("Message transmitted!\n\r");
	}
	else return;
}

void CAN_Rx(void){

	char str[8];
	char msg[8];
	uint8_t crx[8];
	RxHeader.DLC = 8;
	RxHeader.IDE = CAN_ID_EXT;
	RxHeader.RTR = CAN_RTR_DATA;
	RxHeader.ExtId = RemoteID;


	//If there is no message
	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, crx) != HAL_OK){
		serialMsg("CAN_Rx fault\n\r");
		return;
	}
	sprintf(msg, "%x", (unsigned int)crx);
	serialMsg(msg);
	sprintf(str, "%x", RxHeader.ExtId);
	serialMsg(str);
}


void CAN_filterConfig(void){
	CAN_FilterTypeDef filterConfig;
	filterConfig.FilterBank = 0;
	filterConfig.FilterActivation = ENABLE;
	filterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filterConfig.FilterIdHigh = 0x0000;
	filterConfig.FilterIdLow = 0x0000;
	filterConfig.FilterMaskIdHigh = 0x0000;
	filterConfig.FilterMaskIdLow = 0x0000;
	filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	filterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

	HAL_CAN_ConfigFilter(&hcan1, &filterConfig);
}


/* user delay function */
void delay(uint16_t delay){ //one microsecond
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<delay);
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
	serialMsg("Error happened!\r\n");
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
