/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "Max31865.h"
#include <string.h>
#include <stdio.h>
#include "adc.h"
#include "math.h"
#include "ADXL.h"
#include "stdbool.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
ADXL_InitTypeDef * adxl;
adxlStatus adxlStat;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId TP100Handle;
osThreadId TBreakOutHandle;
osThreadId ACC_ReadHandle;
osThreadId Vibration_TestHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void TP100_Test(void const * argument);
void TBreakOut_Init(void const * argument);
void ACC_Read_Init(void const * argument);
void Vibration_Test_Init(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of TP100 */
  osThreadDef(TP100, TP100_Test, osPriorityNormal, 0, 128);
  TP100Handle = osThreadCreate(osThread(TP100), NULL);

  /* definition and creation of TBreakOut */
  osThreadDef(TBreakOut, TBreakOut_Init, osPriorityNormal, 0, 128);
  TBreakOutHandle = osThreadCreate(osThread(TBreakOut), NULL);

  /* definition and creation of ACC_Read */

 // osThreadDef(ACC_Read, ACC_Read_Init, osPriorityNormal, 0, 128);
  //ACC_ReadHandle = osThreadCreate(osThread(ACC_Read), NULL);

  /* definition and creation of Vibration_Test */
  //osThreadDef(Vibration_Test, Vibration_Test_Init, osPriorityNormal, 0, 128);
  //Vibration_TestHandle = osThreadCreate(osThread(Vibration_Test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TP100_Test */
/**
* @brief Function implementing the TP100 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TP100_Test */
void TP100_Test(void const * argument)
{
  /* USER CODE BEGIN TP100_Test */
	Max31865_t  pt100;
	bool        pt100isOK;
	float       pt100Temp;
	Max31865_init(&pt100,&hspi2,GPIOC,GPIO_PIN_1,2,50);
	char UART_SEND[30];
  /* Infinite loop */
  for(;;)
  {
	  	 float t;
	     pt100isOK = Max31865_readTempC(&pt100,&t);
	     pt100Temp = Max31865_Filter(t,pt100Temp,1);   //  << For Smoothing data
	     HAL_Delay(1000);
	     sprintf(UART_SEND,"Temp is %i °C \r\n",(uint8_t)pt100Temp);
	     HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);

	     if(pt100isOK != true){
	    	 UART_SEND[20] = "Sensor Failure\r\n";
	    	 HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);
	    	 HAL_Delay(500);
	     }
	    osDelay(1);
  }
  /* USER CODE END TP100_Test */
}

/* USER CODE BEGIN Header_TBreakOut_Init */
/**
* @brief Function implementing the TBreakOut thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TBreakOut_Init */
void TBreakOut_Init(void const * argument)
{
  /* USER CODE BEGIN TBreakOut_Init */
	const double balance_resistance = 72600;
	const double adc_res = 4096;
	const double beta_value = 20564;
	const double room_tempK = 294.15;
	const double room_tempR = 70000;
//---------------------------------------------------//
	volatile uint32_t value_analog;
	volatile double resistance_thermistor;
	volatile double temp_kelvin;
	volatile double temp_celcius;

	char UART_SEND[30];
  /* Infinite loop */
  for(;;)
  {

	  HAL_ADC_Start(&hadc1);

	  if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK){
		  	   	value_analog = HAL_ADC_GetValue(&hadc1);
	      }

	  resistance_thermistor = balance_resistance * ((adc_res/value_analog)-1);

	  temp_kelvin = (beta_value * room_tempK) / (beta_value + (room_tempK * log(resistance_thermistor/room_tempR)));

	  temp_celcius = temp_kelvin - 273.15;

	  sprintf(UART_SEND,"NTC Temp is %d °C \r\n",(uint8_t)temp_celcius);
	  HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	//  UART_SEND[strlen(UART_SEND)]=0;
    osDelay(1);
  }
  /* USER CODE END TBreakOut_Init */
}

/* USER CODE BEGIN Header_ACC_Read_Init */
/**
* @brief Function implementing the ACC_Read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ACC_Read_Init */
void ACC_Read_Init(void const * argument)
{
  /* USER CODE BEGIN ACC_Read_Init */
	struct adxl adxl1;
	adxl1.SPIMode = SPIMODE_4WIRE;
	adxl1.LPMode = LPMODE_NORMAL;
	adxl1.Rate = BWRATE_100;
	adxl1.Range = RANGE_8G;									//Initial configuration of the SPI and MODULE
	adxl1.Resolution = RESOLUTION_10BIT;
	adxl1.Justify = JUSTIFY_MSB;
	adxl1.AutoSleep = AUTOSLEEPOFF;
	adxl1.LinkMode = LINKMODEOFF;

	if(ADXL_Init(&adxl1) != ADXL_OK){


		 HAL_UART_Transmit(&huart2, "ACC Unit Failure", 16, HAL_MAX_DELAY);

	}
				//Sending configuration to the module

	float test_value=0;
	float acc_value[3];
	//char UART_SEND[30];
  /* Infinite loop */
  for(;;)
  {
	  ADXL_getAccel(&acc_value, test_value);
	  /*
	  sprintf(UART_SEND,"Acc X is %f °C \r\n",(double)acc_value[0]);
	  HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);
	  HAL_Delay(600);

	  sprintf(UART_SEND,"Acc Y is %f °C \r\n",(double)acc_value[1]);
	  HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);
	  HAL_Delay(20);


	  sprintf(UART_SEND,"Acc Z is %f °C \r\n",(double)acc_value[2]);
	  HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);
	  HAL_Delay(20);
	  */
	//HAL_Delay(1000);
	osDelay(1);
  }
  /* USER CODE END ACC_Read_Init */
}

/* USER CODE BEGIN Header_Vibration_Test_Init */
/**
* @brief Function implementing the Vibration_Test thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Vibration_Test_Init */
void Vibration_Test_Init(void const * argument)
{
  /* USER CODE BEGIN Vibration_Test_Init */
  /* Infinite loop */




	bool ifShock = false;
	uint8_t shockVal = 1;
	volatile uint16_t lastAlarm = 250;
	char UART_SEND[30];
	char ALARM[20] = "Shock detected";
  for(;;)
  {
	  /*shockVal = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

	  	  if (shockVal == 0){

	  		lastAlarm = TIM3->CNT;		//Reading the timer value from the register
	  		TIM3->ARR;					//Resetting the timer register value

	  		sprintf(UART_SEND,"Time since last shock %i ms \r\n",(uint16_t)lastAlarm);
	  		HAL_UART_Transmit(&huart2, (uint8_t*)UART_SEND, strlen(UART_SEND), HAL_MAX_DELAY);
	  			if(!ifShock){

	  				HAL_UART_Transmit(&huart2, (uint8_t*)ALARM, strlen(ALARM), HAL_MAX_DELAY);
	  			}
	  	  }

*/
    osDelay(1);
  }

  /* USER CODE END Vibration_Test_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
