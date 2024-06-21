/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rc.h"
#include "usart.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_VAL_LEFT_X	adc_value[0]
#define ADC_VAL_LEFT_Y	adc_value[1]
#define ADC_VAL_RIGHT_Y	adc_value[2]
#define ADC_VAL_RIGHT_X	adc_value[3]

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t thread1_Handle;
const osThreadAttr_t thread1_attr = {
  .name = "thread1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal+3,
};
osThreadId_t thread2_Handle;
const osThreadAttr_t thread2_attr = {
  .name = "thread2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal+2,
};
osThreadId_t thread3_Handle;
const osThreadAttr_t thread3_attr = {
  .name = "thread3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal+1,
};
osThreadId_t thread4_Handle;
const osThreadAttr_t thread4_attr = {
  .name = "thread4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


// global variables
uint8_t bt_recv_done_flag = 0;
uint8_t body_control = 1;
uint8_t dir_l = FORWARD;
uint8_t dir_r = FORWARD;
uint8_t rc_state = STATE_STOP;
uint16_t speed_f		   = 500;
uint16_t speed_b		   = 500;
int16_t speed_l		   = 0;
int16_t speed_r		   = 0;
int16_t turn_left_speed_l  = -500;
int16_t turn_left_speed_r  = 500;
int16_t turn_right_speed_l = 500;
int16_t turn_right_speed_r = -500;
uint8_t bt_rx_data[1];
uint8_t rx_data[1];
uint8_t bt_rx_buffer[50] = {0,};
uint8_t rx_buffer[50] = {0,};
uint16_t adc_value[4] = {0,};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void thread1(void *argument);
void thread2(void *argument);
void thread3(void *argument);
void thread4(void *argument);

PUTCHAR_PROTOTYPE{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	static uint8_t bt_rx_cnt = 0;

	if (huart->Instance == USART6) {
		// Bluetooth
		HAL_UART_Receive_IT(&huart6, bt_rx_data, 1);

		if (bt_rx_data[0]=='\n' || bt_rx_data[0]=='\r') {
			bt_rx_buffer[bt_rx_cnt] = '\0';
			HAL_UART_Transmit(&huart6, bt_rx_buffer, bt_rx_cnt, 1000);
			bt_rx_cnt = 0;
			bt_recv_done_flag = 1;
		}
		else {
			bt_rx_buffer[bt_rx_cnt++] = bt_rx_data[0];
		}
	}

	else if (huart->Instance == USART2) {
		// USB
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_data, 1);
		HAL_UART_Transmit(&huart2, (uint8_t *)rx_data, 1, 1000);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

	// Initialize the motors
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	// Initialize the bluetooth and USB DMA
  	// HAL_UART_Receive_DMA(&huart6, (uint8_t *)bt_rx_buffer, 1);
	// HAL_UART_Receive_DMA(&huart2, (uint8_t *)rx_buffer, 1);

	// Initialize the bluetooth and USB Interrupt
	HAL_UART_Receive_IT(&huart6, (uint8_t *)bt_rx_data, 1);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_data, 1);

	printf("RTOS Begin\n");
	
	wheel_stop();

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	// ssd1306_SetCursor(15, 30);
	// ssd1306_WriteString("RC CAR PROJECT", Font_7x10, White);
	// ssd1306_SetCursor(25, 50);
	// ssd1306_WriteString("67 JO 8228", Font_7x10, White);
	// ssd1306_UpdateScreen();

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  thread1_Handle = osThreadNew(thread1, NULL, &thread1_attr);
  thread2_Handle = osThreadNew(thread2, NULL, &thread2_attr);
  thread3_Handle = osThreadNew(thread3, NULL, &thread3_attr);
  thread4_Handle = osThreadNew(thread4, NULL, &thread4_attr);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  	for(;;){
		// Heartbeat
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		osDelay(500);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// input adc value from controller
void thread1(void *argument){
	char *adc[4];

	for(;;){
		if (bt_recv_done_flag == 1) {
			bt_recv_done_flag = 0; // clear flag

			for (uint8_t i = 0; i < 4; i++) {
				adc[i] = strtok( (i==0) ? bt_rx_buffer :NULL , ",");
				adc_value[i] = atoi(adc[i]);
			}

			// printf("ADC: %d\t %d\t %d\t %d\n", adc_value[0], adc_value[1], adc_value[2], adc_value[3]);

			// memset(bt_rx_buffer, 0, sizeof(bt_rx_buffer));
		osDelay(20);	
		}
	}
}

void thread2(void *argument){
	int16_t foward_speed = 0;
	int16_t turn_speed = 0;
	// uint8_t turn_dir;
	// int16_t speed_l = 0;
	// int16_t speed_r = 0;

	for(;;){

		// 직진 속도 계산
		foward_speed = (ADC_VAL_LEFT_Y >> 1) - 1000 ; // -1000 ~ 1000
		turn_speed = (ADC_VAL_RIGHT_X >> 1) - 1000; // -1000 ~ 1000 오른쪽이 큰값

		speed_l = foward_speed + turn_speed ;
		speed_r = foward_speed - turn_speed ;
		
		if (speed_l < 0) {
			dir_l = BACKWARD;
			speed_l = abs(speed_l);
		}
		else {
			dir_l = FORWARD;
		}

		if (speed_r < 0) {
			dir_r = BACKWARD;
			speed_r = abs(speed_r);
		}
		else {
			dir_r = FORWARD;
		}

		if (speed_l > RC_MAX_SPEED) {
			speed_l = RC_MAX_SPEED;
		}
		if (speed_r > RC_MAX_SPEED) {
			speed_r = RC_MAX_SPEED;
		}
		
		if (speed_l < RC_MIN_SPEED+100) {
			speed_l = 0;
		}
		if (speed_r < RC_MIN_SPEED+100) {
			speed_r = 0;
		}

		left_wheel_dir(dir_l);
		right_wheel_dir(dir_r);
		left_wheel_set_speed(speed_l);
		right_wheel_set_speed(speed_r);

		osDelay(20);
	}
}

void thread3(void *argument){
	char tmp[10];
	for(;;){
		__itoa(speed_l, tmp, 10);
		ssd1306_Fill(Black);
		ssd1306_SetCursor(15, 0);
		ssd1306_WriteString(dir_l ? "+" : "-", Font_7x10, White);
		ssd1306_WriteString(tmp, Font_7x10, White);
		__itoa(speed_r, tmp, 10);
		ssd1306_WriteString(" / ", Font_7x10, White);
		ssd1306_WriteString(dir_r ? "+" : "-", Font_7x10, White);
		ssd1306_WriteString(tmp, Font_7x10, White);
		ssd1306_UpdateScreen();
		osDelay(50);
	}
}

void thread4(void *argument){
	for(;;){
			// printf("L: %d\t R: %d\t LEFT: %d\t RIGHT: %d\n",dir_l, dir_r, speed_l, speed_r);

			osDelay(100);
		}
}
/* USER CODE END Application */

