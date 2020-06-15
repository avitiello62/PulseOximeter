/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"
#include "lcd.h"
#include <stdio.h>
#include "datastructure.h"
#include "mqtt_log.h"
#include <stdlib.h>
#include "ring_buffer.h"
#include "configuration.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern hr_ox_t hr_ox;
extern boot_status conf_status ;
extern char conf_buffer[CONFIGURATION_LENGTH+1];
extern configuration conf;
extern RingBuffer dma_buffer;
static volatile uint32_t _millis;
extern pwm_status pwm;
extern buzzer_status buzzer;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  _millis++;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */

  /* USER CODE END I2C1_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_EV_IRQn 1 */

  /* USER CODE END I2C1_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

  /* USER CODE END I2C1_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c1);
  /* USER CODE BEGIN I2C1_ER_IRQn 1 */

  /* USER CODE END I2C1_ER_IRQn 1 */
}

/**
  * @brief This function handles I2C2 event interrupt.
  */
void I2C2_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_EV_IRQn 0 */

  /* USER CODE END I2C2_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_EV_IRQn 1 */

  /* USER CODE END I2C2_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C2 error interrupt.
  */
void I2C2_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_ER_IRQn 0 */

  /* USER CODE END I2C2_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c2);
  /* USER CODE BEGIN I2C2_ER_IRQn 1 */

  /* USER CODE END I2C2_ER_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/**
  * @brief This function handles I2C3 event interrupt.
  */
void I2C3_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_EV_IRQn 0 */

  /* USER CODE END I2C3_EV_IRQn 0 */
  HAL_I2C_EV_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_EV_IRQn 1 */

  /* USER CODE END I2C3_EV_IRQn 1 */
}

/**
  * @brief This function handles I2C3 error interrupt.
  */
void I2C3_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C3_ER_IRQn 0 */

  /* USER CODE END I2C3_ER_IRQn 0 */
  HAL_I2C_ER_IRQHandler(&hi2c3);
  /* USER CODE BEGIN I2C3_ER_IRQn 1 */

  /* USER CODE END I2C3_ER_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback	(	TIM_HandleTypeDef * 	htim	)	{


	if(htim->Instance==TIM9){
		if(conf_status!=SYSTEM_BOOTED){



			if(conf_status==CONFIGURATION_NOT_LOADED){

				load_conf();
				log_system_configuration_loaded();
			}
			conf_status=SYSTEM_BOOTED;
			log_system_boot();
			log_transmit();
			HAL_TIM_Base_Start_IT(&htim1);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			mqtt_init_communication(conf.mqtt_server);
			lcd_send_status(hr_ox.hr_aggregated,hr_ox.ox_aggregated);
			htim9.Init.Period = 4999;
			if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
			  {
			    Error_Handler();
			  }
			  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_SR_UIF);
		}
		else{
			if(hr_ox.status==SENSOR_ACTIVE){
				hr_ox.hr_aggregated=hr_ox.hr;
				hr_ox.ox_aggregated=hr_ox.ox;
				buzzer_check(hr_ox.hr_aggregated,hr_ox.ox_aggregated);
				lcd_send_status(hr_ox.hr_aggregated,hr_ox.ox_aggregated);
			}
		}
	}
	if(htim->Instance==TIM1){


		if(hr_ox.hr==0)
			hr_ox.ox=0;
		log_status( hr_ox.hr,hr_ox.ox);
		configuration_check_tr(hr_ox);

		log_transmit();


	}
	if(htim->Instance == TIM10){
		 mqtt_set_hearth_rate(hr_ox.hr_aggregated);
		 mqtt_set_oximeter(hr_ox.ox_aggregated);

		 if(send_mqtt()==0){
			 log_mqtt_message_sent();
			 log_transmit();
		 }

	}
	if(htim->Instance == TIM5){
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	}
	if(htim->Instance == TIM11){



		if(hr_ox.status==SENSOR_INACTIVE)
		  {
				hr_ox.status=SENSOR_ACTIVE;
			  HAL_TIM_Base_Start_IT(&htim4);
			  HAL_TIM_Base_Stop_IT(&htim11);
			  HAL_TIM_Base_Start_IT(&htim5);
			  log_system_activated();
			  log_transmit();
			  /*qui bisogna cominciare a leggere i dati dal sensore (funzione che starta tutto)*/

		  }
		  else
		  {
			  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			  HAL_TIM_Base_Stop_IT(&htim4);
			  HAL_TIM_Base_Stop_IT(&htim5);
			  HAL_TIM_Base_Stop_IT(&htim11);
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
			  hr_ox=hr_ox_init();
			  log_system_deactivated();
			  log_transmit();
			  /*qui bisogna mettere in pausa la lettura del sensore qualora fosse in esecuzione(funzione che mette in pausa)*/
		  }
	}
	if(htim->Instance == TIM4){

		MAX30100_read_sensor();
	}
	if (htim->Instance == TIM3) {
			if (htim3.Init.Prescaler == 4799 && buzzer.sound_number_ox > 0) {
				HAL_TIM_Base_Stop_IT(&htim3);
				htim3.Init.Prescaler = 320;
				if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
				  {
				    Error_Handler();
				  }
				  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
				HAL_TIM_Base_Start_IT(&htim3);
			} else if (htim3.Init.Prescaler == 4799
					&& buzzer.sound_number_ox == 0) {
				HAL_TIM_Base_Stop_IT(&htim3);
			}

			if (htim3.Init.Prescaler == 320) {
				if (buzzer.sound_number_hr > 0) {
					if (pwm == PWM_STOPPED) {
						HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
						pwm = PWM_RUNNING;
					} else if (pwm == PWM_RUNNING) {
						HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
						pwm = PWM_STOPPED;
						buzzer.sound_number_hr--;
						if (buzzer.sound_number_hr == 0) {
							HAL_TIM_Base_Stop_IT(&htim3);
							htim3.Init.Prescaler = 4799;
							if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
							  {
							    Error_Handler();
							  }
							  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
							HAL_TIM_Base_Start_IT(&htim3);
						}
					}
				} else if (buzzer.sound_number_ox > 0) {
					if (pwm == PWM_STOPPED) {
						htim2.Instance->CCR1 = 200;
						HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
						pwm = PWM_RUNNING;
					} else if (pwm == PWM_RUNNING) {
						HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
						pwm = PWM_STOPPED;
						buzzer.sound_number_ox--;
						if (buzzer.sound_number_ox == 0) {
							HAL_TIM_Base_Stop_IT(&htim3);
						}
					}
				}
			}
		}

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//sending = 1;
	State state = mqtt.state;
	if(huart->Instance==USART1){

		mqtt_check_state();

		if(mqtt.state==STARTED && state==HANDSHAKE){

			htim10.Init.Prescaler=(uint32_t)(htim10.Init.Prescaler*((float)conf.mqtt_log_period/60.0));
			if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
			  {
			    Error_Handler();
			  }
			  __HAL_TIM_CLEAR_FLAG(&htim10, TIM_SR_UIF);
			HAL_TIM_Base_Start_IT(&htim10);
			log_mqtt_connection_established();
			log_transmit();
		}else if(mqtt.state==MQTT_ERROR ){
			log_mqtt_errors();
			log_transmit();
		}else if(mqtt.state==CONN_ERR){
			log_mqtt_connection_closed();
			log_transmit();
		}
	}
	if(huart->Instance==USART2){
		if(conf_status==CONFIGURATION_NOT_LOADED){
			char conf_array[CONFIGURATION_ATTR_LENGHT];

			char server_array[CONFIGURATION_SEVER_LENGTH+2];
			RingBuffer ring;
			RingBuffer_Init(&ring);
			RingBuffer_Write(&ring, (uint8_t*)conf_buffer, CONFIGURATION_LENGTH);




			conf.hr_low_thresh=read_conf_from_buffer(&ring,conf_array);
			conf.hr_high_thresh=read_conf_from_buffer(&ring,conf_array);


			conf.ox_low_thresh=read_conf_from_buffer(&ring,conf_array);
			conf.ox_high_thresh=read_conf_from_buffer(&ring,conf_array);
			conf.mqtt_log_period=read_conf_from_buffer(&ring,conf_array);

			conf.mqtt_message_period=read_conf_from_buffer(&ring,conf_array);

			RingBuffer_Read(&ring, (uint8_t*)server_array,CONFIGURATION_SEVER_LENGTH);
			server_array[CONFIGURATION_SEVER_LENGTH]='\n';
			server_array[CONFIGURATION_SEVER_LENGTH+1]='\0';
			sprintf(conf.mqtt_server,"%s",(char*)server_array);
			conf.mqtt_server[CONFIGURATION_SEVER_LENGTH+1]='\0';
			conf_status=CONFIGURATION_LOADED_FROM_USER;
			save_conf();
			log_system_configuration_loaded();
			log_transmit();

		}
	}


}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

	//HAL_UART_Receive_DMA(&huart1,(uint8_t*) ACK, 3);

	//MQTT COMMUNICATION requires the sent of an ACK message
	if(huart->Instance==USART1){
		mqtt_get_ack();
	}
}
void millis_init(){
	_millis=0;
	if(SysTick_Config(SystemCoreClock/1000)){
		while(1);
	}
}

uint32_t millis(){
	return _millis;
}
void delay(uint32_t delay){
	static uint32_t delaytime;
	delaytime=millis();
	while(millis()-delaytime<delay);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin==GPIO_PIN_13)
	{
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
			{
				HAL_TIM_Base_Start_IT(&htim11);
			}
			if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
			{
				HAL_TIM_Base_Stop_IT(&htim11);
				__HAL_TIM_SET_COUNTER(&htim11,0);
			}
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
