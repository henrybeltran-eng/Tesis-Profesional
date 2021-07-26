/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "ST7735.h"
#include "GFX_FUNCTIONS.h"
#include "image.h"
#include "MPU_9250.h"
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
extern void Logo_Ecci(void);
extern void get_time(void);
extern void Clock_Ecci(void);

void DisplayDatosGyro (float Gyrox, float Gyroy, float Gyroz,uint16_t x, uint16_t y);
void DisplayDatosAccel (float Ax, float Ay, float Az,uint16_t x, uint16_t y);
void DisplayDatosGyroRPY (float Gyrox, float Gyroy, float Gyroz,uint16_t x, uint16_t y);
void DisplayDatosDistance (float DistanceX, float DistanceY, float DistanceZ,uint16_t x, uint16_t y);
void DisplayDatosMag (float Mx, float My, float Mz ,uint16_t x, uint16_t y);
void MPU9250_CalibracionAG(void);
void MPU9250_CalibracionMag(void);
void Menu(int cambio, int variable);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
char buff[100];
extern int iniSendUart;
int cambio = 0, VarMot = 0, VarSleep = 0, Var = 0;
uint8_t sec,min,hour;
uint8_t week_day,day,month,year;
float adc = 0;

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
  while (1)
  {
  }
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

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
	float pulse = 8399;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
	cambio--;
	HAL_TIM_Base_Start_IT(&htim3);
	if(cambio == -1){
		cambio = 4;
	}
	Menu(cambio, 0);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	float pulse = 8399;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
	cambio++;
	HAL_TIM_Base_Start_IT(&htim3);
	if(cambio == 5){
		cambio = 0;
	}
	Menu(cambio, 0);
  for(int i=0;i<500;i++){
  }
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	/*-----------------------Enter----------------
	 * ----------------------Enter----------------
	 * ----------------------Enter----------------
	 */
	fillScreen(BLACK);
	switch (cambio){
		case(1):
			HAL_TIM_Base_Start_IT(&htim2);
			break;
		case(2):
			HAL_TIM_Base_Start_IT(&htim2);
			break;
		case(3):
			MPU9250_CalibracionAG();
			Menu(cambio,0);
			break;
		case(4):
			MPU9250_CalibracionMag();
			if(VarMag != 2){
				fillScreen(BLACK);
				sprintf(Display_Buff,"Continue?");
				ST7735_WriteString(15 ,30 ,Display_Buff, Font_11x18, WHITE,BLACK);
				sprintf(Display_Buff,"Enter: Si");
				ST7735_WriteString(15 ,50 ,Display_Buff, Font_11x18, WHITE,BLACK);
				sprintf(Display_Buff,"Back: NO");
				ST7735_WriteString(15 ,70 ,Display_Buff, Font_11x18, WHITE,BLACK);
			}
			VarMag++;
			if(VarMag > 2){
				VarMag = 0;
				Menu(cambio,0);
			}
			break;
	  }
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	/*-----------------------Back----------------
	 * ----------------------Back----------------
	 * ----------------------Back----------------
	 */
	HAL_TIM_Base_Stop_IT(&htim2);
	Menu(cambio, 0);
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if(cambio == 1){
		MPU9250_Read_Accel();
		MPU9250_Read_Gyro();
		MPU9250_Read_Mag();
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		DisplayDatosGyro (Gx,Gy,Gz,0,18);
		DisplayDatosAccel (Ax,Ay,Az,0,50);
		DisplayDatosMag (Mgx,Mgy,Mgz,0,82);
		if (iniSendUart == 1){
			sprintf(buff,"%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",Gx,Gy,Gz,Ax,Ay,Az,Mgx,Mgy,Mgz);
			HAL_UART_Transmit(&huart1,buff,strlen(buff),10);
		}
	}
	else if( cambio == 2){
		MPU9250_LectureWithFilter();
		sprintf(buff,"%f / %f / %f / %f / %f / %f ",MPU9250Filtered[0],MPU9250Filtered[1],MPU9250Filtered[2],MPU9250Filtered[3],MPU9250Filtered[4], MPU9250Filtered[5]);
		ST7735_WriteString(0,20,buff,Font_7x10, WHITE,BLACK);
	}
	get_time();
	MPU9250_Read_Temp();
	HAL_ADC_Start(&hadc1);
	adc = (HAL_ADC_GetValue(&hadc1)/4096.0*5.0)/3.7*100;
	HAL_ADC_Stop(&hadc1);
	sprintf(buff,"/%.1fC/%0.1fB",Temp,adc);
	ST7735_WriteString(35,0,buff,Font_7x10, WHITE,BLACK);
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
	float pulse = 8399;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
	if (VarMot > 4){
		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		HAL_TIM_Base_Stop_IT(&htim3);
		VarMot = 0;
	}
	else{
		VarMot++;
	}
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
	if (VarSleep > 60 && VarSleep < 80){
		if(Var == 0){
			fillScreen(BLACK);
			for(int i=0;i<64;i++){
			  for(int j=0;j<64;j++){
				 ST7735_DrawPixel(32+i, 50+j, emoji[j][i]);
			  }
			}
			sprintf(Display_Buff," Sleep... ");
			ST7735_WriteString(40 ,115,Display_Buff, Font_7x10, WHITE,BLACK);
			Var++;
		}
		VarSleep++;
	}
	else if(VarSleep > 80){
		HAL_TIM_Base_Stop_IT(&htim4);
		VarSleep = 0;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
		Var = 0;
	}
	else{
		VarSleep++;
	}
	Clock_Ecci();
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
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

/* USER CODE BEGIN 1 */
void DisplayDatosGyro (float Gyrox, float Gyroy, float Gyroz,uint16_t x, uint16_t y){
	sprintf(Display_Buff,"Gx=%.3f deg/s   ",Gyrox);
	ST7735_WriteString(x,y,Display_Buff, Font_7x10, WHITE,BLACK);
	sprintf(Display_Buff,"GY=%.3f deg/s   ",Gyroy);
	ST7735_WriteString(x,y+10,Display_Buff, Font_7x10, WHITE,BLACK);
	sprintf(Display_Buff,"GZ=%.3f deg/s   ",Gyroz);
	ST7735_WriteString(x,y+20,Display_Buff, Font_7x10, WHITE,BLACK);
}

void DisplayDatosAccel (float Ax, float Ay, float Az,uint16_t x, uint16_t y){
	sprintf(Display_Buff,"Ax=%.3f m/s^2   ",Ax);
	ST7735_WriteString(x,y,Display_Buff, Font_7x10, WHITE,BLACK);
	sprintf(Display_Buff,"Ay=%.3f m/s^2   ",Ay);
	ST7735_WriteString(x,y+10,Display_Buff, Font_7x10, WHITE,BLACK);
	sprintf(Display_Buff,"Az=%.3f m/s^2   ",Az);
	ST7735_WriteString(x,y+20,Display_Buff, Font_7x10, WHITE,BLACK);
}

void DisplayDatosMag (float Mx, float My, float Mz, uint16_t x, uint16_t y){
	sprintf(Display_Buff,"Mx=%.3f uT   ",Mx);
	ST7735_WriteString(x,y,Display_Buff, Font_7x10, WHITE,BLACK);
	sprintf(Display_Buff,"My=%.3f uT   ",My);
	ST7735_WriteString(x,y+10,Display_Buff, Font_7x10, WHITE,BLACK);
	sprintf(Display_Buff,"Mz=%.3f uT   ",Mz);
	ST7735_WriteString(x,y+20,Display_Buff, Font_7x10, WHITE,BLACK);
}

void DisplayDatosGyroRPY (float Gyrox, float Gyroy, float Gyroz,uint16_t x, uint16_t y){
	sprintf(Display_Buff,"Raw=%.3f",Gyrox);
	ST7735_WriteString(x,y,Display_Buff, Font_11x18, WHITE,BLACK);
	sprintf(Display_Buff,"Pch=%.3f",Gyroy);
	ST7735_WriteString(x,y+18,Display_Buff, Font_11x18, WHITE,BLACK);
	sprintf(Display_Buff,"Yaw=%.3f",Gyroz);
	ST7735_WriteString(x,y+36,Display_Buff, Font_11x18, WHITE,BLACK);
}
void DisplayDatosDistance (float DistanceX, float DistanceY, float DistanceZ,uint16_t x, uint16_t y){
	sprintf(Display_Buff,"X=%.3f",DistanceX);
	ST7735_WriteString(x,y,Display_Buff, Font_11x18, WHITE,BLACK);
	sprintf(Display_Buff,"Y=%.3f",DistanceY);
	ST7735_WriteString(x,y+18,Display_Buff, Font_11x18, WHITE,BLACK);
	sprintf(Display_Buff,"Z=%.3f",DistanceZ);
	ST7735_WriteString(x,y+36,Display_Buff, Font_11x18, WHITE,BLACK);
}

void Menu(int cambio, int variable){
	int x =32, y=15;
	fillScreen(BLACK);
	HAL_TIM_Base_Stop_IT(&htim4);
	switch (cambio){
		case(0):
			Logo_Ecci();
			break;
		case(1):
			for(int i=0;i<64;i++){
			  for(int j=0;j<64;j++){
				  ST7735_DrawPixel(x+i, y+j, mpu[j][i]);
			  }
			}
			sprintf(Display_Buff," Datos MPU");
			ST7735_WriteString(7 ,90 ,Display_Buff, Font_11x18, WHITE,BLACK);
			break;
		case(2):
			for(int i=0;i<64;i++){
			  for(int j=0;j<64;j++){
				 ST7735_DrawPixel(x+i, y+j, EULER[j][i]);
			  }
			}
			sprintf(Display_Buff," Desplazamiento y        Angulos");
			ST7735_WriteString(0 ,90,Display_Buff, Font_7x10, WHITE,BLACK);
			break;
		case(3):
			for(int i=0;i<64;i++){
			  for(int j=0;j<64;j++){
				 ST7735_DrawPixel(x+i, y+j, calAG[j][i]);
			  }
			}
			sprintf(Display_Buff,"    Calibracion      Acelerometro  y      Giroscopio");
			ST7735_WriteString(0 ,90 ,Display_Buff, Font_7x10, WHITE,BLACK);
			break;
		case(4):
			for(int i=0;i<64;i++){
			  for(int j=0;j<64;j++){
				 ST7735_DrawPixel(x+i, y+j, calMag[j][i]);
			  }
			}
			sprintf(Display_Buff,"    Calibracion       Magnetrometro");
			ST7735_WriteString(0 ,90 ,Display_Buff, Font_7x10, WHITE,BLACK);
			break;
	  }
}

/*void Print_Image(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t data[]){
	for(int i=0;i<64;i++){
	  for(int j=0;j<64;j++){
		  ST7735_DrawPixel(i, j, data[i][j]);
	  }
   }
}*/

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
