
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int inited=0;
uint32_t test=0;
int SENSOR_EXT_Flag=0;
uint16_t timeSENSOR=0;
uint16_t distanceL=9999,distanceR=9999;
int8_t BluetoothData[2]={0,0};	//Roll,Pitch
uint16_t speedleft = 0;
uint16_t speedright = 0;
uint16_t dirleft=0;
uint16_t dirrightt=0;


uint16_t MOTOR_LEFT = 0xAA;
uint16_t MOTOR_RIGHT = 0xBB;
uint16_t MOTOR_FORWARD = 0xAA;
uint16_t MOTOR_BACKWARD = 0xBB;
uint16_t MOTOR_STOP = 0xCC;
int Change_Speed(uint16_t MOTOR,uint16_t DIRECTION,uint16_t SPEED){
	if(distanceL < 50 || distanceR<50){
		HAL_GPIO_WritePin(MOTOR_R_BACKWARD_GPIO_Port,MOTOR_R_BACKWARD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_R_FORWARD_GPIO_Port,MOTOR_R_FORWARD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_L_BACKWARD_GPIO_Port,MOTOR_L_BACKWARD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_L_FORWARD_GPIO_Port,MOTOR_L_FORWARD_Pin,GPIO_PIN_RESET);
		speedleft = 0;
		speedright = 0;
		return -1;
	}
	if(DIRECTION == MOTOR_STOP){
		HAL_GPIO_WritePin(MOTOR_R_BACKWARD_GPIO_Port,MOTOR_R_BACKWARD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_R_FORWARD_GPIO_Port,MOTOR_R_FORWARD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_L_BACKWARD_GPIO_Port,MOTOR_L_BACKWARD_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MOTOR_L_FORWARD_GPIO_Port,MOTOR_L_FORWARD_Pin,GPIO_PIN_RESET);
		speedleft = 0;
		speedright = 0;
		return 0;
	}
	if(MOTOR != MOTOR_LEFT && MOTOR != MOTOR_RIGHT ) return -1;
	if(DIRECTION != MOTOR_BACKWARD && DIRECTION != MOTOR_FORWARD ) return -1;
	if(SPEED > 100 || SPEED <0) return -1;
	if(MOTOR == MOTOR_RIGHT){
		TIM3->CCR1 = speedright;
		if(DIRECTION == MOTOR_FORWARD){
			HAL_GPIO_WritePin(MOTOR_R_BACKWARD_GPIO_Port,MOTOR_R_BACKWARD_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_R_FORWARD_GPIO_Port,MOTOR_R_FORWARD_Pin,GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(MOTOR_R_FORWARD_GPIO_Port,MOTOR_R_FORWARD_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_R_BACKWARD_GPIO_Port,MOTOR_R_BACKWARD_Pin,GPIO_PIN_SET);
		}
	}
	else{
		TIM3->CCR2 = speedright;
		if(DIRECTION == MOTOR_FORWARD){
			HAL_GPIO_WritePin(MOTOR_L_BACKWARD_GPIO_Port,MOTOR_L_BACKWARD_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_L_FORWARD_GPIO_Port,MOTOR_L_FORWARD_Pin,GPIO_PIN_SET);
		}
		else{
			HAL_GPIO_WritePin(MOTOR_L_FORWARD_GPIO_Port,MOTOR_L_FORWARD_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_L_BACKWARD_GPIO_Port,MOTOR_L_BACKWARD_Pin,GPIO_PIN_SET);
		}
	}
	return 0;
}


uint32_t getUs(void) {

	uint32_t usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
	register uint32_t ms, cycle_cnt;
	do {
		ms = HAL_GetTick();
		cycle_cnt = SysTick->VAL;
	} while (ms != HAL_GetTick());
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

void delayUs(uint16_t micros) {
	uint32_t start = getUs();
	while (getUs() - start < (uint32_t) micros) {
		asm("nop");
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){
		timeSENSOR=65000;
		HAL_TIM_Base_Stop(&htim2);
		TIM2->CNT=0;
		SENSOR_EXT_Flag=4;

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(!inited)return;
	if(GPIO_Pin==echoprawe_Pin){
		if(HAL_GPIO_ReadPin(echoprawe_GPIO_Port,echoprawe_Pin)==GPIO_PIN_SET){
			HAL_TIM_Base_Start(&htim2);
			SENSOR_EXT_Flag=3;
		}
		else{
			timeSENSOR=TIM2->CNT;
			HAL_TIM_Base_Stop(&htim2);
			TIM2->CNT=0;
			SENSOR_EXT_Flag=4;
		}
	}
	else if(GPIO_Pin == echolewe_Pin){
		if(HAL_GPIO_ReadPin(echolewe_GPIO_Port,echolewe_Pin)==GPIO_PIN_SET){
			HAL_TIM_Base_Start(&htim2);
			SENSOR_EXT_Flag=3;
		}
		else{
			timeSENSOR=TIM2->CNT;
			HAL_TIM_Base_Stop(&htim2);
			TIM2->CNT=0;
			SENSOR_EXT_Flag=4;
		}
	}
}
uint32_t Measure_Right(){
  	HAL_GPIO_WritePin(trigprawy_GPIO_Port,trigprawy_Pin,GPIO_PIN_SET);
	delayUs(10);
	HAL_GPIO_WritePin(trigprawy_GPIO_Port,trigprawy_Pin,GPIO_PIN_RESET);

	while(SENSOR_EXT_Flag!=4);
	distanceR=timeSENSOR/58;
	return distanceR;
}
uint32_t Measure_Left(){
  	HAL_GPIO_WritePin(triglewy_GPIO_Port,triglewy_Pin,GPIO_PIN_SET);
	delayUs(10);
	HAL_GPIO_WritePin(triglewy_GPIO_Port,triglewy_Pin,GPIO_PIN_RESET);

	while(SENSOR_EXT_Flag!=4);
	distanceL=timeSENSOR/58;
	return distanceL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(led1_GPIO_Port,led1_Pin);
  if(huart==&huart3){
	  HAL_UART_Receive_IT(&huart3,&BluetoothData,2);
  }
}

void AnglesToSpeed(){

}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  inited=1;
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

  HAL_UART_Receive_IT(&huart3,&BluetoothData,2);
  speedleft=20;
  speedright=20;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  Measure_Left();
//	  Measure_Right();
//	  AnglesToSpeed();
	  speedleft=20;
	  speedright=20;
	  if(BluetoothData[0] < -20 ){
		  dirleft = MOTOR_BACKWARD;
		  dirrightt = MOTOR_FORWARD;
	  }
	  else if(BluetoothData[0] > 20 ){
	  		  dirleft = MOTOR_FORWARD;
	  		  dirrightt = MOTOR_BACKWARD;
	  }
	  else if(BluetoothData[1] > 20 ){
	  		  dirleft = MOTOR_FORWARD;
	  		  dirrightt = MOTOR_FORWARD;
	  }
	  else if(BluetoothData[1] < -20 ){
	  	  		  dirleft = MOTOR_BACKWARD;
	  	  		  dirrightt = MOTOR_BACKWARD;
	  	  }
	  else{
		  speedleft=0;
		  speedright=0;
	  }
	  Change_Speed(MOTOR_LEFT,dirleft,speedleft);
	  Change_Speed(MOTOR_RIGHT,dirrightt,speedright);

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
