/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM1, RH1, TEMP1;
char tstr1[25], rstr1[25];

float Temperature1 = 0;
float Humidity1 = 0;
uint8_t Presence1 = 0;

uint16_t SUM2, RH2, TEMP2;
char tstr2[25], rstr2[25];
    float Temperature2 = 0;
float Humidity2 = 0;
uint8_t Presence2 = 0;

void delay (uint16_t time)
{
/* change your code here for the delay in microseconds */
__HAL_TIM_SET_COUNTER(&htim1, 0);
while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}


//Initialize the pin as GPIO output
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = GPIO_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
//Initialize GPIO pin as input
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Pin = GPIO_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


//Start communication between the MCU and DHT11
void DHT11_Start(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
   Set_Pin_Output(GPIOx, GPIO_Pin);
   HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
   delay(18000);
   HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
   delay(20);
   Set_Pin_Input(GPIOx, GPIO_Pin);
}

//Response from the DHT11
uint8_t DHT11_Check_Response (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
uint8_t Response = 0;
delay (40);
if (!(HAL_GPIO_ReadPin (GPIOx, GPIO_Pin)))
{
delay (80);
if ((HAL_GPIO_ReadPin (GPIOx, GPIO_Pin))) Response = 1;
else Response = -1; // 255
}
while ((HAL_GPIO_ReadPin (GPIOx, GPIO_Pin)));   // wait for the pin to go low

return Response;
}

//Read the temperature and humidity from DHT11
uint8_t DHT11_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
   uint8_t i, j;
   //uint8_t data = 0;
   for (j = 0; j < 8; j++) {
       while (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));
       delay(40);
       if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) {
            i&= ~(1 << (7 - j));
       } else {
           i |= (1 << (7 - j));
       }
       while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));
   }
   return i;
}
uint8_t DHT11_Readi(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
   uint8_t k, l;
   //uint8_t data = 0;
   for (k = 0; k < 8; k++) {
       while (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));
       delay(40);
       if (!(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin))) {
            l&= ~(1 << (7 - k));
       } else {
           l |= (1 << (7 - k));
       }
       while ((HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)));
   }
   return l;
}

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
   SystemClock_Config();
  // MX_GPIO_Init();
   MX_LPUART1_UART_Init();
   MX_TIM1_Init();



   HAL_TIM_Base_Start(&htim1);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
       // SENSOR1
	   DHT11_Start(GPIOA, GPIO_PIN_4);
	          Presence2 = DHT11_Check_Response(GPIOA, GPIO_PIN_4);

	          if (Presence2)
	          {

	              Rh_byte1 = DHT11_Read(GPIOA, GPIO_PIN_4);
	              HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
	              Rh_byte2 = DHT11_Read(GPIOA, GPIO_PIN_4);
	              Temp_byte1 = DHT11_Read(GPIOA, GPIO_PIN_4);
	              Temp_byte2 = DHT11_Read(GPIOA, GPIO_PIN_4);
	              SUM2 = DHT11_Read(GPIOA, GPIO_PIN_4);

	              TEMP2 = Temp_byte1;
	              RH2 = Rh_byte1;

	              Temperature2 = (float) TEMP2 + (float)(Temp_byte2 / 10.0);
	              Humidity2 = (float) RH2 + (float)(Rh_byte2 / 10.0);


	              HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\n\rSensor 2\n\r", 14, 500);
	              sprintf(tstr2, "Temperature2: %.2f°C\n\r", Temperature2);
	              HAL_UART_Transmit(&hlpuart1, (uint8_t *)tstr2, strlen(tstr2), 500);
	              sprintf(rstr2, "Humidity2: %.2f%%\n\r", Humidity2);
	              HAL_UART_Transmit(&hlpuart1, (uint8_t *)rstr2, strlen(rstr2), 500);
	          }
	          else
	          {
	              HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Error in detecting the second DHT11\n\r", 37, 500);
	          }

	          HAL_Delay(2000); // Delay between sensor readings
	      }
       DHT11_Start(GPIOA, GPIO_PIN_0);

       Presence1 = DHT11_Check_Response(GPIOA, GPIO_PIN_0);

       if (Presence1)
       {
           Rh_byte1 = DHT11_Read(GPIOA, GPIO_PIN_0);
           Rh_byte2 = DHT11_Read(GPIOA, GPIO_PIN_0);
           Temp_byte1 = DHT11_Read(GPIOA, GPIO_PIN_0);
           Temp_byte2 = DHT11_Read(GPIOA, GPIO_PIN_0);
           SUM1 = DHT11_Read(GPIOA, GPIO_PIN_0);

           TEMP1 = Temp_byte1;
           RH1 = Rh_byte1;

           Temperature1 = (float) TEMP1 + (float)(Temp_byte2 / 10.0);
           Humidity1 = (float) RH1 + (float)(Rh_byte2 / 10.0);

           HAL_UART_Transmit(&hlpuart1, (uint8_t *)"\n\rSensor 1\n\r", 14, 500);
           sprintf(tstr1, "Temperature1: %.2f°C\n\r", Temperature1);
           HAL_UART_Transmit(&hlpuart1, (uint8_t *)tstr1, strlen(tstr1), 500);
           sprintf(rstr1, "Humidity1: %.2f%%\n\r", Humidity1);
           HAL_UART_Transmit(&hlpuart1, (uint8_t *)rstr1, strlen(rstr1), 500);
       }
       else
       {
           HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Error in detecting the first DHT11\n\r", 36, 500);
       }

       // SENSOR2


}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : PB0 */
//  GPIO_InitStruct.Pin = GPIO_PIN_0;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}

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
