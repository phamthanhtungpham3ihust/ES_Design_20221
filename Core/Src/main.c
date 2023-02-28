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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT.h"
#include "lcd16x2.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*Task_FunctionTypeDef) (void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TransmitBuff_SIZE	35
#define ReceiveBuff_SIZE	10
#define MainBuff_SIZE		8
#define Queue_SIZE			3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
Task_FunctionTypeDef queue[Queue_SIZE];

/* Data of DHT11 Sensor */
float temp = 0;
float humi = 0;
uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;
uint8_t Presence = 0;

/* Data line of LCD1602*/
char data_line_1[16];
char data_line_2[16];

/* Data of HC-SR04 Sensor */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

/* Buffer Array */
uint8_t RxBuffer[ReceiveBuff_SIZE];
uint8_t TxBuffer[TransmitBuff_SIZE];
uint8_t MainBuffer[MainBuff_SIZE];

/* Period Time */
uint8_t DHT11_transmit_period;
uint8_t HCSR04_transmit_period;

/* Temperature Threshold */
uint8_t temp_threshold;

/* Humidity Threshold */
uint8_t humi_threshold;

/* Distance Threshold */
uint8_t dist_threshold;


/* Etc .. */
uint8_t cmd_index = 1;
uint8_t len = 0;

uint32_t start_tick = 0;
uint32_t stop_tick = 0;
uint32_t execute_tick = 0;

bool read_sensor_flag = 0;

uint16_t Count = 0;
uint16_t cnt = 0;
uint16_t cnt_1 = 0;
uint8_t isr_cnt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
// Hiển thị LCD
void LCD_Display(void);

// Nhận UART
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

// Xử lý lệnh UART
void processRecCmd(size_t len);

// Truyen du lieu DHT11 qua UART
void DHT11_UART_Transmit(void);

// Truyen du lieu HSR04 qua UART
void HSR04_UART_Transmit(void);

// Truyen du lieu DHT11 và HSR04 qua UART
void DHT11_HSR04_UART_Transmit(void);

// Delay microsecond
void delay_us(uint16_t time);

// Hàm Callback khi sóng siêu âm phản hồi v�? để đo khoảng cách
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

// Hàm ngắt Timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// Set chân cho DHT11
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

// Khởi tạo giao tiếp giữa DHT11 với Vi đi�?u khiển
void DHT11_Start(void);

// Kiểm tra phản hồi của DHT11
uint8_t DHT11_Check_Response(void);


uint8_t DHT11_Read(void);

// �?�?c cảm biến DHT11
void Read_DHT11_Sensor(void);

// �?�?c cảm biến HCSR04
void Read_HCSR04_Sensor(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Set period, threshold value by button
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	start_tick = HAL_GetTick();
	UNUSED(GPIO_Pin);
	if(GPIO_Pin == GPIO_PIN_0){
		isr_cnt++;
		/* Nhan nut lan 1, dat chu ky truyen UART cac cam bien la 1s */
		if(isr_cnt == 1){
			DHT11_transmit_period = 1;
			HCSR04_transmit_period = 1;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
		}
		/* Nhan nut lan 2, dat chu ky truyen UART cac cam bien la 2s */
		else if(isr_cnt == 2){
			DHT11_transmit_period = 2;
			HCSR04_transmit_period = 2;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
		}
		/* Nhan nut lan 3, dat chu ky truyen UART cac cam bien la 3s */
		else if(isr_cnt == 3){
			DHT11_transmit_period = 3;
			HCSR04_transmit_period = 3;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			isr_cnt = 0;
		}
	}
	//while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
	stop_tick = HAL_GetTick();
	execute_tick = stop_tick - start_tick;
}


void delay_us(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER (&htim1) < time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
			stop_tick = HAL_GetTick();
			execute_tick = stop_tick - start_tick;

		}
	}
}

void Read_HCSR04_Sensor(void)
{
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay_us(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIGGER_GPIO_Port, TRIGGER_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

void LCD_Display(void){
	sprintf(data_line_1,"TEMP HUMI DIST");
	sprintf(data_line_2,"%dC %d  %d", (int)temp, (int)humi,(int)Distance);
	lcd16x2_1stLine();
	lcd16x2_printf(data_line_1);
	lcd16x2_2ndLine();
	lcd16x2_printf(data_line_2);
}

void DHT11_UART_Transmit(void){
	memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "Temp: %doC, Humi: %d%%\n", (int) temp, (int) humi);
	for(size_t i = 0; i < sizeof(TxBuffer); i++){
		if(TxBuffer[i] == '\0'){
			len = i + 1;
		}
	}
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));
}

void HSR04_UART_Transmit(void){
	memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "Dist: %d cm\n", (int)Distance);
	for(size_t i = 0; i < sizeof(TxBuffer); i++){
		if(TxBuffer[i] == '\0'){
			len = i + 1;
		}
	}
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, sizeof(TxBuffer));

}

void DHT11_HSR04_UART_Transmit(void){
	memset(TxBuffer, 0, TransmitBuff_SIZE);
	sprintf((char*)TxBuffer, "Temp: %doC, Humi: %d%%, Dist: %dcm", (int) temp, (int) humi, (int)Distance);
	for(size_t i = 0; i < sizeof(TxBuffer); i++){
		if(TxBuffer[i] == '\0'){
			len = i + 1;
		}
	}
	HAL_UART_Transmit_DMA(&huart1, TxBuffer, len);
}

void processRecCmd(size_t len){
	if(MainBuffer[0] != 0x00 || MainBuffer[len-1] != 0x07){
		return;
	};

	uint8_t cmd_type = MainBuffer[cmd_index]; // MainBuffer[1]

	switch (cmd_type) {
		case 0x01:
			/* some codes */
			DHT11_UART_Transmit();
			//HSR04_UART_Transmit();
		  break;

		case 0x02:
			/* some codes */
			//HSR04_UART_Transmit();
			//DHT11_HSR04_UART_Transmit();

			/* Xoa thong so cai dat tu nut nhan */
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			isr_cnt = 0;

			/* Dat cac thong so cai dat tu lenh UART */
			DHT11_transmit_period = MainBuffer[2];
			HCSR04_transmit_period = MainBuffer[3];
			temp_threshold = MainBuffer[4];
			humi_threshold = MainBuffer[5];
			dist_threshold = MainBuffer[6];
		  break;

		default:
		  break;
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART1){
		for(size_t i = 0; i<Size-1; i++) {
			if(RxBuffer[i] == 0x00){
				memcpy(MainBuffer, &RxBuffer[i], Size-i);
				processRecCmd(Size-i);
				break;
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, MainBuff_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim4)
  {
	  Count++;
	  read_sensor_flag = true;
  }
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_GPIO_Port, DHT11_Pin);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_GPIO_Port, DHT11_Pin, 0);   // pull the pin low
	delay_us(25000);   // wait for 25ms
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, 1);   // pull the pin high
	delay_us(25);   // wait for 25us
	Set_Pin_Input(DHT11_GPIO_Port, DHT11_Pin);    // set as input
}

uint8_t DHT11_Check_Response(void)
{
	uint8_t Response = 0;
	delay_us(40);
	if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))
	{
		delay_us(80);
		if ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin))) Response = 1;
		else Response = -1; // 255
	}
	else{
		return Response;
	}
	while ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read(void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)));   // wait for the pin to go high
		delay_us(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else{
			i|= (1<<(7-j));  // if the pin is high, write 1
		}
		while ((HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin)));  // wait for the pin to go low
	}
	return i;
}

void Read_DHT11_Sensor(void){
	//start_tick = HAL_GetTick();
	DHT11_Start();
    Presence = DHT11_Check_Response();
    if(Presence){
    	Rh_byte1 = DHT11_Read();
    	Rh_byte2 = DHT11_Read();
    	Temp_byte1 = DHT11_Read();
    	Temp_byte2 = DHT11_Read();
    	SUM = DHT11_Read();
    	TEMP = Temp_byte1;
    	RH = Rh_byte1;
    	temp = (float)TEMP;
    	humi = (float)RH;
    }
    //stop_tick = HAL_GetTick();
    //execute_tick = stop_tick - start_tick;
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  queue[0] = Read_HCSR04_Sensor;
  queue[1] = Read_DHT11_Sensor;
  queue[2] = LCD_Display;
  lcd16x2_init_4bits(GPIOB, RS_Pin, E_Pin, GPIOA, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6);

  HAL_TIM_Base_Start_IT(&htim4);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuffer, MainBuff_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 // HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	  if(read_sensor_flag){
		  read_sensor_flag = 0;
		  //DHT_ReadTempHum(&DHT11);
		  //temp = DHT11.Temp;
		  //humi = DHT11.Humi;
		  Read_DHT11_Sensor();
	  }


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
  TIM_IC_InitTypeDef sConfigIC = {0};

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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |TRIGGER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, E_Pin|RS_Pin|DHT11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 PA6
                           TRIGGER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : E_Pin RS_Pin DHT11_Pin */
  GPIO_InitStruct.Pin = E_Pin|RS_Pin|DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
