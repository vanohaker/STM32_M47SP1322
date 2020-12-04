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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEND_CMD      1   // Display instruction (command)
#define SEND_DATA      2   // Display instruction (data)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// start SSD1322(M47SP1322) command
const uint8_t ENABLE_GST 			=	0b00000000; // 10.1.1 Enable Gray Scale Table (00h)
const uint8_t SET_COL_ADDRESS 		=	0b00010101; // 10.1.2 Set Column Address (15h)
const uint8_t WEITE_RAM 			= 	0b01011100; // 10.1.3 Write RAM Command (5Ch)
const uint8_t READ_RAM 				=	0b01011101; // 10.1.4 Read RAM Command (5Dh)
const uint8_t SET_ROW_ADDRESS 		=	0b01110101; // 10.1.5 Set Row Address (75h)
const uint8_t SET_LINE_MODE			=	0b10100000; // 10.1.6 Set Re-map & Dual COM Line Mode (A0h)
const uint8_t SET_START_LINE		=	0b10100001; // 10.1.7 Set Display Start Line (A1h)
const uint8_t SET_DISP_OFSET		=	0b10100010; // 10.1.8 Set Display Offset (A2h)
const uint8_t SET_DISP_MODE_NORMAL	=	0b10100100; // 10.1.9 Set Display Mode - Normal Display (A4h)
const uint8_t SET_DISP_MODE_ON		=	0b10100101; // 10.1.9 Set Display Mode - Display ON (A5h)
const uint8_t SET_DISP_MODE_OFF		=	0b10100110; // 10.1.9 Set Display Mode - Display OFF (A6h)
const uint8_t SET_DISP_MODE_INVERS	=	0b10100111; // 10.1.9 Set Display Mode - Inverse Display (A7h)
const uint8_t PARTAL_DISP_ENABLE	=	0b10101000; // 10.1.10 Enable Partial Display (A8h)
const uint8_t PARTAL_DISP_DISABLE	=	0b10101001; // 10.1.11 Exit Partial Display (A9h)
const uint8_t SET_FUNCTION			=	0b10101011; // 10.1.12 Set Function selection (ABh)
const uint8_t SET_DISP_ON			=	0b10101110; // 10.1.13 Set Display ON (AEh)
const uint8_t SET_DISP_OFF			=	0b10101111; // 10.1.13 Set Display OFF (AFh)
const uint8_t SET_PHASE_LEN			=	0b10110001; // 10.1.14 Set Phase Length (B1h)
const uint8_t SET_FRONT_DIV_OSC_FRQ	=	0b10110011; // 10.1.15 Set Front Clock Divider / Oscillator Frequency (B3h)
const uint8_t SET_DISP_ENH_A		=	0b10110100; // 10.1.16 Display Enhancement A (B4h)
const uint8_t SET_GPIO				=	0b10110101; // 10.1.17 Set GPIO (B5h)
const uint8_t SET_PRE_CHARGE_PERIOD	=	0b10110110; // 10.1.18 Set Second Pre-charge period (B6h)
const uint8_t SET_GST				=	0b10111000; // 10.1.19 Set Gray Scale Table (B8h)
const uint8_t SEL_DEF_LINEAR_GST	=	0b10111001; // 10.1.20 Select Default Linear Gray Scale Table (B9h)
const uint8_t SET_PRE_CHG_VOLTAGE	=	0b10111011; // 10.1.21 Set Pre-charge voltage (BBh)
const uint8_t SET_VCOMH				=	0b10111110; // 10.1.22 Set V COMH Voltage (BEh)
const uint8_t SET_CONTRAST			=	0b11000001; // 10.1.23 Set Contrast Current (C1h)
const uint8_t SET_MASTER_CURRENT	=	0b11000111; // 10.1.24 Master Current Control (C7h)
const uint8_t SET_MULTIPLEX_RATIO	=	0b11001011; // 10.1.25 Set Multiplex Ratio (CAh)
const uint8_t SET_DISP_ENH_B		=	0b11010001; // 10.1.26 Display Enhancement B (D1h)
const uint8_t SET_COMMAND_LOCK		=	0b11111101; // 10.1.27 Set Command Lock (FDh)
// end SSD1322(M47SP1322) command

unsigned char sendType;
uint8_t aTxBuffer = 0b00000000;
uint8_t aRxBuffer[] = {0x00};

const uint16_t MAXROWS =   64;
const uint16_t MAXCOLS =  256;
const uint16_t Col0Off =  112;
const uint16_t ColDiv  =    4;

uint16_t curDisplayRow = 0;
uint16_t curDisplayCol = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

//reset display //
void Reset_Device(void);
void Init_Device(void);
// send data //
void displaySend(unsigned char sendType, uint8_t v);
void displaySendType(unsigned char sendType);
void displaySendData(uint8_t v);
void displaySendEnd(void);
void Set_Column_Address(unsigned char a, unsigned char b);
void Set_Row_Address(unsigned char a, unsigned char b);
void Set_Write_RAM();
void FillDisplay(void);
void ClearDisplay(void);

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* RESET DISPLAY */
  Reset_Device();
  Init_Device();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  FillDisplay();
	  HAL_Delay(1000);
	  ClearDisplay();
	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Pin|CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RST_Pin DC_Pin CS_Pin */
  GPIO_InitStruct.Pin = RST_Pin|DC_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Reset_Device(void) {
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET); // Ставми DC в low
	HAL_Delay(500);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET); // Ставми RESET в low
	HAL_Delay(2); // t1 минимум 0.1 милисекунда
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET); // Ставми RESET в h
	HAL_Delay(200); // t2 = t1 + HAL_Delay(1)
	displaySend(SEND_CMD, SET_DISP_OFF);
	HAL_Delay(400); // t AF

}

void Init_Device(void) {
	displaySend(SEND_CMD, SET_COMMAND_LOCK); 			// 0xFD Устанавливаем блокировку комманд дисплея
	displaySend(SEND_DATA, 0x12);		 				// 0x12 0x12 = Заблокировано; 0x16 = Разблокировано
	displaySend(SEND_CMD, SET_DISP_MODE_OFF);			// 0xA4 Выключаем дисплей
	displaySend(SEND_CMD, SET_FRONT_DIV_OSC_FRQ); 		// 0xB3 Устанавливаем коэффициент деления DCLK и частоту CLK
	// A[3:0] Коэффициент деления (1-1024) [0001] - деление на 2
	// A[7:4] Частота [1001]
	displaySend(SEND_DATA, 0x91);						// 0x91 0b10010001
	displaySend(SEND_CMD, SET_MULTIPLEX_RATIO);			// 0xCA Установка включенных выводов COM (Set MUX Ratio)
	displaySend(SEND_DATA, 0x7F);						// 0x7F Включает все 128 выводов по горизонтали
	displaySend(SEND_CMD, SET_DISP_OFSET);				// 0xA2 Задаёт привязку начальной строки экрана
	displaySend(SEND_DATA, 0x00);						// 0x00 Начальная строка ROW0
	displaySend(SEND_CMD, SET_START_LINE);				// 0xA1 Начальная строка отображения
	displaySend(SEND_DATA, 0x00); 						// 0x00	Начальная строка ROW0
	displaySend(SEND_CMD, SET_LINE_MODE);				// 0xA0 Настройка последовательности отображения пикселей
	displaySend(SEND_DATA, 0x10); 						// 0x10 00010000
	/* A[0] = 0 установлен в 0, драйвер настроен в режим инкремента горизонтального адреса.
	 * A[1] = 0 (состояние по умолчанию после сброса): столбцы RAM 0 .. 119 привязаны к SEG0-SEG3 .. SEG476-SEG479
	 * A[2] = 0 (состояние по умолчанию после сброса): биты данных привязаны напрямую
	 * A[3] = 0
	 * A[4] = 1: сканирование снизу вверх
	 * A[5] = 0 (состояние по умолчанию после сброса): запрет разделения COM на четные / нечетные, назначение выводов COM последовательное, COM127, COM126...COM65, COM64...SEG479...SEG0...COM0 COM1...COM62, COM63
	 * A[6] = 0
	 * A[7] = 0
	 * */
	displaySend(SEND_DATA, 0x01); 						// 0x11 00000001
	/* B[4] = 0 (состояние по умолчанию после сброса): двойной режим для COM запрещен, см. рис. 10-6
	 * */
	displaySend(SEND_CMD, SET_GPIO); 					// 0xB5 Управление GPIO дисплея
	displaySend(SEND_DATA, 0x00); 						// 0x00 Выключены
	displaySend(SEND_CMD, SET_FUNCTION);				// 0xAB Питание ядра VDD
	displaySend(SEND_DATA, 0x01); 						// 0x01 внутренний регулятор
	displaySend(SEND_CMD, SET_DISP_ENH_A); 				// 0xB4 регулировка А
	displaySend(SEND_DATA, 0xA2); 						// 0xA2 A[1:0] = 10b: Internal VSL [reset]
	displaySend(SEND_DATA, 0xB5); 						// 0xB5 B[7:3] = 10110b: Normal [reset]
	displaySend(SEND_CMD, SET_CONTRAST); 				// 0xC1 Установка контраста 00h-ffh
	displaySend(SEND_DATA, 0x7F); 						// 0x7F Контраст
	displaySend(SEND_CMD, SET_MASTER_CURRENT); 			// 0xC7 управления выходным током SEG
	displaySend(SEND_DATA, 0x0F); 						// 0x0F A[3:0] = 1111b, без изменений (состояние после сброса)
	displaySend(SEND_CMD, SEL_DEF_LINEAR_GST); 			// 0xB9
	displaySend(SEND_CMD, SET_PHASE_LEN); 				// 0xB1
	displaySend(SEND_DATA, 0xE2); 						// 0xE2
	//        Phase 2 period (first pre-charge phase length) = 14 DCLKs
    displaySend(SEND_CMD, SET_DISP_ENH_B); 				// 0xD1
    displaySend(SEND_DATA, 0xA2); 						// 0xA2
    displaySend(SEND_DATA, 0x20); 						// 0x20
    displaySend(SEND_CMD, SET_PRE_CHG_VOLTAGE); 		// 0xBB
    displaySend(SEND_DATA, 0x17); 						// 0x17
    displaySend(SEND_CMD, SET_PRE_CHARGE_PERIOD); 		// 0xB6
    displaySend(SEND_DATA, 0x08); 						// 0x08
    displaySend(SEND_CMD, SET_VCOMH); 					// 0xBE
    displaySend(SEND_DATA, 0x04); 						// 0x04
    displaySend(SEND_CMD, SET_DISP_MODE_NORMAL); 		// 0xA6
    displaySend(SEND_CMD, PARTAL_DISP_DISABLE); 		// 0xA9
    displaySend(SEND_CMD, SET_DISP_MODE_ON); 			// 0xA5
    HAL_Delay(10);

}

void ClearDisplay(void){
	displaySend(SEND_CMD, SET_DISP_MODE_NORMAL);
	Set_Column_Address(0x00, MAXCOLS-1);
	Set_Row_Address(0x00, MAXROWS-1);
	Set_Write_RAM();
	displaySendType(SEND_DATA);
    for(int i=0; i<MAXROWS; i++)
    {
        for(int j=0; j<MAXCOLS/2; j++)
        {
            displaySendData(0x00);
        }
    }
    displaySendEnd();
    displaySend(SEND_CMD, SET_DISP_MODE_OFF);
}

void FillDisplay(void) {
	Set_Column_Address(0x00, MAXCOLS-1);
	Set_Row_Address(0x00, MAXROWS-1);
	Set_Write_RAM();
	displaySendType(SEND_DATA);
	for(int i=0; i<MAXROWS; i++)
	{
		for(int j=0; j<MAXCOLS/2; j++)
		{
			displaySendData(0xFF);
		}
	}
	displaySendEnd();
}

void Set_Column_Address(unsigned char a, unsigned char b)
{
    curDisplayCol = a;
    displaySend(SEND_CMD, SET_START_LINE);
    displaySend(SEND_DATA, (Col0Off+a)/ColDiv);
    displaySend(SEND_DATA, (Col0Off+b)/ColDiv);
}

void Set_Row_Address(unsigned char a, unsigned char b)
{
    curDisplayRow = a;
    displaySend(SEND_CMD, SET_ROW_ADDRESS);
    displaySend(SEND_DATA, a);
    displaySend(SEND_DATA, b);
}

void Set_Write_RAM()  // Enable MCU to Write into RAM
{
    displaySend(SEND_CMD, 0x5C);
}

void displaySend(unsigned char sendType, uint8_t v) {
	displaySendType(sendType); // Какой тип данных будем посылать дисплею? Данные или команду.
	displaySendData(v);
	HAL_GPIO_TogglePin(DC_GPIO_Port, DC_Pin);
	displaySendEnd();
}

void displaySendType(unsigned char sendType) {
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET); // Перед передачей данных ставим CS в 0
	if (sendType == SEND_DATA) {
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET); // Если данные то ставим DC в 1
	} else if (sendType == SEND_CMD) {
		HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET); // Если данные то ставим DC в 0
	}
}
void displaySendData(uint8_t v){
	uint8_t aTxBuffer = v;
	HAL_SPI_Transmit(&hspi1, &aTxBuffer, 1, 100);
}
void displaySendEnd(void){
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET); // После конца передачи ставим CS в 1
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
