/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "string.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  UART_RX_0 = 48,
  UART_RX_1,
  UART_RX_2,
  UART_RX_NONE = 0xFF,
}UARTCommandTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE                     1

#define BLEUIO_NOT_READY_MSG	"\r\nBleuIO not ready yet.\r\n"

// Dongle commands
#define DONGLE_CMD_ATI "ATI\r\n"
#define DONGLE_CMD_AT_ADVSTART "AT+ADVSTART\r\n"
#define DONGLE_CMD_AT_ADVSTOP "AT+ADVSTOP\r\n"
#define DONGLE_CMD_SET_ADV_NAME "AT+ADVRESP=14:09:42:6C:65:75:49:4F:20:50:49:52:20:44:65:74:65:63:74:65:64\r\n"

#define PIR_TIMEOUT_REFRESH		20  // Timeout after PIR triggering before PIR sensor can trigger again
#define ADV_TIMER_REFRESH		25 // How long the advertising will continue. Will refresh every time PIR is triggered.

//  ADV_TIMER_REFRESH should be equal or greater than PIR_TIMEOUT_REFRESH	(ADV_TIMER_REFRESH >= PIR_TIMEOUT_REFRESH)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static char uart_tx_buf[250];
int uart_buf_len;
bool isBleuIOReady;
uint8_t dongle_response[244];
static UARTCommandTypeDef uartStatus;
static uint8_t aRxBuffer[RXBUFFERSIZE];
static int RX_value;

int pirAdvTimer;
int pirActivationTimeout;

bool isAdvertising;
bool isAdvNameSet;
bool pirIRQEnabled;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM16_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void handleUartInput(UARTCommandTypeDef cmd);
void dongle_interpreter(uint8_t * input);
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_HOST_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  isBleuIOReady = false;
  isAdvertising= false;
  isAdvNameSet= false;
  pirIRQEnabled = true;
  pirAdvTimer = 0;
  pirActivationTimeout = 0;
  uartStatus = UART_RX_NONE;

  // Starts uart recieve interrupt mode
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE);

  // Turns on all LEDs on start up
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_Delay(1000);

  // Turns off all LEDs except Red
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

  // Sends welcome message to uart
  uart_buf_len = sprintf(uart_tx_buf, "\r\nWelcome to STM32 BleuIO PIR Sensor Example!\r\n\r\n\r\nWaiting for BleuIO...");
  HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
  HAL_TIM_Base_Start_IT(&htim16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // Simple handler for uart input
    handleUartInput(uartStatus);
//    // Inteprets the dongle data
    dongle_interpreter(dongle_response);

// 	Setting Advertising name
    if(isBleuIOReady && !isAdvNameSet)
    {
    	// Setting name in response data to 'BleuIO PIR Detected'
    	writeToDongle((uint8_t*)DONGLE_CMD_SET_ADV_NAME);
    	HAL_Delay(200);
    	isAdvNameSet = true;
		uart_buf_len = sprintf(uart_tx_buf, "\r\n\r\nDevice name set.\r\nBleuIO will now advertise if PIR detect movement!\r\n\r\n\r\n");
		HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
    }

//     Stops advertising if advertising and pirAdvTimer has run out.
    if(isAdvertising && pirAdvTimer < 0)
    {
    	writeToDongle((uint8_t*)DONGLE_CMD_AT_ADVSTOP);
    	HAL_Delay(200);
    	isAdvertising = false;
		uart_buf_len = sprintf(uart_tx_buf, "\r\nNo movement registered for %i seconds. Turning advertising off...\r\n\r\n\r\n", ADV_TIMER_REFRESH - PIR_TIMEOUT_REFRESH);
		HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 6400 -1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000 -1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_OTG_FS_PWR_EN_GPIO_Port, USB_OTG_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PIR_IRQ_SIGNAL_Pin */
  GPIO_InitStruct.Pin = PIR_IRQ_SIGNAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PIR_IRQ_SIGNAL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_OTG_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &huart3)
	{
		RX_value = (int)aRxBuffer[0];
		uartStatus = UART_RX_NONE;

		switch(RX_value)
		{
			case UART_RX_0:
			{
				uartStatus = UART_RX_0;
				break;
			}
			case UART_RX_1:
			{
				uartStatus = UART_RX_1;
				break;
			}
			case UART_RX_2:
			{
				uartStatus = UART_RX_2;
				break;
			}
			default:
			{
				uartStatus = UART_RX_NONE;
				break;
			}
		}
		// Resets uart recieve interrupt mode
		HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, RXBUFFERSIZE);
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// If The INT Source Is  and both isBleuIOReady and isAdvNameSet is true continue
	// Here we check if the source of the interrupt is our PIR sensor (EXTI0 (A0 Pin)), we also make sure that the dongle is ready
    if((GPIO_Pin == PIR_IRQ_SIGNAL_Pin) && isBleuIOReady && isAdvNameSet)
    {
    	// To ensure the PIR sensor doesn't trigger constantly we put a timeout period before it can trigger again.
    	if(pirActivationTimeout < 0)
    	{
			pirActivationTimeout = PIR_TIMEOUT_REFRESH;

			pirAdvTimer = ADV_TIMER_REFRESH;

			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle The YELLOW LED
			pirIRQEnabled = false;
			uart_buf_len = sprintf(uart_tx_buf, "\r\nMovement registered! %s\r\n\r\n\r\n", isAdvertising ? "" : "Turning on advertising...");
			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
			if(!isAdvertising)
			{
				writeToDongle((uint8_t*)DONGLE_CMD_AT_ADVSTART);
				isAdvertising = true;
			}
    	}
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &htim16)
	{
		pirAdvTimer--;
		pirActivationTimeout--;
	}
}

/**
  * @brief Simple uart input handler
  * @retval None
  */
void handleUartInput(UARTCommandTypeDef cmd)
{
	switch(cmd)
	{
		case UART_RX_0:
		{
			// 0

			// Example of how to use UART to input commands to dongle.
			// For response must uncomment line in usb_host.c to allow dongle output to be printed to UART.

//			uart_buf_len = sprintf(uart_tx_buf, "\r\n(0 pressed)\r\n");
//			HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
//			if(isBleuIOReady)
//			{
//				writeToDongle((uint8_t*)DONGLE_CMD_ATI);
//			} else
//			{
//				uart_buf_len = sprintf(uart_tx_buf, BLEUIO_NOT_READY_MSG);
//				HAL_UART_Transmit(&huart3, (uint8_t *)uart_tx_buf, uart_buf_len, HAL_MAX_DELAY);
//			}

			uartStatus = UART_RX_NONE;
			break;
		}
		case UART_RX_1:
		{
			// 1

			uartStatus = UART_RX_NONE;
			break;
		}
		case UART_RX_2:
		{
			// 2


			uartStatus = UART_RX_NONE;
			break;
		}
		case UART_RX_NONE:
		{
			break;
		}
		default:
		{
			uartStatus = UART_RX_NONE;
			break;
		}
	}
}

/**
  * @brief Simple dongle interpreter
  * @retval None
  */
void dongle_interpreter(uint8_t * input)
{

	// Example of a simple interpreter
//	if(strlen((char *)input) != 0)
//	{
//		if(strstr((char *)input, "\r\nADVERTISING...") != NULL)
//		{
//			// Do something
//		}
//		if(strstr((char *)input, "\r\nADVERTISING STOPPED.") != NULL)
//		{
//			// Do something
//		}
//	}
	memset(&dongle_response, 0, RSP_SIZE);
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
