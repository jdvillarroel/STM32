/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "FLASH_M4.h"
#include <string.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	FLASH_BASE_SECTOR_ADDRESS			0x08060000U
#define START_FRAME							(uint8_t)0x4A
#define END_FRAME							(uint8_t)0x56

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ************** CAN related variables **************** //
CAN_FilterTypeDef CANFilter;
CAN_TxHeaderTypeDef CANTXHeader;
CAN_RxHeaderTypeDef CANRXHeader;
uint8_t CANTXBuffer[8];
uint8_t CANRXBuffer[8];
uint32_t CANMailbox;
volatile uint8_t CANInitFlag = 0;
volatile uint8_t CANStopFlag = 0;
volatile uint8_t CANSendDataFlag = 0;
volatile uint8_t isCANConfig = 0;
volatile uint8_t CANSendData = 0;

uint32_t CANParamemeters[30] = {};

typedef enum {
	waitCmd = 0,
	CANListen,
	CANStop,
	resetMCU,
	writeFlashBuffer,
	readFlashBuffer,
	writeFlashBufferToFlash,
	readFlashToFlashBuffer,
	writeDefaultToFlash,
	CANStart
}StateType;

StateType state;

uint32_t flashBuffer[30];
volatile uint8_t writeFlashFlag = 0;

uint32_t counter = 0;
uint8_t aux;

// ************* UART related variables ************** //
uint8_t uRXBuffer[10];
volatile uint8_t cmdReceivedFlag = 0;

typedef struct {
	uint8_t startFrame;
	uint8_t actionCode;
	uint8_t bufferAddress;
	union {
		uint32_t word;
		uint8_t byte[4];
	} data;
	union {
		uint16_t hWord;
		uint8_t byte[2];
	} errorCheck;
	uint8_t endFrame;
} CommandFrameType;

CommandFrameType uCommand;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void CANFilterConfiguration(CAN_FilterTypeDef *_CANFilter);
void CANInit(void);
void CANConfig(void);

void CANFlashDefaults(void);
void loadParamFromFlash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char *initMsg[5] = {
		"//**********************************************//\r\n",
		"// *****           CANbus Reader          ***** //\r\n",
		"// *****     Prototype version v1.0.1     ***** //\r\n",
		"// *****    Designed by: JD Villarroel    ***** //\r\n",
		"//**********************************************//\r\n"
};

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  // Send initial message to a serial console.
  for (aux = 0; aux < 5; aux++)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t*)initMsg[aux], strlen(initMsg[aux]), 1000);
  }

  // Initialize interrupt reception from console.
  HAL_UART_Receive_IT(&huart2, uRXBuffer, 10);

  // Load params from flash.
  loadParamFromFlash();

  // Initialize timer with interrupts.
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check for received commands from console.
	  switch(state)
	  {
		  case waitCmd:
			  break;

		  case CANListen:
			  CANConfig();
			  CANSendData = 0;
			  state = waitCmd;
			  break;

		  case CANStop:
			  CANSendData = 0;
			  break;

		  case resetMCU:
			  HAL_NVIC_SystemReset();
			  break;

		  case writeFlashBuffer:
			  flashBuffer[uCommand.bufferAddress] = uCommand.data.word;
			  state = waitCmd;
			  break;

		  case readFlashBuffer:
			  break;

		  case writeFlashBufferToFlash:
			  Flash_Write_Data(FLASH_BASE_SECTOR_ADDRESS, flashBuffer, 30);
			  state = waitCmd;
			  break;

		  case readFlashToFlashBuffer:
			  break;

		  case writeDefaultToFlash:
			  CANFlashDefaults();
			  state = waitCmd;
			  break;

		  case CANStart:
			  CANConfig();
			  CANSendData = 1;
			  state = waitCmd;
			  break;

		  default:
			  break;
	  }

	  if (CANSendDataFlag && CANSendData)
	  {
		  // Put data in buffer to be transmitted.
		  if (HAL_CAN_AddTxMessage(&hcan1, &CANTXHeader, CANTXBuffer, &CANMailbox) != HAL_OK)
		  {
			  Error_Handler();
		  }

		  HAL_GPIO_TogglePin(LEDW_GPIO_Port, LEDW_Pin);

		  // Reset flag.
		  CANSendDataFlag = 0;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 36;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 900;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDY_Pin|LEDW_Pin|LEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDY_Pin LEDW_Pin LEDB_Pin */
  GPIO_InitStruct.Pin = LEDY_Pin|LEDW_Pin|LEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void CANFilterConfiguration(CAN_FilterTypeDef *_CANFilter)
{
	// Initialize filters.
	if (HAL_CAN_ConfigFilter(&hcan1, _CANFilter) != HAL_OK)
	{
		Error_Handler();
	}
}

void CANInit(void)
{
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = CANParamemeters[29];
	hcan1.Init.Mode = CANParamemeters[1];
	hcan1.Init.SyncJumpWidth = CANParamemeters[2];
	hcan1.Init.TimeSeg1 = CANParamemeters[3];
	hcan1.Init.TimeSeg2 = CANParamemeters[4];
	hcan1.Init.TimeTriggeredMode = CANParamemeters[5];
	hcan1.Init.AutoBusOff = CANParamemeters[6];
	hcan1.Init.AutoWakeUp = CANParamemeters[7];
	hcan1.Init.AutoRetransmission = CANParamemeters[8];
	hcan1.Init.ReceiveFifoLocked = CANParamemeters[9];
	hcan1.Init.TransmitFifoPriority = CANParamemeters[10];
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}

	CANFilter.FilterActivation = CANParamemeters[11];
	CANFilter.FilterBank = CANParamemeters[12];
	CANFilter.FilterFIFOAssignment = CANParamemeters[13];
	CANFilter.FilterIdHigh = CANParamemeters[14];
	CANFilter.FilterIdLow = CANParamemeters[15];
	CANFilter.FilterMaskIdHigh = CANParamemeters[16];
	CANFilter.FilterMaskIdLow = CANParamemeters[17];
	CANFilter.FilterMode = CANParamemeters[18];
	CANFilter.FilterScale = CANParamemeters[19];
	CANFilter.SlaveStartFilterBank = CANParamemeters[20];
}
void CANConfig(void)
{
	CANInit();

	// Initialize Filters.
	CANFilterConfiguration(&CANFilter);

	isCANConfig = 1;

	// Start CAN module.
	HAL_CAN_Start(&hcan1);

	// Activate interrupt mask.
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	CANTXHeader.DLC = CANParamemeters[21];
	CANTXHeader.ExtId = CANParamemeters[22];
	CANTXHeader.IDE = CANParamemeters[23];
	CANTXHeader.RTR = CANParamemeters[24];
	CANTXHeader.StdId = CANParamemeters[25];
	CANTXHeader.TransmitGlobalTime = CANParamemeters[26];

	CANTXBuffer[0] = 'C';
	CANTXBuffer[1] = 'A';
	CANTXBuffer[2] = 'N';
	CANTXBuffer[3] = ' ';
	CANTXBuffer[4] = 'I';
	CANTXBuffer[5] = 'n';
	CANTXBuffer[6] = 'i';
	CANTXBuffer[7] = 't';

	HAL_CAN_AddTxMessage(&hcan1, &CANTXHeader, CANTXBuffer, &CANMailbox);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(LEDY_GPIO_Port, LEDY_Pin);

	counter++;
	if (counter >= 6)
	{
//		CANInitFlag = 1;
		CANSendDataFlag = 1;
		counter = 0;
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	char msg[30];

	HAL_GPIO_TogglePin(LEDB_GPIO_Port, LEDB_Pin);

	if (HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CANRXHeader, CANRXBuffer))
	{
		Error_Handler();
	}

	sprintf(
			msg,
			"NodeID: %x  %x %x %x %x %x %x %x %x\r\n",
			CANRXHeader.StdId,
			CANRXBuffer[0],
			CANRXBuffer[1],
			CANRXBuffer[2],
			CANRXBuffer[3],
			CANRXBuffer[4],
			CANRXBuffer[5],
			CANRXBuffer[6],
			CANRXBuffer[7]
			);

//	sprintf(msg, "Data received...\r\n");

	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		// Transmit data received for verification.
		HAL_UART_Transmit_IT(huart, uRXBuffer, 10);

		// Transfer data received to command structure.
		uCommand.startFrame = uRXBuffer[0];
		uCommand.actionCode = uRXBuffer[1];
		uCommand.bufferAddress = uRXBuffer[2];
		uCommand.data.byte[0] = uRXBuffer[3];
		uCommand.data.byte[1] = uRXBuffer[4];
		uCommand.data.byte[2] = uRXBuffer[5];
		uCommand.data.byte[3] = uRXBuffer[6];
		uCommand.errorCheck.byte[0] = uRXBuffer[7];
		uCommand.errorCheck.byte[1] = uRXBuffer[8];
		uCommand.endFrame = uRXBuffer[9];

		// Verify start and end frame to continue.
		if (uCommand.startFrame == START_FRAME && uCommand.endFrame == END_FRAME)
		{
		  state = uCommand.actionCode;
		}

		// Receive command flag set.
		cmdReceivedFlag = 1;

		// Enable UART reception using interrupts.
		HAL_UART_Receive_IT(huart, uRXBuffer, 10);
	}
}

void loadParamFromFlash(void)
{
	char *msgOK = "Parameters loaded OK\r\n";

	Flash_Read_Data(FLASH_BASE_SECTOR_ADDRESS, flashBuffer, 30);

	for (uint8_t i = 0; i < 30; i++)
	{
		CANParamemeters[i] = flashBuffer[i];
//		writeFlashBuffer[i] = readFlashBuffer[i];
	}

	HAL_UART_Transmit(&huart2, (uint8_t*)msgOK, strlen(msgOK), 100);
}

void CANFlashDefaults(void)
{
	flashBuffer[29] = 36;
	flashBuffer[1] = CAN_MODE_LOOPBACK;
	flashBuffer[2] = CAN_SJW_1TQ;
	flashBuffer[3] = CAN_BS1_16TQ;
	flashBuffer[4] = CAN_BS2_8TQ;
	flashBuffer[5] = DISABLE;
	flashBuffer[6] = DISABLE;
	flashBuffer[7] = DISABLE;
	flashBuffer[8] = DISABLE;
	flashBuffer[9] = DISABLE;
	flashBuffer[10] = DISABLE;

	/*
	 * This group of parameters correspond to the CAN filter structure.
	 */
	flashBuffer[11] = CAN_FILTER_ENABLE;			// FilterActivation
	flashBuffer[12] = 0;							// FilterBank
	flashBuffer[13] = CAN_FILTER_FIFO0;				// FilterFIFOAssignment
	flashBuffer[14] = 0x0;							// FilterIdHigh
	flashBuffer[15] = 0x0;							// FilterIdLow
	flashBuffer[16] = 0x0;							// FilterMaskIdHigh
	flashBuffer[17] = 0x0;							// FilterMaskIdLow
	flashBuffer[18] = CAN_FILTERMODE_IDMASK;		// FilterMode
	flashBuffer[19] = CAN_FILTERSCALE_32BIT;		// FilterScale
	flashBuffer[20] = 13;							// SlaveStartFilterBank

	flashBuffer[21] = 8;							// DLC
	flashBuffer[22] = 0;							// ExtId
	flashBuffer[23] = CAN_ID_STD;					// IDE
	flashBuffer[24] = CAN_RTR_DATA;					// RTR
	flashBuffer[25] = 0x446;						// StdId
	flashBuffer[26] = DISABLE;						// TransmitGlobalTime

  Flash_Write_Data((FLASH_BASE_SECTOR_ADDRESS), flashBuffer, 30);
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
