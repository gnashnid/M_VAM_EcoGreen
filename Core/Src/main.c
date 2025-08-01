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
#include "stdbool.h"
#include "stdlib.h"
#include "DFPLAYER_MINI.h"
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
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8], uart_data[50], oneFloor, tenFloor, floorNow, speed, speakerButton[5][4], button[3];
bool speakFloor, speakDoorOpen, speakDoorClose, speakDiriectionUp, speakDirectionDown, speakFire, speakOverLoad,
	 isSpeakFloor, isSpeakDoorOpen, isSpeakDoorClose, isSpeakDiriectionUp, isSpeakDirectionDown,
	 isSpeakFire, isSpeakOverLoad, detect_speed, read_speed;
uint32_t timer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void speaker_floor(uint8_t tenFloor, uint8_t oneFloor);
void Set_speed_can(CAN_HandleTypeDef hcan, bool detect_speed, uint8_t speed);
void speaker_push_button(uint8_t speakerButton[5][4]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		HAL_NVIC_SystemReset();
	}
	read_speed = true;
	if (RxHeader.StdId == 0x501)
	{
		if ((RxData[0]&0x04) == 0x04)
		{
			if (RxData[2] == 0x07) // floor
			{
				speakFloor = true;
				speakFire = false;
				speakOverLoad = false;
				speakDoorOpen = false;
				speakDoorClose = false;
				speakDiriectionUp = false;
				speakDirectionDown = false;
			} else if (RxData[2] == 0x04) // door
			{
				if (RxData[4] == 0x01)
				{
					speakDoorOpen = true;
					speakDoorClose = false;
				} else if (RxData[4] == 0x02)
				{
					speakDoorOpen = false;
					speakDoorClose = true;
				}
				speakFloor = false;
				speakFire = false;
				speakOverLoad = false;
				speakDiriectionUp = false;
				speakDirectionDown = false;
			}  else if (RxData[2] == 0x01) // fire
			{
				speakFloor = false;
				speakFire = true;
				speakOverLoad = false;
				speakDoorOpen = false;
				speakDoorClose = false;
				speakDiriectionUp = false;
				speakDirectionDown = false;
			}  else if (RxData[2] == 0x05) // over load
			{
				speakFloor = false;
				speakFire = false;
				speakOverLoad = true;
				speakDoorOpen = false;
				speakDoorClose = false;
				speakDiriectionUp = false;
				speakDirectionDown = false;
			}  else if (RxData[2] == 0x0B) // direction
			{
				if (RxData[3] == 0x01) // up
				{
					speakDiriectionUp = true;
					speakDirectionDown = false;
				} else if (RxData[3] == 0x02) // down
				{
					speakDiriectionUp = false;
					speakDirectionDown = true;
				}
				speakFloor = false;
				speakFire = false;
				speakOverLoad = false;
				speakDoorOpen = false;
				speakDoorClose = false;
			}
		} else
		{
			speakFloor = false;
			speakFire = false;
			speakOverLoad = false;
			speakDoorOpen = false;
			speakDoorClose = false;
			speakDiriectionUp = false;
			speakDirectionDown = false;
		}
	} else if (RxHeader.StdId == 0x502)
	{
		oneFloor = RxData[2];
		tenFloor = RxData[1];
	} else if (RxHeader.StdId == 0x503)
	{
		if ((RxData[0] & 0x01) ==  0x01)
		{
			if (RxData[0] != button[0] || RxData[2] != button[1] || RxData[3] != button[2])
			{
				button[0] = RxData[0];
				button[1] = RxData[2];
				button[2] = RxData[3];
				for (uint8_t i=0; i<5; i++)
				{
					if (speakerButton[i][0] == 0)
					{
						speakerButton[i][0] = 1;
						if ((RxData[0] & 0x04) ==  0x04)
						{
							speakerButton[i][1] = 1;
						} else
						{
							speakerButton[i][1] = 0;
						}
						speakerButton[i][2] = button[1];
						speakerButton[i][3] = button[2];
						return;
					}
				}
			}
		}
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == huart2.Instance)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_data, 50);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim3.Instance)
	{
		HAL_IWDG_Refresh(&hiwdg);
	}
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
  __HAL_DBGMCU_FREEZE_IWDG();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  while (!detect_speed)
  {
	  if (!read_speed)
	  {
		  if (abs(HAL_GetTick() - timer) > 500)
		  {
			  timer = HAL_GetTick();
			  speed ++;
			  if (speed > 3)
			  {
				  speed = 1;
			  }
			  Set_speed_can(hcan, detect_speed, speed);
		  }
	  } else
	  {
		  detect_speed = true;
		  Set_speed_can(hcan, detect_speed, speed);
	  }
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart_data, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  DF_Init(30);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  speaker_push_button(speakerButton);

	  if (speakFloor)
	  {
		  if (!isSpeakFloor)
		  {
//			  floorNow = speaker_floor_funtion(tenFloor, oneFloor);
			  DF_Play(floorNow);
			  while (uart_data[6] != floorNow);
			  isSpeakFloor = true;
		  }
	  } else
	  {
		  isSpeakFloor = false;
	  }
	  if (speakDoorOpen) // door open
	  {
		  if (!isSpeakDoorOpen)
		  {
			  DF_Play(35);
			  while (uart_data[6] != 35);
			  isSpeakDoorOpen = true;
		  }
	  } else
	  {
		  isSpeakDoorOpen = false;
	  }
	  if (speakDoorClose) // door close
	  {
		  if (!isSpeakDoorClose)
		  {
			  DF_Play(36);
			  while (uart_data[6] != 36);
			  isSpeakDoorClose = true;
		  }
	  } else
	  {
		  isSpeakDoorClose = false;
	  }
	  if (speakDiriectionUp) // direction up
	  {
		  if (!isSpeakDiriectionUp)
		  {
			  DF_Play(37);
			  while (uart_data[6] != 37);
			  isSpeakDiriectionUp = true;
		  }
	  } else
	  {
		  isSpeakDiriectionUp = false;
	  }
	  if (speakDirectionDown) // direction dowwn
	  {
		  if (!isSpeakDirectionDown)
		  {
			  DF_Play(38);
			  while (uart_data[6] != 38);
			  isSpeakDirectionDown = true;
		  }
	  } else
	  {
		  isSpeakDirectionDown = false;
	  }
	  if (speakFire) // fire
	  {
		  DF_Play(39);
		  while (uart_data[6] != 39);
		  HAL_Delay(1000);
		  speakFire = false;
	  }

	  if (speakOverLoad) // overload
	  {
		  DF_Play(40);
		  while (uart_data[6] != 40);
		  HAL_Delay(1000);
		  speakOverLoad = false;
	  }
	  HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 36;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(S_CAN_GPIO_Port, S_CAN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : S_CAN_Pin */
  GPIO_InitStruct.Pin = S_CAN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(S_CAN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void Set_speed_can(CAN_HandleTypeDef hcan, bool detect_speed, uint8_t speed)
{
	hcan.Init.Mode = CAN_MODE_SILENT;
	if (speed == 1)
	{
		hcan.Init.Prescaler = 30;
		hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
	} else if (speed == 2)
	{
		hcan.Init.Prescaler = 40;
		hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
	} else if (speed == 3)
	{
		hcan.Init.Prescaler = 60;//80
		hcan.Init.TimeSeg1 = CAN_BS1_16TQ;//15
		hcan.Init.TimeSeg2 = CAN_BS2_7TQ;//2
	}
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.SyncJumpWidth = CAN_SJW_3TQ;
	hcan.Init.AutoBusOff = ENABLE;

	HAL_CAN_Init(&hcan);
	if (detect_speed)
	{
		CAN_FilterTypeDef canfilterconfig;
		canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
		canfilterconfig.FilterBank = 0;
		canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		canfilterconfig.FilterIdHigh = 0x501<<5;
		canfilterconfig.FilterIdLow = 0;
		canfilterconfig.FilterMaskIdHigh = 0xFFF<<5;
		canfilterconfig.FilterMaskIdLow = 0;
		canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
		canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
		canfilterconfig.SlaveStartFilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
		canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
		canfilterconfig.FilterBank = 1;
		canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		canfilterconfig.FilterIdHigh = 0x502<<5;
		canfilterconfig.FilterIdLow = 0;
		canfilterconfig.FilterMaskIdHigh = 0xFFF<<5;
		canfilterconfig.FilterMaskIdLow = 0;
		canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
		canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
		canfilterconfig.SlaveStartFilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
		canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
		canfilterconfig.FilterBank = 3;
		canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		canfilterconfig.FilterIdHigh = 0x503<<5;
		canfilterconfig.FilterIdLow = 0;
		canfilterconfig.FilterMaskIdHigh = 0xFFF<<5;
		canfilterconfig.FilterMaskIdLow = 0;
		canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
		canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
		canfilterconfig.SlaveStartFilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
	} else
	{
		CAN_FilterTypeDef canfilterconfig;
		canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
		canfilterconfig.FilterBank = 0;
		canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		canfilterconfig.FilterIdHigh = 0<<5;
		canfilterconfig.FilterIdLow = 0;
		canfilterconfig.FilterMaskIdHigh = 0<<5;
		canfilterconfig.FilterMaskIdLow = 0;
		canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
		canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
		canfilterconfig.SlaveStartFilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
	}
	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void speaker_floor(uint8_t tenFloor, uint8_t oneFloor)
{
	uint8_t floor;
	if (tenFloor == 0x48) // H
	{
		if (oneFloor == 0x31)
		{
			floor = 33;
		} else if (oneFloor == 0x32)
		{
			floor = 34;
		}
	} else if (tenFloor == 0x20)
	{
		floor = oneFloor - 0x30;
	} else
	{
		floor = (tenFloor - 0x30)*10 + (oneFloor - 0x30);
	}
	DF_Play(floor);
	while (uart_data[6] != floor);
}
void speaker_push_button(uint8_t speakerButton[5][4])
{
	if (speakerButton[0][0] == 1)
	{
		if (speakerButton[0][1] == 1)
		{
			// đọc cancel
		}
		// đọc tầng
		speaker_floor(speakerButton[0][2], speakerButton[0][3]);
		for (uint8_t i=0; i<4; i++)
		{
			speakerButton[i][0] = speakerButton[i+1][0];
			speakerButton[i][1] = speakerButton[i+1][1];
			speakerButton[i][2] = speakerButton[i+1][2];
			speakerButton[i][3] = speakerButton[i+1][3];
		}
		speakerButton[4][0] = 0;
		speakerButton[4][1] = 0;
		speakerButton[4][2] = 0;
		speakerButton[4][3] = 0;
	}
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
#ifdef USE_FULL_ASSERT
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
