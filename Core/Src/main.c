/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum { false, true } bool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */

uint8_t can_id=0;								//CAN ID

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailBox;
uint8_t RxData[8];
uint8_t TxData[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void init_can_id(void);
void send_ack(void);
void activar(GPIO_TypeDef *GPIO_I_Port, uint16_t GPIO_I_Pin, GPIO_TypeDef *GPIO_O_Port, uint16_t GPIO_O_Pin);
void desactivar(GPIO_TypeDef *GPIO_I_Port, uint16_t GPIO_I_Pin, GPIO_TypeDef *GPIO_O_Port, uint16_t GPIO_O_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * TOPICOS MQTT
 */

/*
 * ADC CHANNELS
 * LUZ CABECERA ANALOG = CHANNEL 7
 * LUZ MESA ANALOG = CHANNEL 8
 * LUZ TECHO ANALOG = CHANNEL 9
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	//send_ack();			//ENVIANDO MENSAJE AL MB CONFIRMANDO QUE LLEGO EL MENSAJE OK

	/*
	 * INGRESO: PRENDE LUZ TECHO Y LUZ DE NUMERO DE CABINA
	 */
	if(RxData[1] == 0x01){
		if(RxData[0]==0x01){
			activar(I_TECHO_GPIO_Port, I_TECHO_Pin, TECHO_GPIO_Port, TECHO_Pin);
			desactivar(I_MESA_GPIO_Port, I_MESA_Pin, MESA_GPIO_Port, MESA_Pin);
			desactivar(I_CABECERA_GPIO_Port, I_CABECERA_Pin, CABECERA_GPIO_Port, CABECERA_Pin);
			HAL_GPIO_WritePin(LETRERO_GPIO_Port, LETRERO_Pin, GPIO_PIN_SET);
		}

		send_ack();
	}

	/*
	 * APAGADO: APAGAR TODA LAS LUCES
	 */
	else if(RxData[1] == 0x02){
		if(RxData[0]==0x01){
			desactivar(I_TECHO_GPIO_Port, I_TECHO_Pin, TECHO_GPIO_Port, TECHO_Pin);
			desactivar(I_MESA_GPIO_Port, I_MESA_Pin, MESA_GPIO_Port, MESA_Pin);
			desactivar(I_CABECERA_GPIO_Port, I_CABECERA_Pin, CABECERA_GPIO_Port, CABECERA_Pin);
			HAL_GPIO_WritePin(LETRERO_GPIO_Port, LETRERO_Pin, GPIO_PIN_RESET);
		}

		send_ack();
	}


	/*
	 * LUZ-TECHO: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DEL TECHO
	 */
	else if(RxData[1] == 0x04){
		if(RxData[0]==0x01){
			activar(I_TECHO_GPIO_Port, I_TECHO_Pin, TECHO_GPIO_Port, TECHO_Pin);
		}
		else if(RxData[0]==0x00){
			desactivar(I_TECHO_GPIO_Port, I_TECHO_Pin, TECHO_GPIO_Port, TECHO_Pin);
		}

		send_ack();
	}

	/*
	 * LUZ-CABECERA: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DE LA CABECERA
	 */
	else if(RxData[1] == 0x05){
		if(RxData[0]==0x01){
			activar(I_CABECERA_GPIO_Port, I_CABECERA_Pin, CABECERA_GPIO_Port, CABECERA_Pin);
		}
		else if(RxData[0]==0x00){
			desactivar(I_CABECERA_GPIO_Port, I_CABECERA_Pin, CABECERA_GPIO_Port, CABECERA_Pin);
		}

		send_ack();
	}

	/*
	 * LUZ-MESA: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DE LA MESA
	 */
	else if(RxData[1] == 0x06){
		if(RxData[0]==0x01){
			activar(I_MESA_GPIO_Port, I_MESA_Pin, MESA_GPIO_Port, MESA_Pin);
		}
		else if(RxData[0]==0x00){
			desactivar(I_MESA_GPIO_Port, I_MESA_Pin, MESA_GPIO_Port, MESA_Pin);
		}

		send_ack();
	}

	/*
	 * LUZ-NCABINA: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DEL INDICADOR DEL NUMERO DE CABINA
	 */
	else if(RxData[1] == 0x07){
		if(RxData[0]==0x01){
			HAL_GPIO_WritePin(LETRERO_GPIO_Port, LETRERO_Pin, GPIO_PIN_SET);
		}
		else if(RxData[0]==0x00){
			HAL_GPIO_WritePin(LETRERO_GPIO_Port, LETRERO_Pin, GPIO_PIN_RESET);
		}

		send_ack();
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  init_can_id();
  MX_CAN_Init();

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = can_id<<5;
  //canfilterconfig.FilterIdHigh = 0;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x7FF<<5;
  //canfilterconfig.FilterMaskIdHigh = 0;
  canfilterconfig.FilterMaskIdLow = 0x0000 ;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 0;

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, TECHO_Pin|CABECERA_Pin|MESA_Pin|LETRERO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : I_CABECERA_Pin I_MESA_Pin I_TECHO_Pin */
  GPIO_InitStruct.Pin = I_CABECERA_Pin|I_MESA_Pin|I_TECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ID_CAN_1_Pin ID_CAN_2_Pin ID_CAN_3_Pin ID_CAN_4_Pin
                           ID_CAN_5_Pin */
  GPIO_InitStruct.Pin = ID_CAN_1_Pin|ID_CAN_2_Pin|ID_CAN_3_Pin|ID_CAN_4_Pin
                          |ID_CAN_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TECHO_Pin CABECERA_Pin MESA_Pin LETRERO_Pin */
  GPIO_InitStruct.Pin = TECHO_Pin|CABECERA_Pin|MESA_Pin|LETRERO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * MENSAJE ACK AL MOTHERBOARD
 */

void send_ack(void){
	TxHeader.DLC = 2;
	TxHeader.ExtId = 0;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x1B;							//MB ID_CAN
	TxHeader.TransmitGlobalTime = DISABLE;

	TxData[0] = 0x09;
	TxData[1] = can_id;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
}

/*
 * PRENDER LA LUZ
 */

void activar(GPIO_TypeDef *GPIO_I_Port, uint16_t GPIO_I_Pin, GPIO_TypeDef *GPIO_O_Port, uint16_t GPIO_O_Pin){
	if (HAL_GPIO_ReadPin(GPIO_I_Port, GPIO_I_Pin)) return;
	else{
		HAL_GPIO_WritePin(GPIO_O_Port, GPIO_O_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		if (HAL_GPIO_ReadPin(GPIO_I_Port, GPIO_I_Pin)) return;
		HAL_GPIO_WritePin(GPIO_O_Port, GPIO_O_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
		if (HAL_GPIO_ReadPin(GPIO_I_Port, GPIO_I_Pin)) return;
	}
}

/*
 * APAGAR LA LUZ
 */

void desactivar(GPIO_TypeDef *GPIO_I_Port, uint16_t GPIO_I_Pin, GPIO_TypeDef *GPIO_O_Port, uint16_t GPIO_O_Pin){
	if (!HAL_GPIO_ReadPin(GPIO_I_Port, GPIO_I_Pin)) return;
	else{
		HAL_GPIO_WritePin(GPIO_O_Port, GPIO_O_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		if (!HAL_GPIO_ReadPin(GPIO_I_Port, GPIO_I_Pin)) return;
		HAL_GPIO_WritePin(GPIO_O_Port, GPIO_O_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
		if (!HAL_GPIO_ReadPin(GPIO_I_Port, GPIO_I_Pin)) return;
	}
}


/*
 * INICIALIZACION DEL CAN ID
 */
void init_can_id(void){
	if(HAL_GPIO_ReadPin(ID_CAN_1_GPIO_Port, ID_CAN_1_Pin))	can_id += pow(2,0);
	else can_id += 0;
	if(HAL_GPIO_ReadPin(ID_CAN_2_GPIO_Port, ID_CAN_2_Pin))	can_id += pow(2,1);
	else can_id += 0;
	if(HAL_GPIO_ReadPin(ID_CAN_3_GPIO_Port, ID_CAN_3_Pin))	can_id += pow(2,2);
	else can_id += 0;
	if(HAL_GPIO_ReadPin(ID_CAN_4_GPIO_Port, ID_CAN_4_Pin))	can_id += pow(2,3);
	else can_id += 0;
	if(HAL_GPIO_ReadPin(ID_CAN_5_GPIO_Port, ID_CAN_5_Pin))	can_id += pow(2,4);
	else can_id += 0;
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
