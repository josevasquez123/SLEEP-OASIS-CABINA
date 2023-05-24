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
#define N_SAMPLES 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

uint8_t can_id = 0x03;								//CAN ID

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint32_t TxMailBox;
uint8_t RxData[8];

ADC_ChannelConfTypeDef sConfig;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void init_can_id(void);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);
float average(uint32_t channel);
bool led_status(uint32_t channel);
void activar(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t channel);
void desactivar(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * TOPICOS MQTT
 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

	/*
	 * CHECK-IN: PRENDE LUZ TECHO Y LUZ DE NUMERO DE CABINA
	 */
	if(RxData[1] == 0x01){
		if(RxData[0]==0x01){					//PRENDER
			activar(LUZ_TECHO_GPIO_Port, LUZ_TECHO_Pin, ADC_CHANNEL_2);
			desactivar(LUZ_MESA_GPIO_Port, LUZ_MESA_Pin, ADC_CHANNEL_4);
			activar(LUZ_CABECERA_GPIO_Port, LUZ_CABECERA_Pin, ADC_CHANNEL_3);
			HAL_GPIO_WritePin(LUZ_NCABINA_GPIO_Port, LUZ_NCABINA_Pin, GPIO_PIN_SET);
		}
	}

	/*
	 * STAND-BY: ESTADO DE ESPERA HASTA QUE ENTRE UN CLIENTE
	 */
	else if(RxData[1] == 0x02){
		if(RxData[0]==0x01){
			desactivar(LUZ_TECHO_GPIO_Port, LUZ_TECHO_Pin, ADC_CHANNEL_2);
			desactivar(LUZ_MESA_GPIO_Port, LUZ_MESA_Pin, ADC_CHANNEL_4);
			activar(LUZ_CABECERA_GPIO_Port, LUZ_CABECERA_Pin, ADC_CHANNEL_3);
			HAL_GPIO_WritePin(LUZ_NCABINA_GPIO_Port, LUZ_NCABINA_Pin, GPIO_PIN_RESET);
		}
	}

	/*
	 * REINGRESO: CUANDO UNA PERSONA SE VA AL BAÑO Y CIERRA SU PUERTA, CUANDO VUELVE
	 * REINGRESA DENUEVO Y SE LE ABRE LA PUERTA DE SU CABINA
	 */
	else if(RxData[1] == 0x03){
		if(RxData[0]==0x01){
			HAL_GPIO_WritePin(CANTONERA_GPIO_Port, CANTONERA_Pin, GPIO_PIN_SET);
		}
	}

	/*
	 * PUERTA: MANERA MANUAL DE ABRIR O CERRAR LA CANTONERA DE LA PUERTA
	 */
	else if(RxData[1] == 0x04){
		if(RxData[0]==0x01){
			HAL_GPIO_WritePin(CANTONERA_GPIO_Port, CANTONERA_Pin, GPIO_PIN_SET);
		}
		else if(RxData[0]==0x00){
			HAL_GPIO_WritePin(CANTONERA_GPIO_Port, CANTONERA_Pin, GPIO_PIN_RESET);
		}
	}

	/*
	 * LUZ-TECHO: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DEL TECHO
	 */
	else if(RxData[1] == 0x05){
		if(RxData[0]==0x01){
			activar(LUZ_TECHO_GPIO_Port, LUZ_TECHO_Pin, ADC_CHANNEL_2);
		}
		else if(RxData[0]==0x00){
			desactivar(LUZ_TECHO_GPIO_Port, LUZ_TECHO_Pin, ADC_CHANNEL_2);
		}
	}

	/*
	 * LUZ-CABECERA: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DE LA CABECERA
	 */
	else if(RxData[1] == 0x06){
		if(RxData[0]==0x01){
			activar(LUZ_CABECERA_GPIO_Port, LUZ_CABECERA_Pin, ADC_CHANNEL_3);
		}
		else if(RxData[0]==0x00){
			desactivar(LUZ_CABECERA_GPIO_Port, LUZ_CABECERA_Pin, ADC_CHANNEL_3);
		}
	}

	/*
	 * LUZ-MESA: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DE LA MESA
	 */
	else if(RxData[1] == 0x07){
		if(RxData[0]==0x01){
			activar(LUZ_MESA_GPIO_Port, LUZ_MESA_Pin, ADC_CHANNEL_4);
		}
		else if(RxData[0]==0x00){
			desactivar(LUZ_MESA_GPIO_Port, LUZ_MESA_Pin, ADC_CHANNEL_4);
		}
	}

	/*
	 * LUZ-NCABINA: MANERA MANUAL DE PRENDER Y APAGAR LA LUZ DEL INDICADOR DEL NUMERO DE CABINA
	 */
	else if(RxData[1] == 0x08){
		if(RxData[0]==0x01){
			HAL_GPIO_WritePin(LUZ_NCABINA_GPIO_Port, LUZ_NCABINA_Pin, GPIO_PIN_SET);
		}
		else if(RxData[0]==0x00){
			HAL_GPIO_WritePin(LUZ_NCABINA_GPIO_Port, LUZ_NCABINA_Pin, GPIO_PIN_RESET);
		}
	}
}

/*
 * BOTON PANICO EXT INT
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	currentMillis = HAL_GetTick();

	if(GPIO_Pin == PUERTA_INT_Pin && (currentMillis - previousMillis > 500)){
		uint8_t TxData[2] = {0x09, can_id};			//{BOTON_PANICO, ID CABINA}
		TxHeader.DLC = 2;
		TxHeader.ExtId = 0;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.StdId = 0x19;						//ID DEL MB -> 25 = 0x19
		TxHeader.TransmitGlobalTime = DISABLE;

		HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailBox);
		previousMillis = currentMillis;
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
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  //init_can_id();
  MX_CAN_Init();

  HAL_ADC_Start(&hadc1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
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
  HAL_GPIO_WritePin(GPIOA, LUZ_TECHO_Pin|LUZ_CABECERA_Pin|CANTONERA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LUZ_NCABINA_Pin|LUZ_MESA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LUZ_TECHO_Pin LUZ_CABECERA_Pin CANTONERA_Pin */
  GPIO_InitStruct.Pin = LUZ_TECHO_Pin|LUZ_CABECERA_Pin|CANTONERA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LUZ_NCABINA_Pin LUZ_MESA_Pin */
  GPIO_InitStruct.Pin = LUZ_NCABINA_Pin|LUZ_MESA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PUERTA_INT_Pin PANIC_INT_Pin */
  GPIO_InitStruct.Pin = PUERTA_INT_Pin|PANIC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ID_CAN_3_Pin ID_CAN_4_Pin ID_CAN_5_Pin ID_CAN_2_Pin
                           ID_CAN_1_Pin */
  GPIO_InitStruct.Pin = ID_CAN_3_Pin|ID_CAN_4_Pin|ID_CAN_5_Pin|ID_CAN_2_Pin
                          |ID_CAN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/*
 * PRENDER LA LUZ
 */

void activar(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t channel){
	if (led_status(channel)) return;
	else{
		while(!led_status(channel)){
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
			if (led_status(channel)) return;
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
			if (led_status(channel)) return;
		}
	}
}

/*
 * APAGAR LA LUZ
 */

void desactivar(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, uint32_t channel){
	if (!led_status(channel)) return;
	else{
		while(led_status(channel)){
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
			if (!led_status(channel)) return;
			HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
			if (!led_status(channel)) return;
		}
	}
}

/*
 * VER EL ESTADO DE LA LUZ, SI ESTA APAGADO O PRENDIDO
 */

bool led_status(uint32_t channel){
  float voltage_led = average(channel);
  if(voltage_led > 20.0){
    return true;
  }
  else{
    return false;
  }
}

/*
 * AVERAGE DE LA LECTURA DEL ADC PARA LAS LUCES
 */

float average(uint32_t channel)
{
  float sum = 0 ;
  for (int i = 0 ; i < N_SAMPLES ; i++){
	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	int analogVal = HAL_ADC_GetValue(&hadc1);
    float voltage = fmap(analogVal, 0, 4096, 0.0, 25.0);
    sum += voltage;
    HAL_Delay(10);
  }
  return sum / N_SAMPLES;
}

/*
 *  FORMULA QUE PASA DE 0 - 25V A 0 - 3.3V
 */

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
