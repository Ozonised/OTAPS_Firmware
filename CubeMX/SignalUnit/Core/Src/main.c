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
 *
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
#include "nrf24.h"
#include "config.h"
#include "nodes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	SLAVE, MASTER
} NodeType;

typedef struct CommunicatingNode
{
	NRF24 *nrf;
	Payload *pl;
	struct CommunicatingNode *next;
	uint8_t nodeID;
} ComNode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
static NRF24 nrf1 = { &hspi1, NRF_CSN1_GPIO_Port, NRF_CSN1_Pin,
NRF_CE1_Pin, NRF_IRQ1_Pin };
static NRF24 nrf2 = { &hspi1, NRF_CSN2_GPIO_Port, NRF_CSN2_Pin,
NRF_CE2_Pin, NRF_IRQ2_Pin };
static NRF24 nrf3 = { &hspi1, NRF_CSN3_GPIO_Port, NRF_CSN3_Pin,
NRF_CE3_Pin, NRF_IRQ3_Pin };
static Payload locomotiveNodePayload = { 0 }, prevNodePayload = { 0 }, nextNodePayload = { 0 };
static ComNode prevSignalNode = { 0 }, nextSignalNode = { 0 }, locomotiveNode = { 0 };
static ComNode *currentComNode = NULL;

static volatile bool nrf1IRQTriggered = false, nrf2IRQTriggered = false, nrf3IRQTriggered = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void inline radioInit(NRF24 *nrf24, const uint8_t *address, uint8_t RFChannel, uint8_t operationMode);
static void inline masterNode(void);
static void inline slaveNode(void);
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
	NodeType nodeType = (THIS_NODE_NUM % 2) ? SLAVE : MASTER;
	uint8_t nrf24OperationMode = nRF24_MODE_RX;
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
	MX_I2C1_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	signalStateInit();

	// initialize the nrf module for lower signal node and prev signal node structure
	if (thisNode.prev != NULL)
	{
		radioInit(&nrf1, PREV_NODE_ADDRESS, NRF24_LOWER_NODE_RF_CHANNEL, nrf24OperationMode);
		prevSignalNode.nrf = &nrf1;
		prevSignalNode.pl = &prevNodePayload;
		prevSignalNode.next = &nextSignalNode;
		prevSignalNode.nodeID = thisNode.prev->nodeID;
	}

	// initialize the nrf module for lower signal node and next signal node structure
	if (thisNode.next != NULL)
	{
		radioInit(&nrf2, NEXT_NODE_ADDRESS, NRF24_HIGHER_NODE_RF_CHANNEL, nrf24OperationMode);

		nextSignalNode.nrf = &nrf2;
		nextSignalNode.pl = &nextNodePayload;
		nextSignalNode.next = &prevSignalNode;
		nextSignalNode.nodeID = thisNode.next->nodeID;
	}

	radioInit(&nrf3, LOCOMOTIVE_NODE_ADDRESS, NRF24_LOCOMOTIVE_NODE_RF_CHANNEL, nRF24_MODE_RX);

	locomotiveNode.nrf = &nrf3;
	nextSignalNode.pl = &locomotiveNodePayload;
	nextSignalNode.next = NULL;

	// if node type is master then set nrf module into PTX mode and start communication with previous node first
	// else set nrf module into PRX and start communication with the next node
	if (nodeType == MASTER)
	{
		nrf24OperationMode = nRF24_MODE_TX;
		currentComNode = &prevSignalNode;
	} else
	{
		nrf24OperationMode = nRF24_MODE_RX;
		currentComNode = &nextSignalNode;
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	updateTxPayload(&prevNodePayload, thisNode.prev->nodeID);
	updateTxPayload(&nextNodePayload, thisNode.next->nodeID);
	// update locomotive node payload

	while (1)
	{
		// reset axle counter and signal state
		if (HAL_GPIO_ReadPin(SIGNAL_RST_SW_GPIO_Port, SIGNAL_RST_SW_Pin) == GPIO_PIN_RESET)
		{
			axleCounter = 0;
			currentSignalState = &green;
		}

		switch (nodeType)
		{
		case MASTER:

			break;

		case SLAVE:
			break;

		default:
			break;
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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, NRF_CE3_Pin | NRF_CE2_Pin | LED_RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, NRF_CSN3_Pin | NRF_CSN2_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, NRF_CE1_Pin | LED_YELLOW1_Pin | LED_GREEN_Pin | LED_YELLOW2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF_CSN1_GPIO_Port, NRF_CSN1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : NRF_CE3_Pin NRF_CSN3_Pin NRF_CE2_Pin NRF_CSN2_Pin
	 LED_RED_Pin */
	GPIO_InitStruct.Pin = NRF_CE3_Pin | NRF_CSN3_Pin | NRF_CE2_Pin | NRF_CSN2_Pin | LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_IRQ2_Pin */
	GPIO_InitStruct.Pin = NRF_IRQ2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NRF_IRQ2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_CE1_Pin NRF_CSN1_Pin LED_YELLOW1_Pin LED_GREEN_Pin
	 LED_YELLOW2_Pin */
	GPIO_InitStruct.Pin = NRF_CE1_Pin | NRF_CSN1_Pin | LED_YELLOW1_Pin | LED_GREEN_Pin | LED_YELLOW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_IRQ1_Pin PROX_SNSR_Pin NRF_IRQ3_Pin */
	GPIO_InitStruct.Pin = NRF_IRQ1_Pin | PROX_SNSR_Pin | NRF_IRQ3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SIGNAL_RST_SW_Pin */
	GPIO_InitStruct.Pin = SIGNAL_RST_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SIGNAL_RST_SW_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Initializes NRF24 modules
 *
 * @param nrf24 pointer to NRF24 object
 * @param address pointer to 5 byte address
 * @param RFChannel radio frequency channel, value from 0 to 124
 * @param operationMode one of nRF24_MODE_xx values
 *
 * @return None
 */
static void inline radioInit(NRF24 *nrf24, const uint8_t *address, uint8_t RFChannel, uint8_t operationMode)
{
	if (nrf24 != NULL)
	{
		nRF24_Init(nrf24);
		nRF24_SetAddrWidth(nrf24, 5);
		nRF24_SetRFChannel(nrf24, RFChannel);
		nRF24_SetDataRate(nrf24, NRF24_DATA_RATE);
		nRF24_SetCRCScheme(nrf24, nRF24_CRC_2byte);
		nRF24_SetTXPower(nrf24, NRF24_TX_PWR);
		nRF24_SetAutoRetr(nrf24, NRF24_AUTO_RETRY_DELAY, NRF24_AUTO_RETRY_COUNT);
		nRF24_SetAddr(nrf24, nRF24_PIPETX, address);
		nRF24_SetAddr(nrf24, nRF24_PIPE0, address);
		nRF24_EnableAA(nrf24, nRF24_PIPE0);
		nRF24_SetOperationalMode(nrf24, operationMode);
		nRF24_SetDynamicPayloadLength(nrf24, nRF24_DPL_ON);
		nRF24_SetPayloadWithAck(nrf24, 1);
		nRF24_SetPowerMode(nrf24, nRF24_PWR_UP);
	}
}

static void inline masterNode(void)
{
	static unsigned long currentMillis, prevTxMillis;
	uint8_t payloadLength = PAYLOAD_LENGTH;
	bool payload1Valid = false, payload2Valid = false;

	currentMillis = HAL_GetTick();

	if (currentMillis - prevTxMillis >= TX_INTERVAL)
	{
		// Deassert the CE pin (in case if it still high)
		nRF24_CE_L(currentComNode->nrf);

		// Transfer a data from the specified buffer to the TX FIFO
		nRF24_WritePayload(currentComNode->nrf, currentComNode->pl->transmitPayload, PAYLOAD_LENGTH);

		// Start a transmission by asserting CE pin (must be held at least 10us)
		nRF24_CE_H(currentComNode->nrf);

		prevTxMillis = currentMillis;
	}

	// payload received from previous node
	if (nrf1IRQTriggered && currentComNode == &prevSignalNode)
	{
		if (nRF24_ReadPayloadDpl(currentComNode->nrf, prevNodePayload.receivePayload, &payloadLength) == nRF24_RX_PIPE0)
		{
			payload1Valid = isPayLoadValid(&prevNodePayload, currentComNode->nodeID);
			if (payload1Valid)
			{
				extractPayloadData(&prevNodePayload, currentComNode->nodeID);
			}
		}

		// switch to next node
		if (currentComNode->next != NULL)
		{
			currentComNode = currentComNode->next;
		}

		nrf1IRQTriggered = false;
	}

	// payload received from next node
	if (nrf2IRQTriggered && currentComNode == &nextSignalNode)
	{
		payloadLength = PAYLOAD_LENGTH;
		if (nRF24_ReadPayloadDpl(currentComNode->nrf, nextNodePayload.receivePayload, &payloadLength) == nRF24_RX_PIPE0)
		{
			payload2Valid = isPayLoadValid(&nextNodePayload, currentComNode->nodeID);
			if (payload2Valid)
			{
				extractPayloadData(&nextNodePayload, currentComNode->nodeID);
			}
		}

		if (currentComNode->next != NULL)
		{
			currentComNode = currentComNode->next;
		}
		nrf2IRQTriggered = false;
	}

	// payload received from locomotive node
	if (nrf3IRQTriggered)
	{
		payloadLength = PAYLOAD_LENGTH;
		if (nRF24_ReadPayloadDpl(locomotiveNode.nrf, locomotiveNodePayload.receivePayload, &payloadLength)
				== nRF24_RX_PIPE0)
		{
			// change this verification method
			if (isPayLoadValid(&locomotiveNodePayload, currentComNode->nodeID))
			{
				extractPayloadData(&locomotiveNodePayload, currentComNode->nodeID);
			}
		}
		nrf3IRQTriggered = false;
	}

	if (payload1Valid || payload2Valid)
	{
		updateTxPayload(&prevNodePayload, thisNode.prev->nodeID);
		updateTxPayload(&nextNodePayload, thisNode.next->nodeID);
		// update locomotive node payload

		if (thisNode.nodeReady != true)
		{
			thisNode.nodeReady = isNodeReady();

			// all led ON
			HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED_YELLOW1_GPIO_Port, LED_GREEN_Pin | LED_YELLOW1_Pin | LED_YELLOW2_Pin, GPIO_PIN_SET);
		} else
		{
			updateSignalState();
			setSignalLeds();
		}
	}

}

static void inline slaveNode(void)
{
	static unsigned long currentMillis, prevTxMillis;

	currentMillis = HAL_GetTick();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	// port code from previous iteration

	// deassert nrfs ce pin
	// set IRQ trigger flags
	if (GPIO_Pin == NRF_IRQ1_Pin)
	{
		nrf1IRQTriggered = true;
		nRF24_CE_L(&nrf1);
	}
	if (GPIO_Pin == NRF_IRQ2_Pin)
	{
		nrf2IRQTriggered = true;
		nRF24_CE_L(&nrf2);
	}
	if (GPIO_Pin == NRF_IRQ3_Pin)
	{
		nrf3IRQTriggered = true;
		nRF24_CE_L(&nrf3);
	}

	if (GPIO_Pin == PROX_SNSR_Pin)
	{
		if (HAL_GPIO_ReadPin(PROX_SNSR_GPIO_Port, PROX_SNSR_Pin) == GPIO_PIN_RESET)
		{
			axleCounter++;
		}

		if (currentSignalState->state != RED)
		{
			currentSignalState = &red;

			if (THIS_NODE_NUM == 1)
			{
				// for first node
				// check next node axle counter
				// if it is greater than this node than train is moving towards lower node
				// else it is moving towards higher node
				if (thisNode.next->axleCount > axleCounter)
				{
					trainDir = TO_LOWER_NODE;
				} else if (thisNode.next->axleCount < axleCounter)
				{
					trainDir = TO_HIGHER_NODE;
				} else
				{
					trainDir = TRAIN_DIR_NOT_KNOWN;
				}
			}
			else if (THIS_NODE_NUM == TOTAL_NO_OF_NODES)
			{
				// for last node
				// check prev node axle counter
				// if it is greater than this node than train is moving towards higher node
				// else it is moving towards lower node
				if (thisNode.prev->axleCount > axleCounter)
				{
					trainDir = TO_HIGHER_NODE;
				} else if (thisNode.prev->axleCount < axleCounter)
				{
					trainDir = TO_LOWER_NODE;
				} else
				{
					trainDir = TRAIN_DIR_NOT_KNOWN;
				}
			}
			else
			{
				// for  center node
				// check if prev node axle counter is greater than next node axle counter, train is moving towards higher node
				// else if prev node axle counter is less than next node axle counter, train is moving towards lower node
				if (thisNode.prev->axleCount > axleCounter && thisNode.next->axleCount < axleCounter)
				{
					trainDir = TO_HIGHER_NODE;
				} else if (thisNode.prev->axleCount < axleCounter && thisNode.next->axleCount > axleCounter)
				{
					trainDir = TO_LOWER_NODE;
				} else
				{
					trainDir = TRAIN_DIR_NOT_KNOWN;
				}
			}
		}
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
