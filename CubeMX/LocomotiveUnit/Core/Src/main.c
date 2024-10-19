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
#include <stdbool.h>
#include "nrf24.h"
#include "ui.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
static volatile bool nrf1IRQTriggered = false;

static NRF24 nrf24 = { &hspi1, NRF_CSN1_GPIO_Port, NRF_CSN1_Pin,
NRF_CE1_Pin, NRF_IRQ1_Pin };

Locomotive locomotive = { 0 };

SSD1306 display;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

static void inline setSpeed(uint8_t percentage)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (percentage * MAX_PWM_VAL) / 100);
}

static void inline locomotiveOperation(void);
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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	locomotive.id = LOCOMOTIVE_NODE_ID;
	locomotive.dir = TO_HIGHER_NODE;
	locomotive.comNodeNo = 1;
	locomotive.comNodeID = NODE_IDS[locomotive.comNodeNo - 1];

	nRF24_Init(&nrf24);
	nRF24_SetAddrWidth(&nrf24, 5);
	nRF24_SetRFChannel(&nrf24, RF_CHANNEL);
	nRF24_SetDataRate(&nrf24, NRF24_DATA_RATE);
	nRF24_SetCRCScheme(&nrf24, nRF24_CRC_2byte);
	nRF24_SetTXPower(&nrf24, NRF24_TX_PWR);
	nRF24_SetAutoRetr(&nrf24, NRF24_AUTO_RETRY_DELAY, NRF24_AUTO_RETRY_COUNT);
	nRF24_EnableAA(&nrf24, nRF24_PIPE0);
	nRF24_SetOperationalMode(&nrf24, nRF24_MODE_TX);
	nRF24_SetDynamicPayloadLength(&nrf24, nRF24_DPL_ON);
	nRF24_SetPayloadWithAck(&nrf24, 1);
	nRF24_SetPowerMode(&nrf24, nRF24_PWR_UP);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	UI_Init(&display);
	updateUI(&display, &locomotive);
	while (1)
	{
		/* USER CODE END WHILE */
		locomotiveOperation();
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
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 100;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 400;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, NRF_CE1_Pin | LED_YELLOW1_Pin | LED_GREEN_Pin | LED_YELLOW2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NRF_CSN1_GPIO_Port, NRF_CSN1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_CE1_Pin NRF_CSN1_Pin LED_YELLOW1_Pin LED_GREEN_Pin
	 LED_YELLOW2_Pin */
	GPIO_InitStruct.Pin = NRF_CE1_Pin | NRF_CSN1_Pin | LED_YELLOW1_Pin | LED_GREEN_Pin | LED_YELLOW2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_IRQ1_Pin */
	GPIO_InitStruct.Pin = NRF_IRQ1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NRF_IRQ1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TRAIN_DIR_SW_Pin */
	GPIO_InitStruct.Pin = TRAIN_DIR_SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TRAIN_DIR_SW_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_RED_Pin */
	GPIO_InitStruct.Pin = LED_RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void inline locomotiveOperation(void)
{
	static unsigned long prevTxMillis, prevSwitchMillis;
	static Signal prevSignalState = SIGNAL_NOT_KNOWN;
	static Payload payload = { 0 };
	bool isPayloadValid = false, currentSwitchState, switchedNode = false;
	static bool prevSwitchState = true;

	unsigned long currentMillis = HAL_GetTick();

	// train dir switch state
	if (currentMillis - prevSwitchMillis >= SW_SAMPLING_INTERVAL)
	{
		currentSwitchState = HAL_GPIO_ReadPin(TRAIN_DIR_SW_GPIO_Port, TRAIN_DIR_SW_Pin);

		// switch has been pressed, change locomotive direction
		if (currentSwitchState == GPIO_PIN_RESET && prevSwitchState == GPIO_PIN_SET)
		{
			switch (locomotive.dir)
			{
			case TO_HIGHER_NODE:
				locomotive.dir = TO_LOWER_NODE;
				locomotive.comNodeNo = TOTAL_NO_OF_NODES;
				locomotive.comNodeID = NODE_IDS[locomotive.comNodeNo - 1];
				break;

			default:
				locomotive.dir = TO_HIGHER_NODE;
				locomotive.comNodeNo = 1;
				locomotive.comNodeID = NODE_IDS[locomotive.comNodeNo - 1];
				break;
			}
			prevSwitchState = currentSwitchState;
		}
		prevSwitchMillis = currentMillis;
	}

	if (currentMillis - prevTxMillis >= TX_INTERVAL)
	{
		nRF24_SetAddr(&nrf24, nRF24_PIPETX, NODE_ADDRESS[locomotive.comNodeNo]);
		nRF24_SetAddr(&nrf24, nRF24_PIPE0, NODE_ADDRESS[locomotive.comNodeNo]);

		// Deassert the CE pin (in case if it still high)
		nRF24_CE_L(&nrf24);

		if (nRF24_GetStatus_TXFIFO(&nrf24) == nRF24_STATUS_TXFIFO_FULL)
			nRF24_FlushTX(&nrf24);

		if (nRF24_GetStatus_RXFIFO(&nrf24) == nRF24_STATUS_RXFIFO_FULL)
			nRF24_FlushRX(&nrf24);

		// Transfer a data from the specified buffer to the TX FIFO
		nRF24_WritePayload(&nrf24, payload.transmitPayload, PAYLOAD_LENGTH);

		// Start a transmission by asserting CE pin (must be held at least 10us)
		nRF24_CE_H(&nrf24);

		prevTxMillis = currentMillis;
	}

	if (nrf1IRQTriggered)
	{
		uint8_t payloadLength = PAYLOAD_LENGTH;
		uint8_t status = 0;

		status = nRF24_GetStatus(&nrf24);

		// nRF24_FLAG_RX_DR bit is when the ack payload is received from PRX
		if ((status & nRF24_FLAG_RX_DR )
				&& nRF24_ReadPayloadDpl(&nrf24, payload.receivePayload, &payloadLength) == nRF24_RX_PIPE0)
		{
			isPayloadValid = isPayLoadValid(&locomotive, &payload);
			if (isPayloadValid)
			{
				extractPayloadData(&locomotive, &payload);
				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
			}
		}

		// Deassert the CE pin (Standby-II --> Standby-I)
		nRF24_CE_L(&nrf24);
		nRF24_ClearIRQFlags(&nrf24);

		nrf1IRQTriggered = false;
	}

	if (isPayloadValid)
	{
		updateLocomotiveState(&locomotive);

		// if the current communicating node signal turned red but was previously green, double yellow or yellow
		// then the locomotive as reached the current communicating signal.
		if (locomotive.signalData[0] == RED
				&& (prevSignalState == GREEN || prevSignalState == DOUBLE_YELLOW || prevSignalState == YELLOW))
		{
			// switching to next communication node
			switch (locomotive.dir)
			{
			case TO_HIGHER_NODE:
				if (locomotive.comNodeNo < TOTAL_NO_OF_NODES)
					locomotive.comNodeNo++;
				break;

			case TO_LOWER_NODE:
				if (locomotive.comNodeNo > 1)
					locomotive.comNodeNo--;
				break;

			default:
				break;
			}
			switchedNode = true;
			prevSignalState = locomotive.signalData[0];
		}

		// the train switches the node when the signal it is communicating with turned RED
		// signifying that the train crossed that signal.
		// However, this changes the locomotive state to STOP which makes the train to apply brakes until it has acquired the
		// state of the next signal. To avoid this behavior, a check is made to see whether the train has switched nodes
		// if it has then it wont change the speed
		if (switchedNode == false)
		{
			// set the speed as per the locomotive state
			switch (locomotive.state)
			{
			case GO:
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				setSpeed(60);
				break;
			case SLOW_DOWN:
				setSpeed(30);
				break;

			case STOP:
				setSpeed(0);
				break;

			default:
				break;
			}
		}
		updateUI(&display, &locomotive);
		switchedNode = false;
		isPayloadValid = false;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == NRF_IRQ1_Pin)
	{
		nrf1IRQTriggered = true;
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
