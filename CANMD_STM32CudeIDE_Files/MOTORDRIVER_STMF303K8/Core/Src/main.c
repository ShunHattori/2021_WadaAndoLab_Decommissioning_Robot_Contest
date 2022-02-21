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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "xprintf.h"
#include "uart_interface.h"
#include "math_sine_table.h"
#include "stdio.h"
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;
int pattern[4];
int output = 0;

enum RunningMode {
	WAIT = 0, CAN_OUT_ZERO, CAN_RECEIVING, //(but not for me or value is zero)
	CAN_OUT_HIGHSIDE,
	CAN_OUT_LOWSIDE,
} RUNNINGMODE;

enum ButtonMode {
	NONE = 0, HIGHSIDE, LOWSIDE, WAVE,
} BUTTONMODE;

enum WS2812_Pattern {
	BLUE = 0x002856, //OK
	RED = 0xff0000, //OK
	YELLOW = 0x484000, //OK
	PURPLE = 0x260533, //OK
	GREEN = 0x005602, //OK
	ORANGE = 0x3a1700, //OK
} WS2812_PATTERN;

int motor1_output, motor2_output, motor3_output, motor4_output;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void Timer_Start(void);
static void GPIO_Start(void);
static void PWM_Generation(TIM_HandleTypeDef, int, int);
static void CAN_Filter_Init(void);
static int CAN_Polling(void);
static void PARSE_CAN_FRAME(void);
static void WS2812_Applyment(int, int);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

void WS2812_write_PA8(register uint8_t byte) {
	__disable_irq();
	for (int i = 0; i < 8; i++) {
		GPIOA->BSRR = (uint32_t) GPIO_PIN_8;
		if (byte & 0x80) {
			__NOP(); // 800 +- 150 sample 14
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			GPIOA->BRR = (uint32_t) GPIO_PIN_8;
			__NOP(); // 450 +- 150 sample 7
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		} else {
			__NOP(); // 400 +- 150 sample 6
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			GPIOA->BRR = (uint32_t) GPIO_PIN_8;
			__NOP(); // 850 +- 150 sample 15
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
			__NOP();
		}
		byte = byte << 1;
	}
	__enable_irq();
}

void WS2812_apply_color(register uint32_t byte) {
	//	for (int i = 3; i > 0; i--) {
	//		WS2812_write_PA5((byte >> ((i - 1) * 8)) & 0xFF);
	//	}
	// G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0
	WS2812_write_PA8((byte >> 8) & 0xFF);  // RED 8bit
	WS2812_write_PA8((byte >> 16) & 0xFF); // GREEN 8bit
	WS2812_write_PA8((byte >> 0) & 0xFF);  // BLUE 8bit
}

static void WS2812_Applyment(int index, int _pattern) {
	switch (_pattern) {
	case WAIT:
		WS2812_apply_color(PURPLE);
		break;
	case CAN_OUT_ZERO:
		WS2812_apply_color(GREEN);
		break;
	case CAN_OUT_HIGHSIDE:
		WS2812_apply_color(ORANGE);
		break;
	case CAN_OUT_LOWSIDE:
		WS2812_apply_color(BLUE);
		break;
	}
}

static void PWM_Generation(TIM_HandleTypeDef _htim, int channel_set, int _PWM) {
//wrapping value within htim2.Init.Period*0.95
	//CANデータ�?�2byteフル�?�使�?��?�値�?�飛ん�?��??る。　�??れをhtim2.Init.Period�?�スケール�?�変�?��?��?�利用
	//上�?値�?�固定�?��?�プログラムを�?��?�る。周波数を変更�?��?��?��?��?�も自動的�?�変更�?��?�応�?�れる。（ベンリー）
	int16_t pwm = _PWM / (256.0 * 256.0 / 2.0 / _htim.Init.Period);
	int16_t Period_95perc = (_htim.Init.Period * 0.95);
	static int16_t HIGH_SIDE = 0, LOW_SIDE = 0;
	if (pwm < -Period_95perc) {
		pwm = -Period_95perc;
	}
	if (Period_95perc < pwm) {
		pwm = Period_95perc;
	}
	if (pwm < 0) {
		HIGH_SIDE = 0;
		LOW_SIDE = -pwm;
	} else if (0 < pwm) {
		HIGH_SIDE = pwm;
		LOW_SIDE = 0;
	} else {
		HIGH_SIDE = 0;
		LOW_SIDE = 0;
	}
	//回路設計�?�都�?�上1�?ート�?��?�HIGH　LOW�?��??転�?��?��?�る�?��?��?ート�?�よ�?��?��?ャン�?ルを切り替�?�れるよ�?��?��?��?��?�る
	if (channel_set == 1) { //�?��?��?ート�?��?�周り�?��?�れ�?��?�る
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_2, HIGH_SIDE);
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_1, LOW_SIDE);
	}
	if (channel_set == 2) {
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_3, HIGH_SIDE);
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_4, LOW_SIDE);
	}
	if (channel_set == 3) {
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_1, HIGH_SIDE);
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_2, LOW_SIDE);
	}
	if (channel_set == 4) {
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_3, HIGH_SIDE);
		__HAL_TIM_SET_COMPARE(&_htim, TIM_CHANNEL_4, LOW_SIDE);
	}

}

//フィルタ�?�設定�?�る�?��?��?�よ�?��?�ソフト的�?�ID�?�別�?��?��??�?�も�?��?�
//#define CAN_Identifer 0x001

static void PARSE_CAN_FRAME(void) {
//	if (RxHeader.StdId != CAN_Identifer) {
//		return;
//	}

	static int16_t latest[4];
	for (int i = 0; i < 4; i++) {
		latest[i] = (int16_t) ((int16_t) (RxData[i * 2 + 1] << 8)
				| RxData[i * 2]);
	}
	motor1_output = latest[0];
	motor2_output = latest[3];
	motor3_output = latest[1];
	motor4_output = latest[2];

	for (int i = 0; i < 4; i++) {
		if (latest[i] == 0) {
			pattern[i] = CAN_OUT_ZERO;
		} else if (latest[i] > 0) {
			pattern[i] = CAN_OUT_HIGHSIDE;
		} else if (latest[i] < 0) {
			pattern[i] = CAN_OUT_LOWSIDE;
		}
	}

	return;
}

static int CAN_Polling(void) {
	static int idle_counter = 0;
	idle_counter++;
	if (idle_counter > 200) {
		pattern[0] = WAIT;
		pattern[1] = WAIT;
		pattern[2] = WAIT;
		pattern[3] = WAIT;
		motor1_output = 0;
		motor2_output = 0;
		motor3_output = 0;
		motor4_output = 0;
	}

	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) == 0) {
		/* Reception Missing */
		return 1;
	}
	idle_counter = 0;
	xprintf("CAN PACKET RECEIVED\r\n");

#ifdef DEBUGMODE
	xprintf("fill level:%d\r\n",
			HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0));
#endif

	if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData)
			!= HAL_OK) {
		/* Reception Error */
		return 2;
	}

#ifdef DEBUGMODE
	xprintf("id:%d,data[0]:%d\r\n", RxHeader.StdId, RxData[0]);
#endif

	//success receiving
	return HAL_OK;
}

// フィルタモード　設定�?�考
// https://hsdev.co.jp/stm32-can/
static void CAN_Filter_Init(void) {
//setting CAN filters
	uint32_t fId1 = 0x005 << 5; // フィルターID1
	uint32_t fId2 = 0x000 << 5; // フィルターID2
	uint32_t fId3 = 0x000 << 5; // フィルターID3
	uint32_t fId4 = 0x000 << 5; // フィルターID4
	CAN_FilterTypeDef sFilterConfig;
	sFilterConfig.FilterIdHigh = fId1;                  // フィルターID1
	sFilterConfig.FilterIdLow = fId2;                  // フィルターID2
	sFilterConfig.FilterMaskIdHigh = fId3;                  // フィルターID3
	sFilterConfig.FilterMaskIdLow = fId4;                  // フィルターID4
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // 16モード
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;      // FIFO0�?�格�?
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST; // IDリストモード
	sFilterConfig.SlaveStartFilterBank = 14;
	sFilterConfig.FilterActivation = ENABLE;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}
}

static void Timer_Start(void) {
//Initialize timers for PWM generation
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

static void GPIO_Start(void) {
//Activate GATE DRIVER SHUTDOWN PIN to HIGH LEVEL(ENABLE)
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
//Set PWM duty to ZERO (for safety)
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	//provide function logic to xprint source
	xdev_out(uart_putc);
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
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	xprintf("--MotorDriver Historia V1 2021/03/08--\r\n");
	xprintf("Starting up...\r\n");
	Timer_Start();
	GPIO_Start();
	CAN_Filter_Init();
	HAL_CAN_Start(&hcan);
	xprintf("Started up successfully.\r\n");

	while (1) {
		if (CAN_Polling() == HAL_OK) {
			PARSE_CAN_FRAME();
		}

		xprintf("current output:%d \r\n", output);
		printf("current output:%d \r\n", output);

		PWM_Generation(htim2, 1, motor1_output);
		PWM_Generation(htim2, 2, motor2_output);
		PWM_Generation(htim3, 3, motor3_output);
		PWM_Generation(htim3, 4, motor4_output);

		//LED update
		uint8_t is_eligible_to_update = 0;
		for (int i = 0; i < 4; i++) {
			static int previous_pattern[4] = { 999, 999, 999, 999 };
			static int token = 0;
			token++;
			if (previous_pattern[i] == pattern[i] && token < 100)
				continue;
			token = 0;
			previous_pattern[i] = pattern[i];
			is_eligible_to_update = 1;
		}
		if (is_eligible_to_update) {
			is_eligible_to_update = 0;
			for (int i = 0; i < 4; i++) {
				WS2812_Applyment(i, pattern[i]);
			}
			uint32_t tickstart = HAL_GetTick();
			uint32_t wait = 1;
			while ((HAL_GetTick() - tickstart) < wait) {
			}
		}
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */

	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN;
	hcan.Init.Prescaler = 2;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_14TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */

	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2399;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2399;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 230400;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : PA2 PA3 PA5 PA7
	 PA8 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7
			| GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
