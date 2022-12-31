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
#include "common.h"

#define FALSE (0)
#define TRUE (1)

/* Ringbuffer variables */
volatile ringbuff_t *rb_cm4_to_cm7 = (void*) BUFF_CM4_TO_CM7_ADDR;
volatile ringbuff_t *rb_cm7_to_cm4 = (void*) BUFF_CM7_TO_CM4_ADDR;

uint8_t RxData[10];
const int UART_DATALEN = 10;
uint8_t toggleFlag1;
uint8_t toggleFlag2;
uint8_t toggleFlag3;
uint8_t toggleFlag4;

#define CENTERPOS1 (1.45)	//F- B+
#define CENTERPOS2 (1.66)	//F+ B-
#define CENTERPOS3 (1.32)	//F- B+
#define CENTERPOS4 (1.78)	//F+ B-

#define STRIDESTEP (0.5)
#define SHORTSTEP (0.3)
#define BUFFERSTEP_F_1 (0)
#define BUFFERSTEP_B_1 (0)
#define BUFFERSTEP_F_2 (0.1)
#define BUFFERSTEP_B_2 (0.1)
#define BUFFERSTEP_F_3 (0)
#define BUFFERSTEP_B_3 (0)
#define BUFFERSTEP_F_4 (-0.1)
#define BUFFERSTEP_B_4 (0)

void InitWalk(void);
void DoWalk(void);
void InitTurnRight(void);
void InitTurnLeft(void);
void DoTurnRight(void);
void DoTurnLeft(void);
void InitSidestepRight(void);
void InitSidestepLeft(void);
void DoSidestepRight(void);
void DoSidestepLeft(void);
void ToggleServo1(float stride, float buffer_f, float buffer_b);
void ToggleServo2(float stride, float buffer_f, float buffer_b);
void ToggleServo3(float stride, float buffer_f, float buffer_b);
void ToggleServo4(float stride, float buffer_f, float buffer_b);
void InitServo1(void);
void InitServo2(void);
void InitServo3(void);
void InitServo4(void);
void DoStop(void);

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	uint32_t i = 0, time, t1, t2;
	uint8_t readSig[] = { 0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE };
	uint8_t rotSig[] = { 0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29, 0x00, 0xFF, 0x03,
			0x00, 0x00, 0x00, 0x00, 0xC6 };
	uint8_t rotSig2[] = { 0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29, 0x00, 0x00, 0x08,
			0x00, 0x00, 0x00, 0x00, 0xC0 };
	/* USER CODE END 1 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
	PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_UART5_Init();
	MX_USART6_UART_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	HAL_UART_Receive_IT(&huart5, RxData, UART_DATALEN);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	DoStop();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (!ringbuff_is_ready(rb_cm4_to_cm7)
			|| !ringbuff_is_ready(rb_cm7_to_cm4)) {
	}

	/* Write message to buffer */
	//ringbuff_write(rb_cm4_to_cm7, "[CM4] Core ready\r\n", 18);
	/* Set default time */
	time = t1 = t2 = HAL_GetTick();
	char str[10];
	//InitWalk();
	InitSidestepRight();

	while (1) {
		size_t len;
		void *addr;

		time = HAL_GetTick();

		/* Send data to CPU1 */
		if (time - t1 >= 1000) {
			t1 = time;
			char c = '0' + (++i % 10);

			/* Write to buffer from CPU2 to CPU1 */
			ringbuff_write(rb_cm4_to_cm7, "[CM4] Call: ", 12);
			ringbuff_write(rb_cm4_to_cm7, &c, 1);
			ringbuff_write(rb_cm4_to_cm7, "\r\n", 2);
		}

		/* Toggle LED */
		if (time - t2 >= 500) {
			t2 = time;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

			if (strncmp(str, "STOP", 4) == 0) {
				DoStop();
			} else if (strncmp(str, "WALK", 4) == 0) {
				DoWalk();
			} else if (strncmp(str, "SIDESTR", 7) == 0) {
				DoSidestepRight();
			} else if (strncmp(str, "SIDESTL", 7) == 0) {
				DoSidestepLeft();
			} else if (strncmp(str, "TURNR", 5) == 0) {
				DoTurnRight();
			} else if (strncmp(str, "TURNL", 5) == 0) {
				DoTurnLeft();
			}
		}

		/* Check if CPU1 sent some data to CPU2 core */
		while ((len = ringbuff_get_linear_block_read_length(rb_cm7_to_cm4)) > 0) {
			addr = ringbuff_get_linear_block_read_address(rb_cm7_to_cm4);

			strncpy(str, addr, UART_DATALEN);
			HAL_UART_Transmit(&huart5, addr, UART_DATALEN, 1000);

			if (strncmp(str, "STOP", 4) == 0) {
				DoStop();
			} else if (strncmp(str, "WALK", 4) == 0) {
				InitWalk();
			} else if (strncmp(str, "SIDESTR", 7) == 0) {
				InitSidestepRight();
			} else if (strncmp(str, "SIDESTL", 7) == 0) {
				InitSidestepLeft();
			} else if (strncmp(str, "TURNR", 5) == 0) {
				InitTurnRight();
			} else if (strncmp(str, "TURNL", 5) == 0) {
				InitTurnLeft();
			}

			/*
			 * `addr` holds pointer to beginning of data array
			 * which can be used directly in linear form.
			 *
			 * Its length is `len` bytes
			 */
			/* Process data here */

			/* Mark buffer as read to allow other writes from CPU1 */
			ringbuff_skip(rb_cm7_to_cm4, len);
		}

		if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE) ||
		__HAL_UART_GET_FLAG(&huart5, UART_FLAG_NE) ||
		__HAL_UART_GET_FLAG(&huart5, UART_FLAG_FE) ||
		__HAL_UART_GET_FLAG(&huart5, UART_FLAG_PE)) {
			HAL_UART_Abort(&huart5);
			HAL_UART_Receive_IT(&huart5, RxData, UART_DATALEN);
		}
	}
	/* USER CODE END 3 */
}

void InitWalk(void) {
	toggleFlag1 = FALSE;
	toggleFlag2 = FALSE;
	toggleFlag3 = TRUE;
	toggleFlag4 = TRUE;
}

void DoWalk(void) {
	ToggleServo1(STRIDESTEP, BUFFERSTEP_F_1, BUFFERSTEP_B_1);
	ToggleServo2(STRIDESTEP, BUFFERSTEP_F_2, BUFFERSTEP_B_2);
	ToggleServo3(SHORTSTEP, BUFFERSTEP_F_3, BUFFERSTEP_B_3);
	ToggleServo4(SHORTSTEP, BUFFERSTEP_F_4, BUFFERSTEP_B_4);
}

void InitTurnLeft(void) {
	toggleFlag1 = FALSE;
	toggleFlag2 = FALSE;
	toggleFlag3 = FALSE;
	toggleFlag4 = FALSE;
}

void DoTurnLeft(void) {
	ToggleServo1(STRIDESTEP, BUFFERSTEP_F_1, BUFFERSTEP_B_1);
	ToggleServo2(SHORTSTEP, BUFFERSTEP_F_2, BUFFERSTEP_B_2);
	ToggleServo3(STRIDESTEP, BUFFERSTEP_F_3, BUFFERSTEP_B_3);
	ToggleServo4(SHORTSTEP, BUFFERSTEP_F_4, BUFFERSTEP_B_4);
}

void InitTurnRight(void) {
	toggleFlag1 = FALSE;
	toggleFlag2 = FALSE;
	toggleFlag3 = FALSE;
	toggleFlag4 = FALSE;
}

void DoTurnRight(void) {
	ToggleServo1(SHORTSTEP, BUFFERSTEP_F_1, BUFFERSTEP_B_1);
	ToggleServo2(STRIDESTEP, BUFFERSTEP_F_2, BUFFERSTEP_B_2);
	ToggleServo3(SHORTSTEP, BUFFERSTEP_F_3, BUFFERSTEP_B_3);
	ToggleServo4(STRIDESTEP, BUFFERSTEP_F_4, BUFFERSTEP_B_4);
}

void InitSidestepRight(void) {
	toggleFlag1 = FALSE;
	toggleFlag2 = FALSE;
	toggleFlag3 = TRUE;
	toggleFlag4 = TRUE;
}

void DoSidestepRight(void) {
	ToggleServo1(SHORTSTEP, BUFFERSTEP_F_1, BUFFERSTEP_B_1);
	ToggleServo2(STRIDESTEP, BUFFERSTEP_F_2, BUFFERSTEP_B_2);
	ToggleServo3(SHORTSTEP, BUFFERSTEP_F_3, BUFFERSTEP_B_3);
	ToggleServo4(STRIDESTEP, BUFFERSTEP_F_4, BUFFERSTEP_B_4);
}

void InitSidestepLeft(void) {
	toggleFlag1 = TRUE;
	toggleFlag2 = TRUE;
	toggleFlag3 = FALSE;
	toggleFlag4 = FALSE;
}

void DoSidestepLeft(void) {
	ToggleServo1(STRIDESTEP, BUFFERSTEP_F_1, BUFFERSTEP_B_1);
	ToggleServo2(SHORTSTEP, BUFFERSTEP_F_2, BUFFERSTEP_B_2);
	ToggleServo3(STRIDESTEP, BUFFERSTEP_F_3, BUFFERSTEP_B_3);
	ToggleServo4(SHORTSTEP, BUFFERSTEP_F_4, BUFFERSTEP_B_4);

}

void DoStop(void) {
	InitServo1();
	InitServo2();
	InitServo3();
	InitServo4();
}

void ToggleServo1(float stride, float buffer_f, float buffer_b) {

	if (toggleFlag1) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
				(((CENTERPOS1-stride+buffer_f)/20)*40000));
		toggleFlag1 = FALSE;
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
				((CENTERPOS1+stride-buffer_b)/20)*40000);
		toggleFlag1 = TRUE;
	}
}
void ToggleServo2(float stride, float buffer_f, float buffer_b) {

	if (toggleFlag2) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
				(((CENTERPOS2-stride+buffer_b)/20)*40000));
		toggleFlag2 = FALSE;
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
				((CENTERPOS2+stride-buffer_f)/20)*40000);
		toggleFlag2 = TRUE;
	}
}
void ToggleServo3(float stride, float buffer_f, float buffer_b) {

	if (toggleFlag3) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
				(((CENTERPOS3-stride+buffer_f)/20)*40000));
		toggleFlag3 = FALSE;
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
				((CENTERPOS3+stride-buffer_b)/20)*40000);
		toggleFlag3 = TRUE;
	}
}
void ToggleServo4(float stride, float buffer_f, float buffer_b) {

	if (toggleFlag4) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
				(((CENTERPOS4-stride+buffer_b)/20)*40000));
		toggleFlag4 = FALSE;
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,
				((CENTERPOS4+stride-buffer_f)/20)*40000);
		toggleFlag4 = TRUE;
	}
}

void InitServo1(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (CENTERPOS1/20)*40000);
}
void InitServo2(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (CENTERPOS2/20)*40000);
}
void InitServo3(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (CENTERPOS3/20)*40000);
}
void InitServo4(void) {
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, ((CENTERPOS4/20)*40000));
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 99;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 39999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 115200;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
void MX_USART3_UART_Init(void) {

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
	if (HAL_MultiProcessor_Init(&huart3, 0, UART_WAKEUPMETHOD_IDLELINE)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
