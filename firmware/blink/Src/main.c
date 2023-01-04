/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "string.h"
#include <define.h>
#include <mpu6050.h>
#include <pid.h>
#include "SBR1_fis.h"
// #include "median.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	MOTOR1, // = 0
	MOTOR2  // = 1
} motor_id_e;
typedef enum {
	CW, //ClockWise = 0
	CCW //CounterClockWise = 1
} motor_dir_e;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
MPU6050_t mpu_data;
//pid_params_t pid_mtr1;
//pid_params_t pid_mtr2;
//pid_params_t pid_balance;
motor_id_e motor_name;
motor_dir_e motor1_dir, motor2_dir;

uint8_t RX_buff[UART_RX_BUFF_SIZE] = {'\0'};
uint8_t count = 0, count_bl = 0;
//uint16_t pre_enc1 = 0, pre_enc2 = 0;
//float speed1[MAX_COUNT] = {0}, speed2[MAX_COUNT] = {0}, act_speed1, act_speed2;
char MSG[50] = {'\0'};
int16_t dx1, dx2;
//float KP = 20, KI=0, KD=0, SP = 0, SP_mtr1 = -500, SP_mtr2 = 50;
float K1 = K_IN0, K2 = K_IN1, Ku = K_OUT_FUZZY;
uint8_t enable = 0;
int16_t pidval_mtr1, pidval_mtr2;
float inp_fuzzy[2], out_fuzzy;
float inp_pid_mtr;
uint16_t cnt_sp = 0;
float roll, pre_roll, roll_dot;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void app_init(void);
void run_motor(motor_id_e mtr_name, motor_dir_e mtr_dir, uint16_t duty);
int atoi(const char *str);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  app_init();
  SBR1_init();

  HAL_UART_Transmit_DMA(&huart1, "OK\r\n", 4);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) { //Timer 1: f=500Hz -> T = 0.002s
	if (htim->Instance == htim1.Instance) {

//		dx1 = (TIM3->CNT) - pre_enc1;
//		dx2 = (TIM4->CNT) - pre_enc2;
//		pre_enc1 = TIM3->CNT;
//		pre_enc2 = TIM4->CNT;
////		if (dx1>400) dx1 = dx1 - 65535;
////			else if (dx1<-400) dx1 = dx1 + 65535;
////		if (dx2>400) dx2 = dx2 - 65535;
////			else if (dx2<-400) dx2 = dx2 + 65535;
//		speed1[count] = SPEED_SCALE*dx1/0.002f;
//		speed2[count] = SPEED_SCALE*dx2/0.002f;

		count++;
		if (count>=MAX_COUNT) {
			count = 0;
			cnt_sp ++;
			if (cnt_sp>=50) {

				sprintf(MSG, "%d\n\r", (int16_t)inp_pid_mtr);//(int16_t)((float)(roll-(int16_t)roll)*100)
				HAL_UART_Transmit_DMA(&huart1, &MSG, (uint16_t) sizeof(MSG));
				cnt_sp = 0;
//			sprintf(MSG, "%d  -  %d\n\r", (int16_t)inp_pid_mtr, (int16_t)act_speed1);//(int16_t)((float)(roll-(int16_t)roll)*100)
//			HAL_UART_Transmit_DMA(&huart1, &MSG, (uint16_t) sizeof(MSG));
			}


				MPU6050_Read_All(&mpu_data);
				pre_roll = roll;
				roll = mpu_data.KalmanAngleY+ROLL_OFFSET;
				roll_dot = mpu_data.Gy;
//				roll_dot = (roll-pre_roll)/0.010f;
				inp_fuzzy[0] = roll/K1;
				inp_fuzzy[1] = roll_dot/K2;

				SBR1_run(inp_fuzzy, &out_fuzzy);

				if (((roll<=-40) || (roll>=40)) && (enable==1)) {
					enable = 0;
				} else if (((roll>=-40) && (roll<=40)) && (enable==0)) enable = 1;


			inp_pid_mtr = out_fuzzy*Ku;
			if (inp_pid_mtr<=-1999) {
				pidval_mtr1 = -1999;
				pidval_mtr2 = -1999;
			} else if (inp_pid_mtr>=1999) {
				pidval_mtr1 = 1999;
				pidval_mtr2 = 1999;
			}
			else{
				pidval_mtr1 = inp_pid_mtr;
				pidval_mtr2 = inp_pid_mtr;
			}

			if (pidval_mtr1 >= 0) motor1_dir = CW;
				else {
					motor1_dir = CCW;
					pidval_mtr1 = -pidval_mtr1;
				}
			if (pidval_mtr2 >= 0) motor2_dir = CCW;
				else {
					motor2_dir = CW;
					pidval_mtr2 = -pidval_mtr2;
				}
			if (enable==1) {
				run_motor(MOTOR1, motor1_dir, pidval_mtr1);
				run_motor(MOTOR2, motor2_dir, pidval_mtr2);
			} else {
				run_motor(MOTOR1, motor1_dir, 0);
				run_motor(MOTOR2, motor2_dir, 0);
			}
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
	   char *arg_list[6];
	   uint8_t arg_num = 0;
	   char *temp_token = strtok((char*)RX_buff, " ");
	   while (temp_token != NULL) {
		   arg_list[arg_num] = temp_token;
		   arg_num ++;
		   temp_token = strtok(NULL, " ");
	   }
	   if (strstr(arg_list[0], "K1") != NULL) {
		   K1 = (float)atoi(arg_list[1]);
	   } else if (strstr(arg_list[0], "START/STOPPPPPPPPPPP")!= NULL) {
		   if (enable==0) enable = 1;
		   else enable = 0;
		   HAL_UART_Transmit_DMA(&huart1, "ENB-OK", 6);
	   }
	   if (strstr(arg_list[2], "K2") != NULL) {
	   	   K2 = (float)atoi(arg_list[3]);
	   }
	   if (strstr(arg_list[4], "KU") != NULL) {
	   	   Ku = (float)atoi(arg_list[5]);
	   }
//		pid_init(&pid_mtr1, KP, KI, KD, -500, 500, -500, 500, -1999, 1999);
//		pid_init(&pid_mtr2, KP, KI, KD, -500, 500, -500, 500, -1999, 1999);
//	   pid_init(&pid_balance, KP, KI, KD, -500, 500, -500, 500, -750, 750);
	   sprintf(MSG, "K1 %d, K2 %d, KU %d\r\n", (int16_t)K1, (int16_t)K2, (int16_t)Ku);
	   HAL_UART_Transmit_DMA(&huart1,  MSG, strlen(MSG));
	   HAL_UART_Receive_IT(&huart1, (uint8_t*) &RX_buff, UART_RX_BUFF_SIZE);
	}
}
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if (huart->Instance == huart1.Instance) {
//		 HAL_UART_Transmit_DMA(&huart1, NULL, 0);
//	}
//}
void app_init(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&MTR1_PWM_TIMER, MTR1_PWM_CHANNEL, 0);
	__HAL_TIM_SET_COMPARE(&MTR2_PWM_TIMER, MTR2_PWM_CHANNEL, 0);

	HAL_UART_Receive_IT(&huart1, (uint8_t*) &RX_buff, UART_RX_BUFF_SIZE);
	HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, 0);

	MPU6050_Init();
//	pid_init(&pid_mtr1, KP2_MTR, KI2_MTR, KD2_MTR, -1000, 1000, -1000, 1000, -1999, 1999);
//	pid_init(&pid_mtr2, KP2_MTR, KI2_MTR, KD2_MTR, -1000, 1000, -1000, 1000, -1999, 1999);
//	pid_init(&pid_balance, KP_BLC, KI_BLC, KD_BLC, -500, 500, -500, 500, -750, 750);

	HAL_TIM_Base_Start_IT(&htim1);
}
void run_motor(motor_id_e mtr_name, motor_dir_e mtr_dir, uint16_t duty) {
	if (mtr_name == MOTOR1) {
		if (mtr_dir == CCW) {
			HAL_GPIO_WritePin(MTR1_DIR_GPIO_PORT, MTR1_DIR_GPIO_PIN, 0);
			__HAL_TIM_SET_COMPARE(&MTR1_PWM_TIMER, MTR1_PWM_CHANNEL, duty);
		}
		else {
			HAL_GPIO_WritePin(MTR1_DIR_GPIO_PORT, MTR1_DIR_GPIO_PIN, 1);
			__HAL_TIM_SET_COMPARE(&MTR1_PWM_TIMER, MTR1_PWM_CHANNEL, PWM_CNT_PERIOD - duty);
		}
	}
	else if (mtr_name == MOTOR2) {
		if (mtr_dir == CW) {
			HAL_GPIO_WritePin(MTR2_DIR_GPIO_PORT, MTR2_DIR_GPIO_PIN, 0);
			__HAL_TIM_SET_COMPARE(&MTR2_PWM_TIMER, MTR2_PWM_CHANNEL, duty);
		}
		else {
			HAL_GPIO_WritePin(MTR2_DIR_GPIO_PORT, MTR2_DIR_GPIO_PIN, 1);
			__HAL_TIM_SET_COMPARE(&MTR2_PWM_TIMER, MTR2_PWM_CHANNEL, PWM_CNT_PERIOD - duty);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
