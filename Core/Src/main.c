/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2S_HandleTypeDef hi2s3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;



/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotTask */
osThreadId_t RobotTaskHandle;
const osThreadAttr_t RobotTask_attributes = {
  .name = "RobotTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

volatile int32_t encoder1_count = 0;
volatile int32_t encoder2_count = 0;

int32_t encoder1_prev = 0;
int32_t encoder2_prev = 0;

int32_t encoder1_delta = 0;
int32_t encoder2_delta = 0;

uint16_t prev_enc1 = 0;
uint16_t prev_enc2 = 0;

int16_t delta1 = 0;
int16_t delta2 = 0;

uint32_t pwm1 = 50000;
uint32_t pwm2 = 50000;

uint32_t last_time = 0;

float distance1 = 0;
float distance2 = 0;

float Yaw_Angle = 0.0f;

volatile float encoder1_speed = 0.0f;
volatile float encoder2_speed = 0.0f;

volatile float speed_error = 0.0f;

volatile uint16_t base_pwm = 50000;

volatile uint16_t left_pwm = 0;
volatile uint16_t right_pwm = 0;

volatile float Kp = 5.0f;

volatile uint32_t mpu_i2c_result = 99;

volatile HAL_StatusTypeDef i2c_test = HAL_ERROR;

volatile HAL_StatusTypeDef mpu_read_status = HAL_ERROR;

volatile uint8_t mpu6500_ready = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void RobotTaskFunction(void *argument);

/* USER CODE BEGIN PFP */
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Left(void);
void Motor_Right(void);
void Motor_Stop(void);

float HCSR04_Read(void);
float HCSR04_Read2(void);

void MPU6500_Init(void);
HAL_StatusTypeDef MPU6500_Read(void);
void MPU6500_CalibrateGyro(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6500_ADDR  (0x68 << 1)

uint8_t who_am_i;
uint8_t raw_data[14];
volatile HAL_StatusTypeDef mpu_i2c_status = HAL_ERROR;
int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;

float Accel_X, Accel_Y, Accel_Z;
float Gyro_X, Gyro_Y, Gyro_Z;

float Gyro_X_Offset = 0.0f;
float Gyro_Y_Offset = 0.0f;
float Gyro_Z_Offset = 0.0f;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	float HCSR04_Read(void);
	float HCSR04_Read2(void);

	void Motor_Forward(void);
	void Motor_Stop(void);



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
  MX_I2S3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize MPU6500 */
  MPU6500_Init();

  if (mpu6500_ready == 0)
  {
      Error_Handler();
  }

  /* Calibrate gyroscope while robot is stationary */
  MPU6500_CalibrateGyro();

  if (mpu6500_ready == 0)
  {
      Error_Handler();
  }

  /* Start wheel encoders */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* Start motor PWM */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  /* Set motor PWM compare values */
  /* IMPORTANT: Verify TIM4 Period in CubeMX before using 60000 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 60000);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 60000);


  /* Start servo PWM on PA8 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* Set servo to centre position */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 770);

  /* Allow servo to reach centre */
  HAL_Delay(1000);


  /* Initialize encoder tracking */
  prev_enc1 = __HAL_TIM_GET_COUNTER(&htim2);
  prev_enc2 = __HAL_TIM_GET_COUNTER(&htim3);

  last_time = HAL_GetTick();

  /* Start ultrasonic timing base timers */
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RobotTask */
  RobotTaskHandle = osThreadNew(RobotTaskFunction, NULL, &RobotTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE7 PE9 PE11
                           PE12 */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_SCK_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(SPI1_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 Audio_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Motor_Forward_PWM(uint16_t right_pwm, uint16_t left_pwm)
{
    // Right motor = Motor 1 = TIM4 CH1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    // Left motor = Motor 2 = TIM4 CH2
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, right_pwm);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, left_pwm);
}

void Motor_Backward(void)
{
    // Motor 1 Backward
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    // Motor 2 Backward
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 55000);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 55000);
}

void Motor_Left(void)
{
    // Motor A BACKWARD
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

    // Motor B FORWARD
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

    // FULL SPEED
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, htim4.Init.Period);
}

void Motor_Right(void)
{
    // Motor 1 FORWARD
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    // Motor 2 BACKWARD
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, htim4.Init.Period);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, htim4.Init.Period);
}

void Motor_Stop(void)
{
    // Stop Motor 1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

    // Stop Motor 2
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
}



float HCSR04_Read(void)
{
    uint32_t start_tick;
    uint32_t echo_time;

    /* Front sensor: TRIG = PE7, ECHO = PE8 */

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(&htim5, 0);
    while (__HAL_TIM_GET_COUNTER(&htim5) < 2U);

    /* Send 10 us trigger pulse */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

    while (__HAL_TIM_GET_COUNTER(&htim5) < 10U);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);

    /* Wait for echo to become HIGH, with timeout */
    start_tick = HAL_GetTick();

    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start_tick) >= 30U)
        {
            return -1.0f;  /* No echo received */
        }
    }

    /* Measure echo pulse width */
    __HAL_TIM_SET_COUNTER(&htim5, 0);
    start_tick = HAL_GetTick();

    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start_tick) >= 30U)
        {
            return -1.0f;  /* Echo did not end */
        }
    }

    echo_time = __HAL_TIM_GET_COUNTER(&htim5);

    /* Distance in centimetres */
    return (float)echo_time * 0.0343f / 2.0f;
}

float HCSR04_Read2(void)
{
    uint32_t start_tick;
    uint32_t echo_time;

    /* Rear sensor: TRIG = PE12, ECHO = PE13 */

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

    __HAL_TIM_SET_COUNTER(&htim6, 0);
    while (__HAL_TIM_GET_COUNTER(&htim6) < 2U);

    /* Send 10 us trigger pulse */
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);

    while (__HAL_TIM_GET_COUNTER(&htim6) < 10U);

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);

    /* Wait for echo to become HIGH */
    start_tick = HAL_GetTick();

    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start_tick) >= 30U)
        {
            return -1.0f;
        }
    }

    /* Measure echo pulse width */
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    start_tick = HAL_GetTick();

    while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13) == GPIO_PIN_SET)
    {
        if ((HAL_GetTick() - start_tick) >= 30U)
        {
            return -1.0f;
        }
    }

    echo_time = __HAL_TIM_GET_COUNTER(&htim6);

    return (float)echo_time * 0.0343f / 2.0f;
}



void MPU6500_Init(void)
{
    uint8_t check = 0;
    uint8_t data = 0x00;

    who_am_i = 0;
    mpu6500_ready = 0;

    /* Read WHO_AM_I register */
    mpu_i2c_status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6500_ADDR,
        0x75,
        I2C_MEMADD_SIZE_8BIT,
        &check,
        1,
        100
    );

    if (mpu_i2c_status != HAL_OK)
    {
        return;
    }

    who_am_i = check;

    /* MPU6500 should return 0x70 */
    if (who_am_i != 0x70)
    {
        return;
    }

    /* Wake up the MPU6500 */
    mpu_i2c_status = HAL_I2C_Mem_Write(
        &hi2c1,
        MPU6500_ADDR,
        0x6B,
        I2C_MEMADD_SIZE_8BIT,
        &data,
        1,
        100
    );

    if (mpu_i2c_status != HAL_OK)
    {
        return;
    }

    HAL_Delay(100);

    mpu6500_ready = 1;
}


HAL_StatusTypeDef MPU6500_Read(void)
{
    uint8_t sensor_data[14];

    if (mpu6500_ready == 0)
    {
        return HAL_ERROR;
    }

    mpu_read_status = HAL_I2C_Mem_Read(
        &hi2c1,
        MPU6500_ADDR,
        0x3B,
        I2C_MEMADD_SIZE_8BIT,
        sensor_data,
        14,
        100
    );

    if (mpu_read_status != HAL_OK)
    {
        return mpu_read_status;
    }

    Accel_X_RAW = (int16_t)((sensor_data[0] << 8) | sensor_data[1]);
    Accel_Y_RAW = (int16_t)((sensor_data[2] << 8) | sensor_data[3]);
    Accel_Z_RAW = (int16_t)((sensor_data[4] << 8) | sensor_data[5]);

    Gyro_X_RAW = (int16_t)((sensor_data[8] << 8) | sensor_data[9]);
    Gyro_Y_RAW = (int16_t)((sensor_data[10] << 8) | sensor_data[11]);
    Gyro_Z_RAW = (int16_t)((sensor_data[12] << 8) | sensor_data[13]);

    /* Accelerometer: +/- 2g */
    Accel_X = Accel_X_RAW / 16384.0f;
    Accel_Y = Accel_Y_RAW / 16384.0f;
    Accel_Z = Accel_Z_RAW / 16384.0f;

    /* Gyroscope: +/- 250 degrees/second */
    Gyro_X = (Gyro_X_RAW / 131.0f) - Gyro_X_Offset;
    Gyro_Y = (Gyro_Y_RAW / 131.0f) - Gyro_Y_Offset;
    Gyro_Z = (Gyro_Z_RAW / 131.0f) - Gyro_Z_Offset;

    return HAL_OK;
}


void MPU6500_CalibrateGyro(void)
{
    const int samples = 500;
    int successful_samples = 0;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    if (mpu6500_ready == 0)
    {
        return;
    }

    /* Keep the robot stationary */
    HAL_Delay(1000);

    for (int i = 0; i < samples; i++)
    {
        if (MPU6500_Read() == HAL_OK)
        {
            sum_x += Gyro_X_RAW / 131.0f;
            sum_y += Gyro_Y_RAW / 131.0f;
            sum_z += Gyro_Z_RAW / 131.0f;

            successful_samples++;
        }

        HAL_Delay(5);
    }

    if (successful_samples == 0)
    {
        mpu6500_ready = 0;
        return;
    }

    Gyro_X_Offset = sum_x / successful_samples;
    Gyro_Y_Offset = sum_y / successful_samples;
    Gyro_Z_Offset = sum_z / successful_samples;

    /* Verify that a fresh reading is successful */
    if (MPU6500_Read() != HAL_OK)
    {
        mpu6500_ready = 0;
    }
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
      encoder1_count = __HAL_TIM_GET_COUNTER(&htim2);
      encoder2_count = __HAL_TIM_GET_COUNTER(&htim3);

      // Calculate movement since last reading
      encoder1_delta = encoder1_count - encoder1_prev;
      encoder2_delta = encoder2_count - encoder2_prev;

      encoder1_speed = encoder1_delta * 10.0f;
      encoder2_speed = encoder2_delta * 10.0f;

      speed_error = fabsf(encoder1_speed) - fabsf(encoder2_speed);

      float correction = Kp * speed_error;

      float left_command  = base_pwm - correction;
      float right_command = base_pwm + correction;

      if (left_command < 0)
          left_command = 0;

      if (right_command < 0)
          right_command = 0;

      if (left_command > 65535)
          left_command = 65535;

      if (right_command > 65535)
          right_command = 65535;

      left_pwm = (uint16_t)left_command;
      right_pwm = (uint16_t)right_command;

      // Save current values for next cycle
      encoder1_prev = encoder1_count;
      encoder2_prev = encoder2_count;
	  osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_RobotTaskFunction */
/**
* @brief Function implementing the RobotTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotTaskFunction */

void RobotTaskFunction(void *argument)
{
    /* USER CODE BEGIN RobotTaskFunction */

    const float FRONT_STOP_CM = 20.0f;
    const float REAR_SAFE_CM  = 25.0f;

    float left_distance;
    float right_distance;

    static uint32_t last_uart_tick = 0;

    for (;;)
    {
        /* Check the forward path */
        distance1 = HCSR04_Read();

        /* Update IMU */
        static uint32_t previous_imu_tick = 0;
        uint32_t current_imu_tick;
        float dt;

        if (MPU6500_Read() == HAL_OK)
        {
            current_imu_tick = HAL_GetTick();

            if (previous_imu_tick == 0)
            {
                previous_imu_tick = current_imu_tick;
            }

            dt = (current_imu_tick - previous_imu_tick) / 1000.0f;
            previous_imu_tick = current_imu_tick;

            Yaw_Angle += Gyro_Z * dt;
        }

        /* Send telemetry to ESP32-C3 every 1 second */
        if (HAL_GetTick() - last_uart_tick >= 1000)
        {
            char telemetry[150];

            last_uart_tick = HAL_GetTick();

            uint32_t left_encoder_count =
                __HAL_TIM_GET_COUNTER(&htim2);

            uint32_t right_encoder_count =
                __HAL_TIM_GET_COUNTER(&htim3);

            snprintf(telemetry, sizeof(telemetry),
                "DIST:%.2f,ACC:%.2f,%.2f,%.2f,GYRO:%.2f,%.2f,%.2f,YAW:%.2f,ENC_L:%lu,ENC_R:%lu\r\n",
                distance1,
                Accel_X, Accel_Y, Accel_Z,
                Gyro_X, Gyro_Y, Gyro_Z,
                Yaw_Angle,
                (unsigned long)left_encoder_count,
                (unsigned long)right_encoder_count);

            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)telemetry,
                strlen(telemetry),
                100
            );
        }

        /* Handle invalid ultrasonic readings */
        if (distance1 <= 0)
        {
            Motor_Stop();
            osDelay(100);
            continue;
        }

        /* Move forward if the path is clear */
        if (distance1 > FRONT_STOP_CM)
        {
            Motor_Forward_PWM(left_pwm, right_pwm);
            osDelay(50);
            continue;
        }

        /* Obstacle detected */
        Motor_Stop();
        osDelay(300);

        /* CENTRE position */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 770);
        osDelay(500);

        /* Scan LEFT */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1115);
        osDelay(500);

        left_distance = HCSR04_Read();
        osDelay(100);

        /* Return to CENTRE */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 770);
        osDelay(500);

        /* Scan RIGHT */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 425);
        osDelay(500);

        right_distance = HCSR04_Read();
        osDelay(100);

        /* Return to CENTRE */
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 770);
        osDelay(500);

        /* Choose a safe direction */
        if (left_distance > FRONT_STOP_CM &&
            right_distance > FRONT_STOP_CM)
        {
            if (left_distance >= right_distance)
            {
                Motor_Left();
                osDelay(400);
            }
            else
            {
                Motor_Right();
                osDelay(400);
            }

            Motor_Stop();
            osDelay(200);
        }
        else if (left_distance > FRONT_STOP_CM)
        {
            Motor_Left();
            osDelay(400);
            Motor_Stop();
            osDelay(200);
        }
        else if (right_distance > FRONT_STOP_CM)
        {
            Motor_Right();
            osDelay(400);
            Motor_Stop();
            osDelay(200);
        }
        else
        {
            Motor_Stop();
            osDelay(200);

            distance2 = HCSR04_Read2();

            if (distance2 > REAR_SAFE_CM)
            {
                Motor_Backward();
                osDelay(250);
                Motor_Stop();
                osDelay(300);
            }
            else
            {
                Motor_Stop();
                osDelay(500);
            }
        }
    }

    /* USER CODE END RobotTaskFunction */
}

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
