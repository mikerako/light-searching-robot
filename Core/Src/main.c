/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PCF8591_ADDR 		0x90	// Device address = 0
#define PCF8591_DAC_ENABLE 	0x40
#define PCF8591_ADC_CH0		0x40
#define PCF8591_ADC_CH1		0x41
#define PCF8591_ADC_CH2		0x42
#define PCF8591_ADC_CH3		0x43
#define SERVO_MAX			260
#define SERVO_MIN			70
#define DEBUGGING			1		// 1=true, 0=false
#define WATCHDOG			2500000	//TUNE THIS VALUE FOR TIME TO DRIVE WOUT RESCANNING FOR LIGHT
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
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void Read_Distance(uint8_t num_times);
uint8_t Read_Brightness();
void Set_Angle(uint8_t angle, uint32_t max, uint32_t min);
uint8_t Scan_Region_Brightness();
uint8_t Scan_Region_Obstacles();
void Turn_To_Face(uint8_t angle);
void Stop_Moving();
void Turn_Left(uint32_t time, uint32_t speed);
void Turn_Right(uint32_t time, uint32_t speed);
void Move_Forward(uint32_t time, uint32_t speed, _Bool extern_watch);
void Move_Backward(uint32_t time, uint32_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum State {ScanLight=0, FaceLight=1, DriveToLight=2, ObstacleDetected=3,
	ScanAvoidance=4, FaceClearPath=5, InchForward=6};
static uint8_t buf[30];
static uint8_t raw;
static HAL_StatusTypeDef ret;
static volatile uint32_t distance = 0;
static volatile uint32_t capture = 0;
static uint32_t polarity;
static uint8_t light_angle;
static uint8_t clear_angle;
static uint32_t watchdog = WATCHDOG;
static int32_t global_angle = 0;
static uint32_t right_dist = 0;
static uint32_t left_dist = 0;
static _Bool went_right = 0;
static _Bool went_left = 0;
static int inch_counter = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  enum State state;
  state = ScanLight;
  polarity = TIM_INPUTCHANNELPOLARITY_RISING;

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);			//wheel pwm start
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);			//wheel pwm start
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);	//wheel pwm set to 0
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);	//wheel pwm set to 0

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);			//servo pwm start
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 151);	//set servo to 90deg (forward)
  	  	  	  	  	  	  	  	  	  	  	  	  	//pwm range is 50(0deg) to 253(180deg)

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);			//ultrasonic trigger pwm start
  __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);	//set trigger with no pulse (2=20us pulse)

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);		//start ultrasonic echo capture timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch(state){
	case ScanLight:			//0
		light_angle = Scan_Region_Brightness();
		if(light_angle > 90){
			global_angle = global_angle + (light_angle - 90);
		}
		else if(light_angle < 90){
			global_angle = global_angle - (light_angle - 90);
		}
		state = FaceLight;
		break;

	case FaceLight:			//1
		Turn_To_Face(light_angle);
		Set_Angle(90, SERVO_MAX, SERVO_MIN);
		HAL_Delay(1000);	//wait for servo to turn
		state = DriveToLight;
		break;

	case DriveToLight:		//2
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 2);	//set trigger with 20us pulse)
		if(watchdog == 0){
			Stop_Moving();
			watchdog = WATCHDOG;
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);	//set trigger with 20us pulse)
			state = ScanLight;
			break;
		}

		if(distance < 8 && watchdog < (uint32_t)(0.97*WATCHDOG)){
			Stop_Moving();
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);	//set trigger with 20us pulse)
			state = ObstacleDetected;
			watchdog = WATCHDOG;
			break;
		}
		Move_Forward(1000, 200, 1);
		watchdog = watchdog - 1;
		break;

	case ObstacleDetected:	//3
		// look right and take the distance measurement
		Set_Angle(180, SERVO_MAX, SERVO_MIN);
		HAL_Delay(1000);	// wait for servo to turn
		Read_Distance(2);	// read distance twice to protect against errors
		right_dist = distance;

		// look left and take the distance measurement
		Set_Angle(0, SERVO_MAX, SERVO_MIN);
		HAL_Delay(1000);	// wait for servo to turn
		Read_Distance(2);	// read distance twice to protect against errors
		left_dist = distance;

		// case if the right is more open
		if(right_dist > 10 && right_dist > left_dist){
			Turn_To_Face(150);
			state = InchForward;
			went_right = 1;
			break;
		}
		// case if the left is more open
		if(left_dist > 10 && left_dist > right_dist){
			Turn_To_Face(25);
			state = InchForward;
			went_left = 1;
			break;
		}

		// case if the right and left are both blocked
		Turn_To_Face(150);
		Turn_To_Face(150);	// make two 90deg turns
		Read_Distance(2);	// read distance twice to protect against errors

		// case if area behind is also blocked
		// turn back to face front and restart state machine
		if(distance < 10){
			Turn_To_Face(150);
			Turn_To_Face(150);	// make two 90deg turns
			state = ScanLight;
			break;
		}

		// otherwise drive away from obstacles and look for clear space
		Move_Forward(2000, 200, 0);
		Turn_To_Face(150);
		Turn_To_Face(150);	// make two 90deg turns
		state = ScanAvoidance;
		break;

	case ScanAvoidance:		//4
		clear_angle = Scan_Region_Obstacles();
		if(clear_angle > 90){
			global_angle = global_angle + (clear_angle - 90);
		}
		else if(clear_angle < 90){
			global_angle = global_angle - (clear_angle - 90);
		}

		state = FaceClearPath;
		break;

	case FaceClearPath:		//5
		Turn_To_Face(clear_angle);
		Set_Angle(90, SERVO_MAX, SERVO_MIN);
		HAL_Delay(1000);

		Read_Distance(1);
		if(distance < 10){
			state = ObstacleDetected;
			break;
		}

		state = InchForward;
		break;

	case InchForward:		//6
		// case if the robot is going right or left to avoid obstacles
		if(went_right == 1 || went_left == 1){
			// look to the obstacle and check the distance
			inch_counter = 0;
			Set_Angle(180*went_left, SERVO_MAX, SERVO_MIN);	// look right if went_left else look left
			HAL_Delay(50);
			Read_Distance(2);

			// run loop a max of 4 times to check distances and keep inching forward
			// otherwise reset to the ObstacleDetected state
			while(distance < 10){
				if(inch_counter == 4){
					state = ObstacleDetected;
					inch_counter = 0;
					went_right = went_left = 0;
					break;
				}
				// inch forward and check the distance in front of the robot
				Move_Forward(1000, 150, 0);
				Set_Angle(90, SERVO_MAX, SERVO_MIN);
				HAL_Delay(1000);
				Read_Distance(3);

				// case if there is an obstacle detected in front of the robot
				// go back to ObstacleDetected state
				if(distance < 10){
					state = ObstacleDetected;
					inch_counter = 0;
					went_right = went_left = 0;
					break;
				}

				// turn back to face original obstacle and update distance
				Set_Angle(180*went_left, SERVO_MAX, SERVO_MIN);	// look right if went_left else look left
				HAL_Delay(1000);
				Read_Distance(3);
				inch_counter = inch_counter + 1;
			}

			// obstacle was cleared!
			// inch forward one more time to make entire robot clear obstacle
			// move on to re-scan the area for light source
			Move_Forward(1200, 150, 0);
			if(went_right){
				Turn_To_Face(22);
			}
			else if(went_left){
				Turn_To_Face(165);
			}
			Set_Angle(90, SERVO_MAX, SERVO_MIN);
			state = ScanLight;
			went_right = went_left = 0;
		}
		// case left and right were both blocked
		// so, drive back to open space
		else{
			Move_Forward(1000, 150, 0);
			if(clear_angle > 90){
				Set_Angle((180-clear_angle), SERVO_MAX, SERVO_MIN);
			}
			else if(clear_angle < 90){
				Set_Angle((180-clear_angle), SERVO_MAX, SERVO_MIN);
			}
			HAL_Delay(1000);
			Read_Distance(1);
			if(distance < 6){
				//didn't clear object yet
				state = ObstacleDetected;
			}
			else{
				//cleared object
				state = ScanLight;
			}
		}
		break;

	default:
		state = ScanLight;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 840;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 840;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8400;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 50000;
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
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
 * @param 	time: time to move in milliseconds
 * @param 	speed: pwm for motor. 0-100% mapped to 0-500
 * @retval 	None
 */
void Move_Backward(uint32_t time, uint32_t speed){
	//move backward
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(time);
	Stop_Moving();
}


/*
 * @param 	time: time to move in milliseconds
 * @param 	speed: pwm for motor. 0-100% mapped to 0-500
 * @retval 	None
 */
void Move_Forward(uint32_t time, uint32_t speed, _Bool extern_watch){
	//move forward
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

	if(extern_watch == 0){
		HAL_Delay(time);
		Stop_Moving();
	}
}


/*
 * @param 	time: time to move in milliseconds
 * @param 	speed: pwm for motor. 0-100% mapped to 0-500
 * @retval 	None
 */
void Turn_Right(uint32_t time, uint32_t speed){
	//turn right
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_Delay(time);
	Stop_Moving();
}


/*
 * @param 	time: time to move in milliseconds
 * @param 	speed: pwm for motor. 0-100% mapped to 0-500
 * @retval 	None
 */
void Turn_Left(uint32_t time, uint32_t speed){
	//turn left
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_Delay(time);
	Stop_Moving();
}


/*
 * @retval 	None
 */
void Stop_Moving(){
	//set speed to 0
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);	//wheel pwm set to 0
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);	//wheel pwm set to 0
}


/*
 * @brief 	This function turns the zumo to face the angle it is given
 * 			90 degrees is facing forward
 * @param	angle: uint8_t specifying the angle relative to the zumo
 * @retval	None
 */
void Turn_To_Face(uint8_t angle){
	if(angle < 90){
		Turn_Left((90-angle)*9, 180);		//turn for x ms at 20% speed
	}
	else if(angle > 90){
		Turn_Right((angle-90)*9, 200);		//turn for x ms at 20% speed
	}
	return;
}


/*
 * @brief	This function scans the area in front and returns a direction
 * 		  	that has no obstacles
 * @param 	None
 * @retval 	angle: uint8_t angle of max distance
 */
uint8_t Scan_Region_Obstacles(){
	Set_Angle(0, SERVO_MAX, SERVO_MIN);	//move servo to angle 0
	HAL_Delay(1000);					//wait 1sec while servo moves
	uint32_t max_dist = 0; 				//init max brightness to angle 0
	uint8_t max_angle = 0;  			//init max brightness to angle 0
	uint32_t distances[19] = {[0 ... 18]=0};
	uint32_t avg_angle = 0;
	uint8_t counter = 0;

	for(uint32_t angle=0; angle <= 180; angle = angle + 10){
		Set_Angle(angle, SERVO_MAX, SERVO_MIN);
		HAL_Delay(125);
		Read_Distance(1);
		if(distance > max_dist && distance < 150){
			max_dist = distance;
			max_angle = angle;
		}
		distances[angle/10] = distance;
	}

	for(uint32_t i = 0; i <= 18; ++i){
		if(distances[i] >= 15){
			avg_angle = avg_angle + (i*10);
			counter = counter + 1;
		}
	}
	avg_angle = avg_angle / counter;

	if(abs(avg_angle - max_angle) <= 50){
		max_angle = avg_angle;
	}
	return max_angle;
}


/*
 * @brief	This function scans the area in front and returns the location
 * 		  	that has the highest brightness value
 * @param 	None
 * @retval 	angle: uint8_t angle of max brightness
 */
uint8_t Scan_Region_Brightness(){
	Set_Angle(0, SERVO_MAX, SERVO_MIN);	//move servo to angle 0
	HAL_Delay(1000);					//wait 1sec while servo moves
	uint8_t max_bright = 255; 			//init max brightness to angle 0
	uint8_t max_angle = 0;  			//init max brightness to angle 0
	uint16_t brightnesses[19] = {[0 ... 18]=300};
	uint32_t avg_angle = 0;
	uint8_t counter = 0;

	for(uint32_t angle=0; angle <= 180; angle = angle + 10){
		Set_Angle(angle, SERVO_MAX, SERVO_MIN);
		HAL_Delay(125);
		uint8_t brightness = Read_Brightness();
		brightnesses[angle/10] = brightness;
		if(brightness < max_bright){
			max_bright = brightness;
			max_angle = angle;
		}
	}

	for(uint32_t i=0; i <= 18; ++i){
		if(abs(brightnesses[i] - max_bright) <= 10){
			avg_angle = avg_angle + (i*10);
			counter = counter + 1;
		}
	}
	avg_angle = avg_angle / counter;

	if(abs(avg_angle - max_angle) <= 50){
		max_angle = avg_angle;
	}
	return max_angle;
}


/*
 * @brief	This function sets the servo to a specific angle
 * @param 	angle: a uint8_t specifying the angle to go to
 * @param 	max: a uint32_t specifying the max pwm
 * @param 	min: a uint32_t specifying the min pwm
 * @retval 	None
 */
void Set_Angle(uint8_t angle, uint32_t max, uint32_t min){
	float pwm = ((float)(angle))/180;
	pwm = (pwm * (max - min)) + min;
	pwm = (max + min) - pwm;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, (uint32_t)pwm);
	return;
}


/*
 * @brief	This function takes a brightness reading
 * @param 	None
 * @retval 	raw: a uint8_t raw value of brightness reading
 */
uint8_t Read_Brightness(){
	buf[0] = PCF8591_ADC_CH0;
	ret = HAL_I2C_Master_Transmit(&hi2c1, PCF8591_ADDR, buf, 1, HAL_MAX_DELAY);
	if(ret!=HAL_OK){
	  strcpy((char*)buf, "Err Tx 2\r\n");
	}

	ret = HAL_I2C_Master_Receive(&hi2c1, PCF8591_ADDR, buf, 2, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	  strcpy((char*)buf, "Err Rx\r\n");
	}

	raw = buf[1];

	if(DEBUGGING == 1){
		sprintf((char*)buf,
			  "%u raw\r\n",
			  ((unsigned int)raw));
		HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		HAL_Delay(10);
	}

	return raw;
}


/*
 * @brief	This function takes a distance reading
 * @param 	debug: a boolean that will enable putty output
 * @retval 	raw: a uint32_t raw value of distance reading
 */
void Read_Distance(uint8_t num_times){
	//start reading
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 2);	//set trigger with 20us pulse)

	//give time to grab reading
	HAL_Delay(200*num_times);

	//stop sending trigger to read
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0);	//set trigger with no pulse)
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM5){
		//reset counter after input capture interrupt occurs
		if(polarity == TIM_INPUTCHANNELPOLARITY_RISING){
			__HAL_TIM_SetCounter(htim, 0);
			polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

		}
		else if(polarity == TIM_INPUTCHANNELPOLARITY_FALLING){
			distance = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);// - capture;
			distance = distance / 148;
			polarity = TIM_INPUTCHANNELPOLARITY_RISING;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, polarity);
			__HAL_TIM_SetCounter(htim, 0);

			if(DEBUGGING){
				sprintf(buf, "Distance: %lu inches\r\n", distance);
				HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
			}//if
		}//else if
	}//if
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
