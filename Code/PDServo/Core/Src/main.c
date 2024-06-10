/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include "stm32f3xx_hal_gpio.h" // Include the GPIO header file
#include "stm32f3xx_hal_rcc.h" // Include the RCC header file
#include"stm32f3xx_it.h"
#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <Math.h>
#define PERIOD 1100
//#define PWM_PERIOD 20000  // Define the PWM period in clock cycles (e.g., 20 ms at 1 MHz)
volatile const int units_full_circle = 360;                          // Units in a full circle
volatile int dutyScale = 1000;                       // Scale duty cycle to 1/1000ths
volatile float dcMin = 2.9;                             // Minimum duty cycle
volatile float dcMax = 97.1;                            // Maximum duty cycle
volatile const int q2min = 90;                      // For checking if in 1st quadrant
volatile int q3max = 90 * 3;                      // For checking if in 4th quadrant
volatile int turns = 0;
volatile float dc, theta, thetaP, thigh, tlow,  powerOutput, echo_end ,overflow_count,echo_start, totalTargetAngle, error;
volatile int targetAngle = 200;
volatile float dt = 0.0f;
volatile float pulse_width = 0;
uint32_t Error = 0;
uint32_t setpoint = 0;
uint32_t prevsetpoint = 0;
volatile uint32_t tcycle = 0;
volatile float PWM_duty_cycle = 0;
volatile float angel = 0;
volatile float offset  = 0;
volatile float errorAngle =  0 ;
volatile float calculatedAngle = 0;
volatile long previousTime = 0;
uint32_t derivative = 0;
float power = 0;
float prevError = 0 ;
float kD = 0;
uint32_t KP = 1;
volatile bool rising = true;
volatile uint32_t risingEdgeTimestamp = 0;
volatile uint32_t fallingEdgeTimestamp = 0;
volatile const double MAX_PULSE_WIDTH_US = 65535;
volatile bool capture_flag =false;
volatile bool capture_done_flag = false;
bool up = false;
float currentAngle;
volatile uint32_t TimeDelay;
float myPD(uint32_t setpoint);
void TIM3_Configuration(void);
void TIM4_Configuration(void);
void Timer4_Init(void);

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
void TIM4_IRQHandler(void) {
	// Rising or falling edge trigger:
	if (TIM4->SR & TIM_SR_CC1IF) {

		TIM4->SR &= ~TIM_SR_CC1IF;
		if (!capture_flag) {		       // Rising edge
			thigh = TIM4->CCR1;          // Store time of rising edge
			TIM4->CCER ^= TIM_CCER_CC1P; // Switch to capturing falling edge
			capture_flag = true;
			overflow_count = 0;          // Reset overflow, as this is a new time capture
		} else { // Falling edge
			tlow = TIM4->CCR1;           // Store time of the falling edge using CCR
      capture_flag = false;
      capture_done_flag = true;
			TIM4->CCER ^= TIM_CCER_CC1P; // Switch back to capture the rising edge
		}
	}
  
  // Overflow of TIM4, increment the overflow count
  // to ensure proper calculation of time duration.
	if (TIM4->SR & TIM_SR_UIF) {
		TIM4->SR &= ~TIM_SR_UIF;
		if (capture_flag) {
			overflow_count++;
		}
	}
}

#define TIM4_MAX_CLOCK 65536
#define SERVO_ENCODER_MAX_PWM_TIME_MS 1.1

void calculate_elevator_turn_activity(void) {
  if (!capture_done_flag) return;

  total_time_elapsed = (tlow - thigh) + overflow_count * TIM4_MAX_CLOCK; 
  pulse_width = total_time_elapsed/1000;

  PWM_duty_cycle = ((pulse_width) / (SERVO_ENCODER_MAX_PWM_TIME_MS)) * 100;
  float newTheta = (units_full_circle - 1) - ((PWM_duty_cycle - dcMin) * units_full_circle) / (dcMax - dcMin + 1);

  // Calculate turns based on rotational quadrants (see unit circle)
  if ((newTheta < q2min) && (theta > q3max)) { // If 4th to 1st quadrant
    turns++;
  } else if ((theta < q2min) && (newTheta > q3max)) { // If in 1st to 4th quadrant
    turns--;
  }

  capture_done_flag = 
  theta = newTheta;
}

int main(void) {
  HAL_Init();
  __enable_irq();
  SystemClock_Config();
  TIM4_IRQHandler();
  MX_GPIO_Init();
  Timer4_Init();
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_CaptureCallback(&htim4);
  TIM3_Configuration();

  float totalTurns = 2; // Number of turns before reaching the final angle (negative for reverse direction)
  float finalAngle = targetAngle; // Final target angle
  totalTargetAngle = totalTurns * units_full_circle + finalAngle; // Total target angle including turns

  while (1) {
    calculate_elevator_turn_activity();
    powerOutput = myPD(totalTargetAngle);

    if (up) {
      errorAngle = totalTargetAngle - ((turns * units_full_circle) + theta);
      if (errorAngle > 0) {
        if (powerOutput > 4) {
          offset = 30;
        } else {
          offset = 0;
        }
      }
    } else {
      errorAngle = (targetAngle % units_full_circle) - theta;
      if (powerOutput > 4) {
        offset = -40;
      } else {
        offset = 0;
      }
    }

    TIM3->CCR2 = 1500 + offset; // Apply turning speed elevator servo 
    HAL_Delay(20);              // => PID sample time
  }
}

float myPD(uint32_t totalTargetAngle) {
    uint32_t currentTime = HAL_GetTick();
    dt = currentTime - previousTime;
    previousTime = currentTime;

    currentAngle = (fabs(turns) * units_full_circle) + theta;
    error = totalTargetAngle - currentAngle;
    //float derivative = error - prevError; // Calculate the derivative term
    //prevError = error;

    power = fabs(error) * KP; // Add the derivative term
    return power;
}




  void TIM3_Configuration(void)
  {
      RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
      RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

      GPIOB->MODER &= ~(GPIO_MODER_MODER5);
      GPIOB->MODER |= GPIO_MODER_MODER5_1;

      GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL5;
      GPIOB->AFR[0] |= (0b0010 << GPIO_AFRL_AFRL5_Pos);

      RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
      TIM3->PSC = 8 - 1;
      TIM3->ARR = 19999;         // Set auto-reload value (period of PWM signal)
      TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M);
      TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
      TIM3->CCER |= TIM_CCER_CC2E;
      TIM3->CR1 |= TIM_CR1_CEN;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
void Timer4_Init(void) {

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;


    GPIOB->MODER &= ~(GPIO_MODER_MODER6);
    GPIOB->MODER |= GPIO_MODER_MODER6_1;

	GPIOB->AFR[0] &= ~(0xFU << (4U * 6U));
    GPIOB->AFR[0] |= (2U << (4U * 6U));


	TIM4->PSC = 15;

	TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F);

	TIM4->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S);


	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0;

	TIM4->CCER |= TIM_CCER_CC1E;
	TIM4->CCER &= ~TIM_CCER_CC1NP;

	TIM4->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC1DE);

	TIM4->DIER |= TIM_DIER_CC1IE;
	TIM4->DIER |= TIM_DIER_UIE;

	TIM4->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_UDIS |
				   TIM_CR1_CMS | TIM_CR1_ARPE | TIM_CR1_UIFREMAP);
	TIM4->CR1 &= ~TIM_CR1_DIR;
	TIM4->CR1 |= TIM_CR1_CEN;

	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 0);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
