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
#include "PIDController.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STEP_TIME_MS 20					  // Singular step time for the feedback loop
#define TIMER_MAX_VALUE 65536			  // Maximum store value of General Purpose timer
#define SERVO_ENCODER_MAX_PWM_TIME_MS 1.1 // Maximum PWM time of an encoder signal in milliseconds
#define UNITS_FULL_CIRCLE 360			  // Units for a full angular rotation

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile bool capture_flag = false;		 // Used within ISR whether currently needing to capture rising or falling edge
volatile bool capture_done_flag = false; // Used to see whether a full capture is done
volatile bool overflow_flag = false;

volatile uint32_t thigh = 0;		     // Timestamp recorded for rising edge capture
volatile uint32_t tlow = 0;				 // Timestamp recorded for falling edge capture
volatile uint32_t overflow_count = 0;	 // CCR Overflow counter recorded during signal capture

// TODO rename the duty cycle constants to be more reflective

const float dcMin = 2.9;	// Minimum duty cycle (of encoder capture)
const float dcMax = 97.1;	// Maximum duty cycle (of encoder capture)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

void UART_print(const char *string);
void UART_print_formatted(const char *format, ...);

void TIM4_IRQHandler(void);
void Timer4_Init(void);
void TIM3_Configuration(void);

void calculateNewEncoderAngle(float* currentAngle, int* currentTurns);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief UART transmission function to output a string to the serial monitor
  * @param string the string literal to transmit over UART
  * @author Wouter Swinkels
 */
void UART_print(const char *string) {
    if (string != NULL) {
        HAL_UART_Transmit(&huart2, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
    }
}

/**
  * @brief UART transmission of a string with formatting (like in `sprintf()`).
  * @param string the string literal to transmit over UART, (e.g., "distance: %d")
  * @param ... any parameters to be formatted into the string (e.g., int distance)
  * @author Wouter Swinkels
 */
void UART_print_formatted(const char *format, ...) {
    char UARTString[64];

    va_list args;
    va_start(args, format);
    vsnprintf(UARTString, sizeof(UARTString), format, args);
    va_end(args);
    UART_print(UARTString); // Use UART_print to transmit
}

void TIM4_IRQHandler(void) {

	// Rising or falling edge trigger:
	if (TIM4->SR & TIM_SR_CC1IF) {
		TIM4->SR &= ~TIM_SR_CC1IF;
		if (!capture_flag && !capture_done_flag) {		     // Rising edge
			thigh = TIM4->CCR1;          // Store time of rising edge
			TIM4->CCER ^= TIM_CCER_CC1P; // Switch to capturing falling edge
			capture_flag = true;
			overflow_count = 0;          // Reset overflow, as this is a new time capture
			capture_done_flag = false;
			capture_flag = true;
		} else if (capture_flag && !capture_done_flag) { // Falling edge
			tlow = TIM4->CCR1;           // Store time of the falling edge using CCR
			capture_flag = false;
			capture_done_flag = true;
			TIM4->CCER ^= TIM_CCER_CC1P; // Switch back to capture the rising edge
			capture_done_flag = true; // should be set false in other task handler.
		}
	}

  // Overflow of TIM4, increment the overflow count
  // to ensure proper calculation of time duration.
	if (TIM4->SR & TIM_SR_UIF) {
		TIM4->SR &= ~TIM_SR_UIF;
		if (capture_flag && !capture_done_flag) {
			overflow_flag = true;
			overflow_count++;
		}
	}
}

/**
  * @brief TIM3 Initialization Function
  * @details Sets TIM3 to a PWM output signal for a servo motor on a 50Hz frequency
  * @param None
  * @retval None
  */
void TIM3_Configuration(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODER5);
    GPIOB->MODER |= GPIO_MODER_MODER5_1;

    GPIOB->AFR[0] &= ~GPIO_AFRL_AFRL5;
    GPIOB->AFR[0] |= (0b0010 << GPIO_AFRL_AFRL5_Pos);

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 71;
    TIM3->ARR = 19999; // Set auto-reload value to 50Hz (period of 20ms on 1MHz)

    TIM3->CCMR1 &= ~(TIM_CCMR1_OC2M);
    TIM3->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
    TIM3->CCER |= TIM_CCER_CC2E;
    TIM3->CR1 |= TIM_CR1_CEN;

    TIM3->CCR1 = 1500; // Set the signal to 1.5ms to pause the motor initially.
}


/**
  * @brief TIM4 Initialization Function
  * @details Sets TIM4 to capturing both the rising and falling edges on a 1MHz capture rate.
  * @param None
  * @retval None
  */
void Timer4_Init(void) {

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;	// Enable clock for GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable APB clock for TIM4


    GPIOB->MODER &= ~(GPIO_MODER_MODER6); // Configure PB6 to AF mode
    GPIOB->MODER |= GPIO_MODER_MODER6_1;

	GPIOB->AFR[0] &= ~(0xFU << (4U * 6U)); // Configure AF mapping to PB6
    GPIOB->AFR[0] |= (2U << (4U * 6U));


	TIM4->PSC = 71; // Configure PSC for 1MHz clock (72MHZ/72=1MHz)

	// Set TIM4 to Capture Compare input (see CCCER and CCMR entries in datasheet)
	TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1PSC | TIM_CCMR1_IC1F);
	TIM4->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP);
	TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S);

	TIM4->CCMR1 |= TIM_CCMR1_CC1S_0; // Set to input mode

	TIM4->CCER |= TIM_CCER_CC1E;	// Enable capture compare
	TIM4->CCER &= ~TIM_CCER_CC1NP;  // Set to capture both rising and falling

	TIM4->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_CC1DE);
	TIM4->DIER |= TIM_DIER_CC1IE;	// Enable interrupt for capture channel 1
	TIM4->DIER |= TIM_DIER_UIE;		// Enable update interrupt for overflow capture

	// Enable TIM4 for channel 1 (by setting Control Register 1 values):
	TIM4->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_UDIS |
				   TIM_CR1_CMS | TIM_CR1_ARPE | TIM_CR1_UIFREMAP);
	TIM4->CR1 &= ~TIM_CR1_DIR;
	TIM4->CR1 |= TIM_CR1_CEN;

	// Enabe the interrupt service routine for TIm4:
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 0);
}

void calculateNewEncoderAngle(float* currentAngle, int* currentTurns) {
	if (!capture_done_flag) return; // no new encoder capture, angle stays the same.
	capture_done_flag = false;

	// 1. Calculate the capture tick time
	//UART_print("New capture + angle update! ");
	uint32_t ticks = ((tlow-thigh) + (overflow_count* (TIMER_MAX_VALUE + 1))) % (TIMER_MAX_VALUE + 1);
	UART_print_formatted("ticks = %d\n", ticks);

	// 2. Calculate the current angle from the duty cycle of the encoder:
	float PWM_duty_cycle = ((ticks) / (1100.0f)) * 100.0f;
	UART_print_formatted("dc = %d\n", (int)PWM_duty_cycle);
	if (PWM_duty_cycle > 100) return; // wrong duty cycle. May happen with initial captures on bootup
	float motorTheta = (UNITS_FULL_CIRCLE - 1) - ((PWM_duty_cycle - dcMin) * UNITS_FULL_CIRCLE) / (dcMax - dcMin + 1);
	UART_print_formatted("measured = %d\n", (int)motorTheta);

	// 3. Check whether the new angle has made any full cycle turns, and
	//    apply those turns accordingly.

    float angleDifference = motorTheta - *currentAngle;

    if (angleDifference > 180.0f) {
        UART_print("wrapped from 360 to 0");
        while(1);
        (*currentTurns)--;
    } else if (angleDifference < -180.0f) {
        UART_print("wrapped from 0 to 360");
        while(1);
    	(*currentTurns)++;
    }

	// 4. Update the current angle:
	*currentAngle = motorTheta;

	UART_print_formatted("turns=%d\t", *currentTurns);
	UART_print_formatted("angle=%d\n", (int)*currentAngle);


}

void updateMotorSpeed(double normalisedControlUpdate, uint32_t maxTickDeviation) {
	uint32_t newMotorSpeed = (uint32_t)(normalisedControlUpdate * (double)maxTickDeviation);
	TIM3->CCR1 = 1500 + newMotorSpeed;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Setup timers: */

  Timer4_Init(); 		// Initialise TIM4 with input capture on PB6
  TIM3_Configuration(); // Initialise TIM3 to a PWM signal for the motor control

  /* Setup the angles to be used by the motor: */

  float currentAngle = 0.0; // The current angle of the motor
  int currentTurns = 0;     // The current amount of full-degree turns the motor has made (used for calculating total)

  /* Setup the PID controller */

  double Kp = 3.0;
  double Ki = 0.0; // Will evaluate to a PD controller if Ki is zero.
  double Kd = 0.01;
  double timeDelta = 20.0;  // Time delta of step function (used for integral and derivative calculations)
  double maxError = 1000.0; // Max expected (thus point of saturated normalized value) error in degrees

  PIDController* pid = initPIDController(Kp, Ki, Kd, timeDelta, maxError);

  if (pid == NULL) {
	  UART_print("Failed to setup PID controller");
	  while(1); // Halt
  }

  HAL_Delay(1000); // Wait to ensure that initial capture is already completed
  	  	  	     // Otherwise, one gets erronous values.

  // Retrieve the initial angle from the encoder:
  calculateNewEncoderAngle(&currentAngle, &currentTurns);

  double targetDegree = 500.0; // Example target degree.
  setPIDStep(pid, targetDegree, currentAngle);

  while (1) {

	  /*
	   * Layout of activity steps:
	   * 1. Calculating the current encoder angle and updating the PID error
	   * 2. Applying the power using PID updates and power control
	   * 3. Waiting for the step amount
	   */

	  // 1. Calculate the new angle and update the PID error:
	  calculateNewEncoderAngle(&currentAngle, &currentTurns);
	  if (true) continue;

	  updatePIDError(pid, (double)currentAngle);
	  UART_print_formatted("PIDERR=%d", (int)pid->values.currentError);

	  // 2. Calculate the new normalized [-1,1] PID speed and update the motor
	  //    to it with that new speed:
	  double normalizedPowerControl = calculateNormalizedPIDControlValue(pid);
	  updateMotorSpeed(normalizedPowerControl, 300);

	  HAL_Delay(STEP_TIME_MS);
  }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void) {

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
