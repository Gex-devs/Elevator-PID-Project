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
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_conf.h"
#include "stm32f3xx_hal_gpio.h"
#include "stm32f3xx_hal_uart.h"
#include "stdbool.h"

volatile uint32_t start_time2 = 0;
volatile uint32_t end_time2 = 0;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile bool pressed = false;
volatile uint32_t time = 0;
volatile int counter = 0;

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes ----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void configure_GPIO(volatile uint32_t *GPIO_MODER,
		volatile uint32_t *GPIO_OTYPER, volatile uint32_t *GPIO_PUPDR,
		uint8_t pin, uint8_t mode);
void set_interrupt_priority(IRQn_Type IRQn, uint32_t priority);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile bool debounce_done = false;

void EXTI3_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR3; // Clear the pending bit

    if (!debounce_done && !(GPIOB->IDR & GPIO_IDR_3))
    {
        // Button is pressed (assuming it's stable after debounce)
        debounce_done = true;
        start_time = HAL_GetTick();
    }
    else if (debounce_done && (GPIOB->IDR & GPIO_IDR_3))
    {
        // Button is released
        end_time = HAL_GetTick();

        if ((end_time - start_time) < 50)
        {
            // Short press action
            GPIOB->ODR ^= GPIO_ODR_5; // Toggle LED on PB5
        }
        else
        {
            // Long press action
            counter++;
        }

        debounce_done = false; // Reset debounce flag
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR5)
    {
        EXTI->PR |= EXTI_PR_PR5; // Clear the pending bit

        if (!debounce_done && !(GPIOB->IDR & GPIO_IDR_5))
        {
            // Button is pressed (assuming it's stable after debounce)
            debounce_done = true;
            start_time2 = HAL_GetTick();
        }
        else if (debounce_done && (GPIOB->IDR & GPIO_IDR_5))
        {
            // Button is released
            end_time2 = HAL_GetTick();

            if ((end_time2 - start_time2) < 50)
            {
                // Short press action
                // GPIOA->ODR ^= GPIO_ODR_8; // Toggle LED on PA8 (assuming PA8 is connected to an LED)
            }
            else
            {
                // Long press action
                counter--;
            }

            debounce_done = false; // Reset debounce flag
        }
    }
}

int main(void) {

	HAL_Init();

	HAL_InitTick(TICK_INT_PRIORITY);
	SystemClock_Config();
//	  MX_GPIO_Init();
	MX_USART2_UART_Init();

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;

//  SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI3);
//  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB;
//  EXTI->IMR |= EXTI_IMR_IM3;
//  EXTI->FTSR |= EXTI_FTSR_FT3;
//  EXTI->RTSR |= EXTI_RTSR_RT3;
//  NVIC_EnableIRQ(EXTI3_IRQn);
//
//  SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
//  SYSCFG->EXTICR[3] |= (0b010 << SYSCFG_EXTICR4_EXTI13_Pos);
//  EXTI->FTSR |= EXTI_FTSR_TR13;
//  EXTI->RTSR |= EXTI_RTSR_TR13;
//  EXTI->IMR |= EXTI_IMR_MR13;
//  NVIC_EnableIRQ(EXTI15_10_IRQn);
	// Configure EXTI for PB3 (B1 button)
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3; // Clear EXTI3 configuration
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB; // PB3 as EXTI line 3
	EXTI->IMR |= EXTI_IMR_IM3; // Enable interrupt for EXTI line 3
	EXTI->FTSR |= EXTI_FTSR_TR3;
	EXTI->RTSR |= EXTI_RTSR_TR3;

	// Configure EXTI for PB5 (Button connected here)
	SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR2_EXTI5; // Clear EXTI5 configuration
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB; // PB5 as EXTI line 5
	EXTI->IMR |= EXTI_IMR_IM5; // Enable interrupt for EXTI line 5
	EXTI->FTSR |= EXTI_FTSR_FT5; // Trigger on falling edge for EXTI line 5
	EXTI->RTSR |= EXTI_RTSR_RT5; // Trigger on falling edge for EXTI line 5

	// Enable NVIC interrupts for EXTI lines
	NVIC_EnableIRQ(EXTI3_IRQn); // Enable NVIC IRQ for EXTI line 3 (PB3)
	NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable NVIC IRQ for EXTI lines 5 to 9 (PB5 is EXTI line 5)

	configure_GPIO(&GPIOB->MODER, NULL, &GPIOB->PUPDR, 5, 0);
	configure_GPIO(&GPIOB->MODER, NULL, &GPIOB->PUPDR, 3, 0);


	set_interrupt_priority(EXTI3_IRQn, 2);
	set_interrupt_priority(EXTI15_10_IRQn, 1);

	while (1) {

		HAL_Delay(1000000);

	}
}

void set_interrupt_priority(IRQn_Type IRQn, uint32_t priority) {
	uint32_t priority_group = NVIC_GetPriorityGrouping();

	uint32_t priority_value = NVIC_EncodePriority(priority_group, priority, 0);

	// Set the priority for the given interrupt
	NVIC_SetPriority(IRQn, priority_value);
}

void configure_GPIO(volatile uint32_t *GPIO_MODER,
		volatile uint32_t *GPIO_OTYPER, volatile uint32_t *GPIO_PUPDR,
		uint8_t pin, uint8_t mode) {
	*GPIO_MODER &= ~(0b10 << (pin * 2));

	*GPIO_MODER |= (mode << (pin * 2));

	if (GPIO_OTYPER == NULL) {
		*GPIO_PUPDR &= ~(0b10 << (pin * 2));
		*GPIO_PUPDR |= (0b01 << (pin * 2));
	} else if (GPIO_PUPDR == NULL) {
		*GPIO_OTYPER &= ~(0b10 << pin);
		*GPIO_OTYPER |= (0 << pin);
	}
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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
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
//  if (HAL_UART_Init(&huart2) != HAL_OK)
//  {
//    Error_Handler();
//  }
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOF_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}
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
