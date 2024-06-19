/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Wouter Swinkels, Merna Gramoun, Gedewon
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
#include <stdlib.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define STEP_TIME_MS 20					   // Singular step time for the feedback loop
#define PID_ERROR_THRESHOLD_DEGREES 10.0f  // Error threshold before the PID has reached its destination
#define TIMER_MAX_VALUE 65536			   // Maximum store value of General Purpose timer
#define SERVO_ENCODER_MAX_PWM_TIME_MS 1.1  // Maximum PWM time of an encoder signal in milliseconds
#define UNITS_FULL_CIRCLE 360			   // Units for a full angular rotation
#define MAX_PWM_MOTOR_DEVIATION 300		   // Maximum deviation from the still PWM (1500), determines speed

#define UART_BUFFER_SIZE 1				   // Size of UART receive buffer before interrupt
#define CIRCULAR_BUFFER_SIZE 128		   // Circular UART buffer for circular capture

#define MINFLOOR 0
#define MAXFLOOR 4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1; // GPIO UART for receiving any incoming commands and sending info back
UART_HandleTypeDef huart2; // USB UART for uploading and printing debug statements

/* USER CODE BEGIN PV */

/** UART receive variables: **/

uint8_t RxBuffer[UART_BUFFER_SIZE];
uint8_t CircularBuffer[CIRCULAR_BUFFER_SIZE];
volatile uint16_t writeIndex = 0;
volatile uint16_t readIndex = 0;

uint8_t incomingMsgBuffer[32] = {0}; // Message buffer used for parsing UART into a message stream until termination
uint8_t incMsgWriteIndex = 0;	     // Current write index of the message
const char INCOMING_MSG_END = '\n';    // Last sent character to indicate end of message
const char INCOMING_MSG_DELIMITER = ':';

/** Encoder capture variables **/

volatile bool capture_flag = false;		 // Used within ISR whether currently needing to capture rising or falling edge
volatile bool capture_done_flag = false; // Used to see whether a full capture is done

volatile uint32_t thigh = 0;		     // Timestamp recorded for rising edge capture
volatile uint32_t tlow = 0;				 // Timestamp recorded for falling edge capture
volatile uint32_t overflow_count = 0;	 // CCR Overflow counter recorded during signal capture

const float dcMin = 2.9;	// Minimum duty cycle of encoder capture
const float dcMax = 97.1;	// Maximum duty cycle of encoder capture

/** Elevator control functionality variables: **/

// TODO measure these for each floor
const float FLOOR_ANGLES[5] = {-120.0, -1000.0, -1613.0, -2300.0, -2970.0};
int maxElevatorSpeed = 100; // Speed percentage of how fast the elevator can go at most.
int setFloor = 0;
volatile int desiredFloor = 0;

volatile uint32_t start_time2 = 0;
volatile uint32_t end_time2 = 0;
volatile uint32_t start_time = 0;
volatile uint32_t end_time = 0;
volatile bool pressed = false;
volatile uint32_t time = 0;
volatile int counter = 0;
volatile bool debounce_done = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void UART1_Init(void);

void UART_print(const char *string);
void UART_print_formatted(const char *format, ...);

void TIM4_IRQHandler(void);
void Timer4_Init(void);
void TIM3_Configuration(void);

void calculateNewEncoderAngle(float* currentAngle, int* currentTurns);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void USART2_IRQHandler(void);
void UART1_Receive_IT(void);
void ProcessProtocolData(void);

void HandleNewElevatorFloorRequest(int newFloor);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

void EXTI3_IRQHandler(void)
{
    EXTI->PR |= EXTI_PR_PR3; // Clear the pending bit

    if (!debounce_done && !(GPIOB->IDR & GPIO_IDR_3))
    {
        
        debounce_done = true;
        start_time = HAL_GetTick();
    }
    else if (debounce_done && (GPIOB->IDR & GPIO_IDR_3))
    {
        // Button is released
        end_time = HAL_GetTick();

        if ((end_time - start_time) < 50)
        {
            
        }
        else
        {
            // Long press action
            desiredFloor++;
            if (desiredFloor > MAXFLOOR) desiredFloor = MAXFLOOR;
        }

        debounce_done = false; // Reset debounce flag
    }
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR10)
    {
        EXTI->PR |= EXTI_PR_PR6; // Clear the pending bit

        if (!debounce_done && !(GPIOB->IDR & GPIO_IDR_10))
        {
            // Button is pressed (assuming it's stable after debounce)
            debounce_done = true;
            start_time2 = HAL_GetTick();
        }
        else if (debounce_done && (GPIOB->IDR & GPIO_IDR_10))
        {
            // Button is released
            end_time2 = HAL_GetTick();

            if ((end_time2 - start_time2) < 50)
            {
               
            }
            else
            {
                desiredFloor--;
                if (desiredFloor < MINFLOOR) desiredFloor = MINFLOOR;
            }

            debounce_done = false; 
        }
    }
}

void configure_buttons(void) {
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN;
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3; // Clear EXTI3 configuration
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PB; // PB3 as EXTI line 3
	EXTI->IMR |= EXTI_IMR_IM3; // Enable interrupt for EXTI line 3
	EXTI->FTSR |= EXTI_FTSR_TR3;
	EXTI->RTSR |= EXTI_RTSR_TR3;

	// Configure EXTI for PB5 (Button connected here)
	SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10; // Clear EXTI10 configuration
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB; // PB10 as EXTI line 10
	EXTI->IMR |= EXTI_IMR_IM10; // Enable interrupt for EXTI line 10
	EXTI->FTSR |= EXTI_FTSR_FT10; // Trigger on falling edge for EXTI line 10
	EXTI->RTSR |= EXTI_RTSR_RT10; // Trigger on rising edge for EXTI line 10


	// Enable NVIC interrupts for EXTI lines
	 NVIC_EnableIRQ(EXTI3_IRQn); // Enable NVIC IRQ for EXTI line 3 (PB3)
	 NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable NVIC IRQ for EXTI lines 10 to 15 (PB10 is EXTI line 10)

	configure_GPIO(&GPIOB->MODER, NULL, &GPIOB->PUPDR, 10, 0);
	configure_GPIO(&GPIOB->MODER, NULL, &GPIOB->PUPDR, 3, 0);


	set_interrupt_priority(EXTI3_IRQn, 1);
	set_interrupt_priority(EXTI15_10_IRQn, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        printf("UART Error Callback: Error Code %lu\n", huart->ErrorCode);
        if (huart->ErrorCode & HAL_UART_ERROR_NE) {
            printf("Noise Error Detected.\n");
        }
        if (huart->ErrorCode & HAL_UART_ERROR_FE) {
            printf("Framing Error Detected.\n");
        }
        if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
            printf("Overrun Error Detected.\n");
        }
        if (huart->ErrorCode & HAL_UART_ERROR_PE) {
            printf("Parity Error Detected.\n");
        }

        // Clear errors and reinitialize
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        HAL_UART_Receive_IT(huart, RxBuffer, UART_BUFFER_SIZE);
    }
}


void USART2_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        HAL_UART_Receive_IT(&huart2, RxBuffer, UART_BUFFER_SIZE);
    }
    HAL_UART_IRQHandler(&huart2);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Copy received data to circular buffer
        for (int i = 0; i < UART_BUFFER_SIZE; i++) {
            CircularBuffer[writeIndex] = RxBuffer[i];
            writeIndex = (writeIndex + 1) % CIRCULAR_BUFFER_SIZE;
        }

        // Re-enable UART receive interrupt
        HAL_UART_Receive_IT(&huart1, RxBuffer, UART_BUFFER_SIZE);
    }
}

void USART1_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_Receive_IT(&huart1, RxBuffer, UART_BUFFER_SIZE);
    }
    HAL_UART_IRQHandler(&huart1);
}


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

void UART1_print(const char *string) {
    if (string != NULL) {
        HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string), HAL_MAX_DELAY);
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

void UART1_print_formatted(const char *format, ...) {
    char UARTString[64];

    va_list args;
    va_start(args, format);
    vsnprintf(UARTString, sizeof(UARTString), format, args);
    va_end(args);
    UART1_print(UARTString); // Use UART_print to transmit
}

void TIM4_IRQHandler(void) {

	// Rising or falling edge trigger:
	if (TIM4->SR & TIM_SR_CC1IF) {
		TIM4->SR &= ~TIM_SR_CC1IF;
		if (!capture_flag && !capture_done_flag) {	// Rising edge
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
			overflow_count++;
		}
	}
}

/**
  * @brief TIM3 Initialization Function
  * @details Sets TIM3 to a PWM output signal for a servo motor on a 50Hz frequency
  * @param None
  * @retval None
  * @author Merna Gramoun, Wouter Swinkels
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

    TIM3->CCR2 = 1500; // Set the signal to 1.5ms to pause the motor initially.
}


/**
  * @brief TIM4 Initialization Function
  * @details Sets TIM4 to capturing both the rising and falling edges on a 1MHz capture rate.
  * @param None
  * @retval None
  * @author Merna Gramoun, Wouter Swinkels
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



/**
  * @brief Function for handling any received protocol message
  * @details Handles both new floor request and elevator speed change request messages
  * @param message, specific message to process
  * @retval None
  * @author Merna Gramoun, Wouter Swinkels
  */
void HandleNewIncomingMessage(const char* message) {
    // 1. Find the message delimiter:
    const char* separator = strchr(message, INCOMING_MSG_DELIMITER);
    if (separator == NULL) { // No ':' found in the message
        UART_print("invalid msg\n");
        return;
    }

    // 2. Extract prefix/topic and number:
    size_t prefixLength = separator - message;
    char prefix[20];
    strncpy(prefix, message, prefixLength);
    prefix[prefixLength] = '\0';

    const char* numberStr = separator + 1;
    int n = atoi(numberStr);

    // 3. Handle message type accordingly:
    if (strcmp(prefix, "level") == 0) {
    	if (n >= MINFLOOR && n <= MAXFLOOR) {
        	UART_print_formatted("new floor = %d", n);
        	desiredFloor = n;
    	}
    } else if (strcmp(prefix, "speed") == 0) {
    	UART_print("new speed");
    	maxElevatorSpeed = n;
    } else {
        UART_print_formatted("Unknown message type: %s\n", prefix);
    }
}

/**
  * @brief
  * @details
  * @param None
  * @retval None
  * @author Merna Gramoun, Wouter Swinkels
  */
void ProcessProtocolData(void) {
    while (readIndex != writeIndex) {
        // Process the data in the circular buffer:
        uint8_t data = CircularBuffer[readIndex];
        readIndex = (readIndex + 1) % CIRCULAR_BUFFER_SIZE;

        // Write data to the new incoming message:
        incomingMsgBuffer[incMsgWriteIndex] = data;
        incMsgWriteIndex++;

        // Check if a message is done and therefore should be processed:
        if (data == INCOMING_MSG_END) {
        	incomingMsgBuffer[incMsgWriteIndex] = '\0';
        	HandleNewIncomingMessage((const char*)incomingMsgBuffer);
        	incMsgWriteIndex = 0;
            memset(incomingMsgBuffer, 0, sizeof(incomingMsgBuffer)); // clear buffer
        }
    }
}


/**
  * @brief Update current motor angle and full-cycle turns from new encoder measurements
  * @details Uses measured (volatile) encoder measurements
  * @param currentAngle: amount of the current, thus most recently measured, angle
  * @param currentTurns: amount of full-cycle angle turns the motor has already made
  * @retval None, params are updated through pointer references
  * @author Merna Gramoun, Wouter Swinkels
  */
void calculateNewEncoderAngle(float* currentAngle, int* currentTurns) {
	if (!capture_done_flag) return; // no new encoder capture, angle stays the same.

	// 1. Calculate the total tick time for the encoder capture:
	uint32_t ticks = ((tlow-thigh) + (overflow_count* (TIMER_MAX_VALUE + 1))) % (TIMER_MAX_VALUE + 1);
	capture_done_flag = false; // Another capture may now be done again as volatiles have been evaluated

	// 2. Calculate the current encoder angle from the duty cycle of the encoder:
	float PWM_duty_cycle = ((ticks) / (1100.0f)) * 100.0f;
	if (PWM_duty_cycle > 100) return; // wrong duty cycle. May happen with initial captures on bootup
	float motorTheta = (UNITS_FULL_CIRCLE - 1) - ((PWM_duty_cycle - dcMin) * UNITS_FULL_CIRCLE) / (dcMax - dcMin + 1);

    // 3. Check for any full-cycle turnovers and update the
    //    full-cycle turn amount accordingly:
    float angleDifference = motorTheta - *currentAngle;
    if (angleDifference > 180.0f) { // 0 -> 360
        (*currentTurns)--;
    } else if (angleDifference < -180.0f) { // 360 -> 0
    	(*currentTurns)++;
    }

	// 4. Update the current angle:
	*currentAngle = motorTheta;
}

void updateMotorSpeed(double normalisedControlUpdate, uint32_t maxTickDeviation) {
	int newMotorSpeed = (int)(normalisedControlUpdate * (double)maxTickDeviation);

	int calculated = (uint32_t)(1500 + (int)newMotorSpeed);
	UART_print_formatted("calculated=%d", (int)calculated);
	if (calculated >= 1465 && calculated < 1500) {
		calculated = 1460;
	} else if (calculated > 1500 && calculated <= 1520) {
		calculated = 1530;
	}

    TIM3->CCR2 = calculated;
	//UART_print_formatted("us=%d", (int)TIM3->CCR2);
}


void checkNewElevatorFloorRequest(PIDController* pid, float currentAngle, bool* activatePID) {
	if (setFloor == desiredFloor) return; // no floor change, nothing to be done
	if (desiredFloor < MINFLOOR || desiredFloor > MAXFLOOR) return; // floor not valid

	// For the new floor, update the PID controller accordingly:
	float targetDegree = FLOOR_ANGLES[desiredFloor]; // -1 to match array index for floor
	setPIDStep(pid, targetDegree, currentAngle); // Set the new PID step to the floor angle
	setFloor = desiredFloor;
	//UART_print_formatted("Updated floor to %d", setFloor);
	*activatePID = true;
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
  UART1_Init();
  configure_buttons();

  /* Setup timers: */

  Timer4_Init(); 		// Initialise TIM4 with input capture on PB6
  TIM3_Configuration(); // Initialise TIM3 to a PWM signal for the motor control

  /* Setup the angles to be used by the motor: */

  float currentAngle = 180.0; // The current angle of the motor. Initialize at 180 to not have direct turn wraparound.
  int currentTurns = -1;       // The current amount of full-degree turns the motor has made (used for calculating total)

  /* Setup the PID controller */

  double Kp = 3.0;
  double Ki = 0.0; // Using a PD controller; the elevator moves smoothly this way.
  double Kd = 0.01;
  double timeDelta = 20.0;  // Time delta of step function (used for integral and derivative calculations)
  double maxError = 2000.0; // Max expected (thus point of saturated normalized value) error in degrees

  PIDController* pid = initPIDController(Kp, Ki, Kd, timeDelta, maxError);

  if (pid == NULL) {
	  UART_print("Failed to setup PID controller");
	  while(1); // Halt
  }

  HAL_Delay(1000); // Wait to ensure that initial angle capture is already completed

  // Retrieve the initial angle from the encoder:
  while (!(currentAngle > 0.0 && currentAngle < 360.0)) {
	  calculateNewEncoderAngle(&currentAngle, &currentTurns);
  }

  static bool pidActive = false;

  while (1) {
	  // 1. Calculate the new angle by checking encoder capture:
	  calculateNewEncoderAngle(&currentAngle, &currentTurns);
	  float finalAngle = currentAngle + (currentTurns * (float)UNITS_FULL_CIRCLE);
	  UART_print_formatted("FA=%d\n", (int)finalAngle);

	  // 2. Check for any received elevator update requests:
	  //ProcessButtonChanges();
	  ProcessProtocolData();
	  checkNewElevatorFloorRequest(pid, finalAngle, &pidActive);

	  // 3. Run the PID when there is active error:
	  if (pidActive) {
		  // 3.1. Update the PID error
		  updatePIDError(pid, finalAngle);
		  UART_print_formatted("PIDERR = %d\n", (int)pid->values.currentError);

		  float currentErr = pid->values.currentError;

		  // 3.2. Apply the power:
		  double normalisedPowerControl = calculateNormalizedPIDControlValue(pid);

		  updateMotorSpeed(normalisedPowerControl, maxElevatorSpeed * 3);

		  // 3.3. Check whether done:
		  if (currentErr < PID_ERROR_THRESHOLD_DEGREES && currentErr > -PID_ERROR_THRESHOLD_DEGREES) {
			  UART_print_formatted("reached:%d", desiredFloor);
			  UART1_print_formatted("%d", desiredFloor);
			  updateMotorSpeed(0,maxElevatorSpeed * 3);
			  pidActive = false;
		  }


	  }
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

  // Enable UART interrupt
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

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

void UART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 38400;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, RxBuffer, UART_BUFFER_SIZE);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add their own implementation to report the HAL error return state */
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
