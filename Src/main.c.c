/**
 ******************************************************************************
 * FILE               : main.c
 * PROGRAMMER          : NAVREETINDER KAUR
 * DESCRIPTION        : Roulett wheel state-machine
 ******************************************************************************
 * Initially all LEDs will flash in a sequence with a delay of 200ms.
 * When the button is pressed, the delay time of led's reduced to 50ms.
 * When the button is released,the delay of 150ms is added with every flashing
 of led in a sequence.
 * When that delay reaches to 2000ms, the last led will flash 4 times with 500ms
 delay time and then wait for 5sec to start the pattern from initial stage.
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// states used for roulette wheel example
#define WAITING_FOR_PUSHBUTTON_PRESS_STATE		0
#define WAITING_FOR_PUSHBUTTON_RELEASE_STATE	1
#define WAITING_FOR_LED_PATTERN_COMPLETE_STATE	2

// led flash pattern control constants
#define INITIAL_LED_DELAY						200
#define BUTTON_PRESSED_LED_DELAY				50
#define SLOW_DOWN_CONSTANT						150
#define STOP_SPINNING_VALUE						2000
#define WAIT_FOR_NEXT_ROUND_DELAY				5000
#define DONE_BLINK_RATE_DELAY					500
#define NUMBER_OF_FLASHES_WHEN_DONE				4

// delayAfterTurnOn values
#define DO_NOT_PUT_A_DELAY_AFTER_TURN_ON		0
#define PUT_A_DELAY_AFTER_TURN_ON				1

// checkPushButtonDebounced return values
#define BUTTON_PRESSED							1
#define BUTTON_NOT_PRESSED						0

// debounce delay time in ms
#define DEBOUNCE_DELAY							2

// an array holding the pattern of LEDs
#define LED_PATTERN_SIZE	8
static uint16_t ledPattern[] = { LD4_Pin, LD3_Pin, LD5_Pin, LD7_Pin,
LD9_Pin, LD10_Pin, LD8_Pin, LD6_Pin };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
void turnOnLed(uint16_t ledNumber, int delayAfterTurnOnFlag);
int checkPushButtonDebounced(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// FUNCTION      : turnOnLed
// DESCRIPTION   :
// This function will turn on a given LED using supplied GPIO pin number and turn off the last one
// PARAMETERS    :
// ledNumber     : LED pin to turn on
// delayAfterTurnOnFlag : if set put a delay after the led is turn on in
// RETURNS       :Nothing
void turnOnLed(uint16_t ledNumber, int delayAfterTurnOnFlag) {
	// not set yet so set the value to invalid one
	static uint16_t lastLed = 0xffff;

	if (lastLed != 0xffff) {
		// clear the last LED if needed
		HAL_GPIO_WritePin(GPIOE, lastLed, GPIO_PIN_RESET);
	}

	// keep track of last led that we turned on so we can turn it off next time
	lastLed = ledNumber;

	if (delayAfterTurnOnFlag == PUT_A_DELAY_AFTER_TURN_ON)
		HAL_Delay(500);

	// turn on the LED pin on GPIO
	HAL_GPIO_WritePin(GPIOE, lastLed, GPIO_PIN_SET);
}

// FUNCTION      : checkPushButtonDebounced
// DESCRIPTION   :
// This function will return the state of the push button on the STM debounced
// PARAMETERS    : None
// RETURNS       : 0 if not pushed else non zero
int checkPushButtonDebounced(void) {
	int buttonState = BUTTON_NOT_PRESSED;

	// get the initial button state
	if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0)
		buttonState = BUTTON_PRESSED;
	// small delay to bypass bounce on push button
	HAL_Delay( DEBOUNCE_DELAY);

	// check if the button state does not agree with first sample
	// if it does not agree ignore it
	if (buttonState == BUTTON_PRESSED) {
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == 0)
			buttonState = BUTTON_NOT_PRESSED;
	} else {
		if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != 0)
			buttonState = BUTTON_PRESSED;
	}

	return buttonState;
}

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// used to select LED to turn on
		static int ledIndex = 0;
		static uint32_t delayTime = INITIAL_LED_DELAY;
		static int stateMachine = WAITING_FOR_PUSHBUTTON_PRESS_STATE;

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		HAL_Delay(delayTime);
		// turn on the LED (and turn off the old one)
		turnOnLed(ledPattern[ledIndex++], DO_NOT_PUT_A_DELAY_AFTER_TURN_ON);
		// is the pattern ready to repeat
		if (ledIndex >= LED_PATTERN_SIZE)
			ledIndex = 0;

		switch (stateMachine) {
		case WAITING_FOR_PUSHBUTTON_PRESS_STATE:
			delayTime = INITIAL_LED_DELAY;
			// will check the condition whether push button is pressed or not
			if (checkPushButtonDebounced() == BUTTON_PRESSED) {
				// if push-button is pressed, stateMachine will go to the next state.
				stateMachine = WAITING_FOR_PUSHBUTTON_RELEASE_STATE;
			}
			break;
		case WAITING_FOR_PUSHBUTTON_RELEASE_STATE:
			delayTime = BUTTON_PRESSED_LED_DELAY;
			// will check the condition whether push button is pressed or not
			if (checkPushButtonDebounced() == BUTTON_NOT_PRESSED) {
				// if push-button is not pressed, stateMachine will go to the next state
				stateMachine = WAITING_FOR_LED_PATTERN_COMPLETE_STATE;
			}
			break;
		case WAITING_FOR_LED_PATTERN_COMPLETE_STATE:
		default:
			// SLOW_DOWN_CONSTANT will be added each time loop is executed
			delayTime += SLOW_DOWN_CONSTANT;
			// check the condition whether the delayTime is greater than STOP_SPINNING_VALUE
			if (delayTime >= STOP_SPINNING_VALUE) {
				// loop for flashing the last LED 4 times
				ledIndex--;
				for (int i = 0; i < NUMBER_OF_FLASHES_WHEN_DONE; i++) {
					// to blink led with delay of 500ms
					HAL_Delay(DONE_BLINK_RATE_DELAY);
					// calling of a function to turn on led with delay
					turnOnLed(ledPattern[ledIndex], PUT_A_DELAY_AFTER_TURN_ON);
				}
				// delay to wait for next round
				HAL_Delay(WAIT_FOR_NEXT_ROUND_DELAY);
				// Led will flash with a delay of 200ms
				delayTime = INITIAL_LED_DELAY;
				/* next state will be the  WAITING_FOR_PUSHBUTTON_PRESS_STATE
				 after execution of this state*/
				stateMachine = WAITING_FOR_PUSHBUTTON_PRESS_STATE;

			}
			break;
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PA5   ------> SPI1_SCK
 PA6   ------> SPI1_MISO
 PA7   ------> SPI1_MOSI
 PA11   ------> USB_DM
 PA12   ------> USB_DP
 PB6   ------> I2C1_SCL
 PB7   ------> I2C1_SDA
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
	 MEMS_INT2_Pin */
	GPIO_InitStruct.Pin = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin
			| MEMS_INT1_Pin | MEMS_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
	 LD7_Pin LD9_Pin LD10_Pin LD8_Pin
	 LD6_Pin */
	GPIO_InitStruct.Pin = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin
			| LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 PA6 SPI1_MISO_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | SPI1_MISO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DM_Pin DP_Pin */
	GPIO_InitStruct.Pin = DM_Pin | DP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF14_USB;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin | LD7_Pin | LD9_Pin
					| LD10_Pin | LD8_Pin | LD6_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
