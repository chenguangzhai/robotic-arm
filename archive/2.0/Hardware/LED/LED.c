#include "LED.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>



void Set_LED(uint8_t Addr) {
	if (Addr) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}
}

void LED_Turn(void) {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}
