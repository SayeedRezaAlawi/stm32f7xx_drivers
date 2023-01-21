/*
 * 001led_toggle.c
 *	
 *  Created on: Jan 13, 2023
 *      Author: Sayeed Reza Alawi
 * LED toggling
 * toggle
 * toggle leds
 * leds
 */


#include <stdint.h>
#include <stm32f746xx.h>
#include <stm32f746xx_gpio_driver.h>


void delay(void)
{
	for(uint64_t i=0; i < (50000*2); i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed;
    gpioLed.pGPIOx = GPIOI;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

    GPIO_PeriClockControl(GPIOI, ENABLE);
    GPIO_Init(&gpioLed);
//    GPIO_WriteToOutputPin(GPIOI, GPIO_PIN_NO_1,ENABLE);
	while(1)
	{
		GPIO_ToggleOutputPin(GPIOI, GPIO_PIN_NO_1);
		delay();
	}
}
