/*
 * 001led_toggle.c
 *
 *  Created on: Jan 13, 2023
 *      Author: Reza
 */


#include <stdint.h>
#include <stm32f746xx.h>
#include <stm32f746xx_gpio_driver.h>

#define HIGH			1
#define	BTN_PRESSED		HIGH

void delay(void)
{
	for(uint64_t i=0; i < (500000/2); i++);
}

int main(void)
{
    GPIO_Handle_t gpioLed;
    gpioLed.pGPIOx = GPIOI;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


    GPIO_Handle_t gpioButton;
    gpioButton.pGPIOx = GPIOI;
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

    GPIO_PeriClockControl(GPIOI, ENABLE);
    GPIO_Init(&gpioLed);
    GPIO_Init(&gpioButton);
//    GPIO_WriteToOutputPin(GPIOI, GPIO_PIN_NO_1,ENABLE);
	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOI, GPIO_PIN_NO_11) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOI, GPIO_PIN_NO_1);

		}
	}
}
