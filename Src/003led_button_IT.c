/*
 * 001led_toggle.c
 *
 *  Created on: Jan 13, 2023
 *      Author: Reza
 */


#include <stdint.h>
#include<string.h>
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
    GPIO_Handle_t gpioLed, gpioButton;

    memset(&gpioLed,0,sizeof(gpioLed));
    memset(&gpioButton,0,sizeof(gpioButton));

    gpioLed.pGPIOx = GPIOI;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
    gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

    GPIO_PeriClockControl(GPIOI, ENABLE);

    GPIO_Init(&gpioLed);

    gpioButton.pGPIOx = GPIOI;
    gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_PeriClockControl(GPIOI, ENABLE);

    GPIO_Init(&gpioButton);

    GPIO_WriteToOutputPin(GPIOI,GPIO_PIN_NO_1,GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(GPIOI,GPIO_PIN_NO_1,GPIO_PIN_SET);
    // IRQ configuration
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIO15);
    GPIO_IRQITConfig(IRQ_NO_EXTI15_10, ENABLE);
//    GPIO_WriteToOutputPin(GPIOI, GPIO_PIN_NO_1,ENABLE);
	while(1)
	{

	}
}




void EXTI15_10_IRQHandler(void)
{

	GPIO_ToggleOutputPin(GPIOI, GPIO_PIN_NO_1);
	GPIO_IRQHandling(GPIO_PIN_NO_11);
}
