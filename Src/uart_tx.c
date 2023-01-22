/*
 * uart_tx.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Reza
 */

#include <stdio.h>
#include <string.h>
#include "stm32f746xx.h"


#define HIGH			1
#define	BTN_PRESSED		HIGH

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart1_handle;

void USART1_Init(void)
{
	usart1_handle.pUSARTx = USART1;
	usart1_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart1_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart1_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart1_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart1_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart1_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart1_handle);
}


void USART1_GPIOInit(void)
{
	GPIO_Handle_t usart_gpiosTX, usart_gpiosRX;

	usart_gpiosTX.pGPIOx = GPIOA;
	usart_gpiosTX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpiosTX.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpiosTX.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpiosTX.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpiosTX.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART1 TX
	usart_gpiosTX.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_9;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&usart_gpiosTX);

	usart_gpiosRX.pGPIOx = GPIOB;
	usart_gpiosRX.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpiosRX.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpiosRX.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpiosRX.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpiosRX.GPIO_PinConfig.GPIO_PinAltFunMode =7;

	//USART1 RX
	usart_gpiosRX.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&usart_gpiosRX);


}

void GPIO_ButtonInit(void)
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

}


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

	GPIO_ButtonInit();

	USART1_GPIOInit();

    USART1_Init();

    USART_PeripheralControl(USART1,ENABLE);

//    while(1)
//    {
//    	if(GPIO_ReadFromInputPin(GPIOI, GPIO_PIN_NO_11) == BTN_PRESSED)
//    	{
//    		delay();
//    		GPIO_ToggleOutputPin(GPIOI, GPIO_PIN_NO_1);
//
//    	}
//    }

    while(1)
    {
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOI,GPIO_PIN_NO_11) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();
		GPIO_ToggleOutputPin(GPIOI, GPIO_PIN_NO_1);

		USART_SendData(&usart1_handle,(uint8_t*)msg,strlen(msg));

    }

	return 0;
}
