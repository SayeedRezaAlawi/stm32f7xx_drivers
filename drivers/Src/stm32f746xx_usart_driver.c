/*
 * stm32f746xx_usart_driver.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Reza
 */

#include "stm32f746xx_usart_driver.h"


/*
 * Peripheral Clock setup
 */
void USART_Peri_ClockControl(HAL_USART_RegDef_t * pUSARTx, uint8_t EnorDi)
{

}

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t* pUSARTHandle)
{

}
void USART_DeInit(HAL_USART_RegDef_t* pUSARTx)
{

}


/*
 * Data Send and Receive
 */
void USART_SendData(HAL_USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint8_t Len)
{

}
void USART_ReceiveData(HAL_USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint8_t Len)
{

}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len)
{

	return 0;
}
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	return 0;
}


/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

}

/*
 * Other Peripheral Control APIs
 */
void USART_PeripheralControl(HAL_USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{

}
uint8_t USART_GetFlagStatus(HAL_USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	return 0;
}
void USART_ClearFlag(HAL_USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{

}


/*
 * Application callback
 */
void USART_ApplicationEventCallback(HAL_USART_RegDef_t *pUSARTx, uint8_t AppEv)
{

}
