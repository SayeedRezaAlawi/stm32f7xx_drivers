/*
 * stm32f746xx_usart_driver.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Reza
 */

#include "stm32f746xx_usart_driver.h"


/*
 * USART_SetBaudRate
 */
/*****************************************************
 * @fn					- USART_DeInit
 *
 * @brief				- This function is application callback to handle even on USARTx peripheral
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- Event flag
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_SetBaudRate(HAL_USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *BaudRate));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 *BaudRate));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR = tempreg;
}



/*
 * Peripheral Clock setup
 */
/*****************************************************
 * @fn					- USART_Peri_ClockControl
 *
 * @brief				- This function clock for USARTx peripheral
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- Enable/Disable macros
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_Peri_ClockControl(HAL_USART_RegDef_t * pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}else if(pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}else if(pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}else if(pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		}else if(pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}
	}
}

/*
 * Init
 */
/*****************************************************
 * @fn					- USART_Init
 *
 * @brief				- This function initialized the USARTx peripheral
 *
 * @param[in]			- USART peripheral handle structure
 * @param[in]			-
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_Init(USART_Handle_t* pUSARTHandle)
{
	uint32_t tempreg = 0;
/*************************************Configuration of CR1 register*********************************/

	//Implement the code to enable the Clock for given USART peripheral
	USART_Peri_ClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//implement the code to enable transmitter mode
		tempreg |= (1 << USART_CR1_TE);

	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//implement the code to enable receiver mode
		tempreg |= (1 << USART_CR1_RE);

	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//implement the code to enable both receiver and transmitter mode
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

	//implement the code to configure the wordlength
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_8BITS)
	{
		//configure 00 for bit M0 and M1 for 8bits wordlength
//		tempreg &= ((~(1 << 12)) | (~(1 << 28)));
		tempreg &= ~(1 << USART_CR1_M0);
		tempreg &= ~(1 << USART_CR1_M1);

	}else if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
	{
		//configure 01 for bit M0=1 and M1=0 for 8bits wordlength
		tempreg |= (1 << USART_CR1_M0);
		tempreg &= ~(1 << USART_CR1_M1);

	}

	//configuration of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//configure the parity control enable bit field
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//configure the parity control enable bit field
		tempreg |= (1 << USART_CR1_PCE);

		//configure the Parity selection bit field, 0 even parity
		tempreg |= (1 << USART_CR1_PS);

	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/*************************************Configuration of CR2 register*********************************/

	tempreg = 0;

	//implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/*************************************Configuration of CR3 register*********************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);

	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);

	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);

	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}


/*
 * De-init
 */
/*****************************************************
 * @fn					- USART_DeInit
 *
 * @brief				- This function deinitialized the USARTx peripheral
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			-
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_DeInit(HAL_USART_RegDef_t* pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}else if(pUSARTx == UART7)
	{
		UART7_REG_RESET();
	}else if(pUSARTx == UART8)
	{
		UART8_REG_RESET();
	}
}


/*
 * Data Send
 */
/*****************************************************
 * @fn					- USART_SendData
 *
 * @brief				- This function sends the data over USARTx
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- transmit buffer which holds data
 * @param[in]			- length the data holds in tx buffer
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len)
{
	uint16_t *pData;
	//loop over until "len" number of bytes are transfered
	for(uint32_t i=0; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)));

		//check the word length for 9bit or 8bit in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9bit load the DR with 2byte masking the bits other 9 bits
			pData = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->TDR = (*pData & (uint16_t)0x01FF);

			//check if parity is enabled
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transmission, so 9bits of user data will be sent
				//implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//the parity is used in this transfer. So only 8 bits of user data will be sent
				//the 9th bit will be replaced with hardware parity bit
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bits transfer
			pUSARTHandle->pUSARTx->TDR = (*pTxBuffer & (uint8_t)0xFF);

			//implement the code to increment the pTxBuffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

}

/*
 * Receive Data
 */
/*****************************************************
 * @fn					- USART_ReceiveData, blocking
 *
 * @brief				- This function receives the data over USARTx
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- pointer to receive buffer which holds data
 * @param[in]			- length the data holds in rx buffer
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	//loop over until Len number of bytes are transfered
	for(uint32_t i=0; i < Len; i++)
	{
		//Wait till data are ready to be read by flag RXNE
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check if the data received is 9bits or 8bits frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//check if parity is enabled
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask DR with 0x01FF
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->RDR & (uint16_t)0x01FF);

				//Increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//read only 8bits so mask DR with 0xFF
				*pRxBuffer = (pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check if the parity control is enabled
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read only first 8 bits so mask DR with 0xFF
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & (uint8_t)0xFF);
			}
			else
			{
				//read only 7bits so mask DR with 0x7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->RDR & (uint8_t)0x7F);
			}

			//Increment the pRxBuffer
			pRxBuffer++;
		}

	}

}

/*
 * Send Data
 */
/*****************************************************
 * @fn					- USART_SendDataIT, none-blocking
 *
 * @brief				- This function sends the data over USARTx
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- pointer to transmit buffer which holds data
 * @param[in]			- length the data holds in rx buffer
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint8_t Len)
{

	return 0;
}

/*
 * Receive Data
 */
/*****************************************************
 * @fn					- USART_ReceiveData, none-blocking
 *
 * @brief				- This function receives the data over USARTx
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- pointer to receive buffer which holds data
 * @param[in]			- length the data holds in rx buffer
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	return 0;
}


/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************
 * @fn					- USART_IRQInterruptConfig
 *
 * @brief				- This function configures the Interrupt request number on the peripheral side
 *
 * @param[in]			- Interrupt request number
 * @param[in]			- Enable/Disable macros
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*
 * Configure the priority for Interrupt request number
 */
/*****************************************************
 * @fn					- USART_IRQPriorityConfig
 *
 * @brief				- This function configures the priority for the interrupt request number at CPU side
 *
 * @param[in]			- Interrupt request number
 * @param[in]			- Interrupt priority
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/*
 * Handle the USARTx interrupt
 */
/*****************************************************
 * @fn					- USART_IRQHandling
 *
 * @brief				- This function handle the interrupt generated on USARTx
 *
 * @param[in]			- USARTx Handle structure
 * @param[in]			-
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{

}

/*
 * Other Peripheral Control APIs
 */
/*****************************************************
 * @fn					- USART_PeripheralControl
 *
 * @brief				- This function controls the USARTx peripheral
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- Enable/Disable macros
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_PeripheralControl(HAL_USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*
 * Get USART status flag
 */
/*****************************************************
 * @fn					- USART_GetFlagStatus
 *
 * @brief				- This function gets the USARTx peripheral status flag
 * @param[in]			- base address of USART peripheral
 * @param[in]			- flag name
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
uint8_t USART_GetFlagStatus(HAL_USART_RegDef_t *pUSARTx, uint8_t StatusFlagName)
{
	if(pUSARTx->ISR & StatusFlagName)
	{
		return SET;
	}

	return RESET;
}

/*
 * Clear USART flag
 */
/*****************************************************
 * @fn					- USART_ClearFlag
 *
 * @brief				- This function clears the USARTx peripheral flag
 * @param[in]			- base address of USART peripheral
 * @param[in]			- status flag name
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_ClearFlag(HAL_USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
	pUSARTx->ISR &= ~(StatusFlagName);
}


/*
 * Application callback
 */
/*****************************************************
 * @fn					- USART_DeInit
 *
 * @brief				- This function is application callback to handle even on USARTx peripheral
 *
 * @param[in]			- base address of USART peripheral
 * @param[in]			- Event flag
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/
void USART_ApplicationEventCallback(HAL_USART_RegDef_t *pUSARTx, uint8_t AppEv)
{

}
