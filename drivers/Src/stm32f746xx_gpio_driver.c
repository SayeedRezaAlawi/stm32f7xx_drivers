/*
 * stm32f746xx_gpio.c
 *
 *  Created on: Jan 13, 2023
 *      Author: Reza
 */

#include <stm32f746xx.h>
#include <stm32f746xx_gpio_driver.h>


/*
 * Peripheral clock setup
 */
/*****************************************************
 * @fn					- GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for givin GPIO port
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			- Enable or Diable macros
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_PeriClockControl(HAL_GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_EN();
		}else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}else if(pGPIOx == GPIOJ)
		{
			GPIOJ_PCLK_DI();
		}else if(pGPIOx == GPIOK)
		{
			GPIOK_PCLK_DI();
		}
	}
}

/*
 * Peripheral init
 */
/*****************************************************
 * @fn					- GPIO_Init
 *
 * @brief				- This function initialize the GPIO pin
 *
 * @param[in]			- GPIO Handle
 * @param[in]			-
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
//	uint32_t temp = 0;
	int32_t temp;
	//1. Configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the none interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		//the interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_FT)
		{
			//1. Configure FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RT)
		{
			//1. cONFIGURE RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IT_RFT)
		{
			//1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] = (portCode << (temp2 * 4));

		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2. Configure the speed
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. Configure pupd setting
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	//4. configure optype
	temp = 0;
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 = 0;
		uint8_t temp2 = 0;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4* temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4* temp2);
	}


}

/*
 * Peripheral deinit
 */
/*****************************************************
 * @fn					- GPIO_DeInit
 *
 * @brief				- This function deinitialize the GPIO port
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			-
 * @param[in]			-
 *
 * return				- None
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_DeInit(HAL_GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}else if(pGPIOx == GPIOJ)
			{
				GPIOJ_REG_RESET();
			}
}

/*
 * Peripheral data read and write pin
 */
/*****************************************************
 * @fn					- GPIO_ReadFromInputPin
 *
 * @brief				- This function read from GPIO pin
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			- Pin number
 * @param[in]			-
 *
 * return				- value of the pin number from GPIO register
 *
 * @Note				- None
 *
 ****************************************************/

uint8_t GPIO_ReadFromInputPin(HAL_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value =0;
	value = (uint8_t)(pGPIOx->IDR >> PinNumber) & (0x00000001);
	return value;

}

/*
 * Peripheral data read and write port
 */
/*****************************************************
 * @fn					- GPIO_ReadFromInputPort
 *
 * @brief				- This function read the whole GPIO port
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			-
 * @param[in]			-
 *
 * return				- value of output GPIO register
 *
 * @Note				- None
 *
 ****************************************************/

uint16_t GPIO_ReadFromInputPort(HAL_GPIO_RegDef_t *pGPIOx)
{
	uint16_t value =0;
	value = (uint16_t)(pGPIOx->IDR);
		return value;
}

/*
 * Peripheral data write pin
 */
/*****************************************************
 * @fn					- GPIO_WriteToOutputPin
 *
 * @brief				- This function write value to GPIO pin
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			- pin number
 * @param[in]			- value
 *
 * return				- none
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_WriteToOutputPin(HAL_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else if(Value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}


}

/*
 * Peripheral data write port
 */
/*****************************************************
 * @fn					- GPIO_WriteToOutputPort
 *
 * @brief				- This function write value to whole GPIO register
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			- value
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_WriteToOutputPort(HAL_GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR |= Value;
}

/*
 * Peripheral pin value toggle
 */
/*****************************************************
 * @fn					- GPIO_ToggleOutputPin
 *
 * @brief				- This function toggles value to GPIO Pin register
 *
 * @param[in]			- base address of GPIO port
 * @param[in]			- pin number
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_ToggleOutputPin(HAL_GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration
 */
/*****************************************************
 * @fn					- GPIO_IRQITConfig
 *
 * @brief				- This function configure the GPIO interrupt
 *
 * @param[in]			- Interrupt request number
 * @param[in]			- Enable or Diable macros
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 31)
		{
			//program ISER0
			*((uint32_t*)NVIC_ISER0) |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ISER1
			*((uint32_t*)NVIC_ISER1) |= (1 << ( IRQNumber % 32 ));



		}else if(IRQNumber > 64 && IRQNumber < 96 )
		{
			//program ISER2
			*((uint32_t*)NVIC_ISER2) |= (1 << ( IRQNumber % 64 ));
		}
	}
	else
	{
		if(IRQNumber < 31)
		{
			//program ICER0
			*((uint32_t*)NVIC_ICER0) |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1
			*((uint32_t*)NVIC_ICER1) |= (1 << ( IRQNumber % 32 ));

		}else if(IRQNumber > 64 && IRQNumber < 96 )
		{
			//program ICER2
			*((uint32_t*)NVIC_ICER2) |= (1 << ( IRQNumber % 64 ));
		}
	}
}


/*
 * IRQ priority Configuration
 */
/*****************************************************
 * @fn					- GPIO_IRQPriorityConfig
 *
 * @brief				- This function configure the IRQ prriority
 *
 * @param[in]			- Interrupt request priority
 * @param[in]			-
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- None
 *
 ****************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//1. first lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//2. program
	uint32_t *pPR;
	pPR = (uint32_t*)NVIC_PR_BASEADDR;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(pPR + (iprx)) |= (IRQPriority << (shift_amount));


}
/*
 * ISR handling
 */
/*****************************************************
 * @fn					- GPIO_IRQHandling
 *
 * @brief				- This function handle the IRQ on the GPIO specific pin
 *
 * @param[in]			- Pin number
 * @param[in]			-
 * @param[in]			-
 *
 * return				- none
 *
 * @Note				- None
 *
 ****************************************************/

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register correponding tho the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}

}
