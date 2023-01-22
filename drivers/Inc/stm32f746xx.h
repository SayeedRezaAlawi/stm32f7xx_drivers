/*
 * stm32f746xx.h
 *
 *  Created on: Jan 13, 2023
 *      Author: Reza
 */

#ifndef INC_STM32F746XX_H_
#define INC_STM32F746XX_H_

#include "stdint-gcc.h"

#define __vo  volatile

/******************************************START PROCESSOR SPECIFIC DETAILS *******************************/
/*
 * ARM Cortex Mx Processor NVIC ISERs register address
 */
#define	NVIC_ISER0			( (__vo uint32_t)0xE000E100 )
#define	NVIC_ISER1			( (__vo uint32_t)0xE000E104 )
#define	NVIC_ISER2			( (__vo uint32_t)0xE000E108 )
#define	NVIC_ISER3			( (__vo uint32_t)0xE000E10C )

/*
 * ARM Cortex Mx Processor NVIC ICERs register address
 */
#define	NVIC_ICER0			( (__vo uint32_t)0XE000E180 )
#define	NVIC_ICER1			( (__vo uint32_t)0XE000E184 )
#define	NVIC_ICER2			( (__vo uint32_t)0XE000E188 )
#define	NVIC_ICER3			( (__vo uint32_t)0XE000E18C )

/*
 * ARM Cortex Mx Processor NVIC PR register address
 */
#define	NVIC_PR_BASEADDR	( (__vo uint32_t)0xE000E400 )

#define	NO_PR_BITS_IMPLEMENTED		4

/*
 * base addresses of flash and SRAM memories
 */

#define HAL_FLASH_BASEADDR					0x08000000U			/*Base of flash memory*/
#define HAL_SRAM1_BASEADDR					0X20010000U			/*Base of SRAM1 memory*/
#define HAL_SRAM2_BASEADDR					0x2004C000U			/*Base of SRAM2 memory*/
#define HAL_ROM_BASEADDR					0x1FF00000U			/*Base of ROM memory*/
#define HAL_SRAM 							HAL_SRAM1_BASEADDR		/*SRAM base memory address*/


/*
 * AHBx and APBx bus peripheral base addresses
 */

#define HAL_PERIPH_BASEADDR					0x40000000U
#define	HAL_APB1PERIPH_BASEADDR				HAL_PERIPH_BASEADDR
#define HAL_APB2PERIPH_BASEADDR				0x40010000U
#define HAL_AHB1PERIPH_BASEADDR				0x40020000U
#define HAL_AHB2PERIPH_BASEADDR				0x50000000U
#define HAL_AHB3PERIPH_BASEADDR				0xA0000000U

/*
 * Base addresses of peripherals which are hanging on AHB1 BUS
 */

#define HAL_GPIOA_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X0000)
#define HAL_GPIOB_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X0400)
#define HAL_GPIOC_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X0800)
#define HAL_GPIOD_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X0C00)
#define HAL_GPIOE_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X1000)
#define HAL_GPIOF_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X1400)
#define HAL_GPIOG_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X1800)
#define HAL_GPIOH_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X1C00)
#define HAL_GPIOI_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X2000)
#define HAL_GPIOJ_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X2400)
#define HAL_GPIOK_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X2800)

#define HAL_RCC_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X3800)

#define HAL_CRC_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X3000)

#define HAL_FLAHSINTF_BASEADDR				(HAL_AHB1PERIPH_BASEADDR + 0X3C00)

#define HAL_DMA1_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X6000)
#define HAL_DMA2_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X6400)

#define HAL_ETHMAC_BASEADDR					(HAL_AHB1PERIPH_BASEADDR + 0X8000)


/*
 * Base addresses of peripherals which are hanging on AHB2 BUS
 */

#define HAL_DCMI_BASEADDR					0x50050000U

#define HAL_CRYP_BASEADDR					0x50060000U

#define HAL_HASH_BASEADDR					0x50060400U

#define HAL_RNG_BASEADDR					0x50060800U


/*
 * Base addresses of peripherals which are hanging on APB1 BUS
 */

#define	HAL_I2C1_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X5400)
#define	HAL_I2C2_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X5800)
#define	HAL_I2C3_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X5C00)
#define	HAL_I2C4_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X6000)

#define	HAL_CAN1_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X6400)
#define	HAL_CAN2_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X6800)

#define	HAL_TIM2_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X0000)
#define	HAL_TIM3_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X0400)
#define	HAL_TIM4_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X0800)
#define	HAL_TIM5_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X0C00)
#define	HAL_TIM6_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X1000)
#define	HAL_TIM7_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X1400)
#define	HAL_TIM12_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X1800)
#define	HAL_TIM13_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X1C00)
#define	HAL_TIM14_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X2000)
#define	HAL_LPTIM1_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X2400)

#define	HAL_SPI2_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X3800)
#define	HAL_SPI3_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X3C00)

#define	HAL_USART2_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X4400)
#define	HAL_USART3_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X4800)
#define	HAL_UART4_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X4C00)
#define	HAL_UART5_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X5000)
#define	HAL_UART7_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X7800)
#define	HAL_UART8_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X7C00)

#define	HAL_DAC_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X7400)

#define	HAL_PWR_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X7000)

#define	HAL_HDMICEC_BASEADDR				(HAL_APB1PERIPH_BASEADDR + 0X6C00)

#define	HAL_IWDG_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X3000)
#define	HAL_DWDG_BASEADDR					(HAL_APB1PERIPH_BASEADDR + 0X42C00)

/*
 * Base addresses of peripherals which hanging on APB2 BUS
 */

#define	HAL_EXTI_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X3C00)

#define	HAL_TIM1_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X0000)
#define	HAL_TIM8_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X0400)
#define	HAL_TIM9_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X4000)
#define	HAL_TIM10_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X4400)
#define	HAL_TIM11_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X4800)

#define	HAL_USART1_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X1000)
#define	HAL_USART6_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X1400)

#define	HAL_ADC_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X2000)

#define	HAL_SPI1_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X3000)
#define	HAL_SPI4_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X3400)
#define	HAL_SPI5_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X5000)
#define	HAL_SPI6_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X5400)

#define	HAL_SYSCFG_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X3800)

#define	HAL_SDMMC1_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X2C00)

#define	HAL_SAI1_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X5800)
#define	HAL_SAI2_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X5C00)
#define	HAL_LCDTFT_BASEADDR					(HAL_APB2PERIPH_BASEADDR + 0X6800)

/****************************peripheral register definition structures ************/
/*
 * Note: Registers of peripheral are specific to MCU
 */

typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register								address offset 0x00*/
	__vo uint32_t OTYPER;		/*GPIO port output type register						address offset 0x04*/
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register						address offset 0x08*/
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register					address offset 0x0C*/
	__vo uint32_t IDR;			/*GPIO port input data register					    	address offset 0x10*/
	__vo uint32_t ODR;			/*GPIO port output data register						address offset 0x14*/
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register						address offset 0x18*/
	__vo uint32_t LCKR;			/*GPIO port configuration clock register			 		address offset 0x1C*/
	__vo uint32_t AFR[2];		/*GPIO alternate function low and high registers		address offset 0x20, 0x24*/
}HAL_GPIO_RegDef_t;

/****************************peripheral register definition structures ************/
/*
 * Note: Registers of USART peripheral are specific to MCU
 */

typedef struct
{
	__vo uint32_t CR1;			/*GPIO port mode register								address offset 0x00*/
	__vo uint32_t CR2;			/*GPIO port output type register						address offset 0x04*/
	__vo uint32_t CR3;			/*GPIO port output speed register						address offset 0x08*/
	__vo uint32_t BRR;			/*GPIO port pull-up/pull-down register					address offset 0x0C*/
	__vo uint32_t GTPR;			/*GPIO port input data register					    	address offset 0x10*/
	__vo uint32_t RTOR;			/*GPIO port output data register						address offset 0x14*/
	__vo uint32_t RQR;			/*GPIO port bit set/reset register						address offset 0x18*/
	__vo uint32_t ISR;			/*GPIO port configuration clock register			 	address offset 0x1C*/
	__vo uint32_t ICR;			/*GPIO alternate function low and high registers		address offset 0x20*/
	__vo uint32_t RDR;			/*GPIO alternate function low and high registers		address offset 0x24*/
	__vo uint32_t TDR;			/*GPIO alternate function low and high registers		address offset 0x28*/
}HAL_USART_RegDef_t;


/****************************peripheral register definition structures ************/
/*
 * Note: Registers of peripheral are specific to MCU
 */

typedef struct
{
	__vo uint32_t CR;		    /*RCC clock control register									address offset 0x00*/
	__vo uint32_t PLLCFGR;		/*RCC PLL configuration register								address offset 0x04*/
	__vo uint32_t CFGR;			/*RCC clock configuration register								address offset 0x08*/
	__vo uint32_t CIR;			/*RCC clock interrupt register									address offset 0x0C*/
	__vo uint32_t AHB1RSTR;		/*RCC AHB1 peripheral reset register							address offset 0x10*/
	__vo uint32_t AHB2RSTR;		/*RCC AHB2 peripheral reset register							address offset 0x14*/
	__vo uint32_t AHB3RSTR;		/*RCC AHB3 peripheral reset register							address offset 0x18*/
	__vo uint32_t Reserved1;	/*Reserved														address offset 0x1C*/
	__vo uint32_t APB1RSTR;		/*RCC APB1 peripheral reset register							address offset 0x20*/
	__vo uint32_t APB2RSTR;		/*RCC APB2 peripheral reset register							address offset 0x24*/
	__vo uint32_t Reserved2;	/*Reserved														address offset 0x28*/
	__vo uint32_t Reserved3;	/*Reserved														address offset 0x2C*/
	__vo uint32_t AHB1ENR;		/*RCC AHB1 peripheral clock enable register						address offset 0x30*/
	__vo uint32_t AHB2ENR;		/*RCC AHB2 peripheral clock enable register						address offset 0x34*/
	__vo uint32_t AHB3ENR;		/*RCC AHB3 peripheral clock enable register						address offset 0x38*/
	__vo uint32_t Reserved4;	/*Reserved														address offset 0x3C*/
	__vo uint32_t APB1ENR;		/*RCC APB1 peripheral clock enable register						address offset 0x40*/
	__vo uint32_t APB2ENR;		/*RCC APB2 peripheral clock enable register						address offset 0x44*/
	__vo uint32_t Reserved5;	/*Reserved														address offset 0x48*/
	__vo uint32_t Reserved6;	/*Reserved														address offset 0x4C*/
	__vo uint32_t AHB1LPENR;	/*RCC AHB1 peripheral clock enable in low power mode register 	address offset 0x50*/
	__vo uint32_t AHB2LPENR;	/*RCC AHB2 peripheral clock enable in low power mode register	address offset 0x54*/
	__vo uint32_t AHB3LPENR;	/*RCC AHB3 peripheral clock enable in low power mode register	address offset 0x58*/
	__vo uint32_t Reserved7;	/*Reserved														address offset 0x5C*/
	__vo uint32_t APB1LPENR;	/*RCC APB1 peripheral clock enable in low power mode register	address offset 0x60*/
	__vo uint32_t APB2LPENR;	/*RCC APB2 peripheral clock enable in low power mode register	address offset 0x64*/
	__vo uint32_t Reserved8;	/*Reserved														address offset 0x68*/
	__vo uint32_t Reserved9;	/*Reserved														address offset 0x6C*/
	__vo uint32_t BDCR;			/*RCC Backup domain control register							address offset 0x70*/
	__vo uint32_t CSR;			/*RCC clock control & status register							address offset 0x74*/
	__vo uint32_t Reserved10;	/*Reserved														address offset 0x78*/
	__vo uint32_t Reserved11;	/*Reserved														address offset 0x7C*/
	__vo uint32_t SSCGR;		/*RCC spread spectrum clock generation register					address offset 0x80*/
	__vo uint32_t PLLI2SCFGR;	/*RCC PLLI2S configuration register								address offset 0x84*/
}HAL_RCC_RegDef_t;

/*
 * Note: Peripheral register definition structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}HAL_EXTI_RegDef_t;

/*
 * Note: Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
}HAL_SYSCFG_RegDef_t;


/*
 * peripheral definition (Peripheral base addresses typcasted to xxx_RegDef_t)
 */

#define GPIOA 							((HAL_GPIO_RegDef_t*)HAL_GPIOA_BASEADDR)
#define GPIOB 							((HAL_GPIO_RegDef_t*)HAL_GPIOB_BASEADDR)
#define GPIOC 							((HAL_GPIO_RegDef_t*)HAL_GPIOC_BASEADDR)
#define GPIOD 							((HAL_GPIO_RegDef_t*)HAL_GPIOD_BASEADDR)
#define GPIOE 							((HAL_GPIO_RegDef_t*)HAL_GPIOE_BASEADDR)
#define GPIOF 							((HAL_GPIO_RegDef_t*)HAL_GPIOF_BASEADDR)
#define GPIOG 							((HAL_GPIO_RegDef_t*)HAL_GPIOG_BASEADDR)
#define GPIOH 							((HAL_GPIO_RegDef_t*)HAL_GPIOH_BASEADDR)
#define GPIOI 							((HAL_GPIO_RegDef_t*)HAL_GPIOI_BASEADDR)
#define GPIOJ 							((HAL_GPIO_RegDef_t*)HAL_GPIOJ_BASEADDR)
#define GPIOK 							((HAL_GPIO_RegDef_t*)HAL_GPIOK_BASEADDR)

#define RCC								((HAL_RCC_RegDef_t*)HAL_RCC_BASEADDR)

#define EXTI							((HAL_EXTI_RegDef_t*)HAL_EXTI_BASEADDR)

#define SYSCFG							((HAL_SYSCFG_RegDef_t*)HAL_SYSCFG_BASEADDR)


#define USART1 							((HAL_USART_RegDef_t*)HAL_USART1_BASEADDR)
#define USART2 							((HAL_USART_RegDef_t*)HAL_USART2_BASEADDR)
#define USART3 							((HAL_USART_RegDef_t*)HAL_USART3_BASEADDR)
#define UART4 							((HAL_USART_RegDef_t*)HAL_UART4_BASEADDR)
#define UART5 							((HAL_USART_RegDef_t*)HAL_UART5_BASEADDR)
#define USART6 							((HAL_USART_RegDef_t*)HAL_USART6_BASEADDR)
#define UART7 							((HAL_USART_RegDef_t*)HAL_UART7_BASEADDR)
#define UART8 							((HAL_USART_RegDef_t*)HAL_UART8_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()		(RCC->AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()		(RCC->AHB1ENR |= (1 << 10))

/*
 * Clock Enable Macros for DMAx peripherals
 */

#define DMA1_PCLK_EN()		(RCC->AHB1ENR |= (1 << 21))
#define DMA2_PCLK_EN()		(RCC->AHB1ENR |= (1 << 22))


/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))
#define I2C4_PCLK_EN()		(RCC->APB1ENR |= (1 << 24))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()		(RCC->APB2ENR |= (1 << 21))

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()			(RCC->APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()			(RCC->APB1ENR |= (1 << 31))


/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 10))

/*
 * Clock Enable Macros for DMAx peripherals
 */

#define DMA1_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 21))
#define DMA2_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 22))

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))
#define I2C4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 24))

/*
 * Clock Disable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Clock Disable Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 21))

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 31))

/*
 * returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0:\
									 (x == GPIOB) ? 1:\
									 (x == GPIOC) ? 2:\
									 (x == GPIOD) ? 3:\
									 (x == GPIOE) ? 4:\
									 (x == GPIOF) ? 5:\
									 (x == GPIOH) ? 6:\
									 (x == GPIOG) ? 7:\
									 (x == GPIOI) ? 8:\
									 (x == GPIOJ) ? 9:\
									 (x == GPIOK) ? 10: 0 )

/*
 * IRQ (Interrupt Request) 	Numbers of STM32F746x MCU
 *
 */
#define	IRQ_NO_EXTI0				6
#define	IRQ_NO_EXTI1				7
#define	IRQ_NO_EXTI2				8
#define	IRQ_NO_EXTI3				9
#define	IRQ_NO_EXTI4				10
#define	IRQ_NO_EXTI9_5				23
#define	IRQ_NO_EXTI15_10			40

/*
 * macros for all possible priority levels
 *
 */
#define NVIC_IRQ_PRIO0				0
#define NVIC_IRQ_PRIO1				1
#define NVIC_IRQ_PRIO2				2
#define NVIC_IRQ_PRIO3				3
#define NVIC_IRQ_PRIO4				4
#define NVIC_IRQ_PRIO5				5
#define NVIC_IRQ_PRIO6				6
#define NVIC_IRQ_PRIO7				7
#define NVIC_IRQ_PRIO8				8
#define NVIC_IRQ_PRIO9				9
#define NVIC_IRQ_PRIO10				10
#define NVIC_IRQ_PRIO11				11
#define NVIC_IRQ_PRIO12				12
#define NVIC_IRQ_PRIO13				13
#define NVIC_IRQ_PRIO14				14
#define NVIC_IRQ_PRIO15				15

//some genric macros

#define ENABLE 				1
#define DISABLE				0
#define SET 				ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


/*
 * Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0));\
									(RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 1));\
									(RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 2));\
									(RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 3));\
									(RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 4));\
									(RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 5));\
									(RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 6));\
									(RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOG_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 7));\
									(RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 8));\
									(RCC->AHB1RSTR &= ~(1 << 8));}while(0)
#define GPIOJ_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 9));\
									(RCC->AHB1RSTR &= ~(1 << 9));}while(0)
#define GPIOK_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 10));\
									(RCC->AHB1RSTR &= ~(1 << 10));}while(0)
/*
 * Macros to reset DMAx peripherals
 */

#define DMA1_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 21));\
								   (RCC->AHB1RSTR &= ~(1 << 21));}while(0)
#define DMA2_REG_RESET()		do{(RCC->AHB1RSTR |= (1 << 22));\
								   (RCC->AHB1RSTR &= ~(1 << 22));}while(0)
/*
 * Macros to reset I2Cx peripherals
 */

#define I2C1_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 21));\
								   ((RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 22));\
								   ((RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 23));\
								   ((RCC->APB1RSTR &= ~(1 << 23));}while(0)
#define I2C4_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 24));\
								   ((RCC->APB1RSTR &= ~(1 << 24));}while(0)
/*
 * Macros to reset SYSCFG peripheral
 */

#define SYSCFG_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 14));\
								   (RCC->APB2RSTR &= ~(1 << 14));}while(0)
/*
 * Macros to reset SPIx peripherals
 */

#define SPI1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 12));\
								   (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 14));\
								   (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 15));\
								   (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 13));\
								   (RCC->APB2RSTR &= ~(1 << 13));}while(0)
#define SPI5_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 20));\
								   (RCC->APB2RSTR &= ~(1 << 20));}while(0)
#define SPI6_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 21));\
								   (RCC->APB2RSTR &= ~(1 << 21));}while(0)
/*
 * Macros to reset UARTx peripherals
 */

#define USART1_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 4));\
								   (RCC->APB2RSTR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 17));\
								   (RCC->APB1RSTR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 18));\
								   (RCC->APB1RSTR &= ~(1 << 18));}while(0)
#define UART4_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 19));\
								   (RCC->APB1RSTR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 20));\
								   (RCC->APB1RSTR &= ~(1 << 20));}while(0)
#define USART6_REG_RESET()		do{(RCC->APB2RSTR |= (1 << 5));\
								   (RCC->APB2RSTR &= ~(1 << 5));}while(0)
#define UART7_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 30));\
								   (RCC->APB1RSTR &= ~(1 << 30));}while(0)
#define UART8_REG_RESET()		do{(RCC->APB1RSTR |= (1 << 31));\
								   (RCC->APB1RSTR &= ~(1 << 31));}while(0)


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_UE					0
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M0 					12
#define USART_CR1_MME 					13
#define USART_CR1_CMIE 					14
#define USART_CR1_OVER8  				15
#define USART_CR1_M1	  				28



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADDM7   				4
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_CLKEN   				11
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14
#define USART_CR2_SWAP   				15
#define USART_CR2_RXINV  				16
#define USART_CR2_TXINV   				17
#define USART_CR2_DATAINV  				18
#define USART_CR2_MSBFIRST 				19
#define USART_CR2_ABREN   				20
#define USART_CR2_ABRMOD  				21
#define USART_CR2_RTOEN  				23


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11
#define USART_CR3_OVRDIS   				12
#define USART_CR3_DDRE  				13
#define USART_CR3_DEM   				14
#define USART_CR3_DEP   				15
#define USART_CR3_SCARCT0  				17
#define USART_CR3_SCARCNT1  			18
#define USART_CR3_SCARCNT2 				19
/*
 * Bit position definitions USART_SR
 */
/*
 * Bit position definitions USART_ISR
 */

#define USART_ISR_PE        				0
#define USART_ISR_FE        				1
#define USART_ISR_NF        				2
#define USART_ISR_ORE       				3
#define USART_ISR_IDLE       				4
#define USART_ISR_RXNE        				5
#define USART_ISR_TC        				6
#define USART_ISR_TXE        				7
#define USART_ISR_LBDF       				8
#define USART_ISR_CTSIF        				9
#define USART_ISR_CTS						10
#define USART_ISR_RTOF        				11
#define USART_ISR_EOBF       				12
#define USART_ISR_ABRE        				14
#define USART_ISR_ABRF        				15
#define USART_ISR_BUSY       				16
#define USART_ISR_CMF        				17
#define USART_ISR_SBKF						18
#define USART_ISR_RWU        				19
#define USART_ISR_TEACK       				21

#include "stm32f746xx_gpio_driver.h"
#include "stm32f746xx_rcc_driver.h"
#include "stm32f746xx_usart_driver.h"


#endif /* INC_STM32F407XX_H_ */
