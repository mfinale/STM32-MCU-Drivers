/*
 * stm32f446E.h
 *
 *  Created on: Apr 17, 2020
 *      Author: Michael R. Finale
 *      This file contains MCU specific data.
 *      These addresses were defined using STM32F446xx reference manual.
 *      This code is not guaranteed to be compatible with other MCU's
 */

#ifndef INC_STM32F446E_H_
#define INC_STM32F446E_H_

/*includes*/

#include <stdint.h>
#include <stddef.h>


/*********************************************ARM CORTEX M PROCESSOR DATA*******************************************/
/**arm cortex m4 NVIC register addresses*/
#define NVIC_ISER0			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3			((volatile uint32_t*)0xE000E10C)
#define NVIC_ICER0			((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2			((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0xE000E18C)

/*NVIC priority register*/
#define NVIC_PR_BASE_ADDR	((volatile uint32_t*)0xE000E400)

#define NO_OF_PRIO_BITS_IMPLEMENTED						4


/***********************************MCU PERIPHERAL DATA********************************************/

/* base addresses of memory from MCU ref manual (in unsigned integers) */
#define FLASH_BASEADDR			0x08000000U	/* base address of flash memory */
#define SRAM1_BASEADDR			0x20000000U /* base address of SRAM  */
#define SRAM2_BASEADDR			0x2001C000U /* base address of SRAM2 112kb from SRAM 1 */
#define ROM						0x1FFF0000U /* base address of ROM (system memory)*/
#define SRAM   					SRAM1_BASEADDR

/*base addresses of Peripheral Buses */
#define PERIPH_BASEADDR				0X40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x50000000U

/* base addresses of peripherals on AHB1 Bus */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0X1C00)
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)

/* base addresses of peripherals on APB1 Bus */
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0X5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0X5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0X5C00)
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0X3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0X3C00)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0X4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0X4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0X4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0X5000)


/* base addresses of peripherals on APB2 Bus */
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0X3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0X3400)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0X1400)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0X3C00)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0X3800)

/*register structure for GPIO peripherals. Every 32 bits (4 bytes) is in each parameter */

typedef struct
{
	volatile uint32_t MODER;		/*GPIO port mode register Address offset: 0x00 */
	volatile uint32_t OTYPER;		/*GPIO port output type register Address offset: 0x04 */
	volatile uint32_t OSPEEDR;		/*GPIO port output speed register Address offset: 0x08 */
	volatile uint32_t PUPDR;		/*GPIO port pull-up/pull-down register Address offset: 0x0C */
	volatile uint32_t IDR;			/*GPIO port input data register Address offset: 0x10 */
	volatile uint32_t ODR;			/*GPIO port output data register Address offset: 0x14 */
	volatile uint32_t BSRR;			/*GPIO port bit set/reset register Address offset: 0x18 */
	volatile uint32_t LCKR;			/*GPIO port configuration lock register Address offset: 0x1C */
	volatile uint32_t AFR[2];		/*GPIO alternate function low/high register	Address offset: 0x20 */
	//volatile uint32_t AFRH;			/*GPIO alternate function high register Address offset: 0x24 */

} GPIO_RegDef_t;

/*register structure for RCC peripheral. Every 32 bits (4 bytes) is in each parameter */

typedef struct
{
	volatile uint32_t CR;   		/*RCC Clock Control register Address offset: 0x00 */
	volatile uint32_t PLLCFGR;   	/*RCC PLL configuration register Address offset: 0x04 */
	volatile uint32_t CFGR; 		/*RCC clock configuration register Address offset: 0x08*/
	volatile uint32_t CIR;			/*RCC clock interrupt register Address offset: 0x0C*/
	volatile uint32_t AHB1RSTR;		/*RCC AHB1 peripheral reset register Address offset: 0x10*/
	volatile uint32_t AHB2RSTR;		/*RCC AHB2 peripheral reset register Address offset: 0x14 */
	volatile uint32_t AHB3RSTR;		/*RCC AHB3 peripheral reset register Address offset: 0x18 */
	volatile uint32_t RESERVE0;		/* no data at address offset  0x1C*/
	volatile uint32_t APB1RSTR;		/*RCC APB1 peripheral reset register Address offset: 0x20 */
	volatile uint32_t APB2RSTR;		/*RCC APB2 peripheral reset register Address offset: 0x24 */
	volatile uint32_t RESERVE1[2];		/* no data at address offset  0x28*/
	//volatile uint32_t RESERVE2;		/* no data at address offset  0x2C*/
	volatile uint32_t AHB1ENR;		/*RCC AHB1 peripheral clock enable register Address offset: 0x30 */
	volatile uint32_t AHB2ENR;		/*RCC AHB2 peripheral clock enable register Address offset: 0x34 */
	volatile uint32_t AHB3ENR;		/*RCC AHB3 peripheral clock enable register Address offset: 0x38 */
	volatile uint32_t RESERVE2;		/*RESERVED Address offset: 0x3C */
	volatile uint32_t APB1ENR;		/*RCC APB1 peripheral clock enable register Address offset: 0x40*/
	volatile uint32_t APB2ENR;		/*RCC APB2 peripheral clock enable register Address offset: 0x44*/
	volatile uint32_t RESERVED3[2];		/* no data at address offset  0x48*/
	//volatile uint32_t RESERVE5;		/* no data at address offset  0x4C*/
	volatile uint32_t AHB1LPENR;	/*RCC AHB1 peripheral clock enable in low power mode register Address offset: 0x50*/
	volatile uint32_t AHB2LPENR;	/*RCC AHB2 peripheral clock enable in low power mode register Address offset: 0x54*/
	volatile uint32_t AHB3LPENR;	/*RCC AHB3 peripheral clock enable in low power mode register Address offset: 0x58*/
	volatile uint32_t RESERVE4;		/* no data at address offset  0x5C*/
	volatile uint32_t APB1LPENR;	/*RCC APB1 peripheral clock enable in low power mode register Address offset: 0x60*/
	volatile uint32_t APB2LPENR;	/*RCC APB2 peripheral clock enable in low power mode register Address offset: 0x64*/
	volatile uint32_t RESERVE5[2];		/* no data at address offset  0x68*/
	//volatile uint32_t RESERVE8;		/* no data at address offset  0x6C*/
	volatile uint32_t BDCR;			/* RCC Backup domain control register	Address offset: 0x70*/
	volatile uint32_t CSR;			/* RCC clock control & status register	Address offset: 0x74*/
	volatile uint32_t RESERVE6[2];		/* no data at address offset  0x78*/
	//volatile uint32_t RESERVE10;	/* no data at address offset  0x7C*/
	volatile uint32_t SSCGR;		/* RCC clock control & status register	Address offset: 0x80*/
	volatile uint32_t PLLI2SCFGR;	/* RCC PLLI2S configuration register	Address offset: 0x84*/
	volatile uint32_t PLLSAICFGR;	/* RCC PLL configuration register	Address offset: 0x88*/
	volatile uint32_t DCKCFGR;		/* RCC Dedicated Clock Configuration Register	Address offset: 0x8C*/
	volatile uint32_t CKGATENR;		/* RCC clocks gated enable register	Address offset: 0x90*/
	volatile uint32_t DCKCFGR2;		/* RCC dedicated clocks configuration register 2	Address offset: 0x94*/


} RCC_RegDef_t;


/*register structure for EXTI peripheral. Every 32 bits (4 bytes) is in each parameter */
typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;


} EXTI_RegDef_t;


/*register structure for SYSCFG controller. Every 32 bits (4 bytes) is in each parameter */
typedef struct
{
	volatile uint32_t MEMRMP;				/*Address offset: 0x00 */
	volatile uint32_t PMC;					/*Address offset: 0x04 */
	volatile uint32_t EXTICR[4];				/*Address offset: 0x08-014 */
	volatile uint32_t RESERVED1[2];			/*Address offset: 0x18 -01XC */
	volatile uint32_t CMPCR;				/*Address offset: 0x20 */
	volatile uint32_t RESERVED2[2];			/*Address offset: 0x24-0X28 */
	volatile uint32_t CFGR;					/*Address offset: 0x2C */


} SYSCFG_RegDef_t;

/*register structure for SPI peripheral. Every 32 bits (4 bytes) is in each parameter */

typedef struct
{
	volatile uint32_t CR1;				/*Address offset: 0x00 */
	volatile uint32_t CR2;				/*Address offset: 0x04 */
	volatile uint32_t SR;				/*Address offset: 0x08*/
	volatile uint32_t DR;				/*Address offset: 0x0C */
	volatile uint32_t CRCPR;			/*Address offset: 0x10 */
	volatile uint32_t RXCRCR;			/*Address offset: 0x14 */
	volatile uint32_t TXCRCR;			/*Address offset: 0x18 */
	volatile uint32_t I2SCFGR;			/*Address offset: 0x1C */
	volatile uint32_t I2SPR;			/*Address offset: 0x20 */



} SPI_RegDef_t;

/*register structure for I2C peripheral. Every 32 bits (4 bytes) is in each parameter */

typedef struct
{
	volatile uint32_t CR1;				/*Address offset: 0x00 */
	volatile uint32_t CR2;				/*Address offset: 0x04 */
	volatile uint32_t OAR1;				/*Address offset: 0x08*/
	volatile uint32_t OAR2;				/*Address offset: 0x0C */
	volatile uint32_t DR;				/*Address offset: 0x10 */
	volatile uint32_t SR1;				/*Address offset: 0x14 */
	volatile uint32_t SR2;				/*Address offset: 0x18 */
	volatile uint32_t CCR;				/*Address offset: 0x1C */
	volatile uint32_t TRISE;			/*Address offset: 0x20 */
	volatile uint32_t FLTR;				/*Address offset: 0x24 */


} I2C_RegDef_t;





/* PERIPHERAL DEFINITIONS
 * The following lines of code defines macros for each peripheral base address
 * typecasted as a pointer with one of the structs defined above. "A pointer with a struct"
 * This allows easier coding in an application.
 * Ex: pGPIOA->MODER =25; same as *(0x4002000+0x00) =25;
*/

#define GPIOA	   ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	   ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	   ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	   ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	   ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF	   ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG	   ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH	   ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)


#define EXTI   		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)




/*Clock Enable Macros for GPIOx peripherals*/

#define GPIOA_PERICLK_EN()	(RCC->AHB1ENR |=(1<<0))
#define GPIOB_PERICLK_EN()	(RCC->AHB1ENR |=(1<<1))
#define GPIOC_PERICLK_EN()	(RCC->AHB1ENR |=(1<<2))
#define GPIOD_PERICLK_EN()	(RCC->AHB1ENR |=(1<<3))
#define GPIOE_PERICLK_EN()	(RCC->AHB1ENR |=(1<<4))
#define GPIOF_PERICLK_EN()	(RCC->AHB1ENR |=(1<<5))
#define GPIOG_PERICLK_EN()	(RCC->AHB1ENR |=(1<<6))
#define GPIOH_PERICLK_EN()	(RCC->AHB1ENR |=(1<<7))

/*Clock Enable Macros for I2C peripherals*/

#define I2C1_PERICLK_EN()	(RCC->APB1ENR |=(1<<21))
#define I2C2_PERICLK_EN()	(RCC->APB1ENR |=(1<<22))
#define I2C3_PERICLK_EN()	(RCC->APB1ENR |=(1<<23))

/*Clock Enable Macros for SPI peripherals*/

#define SPI1_PERICLK_EN()	(RCC->APB2ENR |=(1<<12))
#define SPI2_PERICLK_EN()	(RCC->APB1ENR |=(1<<14))
#define SPI3_PERICLK_EN()	(RCC->APB1ENR |=(1<<15))
#define SPI4_PERICLK_EN()	(RCC->APB2ENR |=(1<<13))


/*Clock Enable Macros for USART peripherals*/

#define USART1_PERICLK_EN()	(RCC->APB2ENR |=(1<<4))
#define USART2_PERICLK_EN()	(RCC->APB1ENR |=(1<<17))
#define USART3_PERICLK_EN()	(RCC->APB1ENR |=(1<<18))
#define UART4_PERICLK_EN()	(RCC->APB1ENR |=(1<<19))
#define UART5_PERICLK_EN()	(RCC->APB1ENR |=(1<<20))
#define USART6_PERICLK_EN()	(RCC->APB2ENR |=(1<<5))

/*Clock Enable Macros for SYSCFG peripherals*/
#define SYSCFG_PERICLK_EN()	(RCC->APB2ENR |=(1<<14))


/*Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PERICLK_DI()	(RCC->AHB1ENR &=~(1<<7))

/*Clock Disable Macros for I2C peripherals */
#define I2C1_PERICLK_DI()	(RCC->APB1ENR &=~(1<<21))
#define I2C2_PERICLK_DI()	(RCC->APB1ENR &=~(1<<22))
#define I2C3_PERICLK_DI()	(RCC->APB1ENR &=~(1<<23))

/*Clock Disable Macros for SPI peripherals */

#define SPI1_PERICLK_DI()	(RCC->APB2ENR &=~(1<<12))
#define SPI2_PERICLK_DI()	(RCC->APB1ENR &=~(1<<14))
#define SPI3_PERICLK_DI()	(RCC->APB1ENR &=~(1<<15))
#define SPI4_PERICLK_DI()	(RCC->APB2ENR &=~(1<<13))

/*Clock Disable Macros for USART peripherals */
#define USART1_PERICLK_DI()	(RCC->APB2ENR &=~(1<<4))
#define USART2_PERICLK_DI()	(RCC->APB1ENR &=~(1<<17))
#define USART3_PERICLK_DI()	(RCC->APB1ENR &=~(1<<18))
#define UART4_PERICLK_DI()	(RCC->APB1ENR &=~(1<<19))
#define UART5_PERICLK_DI()	(RCC->APB1ENR &=~(1<<20))
#define USART6_PERICLK_DI()	(RCC->APB2ENR &=~(1<<5))


/*Clock Disable Macros for SYSCFG peripherals */
#define SYSCFG_PERICLK_DI()	(RCC->APB2ENR &=~(1<<14)

/*Peripheral Reset Macros from the ref manual, have to set bit 0 of AHB1RSTR to 1 then 0 to reset */
#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<1)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<2)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<3)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<4)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<5)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<6)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<7)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define SPI1_REG_RESET()			do{(RCC->AHB2RSTR |=(1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()			do{(RCC->APB1RSTR |=(1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()			do{(RCC->APB1RSTR |=(1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()			do{(RCC->AHB2RSTR |=(1<<12)); (RCC->APB2RSTR &= ~(1<<13));}while(0)
#define I2C1_REG_RESET()			do{(RCC->APB1RSTR |=(1<<21)); (RCC->APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()			do{(RCC->APB1RSTR |=(1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()			do{(RCC->APB1RSTR |=(1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0)


/*bit position definitions for SPI peripheral */
#define SPI_CR1_CPHA					0
#define SPI_CR1_CPOL					1
#define SPI_CR1_MSTR					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFRST					7
#define SPI_CR1_SSI						8
#define SPI_CR1_SSM						9
#define SPI_CR1_RXONLY					10
#define SPI_CR1_DFF						11
#define SPI_CR1_CRCNXT					12
#define SPI_CR1_CRCEN					13
#define SPI_CR1_BIDIOE					14
#define SPI_CR1_BIDIMODE				15

#define SPI_CR2_RXMDAEN					0
#define SPI_CR2_TXMDAEN					1
#define SPI_CR2_SSOE					2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7

#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7
#define SPI_SR_FRE						8

/*bit position definitions for I2C peripheral */
#define I2C_CR1_PE						0
#define I2C_CR1_SMBUS					1
#define I2C_CR1_SMBTYPE					3
#define I2C_CR1_ENARP					4
#define I2C_CR1_ENPEC					5
#define I2C_CR1_ENGC					6
#define I2C_CR1_NOSTRETCH				7
#define I2C_CR1_START					8
#define I2C_CR1_STOP					9
#define I2C_CR1_ACK						10
#define I2C_CR1_POS						11
#define I2C_CR1_PEC						12
#define I2C_CR1_ALERT					13
#define I2C_CR1_SWRST					15

#define I2C_CR2_FREQ0					0
#define I2C_CR2_FREQ1					1
#define I2C_CR2_FREQ2					2
#define I2C_CR2_FREQ3					3
#define I2C_CR2_FREQ4					4
#define I2C_CR2_FREQ5					5
#define I2C_CR2_ITERREN					8
#define I2C_CR2_ITEVTEN					9
#define I2C_CR2_ITBUFEN					10
#define I2C_CR2_DMAEN					11
#define I2C_CR2_LAST					12

#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT					14
#define I2C_CR1_SMBALERT				15

#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_SMBDEFAULT				5
#define I2C_SR2_SMBHOST					6
#define I2C_SR2_DUALF					7
#define I2C_SR2_PEC0					8
#define I2C_SR2_PEC1					9
#define I2C_SR2_PEC2					10
#define I2C_SR2_PEC3					11
#define I2C_SR2_PEC4					12
#define I2C_SR2_PEC5					13
#define I2C_SR2_PEC6					14
#define I2C_SR2_PEC7					15


#define I2C_CCR_CCR0					0
#define I2C_CCR_CCR1					1
#define I2C_CCR_CCR2					2
#define I2C_CCR_CCR3					3
#define I2C_CCR_CCR4					4
#define I2C_CCR_CCR5					5
#define I2C_CCR_CCR6					6
#define I2C_CCR_CCR7					7
#define I2C_CCR_CCR8					8
#define I2C_CCR_CCR9					9
#define I2C_CCR_CCR10					10
#define I2C_CCR_CCR11					11
#define I2C_CCR_DUTY					14
#define I2C_CCR_FS 						15



/*generic macros*/
#define ENABLE 							1
#define DISABLE 						0
#define SET								ENABLE
#define RESET							DISABLE

/*IRQ numbers of stm32f446E from table 38 vector table of stm32f446e ref manual*/
#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40
#define IRQ_NO_SPI1						42
#define IRQ_NO_SPI2						43
#define IRQ_NO_SPI3						58
#define IRQ_NO_I2C1_EV					31
#define IRQ_NO_I2C1_ER					32
#define IRQ_NO_I2C2_EV					33
#define IRQ_NO_I2C2_ER					34
#define IRQ_NO_I2C3_EV					72
#define IRQ_NO_I2C3_ER					73

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_SPI_driver.h"
#include "stm32f446xx_I2C_driver.h"


#endif /* INC_STM32F446E_H_ */
