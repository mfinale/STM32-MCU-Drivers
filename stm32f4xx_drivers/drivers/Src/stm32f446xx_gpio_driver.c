/*
 * stm32f4446xx_gpio_driver.c
 *
 *  Created on: Apr 20, 2020
 *      Author: User
 */


#include "stm32f446xx_gpio_driver.h"



/*****************************************************************
* Function name:Peripheral Clock Control
* Description:	Enable or disable the peripheral clock for the given GPIO port
* Parameter1:	base address of GPIO port
* Parameter2:	enable or disable
* ReturnVal:	none
* Notes:		none
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERICLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERICLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERICLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERICLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERICLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERICLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERICLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERICLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERICLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERICLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERICLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERICLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERICLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERICLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERICLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERICLK_DI();
		}

	}
}


/*****************************************************************
* Function name: GPIO Init
* Description:	Enables given GPIO port by configuring the mode, speed, pullup/pulldown
*  output type, and alternate functionality. Done by getting desired setting from handle
*  and bit shift it by pin number (it is multiplied by the number of bits
*  required for the setting).
* Parameter1:	Handle of GPIO port
* ReturnVal:	none
* Notes:		none
*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{	uint32_t temp=0;

	//enable peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//see if its not an interrupt mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//configure pin mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear bit fields before setting
		pGPIOHandle->pGPIOx->MODER |=temp; //set bits

	}
	//else configure for  interrupt mode
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure Falling trigger selection register
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding RTSR bit
			EXTI->RTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. configure Rising trigger selection register
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear corresponding FTSR bit
			EXTI->FTSR &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1. configure Falling and Rising trigger selection register
			EXTI->FTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//2. configure GPIO port selection in SYSCFG_EXTIC
		uint8_t temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4; //to get which exti control register
		uint8_t temp2 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4; //to determine section within control register determined above
		uint8_t portcode = 0;

		if(pGPIOHandle->pGPIOx == GPIOA){portcode=0;}
		else if(pGPIOHandle->pGPIOx == GPIOB){portcode=1;}
		else if(pGPIOHandle->pGPIOx == GPIOC){portcode=2;}
		else if(pGPIOHandle->pGPIOx == GPIOD){portcode=3;}
		else if(pGPIOHandle->pGPIOx == GPIOE){portcode=4;}
		else if(pGPIOHandle->pGPIOx == GPIOF){portcode=5;}
		else if(pGPIOHandle->pGPIOx == GPIOG){portcode=6;}
		else if(pGPIOHandle->pGPIOx == GPIOH){portcode=7;}

		SYSCFG_PERICLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);
		//3. enable EXTI interrupt delivery via IMR
		EXTI->IMR |= 1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	//configure speed
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR  &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	//configure pullup/pulldown
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR  &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	//configure output type
	temp=0;
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER  &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp=0;

	if((pGPIOHandle->GPIO_PinConfig.GPIO_PinMode) == GPIO_MODE_ALTFN)
	{
		//Need to revisit this to understand
		uint8_t temp1, temp2;
		temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1]&= ~(0xF<<(4*temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode<<(4*temp2));


	}




}


/*****************************************************************
* Function name: GPIO Init
* Description:	Disables given GPIO port
* Parameter1:	Base Address of a GPIO port
* ReturnVal:	none
* Notes:		none
*/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();;
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}



/*****************************************************************
* Function name: GPIO Pin Read
* Description:	Reads value of specified GPIO pin by
* shifting IDR by pin number to LSB then AND to get desired value
* Parameter1:	Base Address of a GPIO port
* Parameter2:	GPIO pin in range of 0 to 16
* ReturnVal:	0 or 1
* Notes:		none
*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*****************************************************************
* Function name: GPIO Port Read
* Description:	Reads value of specified GPIO pin
* Parameter1:	Base Address of a GPIO port
* ReturnVal:	Current value of all pins on GPIO port as a 16 bit unsigned integer
* Notes:		none
*/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR ;
	return value;

}


/*****************************************************************
* Function name: GPIO Pin Write
* Description:	Sets a specified pin on a given GPIO port ot high or low
* Parameter1:	Base Address of a GPIO port
* Parameter2:	Pin on GPIO port
* Parameter3:	Value to write to pin
* ReturnVal:	none
* Notes:		none
*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,  uint8_t PinNumber, uint8_t Value)
{

	if(Value == SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}
}


/*****************************************************************
* Function name: GPIO Port Write
* Description:	Set value of multiple pins on a GPIO port
* Parameter1:	Base Address of a GPIO port
* Parameter2:	Value to write to pin
* ReturnVal:	none
* Notes:		none
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*****************************************************************
* Function name: GPIO Toggle Pin
* Description:	Toggles the value of a pin on a given GPIO port by XOR
* with bit 1 shifted by pin number
* Parameter1:	Base Address of a GPIO port
* Parameter2:	Pin of GPIO port
* ReturnVal:	none
* Notes:		none
*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1<<PinNumber);


}



/*****************************************************************
* Function name: IRConfig
* Description:	!!!!!!TODO
* Parameter1:
* Parameter2:
* ReturnVal:	none
* Notes:		none
*/
void GPIO_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{

	if(EnorDi== ENABLE)
	{
		if(IRQNumber <=31)
		{
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 &&  IRQNumber<64 )
		{
			*NVIC_ISER1 |= (1<<IRQNumber%32);
		}
		else if(IRQNumber >64 &&  IRQNumber<96 )
		{
			*NVIC_ISER2 |= (1<<IRQNumber%64);
		}


	}
	else
	{
		if(IRQNumber <=31)
		{
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 &&  IRQNumber<64 )
		{
			*NVIC_ICER1 |= (1<<IRQNumber%32);
		}
		else if(IRQNumber >64 &&  IRQNumber<96 )
		{
			*NVIC_ICER2 |= (1<<IRQNumber%64);
		}
	}

}

void GPIO_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
  //determine IPR register
	uint8_t iprx =  IRQNumber/4;
	//determine IPR register section
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8* iprx_section) + (8-NO_OF_PRIO_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx) |= (IRQPriority<<(shift_amount));
}


/*****************************************************************
* Function name: !!!!!TODO
* Description:
* Parameter1:
* Parameter2:	P
* ReturnVal:	none
* Notes:		none
*/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear exti pending register
	if (EXTI->PR& (1<<PinNumber))
	{
		EXTI->PR |= (1<<PinNumber);
	}
}


