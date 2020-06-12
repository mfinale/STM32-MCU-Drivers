/*
 * stm32f4446xx_I2C_driver.c
 *
 *  Created on: Apr 20, 2020
 *      Author: User
 */


#include "stm32f446xx_I2C_driver.h"
static void I2C_GenerateStart(I2C_RegDef_t *pI2Cx);
uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_Prescaler[8] = {2,4,8,16};
uint32_t RCC_GETPCLK1Value(void)
{
	uint32_t pclk1, systemclk;
	uint8_t clksrc, temp, ahbpresc,apb1presc;
	//determine clock source by reading RCC registers
	clksrc = ((RCC->CFGR >>2) & 0x3);
	if (clksrc ==0 )
	{
		//HSI clock source 16 MHz
		systemclk =  16000000;
	}
	else if (clksrc ==1 )
	{
		//HSE clock source (8 MHz for discovery board)
		systemclk =  8000000;
	}
	else if (clksrc ==1 )
	{
		//PLL clock source
		systemclk =  RCC_GetPLLclock();
	}
	//get AHB prescaler value
	temp =  ((RCC->CFGR >>4) & 0xF);
	if (temp<8)
	{
		ahbpresc = 1;
	}
	else
	{
		ahbpresc = AHB_Prescaler[temp-8];
	}
	//get APB1 prescaler
	temp =  ((RCC->CFGR >>10) & 0x7);
	if (temp<7)
	{
		apb1presc = 1;
	}
	else
	{
		apb1presc = APB1_Prescaler[temp-4];
	}
	//calculate peripheral clock 1 frequency
	pclk1 = (systemclk/ahbpresc)/apb1presc;
	return pclk1;
}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PERICLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PERICLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PERICLK_EN();
		}
	}

	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PERICLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PERICLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PERICLK_DI();
		}

	}

}


void I2C_Init(I2C_Handle_t *I2CHandle)
{
	uint32_t tempreg = 0;
	uint16_t ccrvalue;
	// steps must be done when peripheral is disabled
	//1. Configure mode (speed): standard or fast
	if (I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccrvalue =  (RCC_GETPCLK1Value()/(2*I2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= ccrvalue & 0xFFF; //maskout other bits except first 12
	}
	else
	{
		//fastmode
		tempreg |= (1<<15);
		tempreg |= (I2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if (I2CHandle->I2C_Config.I2C_FMDutyCycle==2)
		{
			ccrvalue =  (RCC_GETPCLK1Value()/(3*I2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccrvalue =  (RCC_GETPCLK1Value()/(25*I2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= ccrvalue & 0xFFF; //maskout other bits except first 12
	}
	I2CHandle->pI2Cx->CCR = tempreg;
	//2. FREQ bits must be configured with the APB clock frequency value
	tempreg = 0;
	tempreg |=  RCC_GETPCLK1Value()/1000000;
	I2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); //masks other bits and write to cr2
	//3. Configure device address (applies if slave)
	tempreg=0;
	tempreg |= I2CHandle->I2C_Config.I2C_DeviceAddress<<1  ;
	tempreg |= (1<<14); //bit 14 has to be kept at 1 per ref manual
	I2CHandle->pI2Cx->OAR1 = tempreg;
	//4. Enable Acking
	tempreg=0;
	tempreg |=  (I2CHandle->I2C_Config.I2C_ACKControl <<10);
	I2CHandle->pI2Cx->CR1 = tempreg;
	//5. Configure rise time of I2C pins



}




void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}



/*Interrupt Control*/
/******************************************************************/

//enable the given IRQ
void I2C_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi)
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

//set the priority of a given IRQ
void I2C_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//determine IPR register
	uint8_t iprx =  IRQNumber/4;
	//determine IPR register section
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8* iprx_section) + (8-NO_OF_PRIO_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx) |= (IRQPriority<<(shift_amount));
}

/*******************Blocking Data controls************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname)
{
	if(pI2Cx->SR1 & Flagname)
	{
		return 1;
	}
	return 0;
}

void I2CMasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveaddr)
{
	//1. Generate Start condition
	pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_START);
	//2. Confirm start generation by checking SB flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));
	//3. Send address of the slave + r/w bit set to '0'
	slaveaddr = slaveaddr <<1;
	slaveaddr &= ~(1);
	pI2CHandle->pI2Cx->DR =  slaveaddr;
	//4. Confirm address phase is completed by checking ADDR flag
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));
	//5. Clear ADDR FLAG
	uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
	dummyread =  pI2CHandle->pI2Cx->SR2;
	//6. Send data until len becomes 0
	while (len>0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		len--;

	}
	//7. When len = 0 wait for TXE=1 and BTF=1 then generate stop condition
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));
	pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);



}




