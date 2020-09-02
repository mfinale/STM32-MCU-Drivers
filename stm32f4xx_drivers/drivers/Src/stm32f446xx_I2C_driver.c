/*
 * stm32f4446xx_I2C_driver.c
 *
 *  Created on: May 20, 2020
 *      Author: Michael Finale
 */




#include "stm32f446xx_I2C_driver.h"


uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_Prescaler[8] = {2,4,8,16};



static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	 //check device mode
	if (pI2CHandle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
	{
		//device is in master mode
		if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize==1)
			{
				//first disable ACK
				pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);

				//clear ADDR flag
				uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
				dummyread =  pI2CHandle->pI2Cx->SR2;
				(void)dummyread;


			}
			else
			{
				//clear ADDR flag
				uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
				dummyread =  pI2CHandle->pI2Cx->SR2;
				(void)dummyread;

			}
		}

	}
	else
	{
		//device is in slave mode
		//clear ADDR flag
		uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
		dummyread =  pI2CHandle->pI2Cx->SR2;
		(void)dummyread;

	}
}

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


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi==ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_PE);
	}
}



void I2C_Init(I2C_Handle_t *I2CHandle)
{

	uint32_t tempreg = 0;
	uint16_t ccrvalue;
	uint8_t trise =0;


	//enable peripheral clock
	I2C_PeriClockControl(I2CHandle->pI2Cx,ENABLE );


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
	tempreg = 0;
	//TRISE= (trisemax /tpclkl1)+1 or...fpclk1/frisetimemax+1
	if (I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode (trise max =  1000 nanoseconds)
		trise= (RCC_GETPCLK1Value()/1000000)+1;

	}
	else
	{
		//fast mode (trise max = 300 nanoseconds)
		trise= ((RCC_GETPCLK1Value()*300)/1000000000)+1;

	}
	tempreg = trise;
	I2CHandle->pI2Cx->TRISE = (tempreg &0x3F);



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




/*******************Blocking Data controls************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t Flagname)
{
	if(pI2Cx->SR1 & Flagname)
	{
		return 1;
	}
	return 0;
}

void I2CMasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveaddr, uint8_t SR)
{
	//SR is to enable repeated start to hold the bus from other devices
	//len is in bytes
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
	(void)dummyread;
	//6. Send data until len becomes 0
	while (len>0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		len--;

	}
	//7. When len = 0 wait for TXE=1 and BTF=1 then generate stop condition or repeated start
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_TXE_FLAG));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_BTF_FLAG));
	if (SR == 0 )
	{
		pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
	}




}

void I2CMasterRcvData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveaddr, uint8_t SR)
{
	//len is in bytes
	//SR is to enable repeated start to hold the bus from other devices

	//1. Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_START);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	slaveaddr = slaveaddr <<1;
	slaveaddr |= 1;
	pI2CHandle->pI2Cx->DR =  slaveaddr;

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_ADDR_FLAG));


	//procedure to read only 1 byte from slave
	if(len == 1)
	{
		//Disable Acking
		pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);

		//clear the ADDR flag
		uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
		dummyread =  pI2CHandle->pI2Cx->SR2;
		(void) dummyread;

		//wait until  RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG));


		//generate STOP condition if SR set to 1
		if (SR == 0 )
		{
			pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
		}

		//read data in to buffer
		*pRxbuffer = pI2CHandle->pI2Cx->DR;


	}


	//procedure to read data from slave when Len > 1
	if(len > 1)
	{
		//clear the ADDR flag
		uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
		dummyread =  pI2CHandle->pI2Cx->SR2;
		(void) dummyread;

		//read the data until len becomes zero
		for ( uint32_t i = len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_RXNE_FLAG));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);

				//generate STOP condition if SR set to 1
				if (SR == 0 )
				{
					pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
				}

			}

			//read the data from data register in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->DR;
			//increment the buffer address
			pRxbuffer++;

		}

	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl==I2C_ACK_ENABLE)
	{pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_ACK);}





}



/*******************NON-blocking Data controls************************/


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

//generates interrupt for sending data
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_START);

		//enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);

		//ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);

	}

	return busystate;

}



uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate START Condition
		pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_START);


		//enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1<<I2C_CR2_ITERREN);


	}

	return busystate;
}


void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen>0)
	{
		//1. load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		//2. decrement Txlen
		pI2CHandle->TxLen--;
		//3. increment buffer address
		pI2CHandle->pTxBuffer++;
	}

}



void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize==1)
	{
			*pI2CHandle->pRxBuffer==pI2CHandle->pI2Cx->DR;
			pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize>1)
	{
		if(pI2CHandle->RxLen==2)
		{
			pI2CHandle->pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
		}


		//read DR
		*pI2CHandle->pRxBuffer==pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;

	}
	if(pI2CHandle->RxLen==0)
	{
		//close I2C data reception and notify the application

		//1. generate STOP condition
		if(pI2CHandle->Sr==0)
		{
			pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
		}

		//2. Close I2C rx
		I2C_CloseReceiveData();

		//3. Notify application
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_RX_COMPLETE);
	}

}



// handles events once interrupt is triggered via I2C_MasterReceiveDataIT or I2C_MasterSendDataIT
// TODO: document each event from ref manual
void I2C_EVENT_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for each I2C related interrupt


	uint32_t temp1, temp2, temp3;
	uint8_t slaveaddr;

	//get status of ITEVTEN and ITBUFEN
	temp1 =  pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITEVTEN);
	temp2 =  pI2CHandle->pI2Cx->CR2 & (1<<I2C_CR2_ITBUFEN);

	//1. check if SB EVENT is enabled (only applicable in master mode see ref manual)
	temp3 =  pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_SB);
	if (temp1 && temp3)
	{
		//execute address phase of read or write depending on state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			slaveaddr = pI2CHandle->DevAddr;
			slaveaddr= slaveaddr <<1;
			slaveaddr &= ~(1);
			pI2CHandle->pI2Cx->DR =  slaveaddr;
		}

		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			slaveaddr = pI2CHandle->DevAddr;
			slaveaddr = slaveaddr <<1;
			slaveaddr |= 1;
			pI2CHandle->pI2Cx->DR =  slaveaddr;
		}

	}

	//2. Handle ADDR  Event
	temp3 =  pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_ADDR);
	if (temp1 && temp3)
	{
		//clear ADDR flag
		uint32_t dummyread =  pI2CHandle->pI2Cx->SR1;
		dummyread =  pI2CHandle->pI2Cx->SR2;
		(void) dummyread;
	}

	//3. Handle  BTF event (BYTE TRANSFER FINISHED) if enabled
	temp3 =  pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_BTF);
	if (temp1 && temp3)
	{
		//check if busy in tx or rx
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if(pI2CHandle->pI2Cx->SR1&(1<<I2C_SR1_TXE))
			{
				if (pI2CHandle->TxLen==0)
				{
					//generate stop condition if no repeated start
					if (pI2CHandle->Sr==0)
					{pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);}

					//reset member elements of handle structure
					I2C_CloseSendData();


					//notify calling application that transmission is complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_TX_COMPLETE);
				}

			}
		}

		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;//do nothing
		}



	}


	//4. Handle STOPF Event (only executed in slave mode)
	temp3 =  pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_STOPF);
	if (temp1 && temp3)
	{
		//Clear STOPF FLAG (read SR1 (done above) and write to CR1)
		pI2CHandle->pI2Cx->CR1 |=0x000;

		//Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_STOP);
	}

	//5. Handle TXE Event (TX register empty)
	temp3 =  pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_TXE);
	if (temp1 && temp2 && temp3)
	{
		if ( pI2CHandle->pI2Cx->SR2 &(1<< I2C_SR2_MSL))
		{
			//TXE flag is set and device is in Master mode: Do a data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX )
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}

	}

	//6. Handle RXNE Even (Receive buffer not empty)
	temp3 =  pI2CHandle->pI2Cx->SR1 & (1<<I2C_SR1_RXNE);
	if (temp1 && temp2 && temp3)
	{
		//check device mode
		if ( pI2CHandle->pI2Cx->SR2 &(1<< I2C_SR2_MSL))
		{
			//device is in master mode
			if(pI2CHandle->TxRxState==I2C_BUSY_IN_RX)
			{

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);

			}
		}

	}


}
void I2C_ERROR_IRQHandling(I2C_Handle_t *pI2CHandle);
