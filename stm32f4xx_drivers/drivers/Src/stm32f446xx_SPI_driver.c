/*
 * stm32f446xx_SPI_driver.c
 *
 *  Created on: May 9, 2020
 *      Author: Michael Finale
 */

#include "stm32f446xx_SPI_driver.h"

static void spi_txe_handle(SPI_Handle_t *SPIHandle);
static void spi_rxne_handle(SPI_Handle_t *SPIHandle);
static void spi_over_err_handle(SPI_Handle_t *SPIHandle);

/* Peripheral Clock Control*/



/*****************************************************************
* Function name:Peripheral Clock Control
* Description:	Enable or disable the peripheral clock for the given SPI peripheral
* Parameter1:	base address of GPIO port
* Parameter2:	enable or disable
* ReturnVal:	none
* Notes:		none
*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERICLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PERICLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PERICLK_EN();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PERICLK_EN();
		}


	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERICLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PERICLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PERICLK_DI();
		}
		else if(pSPIx == SPI4)
		{
			SPI4_PERICLK_DI();
		}

	}
}


/*****************************************************************
* Function name: SPI Init
* Description:	Enable SPI peripheral clock and load settings defined
* by given SPI Handler into SPI register
* Parameter1:	handle of the desired SPI peripheral
* ReturnVal:	none
* Notes:		none
*/
void SPI_Init(SPI_Handle_t *SPIHandle)
{
	//enable peripheral clock
	SPI_PeriClockControl(SPIHandle->pSPIx, ENABLE);


	uint32_t tempreg=0;
	//1. device mode
	tempreg |= SPIHandle->SPI_Config.SPI_Devicemode<<SPI_CR1_MSTR;
	//2. Busconfig
	if(SPIHandle->SPI_Config.SPI_Busconfig==SPI_BUS_CONFIG_FD)
	{
		//clear bidi mode/
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);

	}
	else if(SPIHandle->SPI_Config.SPI_Busconfig==SPI_BUS_CONFIG_HD)
	{
		//enable bidi mode
		tempreg |= (1<<SPI_CR1_BIDIMODE);

	}
	else if(SPIHandle->SPI_Config.SPI_Busconfig==SPI_BUS_CONFIG__SIMPLEX_RXONLY)
	{
		//clear bidimode + rx only
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		tempreg |= (1<<SPI_CR1_RXONLY);
	}
	//3. SPI serial clock speed
	tempreg |= SPIHandle->SPI_Config.SPI_Speed <<SPI_CR1_BR;
	//4. DFF
	tempreg |= SPIHandle->SPI_Config.SPI_DFF <<SPI_CR1_DFF;
	//5. CPOL
	tempreg |= SPIHandle->SPI_Config.SPI_CPOL <<SPI_CR1_CPOL;
	//6. CPHA
	tempreg |= SPIHandle->SPI_Config.SPI_CPHA <<SPI_CR1_CPHA;
	//7. SSM
	tempreg |= SPIHandle->SPI_Config.SPI_SSM <<SPI_CR1_SSM;

	//write value in tempreg to the CR1 register
	SPIHandle->pSPIx->CR1 = tempreg;
}

/*****************************************************************
* Function name: SPI DeInit
* Description:	Reset SPI peripheral
* Parameter1:	Register definition of the desired SPI peripheral
* ReturnVal:	none
* Notes:		none
*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		GPIOA_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		GPIOB_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		GPIOC_REG_RESET();
	}
	else if(pSPIx == SPI4)
	{
		GPIOD_REG_RESET();
	}

}

/*****************************************************************
* Function name: SPI_PeripheralControl
* Description:	Enable SPI peripheral clock
* Parameter1:	Register definition of the desired SPI peripheral
* Parameter2:	Enable or disable value
* ReturnVal:	none
* Notes:		none
*/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi==ENABLE)
	{
		pSPIx->CR1 |= (1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR1_SPE);
	}
}

//enable/disable the SSI bit to connect NSS bit to internal voltage source.
//not enabling SSI bit can lead to MODF error
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi==ENABLE)
		{
			pSPIx->CR1 |= (1<<SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1<<SPI_CR1_SSI);
		}
}

//enable disable SSOE bit for controlling nss pin
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi==ENABLE)
	{
		pSPIx->CR2 |= (1<<SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<<SPI_CR2_SSOE);
	}

}


/*******************Blocking Data controls************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t Flagname)
{
	if(pSPIx->SR & Flagname)
	{
		return 1;
	}
	return 0;
}

/* SPI send data (blocking) */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len>0)
	{


		//wait until TX buffer is empty (TXEis empty via status register)
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==0);

		if(pSPIx->CR1 & (1<< SPI_CR1_DFF ))
		{
			// 16 bit dff type cast into 16 bit pointer then dereference
			// I definitely would like to use a less confusing implementation
			//load DR with 2 bytes of data to send and increment the buffer address
			// to point to the next group of data to send
			pSPIx->DR= *((uint16_t  *)pTxBuffer) ;
			len--;
			len--;
			(uint16_t  *)pTxBuffer++;

		}
		else
		{
			// 8 bit dff
			//load DR with 1 byte of data to be sent
			//and increment the buffer address for next byte of data
			pSPIx->DR= *pTxBuffer ;
			len--;
			pTxBuffer++;

		}

	}



}


/* SPI receive data (blocking) */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while(len>0)
	{


		//wait until RX buffer is not empty (RXNE via status register for SPI)
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG==0));
		//check the DFF bit in CR1
		if(pSPIx->CR1 & (1<< SPI_CR1_DFF ))
		{	//16 bit DFF
			//load 2 bytes of data from DR to RX buffer address
			//and increment the buffer address to point to the next group
			// of data to read
			*((uint16_t  *)pRxBuffer)= pSPIx->DR ;
			len--;
			len--;
			(uint16_t  *)pRxBuffer++;

		}
		else
		{
			// 8 bit dff
			//load RXbuffer with one byte of data from DR
			//and increment the buffer address for next byte of data
		   *pRxBuffer =pSPIx->DR ;
			len--;
			pRxBuffer++;

		}

	}


}

/*Interrupt Control*/
/******************************************************************/

//enable the given IRQ
void SPI_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi)
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
void SPI_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//determine IPR register
	uint8_t iprx =  IRQNumber/4;
	//determine IPR register section
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8* iprx_section) + (8-NO_OF_PRIO_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR+iprx) |= (IRQPriority<<(shift_amount));
}



/*Non-blocking Data controls*/
uint8_t SPI_SendDataIT(SPI_Handle_t *SPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
		uint8_t state = SPIHandle->TxState;
		if (state!=SPI_BUSY_IN_TX)
		{
			//1. Save Tx buffer address and len info into handle
			SPIHandle->pTxBuffer = pTxBuffer;
			SPIHandle->TxLen = len;
			//2. Mark SPI state as busy
			SPIHandle->TxState = SPI_BUSY_IN_TX;
			//3. Enable TXEIE interrupt control bit
			SPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
			//4. Transmit Data with ISR code
		}

		return state;

}

uint8_t  SPI_ReceiveDataIT(SPI_Handle_t *SPIHandle,uint8_t *pRxBuffer, uint32_t len )
{
	uint8_t state = SPIHandle->TxState;
	if (state!=SPI_BUSY_IN_RX)
	{
		//1. Save Tx buffer address and len info into handle
		SPIHandle->pRxBuffer = pRxBuffer;
		SPIHandle->RxLen = len;
		//2. Mark SPI state as busy
		SPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable TXEIE interrupt control bit
		SPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
		//4. Transmit Data with ISR code
	}

	return state;


}

void SPI_IRQHandling(SPI_Handle_t *SPIhandle)
{
	uint8_t temp1, temp2;
	//check status register for which event occurred (which flag is set and if interrupt is enabled)
	//handle TXE interrupt
	temp1 = SPIhandle->pSPIx->SR & (1<<SPI_SR_TXE);
	temp2 = SPIhandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE);
	if (temp1 && temp2)
	{
		spi_txe_handle(SPIhandle);
	}
	//handle RXNE interrupt
	temp1 = SPIhandle->pSPIx->SR & (1<<SPI_SR_RXNE);
	temp2 = SPIhandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		spi_rxne_handle(SPIhandle);
	}
	//handle Overrun interrupt
	temp1 = SPIhandle->pSPIx->SR & (1<<SPI_SR_OVR);
	temp2 = SPIhandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{

		spi_over_err_handle(SPIhandle);
	}

}


static void spi_txe_handle(SPI_Handle_t *SPIHandle)
{
	if(SPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF ))
	{
		// 16 bit dff type cast into 16 bit pointer then dereference
		// I definitely would like to use a less confusing implementation
		//load DR with 2 bytes of data to send and increment the buffer address
		// to point to the next group of data to send
		SPIHandle->pSPIx->DR= *((uint16_t  *)SPIHandle->pTxBuffer) ;
		SPIHandle->TxLen--;
		SPIHandle->TxLen--;
		(uint16_t  *)SPIHandle->pTxBuffer++;

	}
	else
	{
		// 8 bit dff
		//load DR with 1 byte of data to be sent
		//and increment the buffer address for next byte of data
		SPIHandle->pSPIx->DR= *SPIHandle->pTxBuffer;
		SPIHandle->TxLen--;
		SPIHandle->pTxBuffer++;

	}

	if (!SPIHandle->TxLen)
	{
		//disable interrupt flag and close spi transmission
		SPI_CloseTransmission(SPIHandle);


	}

}

static void spi_rxne_handle(SPI_Handle_t *SPIHandle)
{
	//check the DFF bit in CR1
	if(SPIHandle->pSPIx->CR1 & (1<< SPI_CR1_DFF ))
	{	//16 bit DFF
		//load 2 bytes of data from DR to RX buffer address
		//and increment the buffer address to point to the next group
		// of data to read
		*((uint16_t  *)SPIHandle->pRxBuffer)= SPIHandle->pSPIx->DR ;
		SPIHandle->RxLen--;
		SPIHandle->RxLen--;
		(uint16_t  *)SPIHandle->pRxBuffer++;

	}
	else
	{
		// 8 bit dff
		//load RXbuffer with one byte of data from DR
		//and increment the buffer address for next byte of data
	   *SPIHandle->pRxBuffer =SPIHandle->pSPIx->DR ;
	   	SPIHandle->RxLen--;
	   	SPIHandle->pRxBuffer++;

	}
	if (!SPIHandle->RxLen)
	{
		//disable interrupt flag and close spi reception
		SPI_CloseReception(SPIHandle);
	}
}


static void spi_over_err_handle(SPI_Handle_t *SPIHandle)
{
	uint8_t temp;
	//clear ovr flag
	if(SPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = SPIHandle->pSPIx->DR;
		temp = SPIHandle->pSPIx->SR;
	}
	(void) temp;

	//inform application
	SPI_ApplicationEventCallback(SPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{	uint8_t temp;
	//disable interrupt flag and close spi transmission
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void) temp;

}
void SPI_CloseTransmission(SPI_Handle_t *SPIHandle)
{
	SPIHandle->pSPIx->CR2  &= ~(1<< SPI_CR2_TXEIE);
	SPIHandle->pTxBuffer=NULL;
	SPIHandle->TxLen= 0;
	SPIHandle->TxState=SPI_READY;
	//application has to implement callback
	SPI_ApplicationEventCallback(SPIHandle, SPI_EVENT_TX_CMPLT);
}
void SPI_CloseReception(SPI_Handle_t *SPIHandle)
{
	//disable interrupt flag and close spi transmission
	SPIHandle->pSPIx->CR2  &= ~(1<< SPI_CR2_RXNEIE);
	SPIHandle->pRxBuffer=NULL;
	SPIHandle->RxLen= 0;
	SPIHandle->RxState=SPI_READY;
	//application has to implement callback
	SPI_ApplicationEventCallback(SPIHandle, SPI_EVENT_RX_CMPLT);
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *SPIhandle, uint8_t AppEvent)
{
	//This is a weak implementation and the application can override this function
}
