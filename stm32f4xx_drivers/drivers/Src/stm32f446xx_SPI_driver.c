/*
 * stm32f446xx_SPI_driver.c
 *
 *  Created on: May 9, 2020
 *      Author: Michael Finale
 */

#include "stm32f446xx_SPI_driver.h"

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
//void SPI_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi);
//void SPI_IRQHandling(SPI_Handle_t *SPIhandle);
//void SPI_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
