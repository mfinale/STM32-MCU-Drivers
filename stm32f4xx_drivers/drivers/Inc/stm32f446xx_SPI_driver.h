/*
 * stm32f446xx_SPI_driver.h
 *
 *  Created on: May 9, 2020
 *      Author: User
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_

#include "stm32f446E.h"

/*SPI configuration structure*/
typedef struct{


	uint8_t SPI_Devicemode;
	uint8_t SPI_Busconfig;
	uint8_t SPI_DFF;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t SPI_Speed;


}SPI_Config_t;


/*SPI Handle structure =  register structure + config structure */
typedef struct{

	SPI_RegDef_t *pSPIx;  /*this pointer holds the spi base address*/
	SPI_Config_t SPI_Config;  /*this holds spi  configuration settings*/

}SPI_Handle_t;





/** SPI DEVICE MODE OPTIONS **/
#define SPI_DEVICE_MASTER					1
#define SPI_DEVICE_SLAVE					0

/** SPI BUS CONFIG OPTIONS **/
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG__SIMPLEX_RXONLY		3



/** SPI CLOCK SPEED (PRESCALAR) OPTIONS **/
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


/** SPI DFF OPTIONS **/
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/** SPI CPOL OPTIONS **/
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0

/** SPI CPHA OPTIONS **/
#define SPI_CPHA_HIGH						1
#define SPI_CPPHA_LOW						0

/** SPI SSM OPTIONS **/
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/**flag definitions of SPI status register**/
#define SPI_TXE_FLAG 				(1<<SPI_SR_TXE)
#define SPI_BUSY_FLAG				(1<<SPI_SR_BSY)
#define SPI_RXNE_FLAG 				(1<<SPI_SR_RXNE)


/*******************************************************************************
 *
 * 							APIs supported by this driver
 *
 *
 * ****************************************************************************/

/* Peripheral Clock Control*/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*  Init and De-init*/
void SPI_Init(SPI_Handle_t *SPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*Blocking Data controls*/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t len );


/*Interrupt Control*/
void SPI_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void SPI_IRQHandling(SPI_Handle_t *SPIhandle);
void SPI_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);


void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*NSS pin control*/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
