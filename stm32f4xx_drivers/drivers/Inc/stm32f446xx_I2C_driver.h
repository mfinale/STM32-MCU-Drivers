/*
 * stm32f446_I2C_driver.h
 *
 *  Created on: JUNE 8, 2020
 *      Author: Michael R. Finale
 *      This file contains I2C driver specific data.
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446E.h"




/*I2C CONFIG STRUCTURE*/
typedef struct{

	uint32_t 	I2C_SCLSpeed;
	uint8_t 	I2C_DeviceAddress;
	uint8_t 	I2C_ACKControl;
	uint16_t 	I2C_FMDutyCycle;



}I2C_PinConfig_t;


/* I2C HANDLE STRUCTURE */
typedef struct{

	I2C_RegDef_t *pI2Cx;  /*this pointer holds the I2C base address*/
	I2C_PinConfig_t I2C_Config;  /*this holds gpio pin configuration settings*/

}I2C_Handle_t;



/*I2C SCL SPEED OPTIONS */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM4K	400000
#define I2C_SCL_SPEED_F2K	200000


/*I2C ACK OPTIONS */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*I2C DUTY CYCLE OPTIONS */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/**flag definitions of I2C status registerS**/
#define I2C_TXE_FLAG 				(1<<I2C_SR1_TXE)
#define I2C_RXNE_FLAG 				(1<<I2C_SR1_RXNE)
#define I2C_SB_FLAG 				(1<<I2C_SR1_SB)
#define I2C_BTF_FLAG 				(1<<I2C_SR1_BTF)
#define I2C_STOPF_FLAG 				(1<<I2C_SR1_STOPF)
#define I2C_BERR_FLAG 				(1<<I2C_SR1_BERR)
#define I2C_ARLO_FLAG 				(1<<I2C_SR1_ARLO)
#define I2C_AF_FLAG 				(1<<I2C_SR1_AF)
#define I2C_OVR_FLAG 				(1<<I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG 			(1<<I2C_SR1_TIMEOUT)
#define I2C_ADDR_FLAG 			(1<<I2C_SR1_ADDR)














/*******************************************************************************
 *
 * 							APIs supported by this driver
 *
 *
 * ****************************************************************************/



/* Peripheral Clock Control*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*  Init and De-init*/
void I2C_Init(I2C_Handle_t *I2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* Data send/receiver*/
void I2CMasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveaddr);


/*Interrupt Control*/
void I2C_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi);
//void I2C_IRQHandling(SPI_Handle_t *SPIhandle);
void I2C_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */

