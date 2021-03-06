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



}I2C_Config_t;


/* I2C HANDLE STRUCTURE */
typedef struct{

	I2C_RegDef_t 	*pI2Cx;  /*this pointer holds the I2C base address*/
	I2C_Config_t 	I2C_Config;  /*this holds gpio pin configuration settings*/
	uint8_t			*pTxBuffer; /*pointer to data in TxBuffer*/
	uint8_t			*pRxBuffer; /*pointer to data in RxBuffer*/
	uint32_t		TxLen; /*length of TX data*/
	uint32_t		RxLen; /*length of TX data*/
	uint8_t			TxRxState;	/*to determine if we are transmitting or receiving*/
	uint8_t			DevAddr; /* slave/device address */
	uint32_t		RxSize;
	uint8_t			Sr;		/* option to enable repeated start */


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

/*I2C Application States */
#define I2C_READY			0
#define I2C_BUSY_IN_RX		1
#define I2C_BUSY_IN_TX		2


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
#define I2C_ADDR_FLAG 				(1<<I2C_SR1_ADDR)


/* I2C application events macros */
#define I2C_EVENT_TX_COMPLETE	0
#define I2C_EVENT_RX_COMPLETE	1
#define I2C_EVENT_STOP			2
#define I2C_ERROR_BERR		    3
#define I2C_ERROR_ARLO 		 	4
#define I2C_ERROR_AF			5
#define I2C_ERROR_OVR			6
#define I2C_ERROR_TIMEOUT		7











/*******************************************************************************
 *
 * 							APIs supported by this driver
 *
 *
 * ****************************************************************************/



/* Peripheral Clock Control*/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/* Peripheral Control */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*  Init and De-init*/
void I2C_Init(I2C_Handle_t *I2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* Data send/receiver*/
void I2CMasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveaddr, uint8_t SR);
void I2CMasterRcvData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveaddr, uint8_t SR);

uint8_t I2CMasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t slaveaddr, uint8_t SR);
uint8_t I2CMasterRcvDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t slaveaddr, uint8_t SR);

/*Extra Peripheral controls*/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*Interrupt Control*/
void I2C_IRQConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void I2C_IRQPriority_Config(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EVENT_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ERROR_IRQHandling(I2C_Handle_t *pI2CHandle);



/*
 * Application Callback
 */

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */

