/*
 * stm32f446_gpio_driver.h
 *
 *  Created on: Apr 20, 2020
 *      Author: Michael R. Finale
 *      This file contains gpio driver specific data.
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446E.h"




/*GPIO configuration settings for each pin*/
typedef struct{

	//use uint 8 as there is a maximum of 15 GPIO pins. uint_32 would be wasteful */
	uint8_t GPIO_PinNumber;			/*!<possible values from GPIO Pin Numbers >*/
	uint8_t GPIO_PinMode;			/*!<possible values from GPIO Pin Modes >*/
	uint8_t GPIO_PinSpeed;			/*!<possible values from GPIO Pin Speeds>*/
	uint8_t GPIO_PinPuPdControl;	/*!<possible values from GPIO Pin pullup/pulldown config >*/
	uint8_t GPIO_PinOPType;			/*!<possible values from GPIO Pin Output type >*/
	uint8_t GPIO_PinAltFuncMode;	/*!<possible values from @GPIO_PINMODES >*/


}GPIO_PinConfig_t;


typedef struct{

	GPIO_RegDef_t *pGPIOx;  /*this pointer holds the GPIO base address*/
	GPIO_PinConfig_t GPIO_PinConfig;  /*this holds gpio pin configuration settings*/

}GPIO_Handle_t;


/**GPIO Pin Numbers **/
#define GPIO_PIN_NUM_0			0
#define GPIO_PIN_NUM_1			1
#define GPIO_PIN_NUM_2			2
#define GPIO_PIN_NUM_3			3
#define GPIO_PIN_NUM_4			4
#define GPIO_PIN_NUM_5			5
#define GPIO_PIN_NUM_6			6
#define GPIO_PIN_NUM_7			7
#define GPIO_PIN_NUM_8			8
#define GPIO_PIN_NUM_9			9
#define GPIO_PIN_NUM_10			10
#define GPIO_PIN_NUM_11			11
#define GPIO_PIN_NUM_12			12
#define GPIO_PIN_NUM_13			13
#define GPIO_PIN_NUM_14			14
#define GPIO_PIN_NUM_15			15



/** GPIO Pin Modes from MCU reference manual**/
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/** GPIO Pin Output type from MCU reference manual**/
#define GPIO_OUT_TYPE_PP		0
#define GPIO_OUT_TYPE_OD		1

/** GPIO Pin Speeds from MCU reference Manual **/
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MED			1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/** GPIO Pin pullup/pulldown config from MCU reference Manual **/
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2






/*******************************************************************************
 *
 * 							APIs supported by this driver
 *
 *
 * ****************************************************************************/

/* Peripheral Clock Control
 * Inputs: Base Address, Enable or Disable*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*  Init and De-init*/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*Data control*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,  uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*Interrupt Control*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */

