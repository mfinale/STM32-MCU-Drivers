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

	//use uint 8 as there is a maximum of 15 GPIO pins. uint_32 would be wasteful */
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



#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
