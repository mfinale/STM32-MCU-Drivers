/*
 * 006SPI_TX_test.c
 *
 *  Created on: May 12, 2020
 *      Author: Michael R Finale
 *      Exercise to test SPI 2 peripheral on stm32f446re mcu.
 *      Code only uses MISO and SCLK lines. MOSI and NSS are disconnected.
 */

#include "stm32f446E.h"
#include "string.h"

//configure GPIO pins to use for SPI. See alternate function table in datasheet.
//formerly used GPIO port b for spi2 however pins are disconnected on nucleo board.
//switched
//PB15 --> MOSI
//PB14 --> MISO
//PB10 --> SCLK
//PB12 --> NSS
// ALT function mode: 5
void SPI2_GPIO_Init(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//sclk pin
	//TO DO: use different pins
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GPIO_Init(&SPIPins);
	//TO DO: use different pins
	//MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&SPIPins);
	//MISO pin
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	//GPIO_Init(&SPIPins);
	//NSS pin
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	//GPIO_Init(&SPIPins);



}

//init SPI2 peripheral
void SPI2_Init(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPI_Config.SPI_Busconfig=SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_Devicemode=SPI_DEVICE_MASTER;
	SPI2handle.SPI_Config.SPI_Speed=SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPI_Config.SPI_DFF=SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA=SPI_CPPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM=SPI_SSM_EN;
	SPI_Init(&SPI2handle);
}

int main(void)
{
	char user_data[] = "hello world";

	SPI2_GPIO_Init();
	SPI2_Init();
	//make NSS signal internal high
	SPI_SSIConfig(SPI2, ENABLE);
	//enable SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);
	//need to have data to send in order to see the clock pulses.
	//sclk is a way to tell the slave when to sample the data
	SPI_SendData(SPI2, (uint8_t *)user_data,  strlen(user_data));
	//check if SPI is still sending data
	while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
	//disable peripheral
	SPI_PeripheralControl(SPI2,DISABLE);


	while(1);

	return 0;
}
