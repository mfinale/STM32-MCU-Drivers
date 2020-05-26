/*
 * 007_spi_tx_arduino.c
 *
 *  Created on: May 19, 2020
 *      Author: Michael R. Finale
 *      Exercise to test SPI transmit with STM32 nucleo as master
 *      and arduino uno as slave/receiver.
 *
 *      See SPI_arduino_setup.png for setup
 */


#include "stm32f446E.h"
#include  "stm32f446xx_gpio_driver.h"
#include "string.h"

void delay(void)
{
	for (uint32_t i=0; i<500000/2 ; i++);
}




//configure GPIO pins to use for SPI. See alternate function table in datasheet.
//formerly used GPIO port b for spi2 however pins are disconnected on nucleo board.
//switched
//PB15 --> MOSI
//PB14 --> MISO
//PB13 --> SCLK
//PB12 --> NSS
// ALT function mode: 5
void SPI2_GPIO_Init(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFuncMode= 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;
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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GPIO_Init(&SPIPins);



}

//init SPI2 peripheral
void SPI2_Init(void){
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx=SPI2;
	SPI2handle.SPI_Config.SPI_Busconfig=SPI_BUS_CONFIG_FD;
	SPI2handle.SPI_Config.SPI_Devicemode=SPI_DEVICE_MASTER;
	SPI2handle.SPI_Config.SPI_Speed=SPI_SCLK_SPEED_DIV8;//generates 2 Mhz
	SPI2handle.SPI_Config.SPI_DFF=SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPOL=SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_CPHA=SPI_CPPHA_LOW;
	SPI2handle.SPI_Config.SPI_SSM=SPI_SSM_DI;//disable SW slave management. Will use HW for this exercise
	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;

	//button configuration
	GpioBtn.pGPIOx =GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);



}

int main(void)
{




	char user_data[] = "hello world";//string length within 255 characters since using 1 byte
	GPIO_ButtonInit();
	SPI2_GPIO_Init();
	SPI2_Init();

	/*
	 * NSS output enable (SSM=0,SSOE = 1): this configuration is only used when the
	 * MCU is set as master. The NSS pin is managed by the hardware. The NSS signal
	 * is driven low as soon as the SPI is enabled in master mode (SPE=1), and is kept
	 * low until the SPI is disabled (SPE =0).
	 */
	SPI_SSIConfig(SPI2, DISABLE);
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		//send spi when reading LOW from button
		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_13)==1);
		delay();//debounce prevention

		//enable SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);
		//first send #of bytes information for slave device to expect
		uint8_t dataLen =  strlen(user_data);
		SPI_SendData(SPI2, &dataLen,  1);
		//send data
		SPI_SendData(SPI2, (uint8_t *)user_data,  strlen(user_data));
		//check if SPI is still sending data
		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));
		//disable peripheral when done
		SPI_PeripheralControl(SPI2,DISABLE);


	}

	return 0;
}
