/*
 * 008_spi_txrx_arduino.c
 *
 *  Created on: May 26, 2020
 *      Author: Michael R. Finale
 *
 *      Refer to SPI_txrx_arduino_setup.png for hardware setup.
 *
 *      Exercise to test SPI Receive with STM32 nucleo as master
 *      and arduino uno as slave/transmitter. Sends commands
 *      to Arduino which is running code that responds to said commands:
 *
 *      CMD_LED_CTRL <PIN NO> <VALUE> -> CONTROL LED ON ARDUINO
 *      CMD_SENSOR_READ <ANALOG PIN NO> -> READ FROM PIN ON ARDUINO
 *      CMD_LED_READ <PIN NO> -> READ STATUS OF LED
 *      CMD_PRINT <LEN> <MESSAGE> - > SEND MESSAGE TO ARDUINO
 *      CMD_ID_READ -> READ ID OF ARDUINO BOARD
 *
 *      1. Use SPI Full duplex mode
 *      2. ST board will be in Master mode and Arduino in Slave mode
 *      3. DFF will be 9 (8buts)
 *      4. Hardware slave management enabled (SSM=0)
 *      5. SCLK Speed = 2 Mhz
 *
 *
 *
 */


#include "stm32f446E.h"
#include  "stm32f446xx_gpio_driver.h"
#include "string.h"

extern void initialise_monitor_handles(void);


//command codes
#define COMMAND_LED_CTRL			0X50
#define COMMAND_SENSOR_READ			0X51
#define COMMAND_LED_READ			0X52
#define COMMAND_PRINT				0X53
#define COMMAND_ID_READ				0X54

#define LED_ON						1
#define LED_OFF						0

//arduino pins
#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4
#define LED_Pin 					9


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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	GPIO_Init(&SPIPins);
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


uint8_t SPI_Verifyresponse(uint8_t response)
{
	if(response==0xF5)
	{
		return 1;
	}

	return 0;
}
int main(void)
{


	uint8_t dummy_write = 0xFF;
	uint8_t dummy_read;
	uint8_t	ackbyte;
	uint8_t analog_read;
	uint8_t args[2];

	initialise_monitor_handles();//semihosting for using printing in eclipse console

	printf("application is running.\n");

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


		//1.)send LED control command
		uint8_t commandcode= COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Receive ack byte
		//SPI communication will not occur on its own unless data is being sent from master.
		//therefore we need to send a dummy byte to push the data from the slave to master
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//verify ack and then send arguments for led ccontrol
		if (SPI_Verifyresponse(&ackbyte))
		{
			args[0]=LED_Pin;
			args[1]=LED_ON;
			SPI_SendData(SPI2, args, 2);
		}


		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_13)==1);
		delay();


		//2.) send sensor read command
		commandcode= COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandcode, 1);
		//dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		//Receive ack byte
		SPI_SendData(SPI2, &dummy_write, 1);
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		//verify ack and then send arguments for sensor read
		if (SPI_Verifyresponse(&ackbyte))
		{
			args[0]=ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			//clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//delay to let adc process
			delay();

			//receive analog data
			SPI_SendData(SPI2, &dummy_write, 1);
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("COMMAND_SENSOR_READ %d.\n",analog_read);

		}

		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_13)==1);
				delay();


		//3.) send COMMAND_ID_READ	 command
//		commandcode= COMMAND_ID_READ;
//		SPI_SendData(SPI2, &commandcode, 1);
//		//dummy read to clear off the RXNE
//		SPI_ReceiveData(SPI2, &dummy_read, 1);
//
//		//Receive ack byte
//		SPI_SendData(SPI2, &dummy_write, 1);
//		SPI_ReceiveData(SPI2, &ackbyte, 1);
//
//		uint8_t id[11];
//		uint32_t i=0;
//		if(SPI_Verifyresponse(&ackbyte))
//		{
//			//read 10 bytes id from the slave
//			for(  i = 0 ; i < 10 ; i++)
//			{
//				//send dummy byte to fetch data from slave
//				SPI_SendData(SPI2,&dummy_write,1);
//				SPI_ReceiveData(SPI2,&id[i],1);
//			}
//
//			id[11] = '\0';
//
//			printf("COMMAND_ID : %s \n",id);
//
//		}





		SPI_PeripheralControl(SPI2,DISABLE);


	}

	return 0;
}
