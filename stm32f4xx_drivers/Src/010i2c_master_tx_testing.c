/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Jun 17, 2020
 *      Author: Michael Finale
 *      Connections required:
 *      Cn5(10)->d15->PB8->i2c1scl->cn10(3)->arduino a5
		Cn5(9)->d15->PB9->i2c1sda->cn10(5)->arduino a4
		cn5(7)->gnd->arduino gnd
		PB8 in AF4 gives i2c1_scl
		PB9 in AF4 gives i2c1_sda
 *
 */

#include "stm32f446E.h"
#include "string.h"
#define MY_ADDR	0x61



I2C_Handle_t I2C1Handle;

void delay(void)
{
	for (uint32_t i=0; i<500000/2 ; i++);
}

void I2C1_GPIO_Init(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFuncMode= 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl=GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber= 8;
	GPIO_Init(&I2CPins);
	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber= 9;
	GPIO_Init(&I2CPins);


}

//init i2c1 peripheral
void I2C1_Init(void){

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;


	I2C_Init(&I2C1Handle);
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

int main(void){

	//init user button on stm board
	GPIO_ButtonInit();

	I2C1_GPIO_Init();
	I2C1_Init();
	//enable I2C1 peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//send some data on button press (slave address is 0x68 set by arduino)
	uint8_t some_data[]=  "We are testing I1c master Tx\n";

	while (1)
	{
		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_13)==1);
		delay();//debounce prevention
		I2CMasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), 0x68);

	}








}
