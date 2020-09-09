/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: September 4th, 2020
 *
 *  I2C communication between stm 32 and arduino but with Interrupts enabled!
 *
 *      Author: Michael Finale
 *      Connections required:
 *      Cn5(10)->d15->PB8->i2c1scl->cn10(3)->arduino a5
		Cn5(9)->d15->PB9->i2c1sda->cn10(5)->arduino a4
		cn5(7)->gnd->arduino gnd
		PB8 in AF4 gives i2c1_scl
		PB9 in AF4 gives i2c1_sda

		command codes are defined in the code loaded on arduino slave

 *
 */

#include "stm32f446E.h"
#include "string.h"
#include <stdio.h>
#define MY_ADDR	0x61
uint8_t receive_complete = RESET;



I2C_Handle_t I2C1Handle;

//semihosting to print to console
extern void initialise_monitor_handles();

//rcv buffer of 32 bytes
uint8_t rcv_buff[32];


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



/* I2C1 error interrupt: run this whenever I2C error interrupt is generated */
void	I2C1_ER_IRQHandler(void)
{
	I2C_ERROR_IRQHandling(&I2C1Handle);
}
/* I2C2 event interrupt: run this whenever I2C event interrupt is generated */
void	I2C1_EV_IRQHandler(void)
{
	I2C_EVENT_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
	if(AppEv== I2C_EVENT_TX_COMPLETE)
	{
		printf("Tx is complete\n");

	}
	else if (AppEv == I2C_EVENT_RX_COMPLETE) {
		printf("Rx is complete\n");
		//signal the flag once RX is complete
		receive_complete = SET;
	}
	else if (AppEv == I2C_ERROR_AF) {
		printf("ERROR ACK FAILURE\n");
		//close send data and generate stop condition on error
		I2C_CloseSendData(pI2CHandle);
		I2C_GenerateStopCondition(I2C1);
	}

}

int main(void){

	uint8_t commandcode;
	uint8_t length;

	//semihosting to print to console
	initialise_monitor_handles();

	printf("Application is running\n");
	//init user button on stm board
	GPIO_ButtonInit();
	//I2C PIN INIT
	I2C1_GPIO_Init();
	//I2C PERIPHERAL CONFIG
	I2C1_Init();
	//I2C IRQ CONFIG
	I2C_IRQConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQConfig(IRQ_NO_I2C1_ER, ENABLE);
	//enable I2C1 peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//manage acking
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);


	while (1)
	{
		while (GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_13)==1);
		delay();//debounce prevention


		// 1. send command to request length of data to be read from slave when not busy
		commandcode = 0x51;
		while(I2CMasterSendDataIT(&I2C1Handle, &commandcode, 1, 0x68, ENABLE) != I2C_READY);

		// 2. Read response of length command from slave when not busy

		while (I2CMasterRcvDataIT(&I2C1Handle, &length, 1, 0x68,  ENABLE) != I2C_READY);

		// 3. Begin command to read 'length' bytes of data
		commandcode = 0x52;
		while(I2CMasterSendDataIT(&I2C1Handle, &commandcode, 1, 0x68, ENABLE) != I2C_READY);

		while(I2CMasterRcvDataIT(&I2C1Handle, rcv_buff, length, 0x68, DISABLE)!= I2C_READY);

		//need to wait until the RXNE interrupt is finished (RX is done): otherwise will
		//not be ready to print data
		receive_complete=RESET;
		 //printf("receive complete is: %s", receive_complete);
		while(receive_complete!=SET){};
		//get rcv_buff from pointer rather than a return

		 rcv_buff[length+1] = '\0';
		 printf("Data : %s", rcv_buff);
		 receive_complete=RESET;







	}








}
