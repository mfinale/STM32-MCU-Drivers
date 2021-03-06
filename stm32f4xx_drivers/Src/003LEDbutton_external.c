/*
 * 003LEDbutton_external.c
 *
 *  Created on: Apr 27, 2020
 *      Author: User
 */



#include "stm32f446E.h"



void delay(void)
{
	for (uint32_t i=0; i<500000; i++);
}




int main(void)
{

	GPIO_Handle_t Gpioled;

	GPIO_Handle_t GpioBtn;

	//led configuration
	Gpioled.pGPIOx =GPIOA;
	Gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_0;
	Gpioled.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Gpioled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	Gpioled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&Gpioled);



	//button configuration
	GpioBtn.pGPIOx =GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);







	while (1)
	{
		if (!GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NUM_12))
				{
					delay();//debounce prevention
					GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_0);
				}


	}



}
