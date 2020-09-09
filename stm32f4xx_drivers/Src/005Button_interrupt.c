/*
 * 005 Button_interrupt.c
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
	GpioBtn.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);



	//IRQ Configuration
	GPIO_IRQPriority_Config(IRQ_NO_EXTI15_10, 15);
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1){}







}

//ISR
void EXTI15_10_IRQHandler (void)
{
	GPIO_IRQHandling(GPIO_PIN_NUM_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_0);
}

