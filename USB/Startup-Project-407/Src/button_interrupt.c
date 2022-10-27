/*
 * button_interrupt.c
 *
 *  Created on: Oct 25, 2022
 *      Author: Martin Villasenor
 */


#include "stm32f407xx.h"
#include <string.h>

void delay(void)
{

	for (uint32_t i=0; i< 500000/2 ; i++);
}

int main(void)
{

	// create handle
	GPIO_Handle_t  GpioLed, GpioButton;

	// initialize the structures
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioButton, 0, sizeof(GpioButton));

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEDD_LOW ;


	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);


	GpioButton.pGPIOx = GPIOD;
	GpioButton.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioButton.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;//GPIO_NO_PUPD;
	GpioButton.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEDD_FAST ;


	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioButton);


	//IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);


	while(1);


	return 0;
}



// ISR for button press on EXTI on D5
void EXTI9_5_IRQHandler(void){

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}
