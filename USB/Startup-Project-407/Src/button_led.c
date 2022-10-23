/*
 * button_led.c
 *
 *  Created on: Oct 23, 2022
 *      Author: Martin Villasenor
 */


#include "stm32f407xx.h"

void delay(void)
{

	for (uint32_t i=0; i< 500000/2 ; i++);
}

int main(void)
{

	// create handle
	GPIO_Handle_t  GpioLed, GpioButton;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEDD_FAST ;


	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioLed);


	GpioButton.pGPIOx = GPIOA;
	GpioButton.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioButton.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioButton.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEDD_FAST ;


	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioButton);



	while(1)
	{

		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_15);
		}



	}

	return 0;
}
