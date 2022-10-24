/*
 * led_button_external.c
 *
 *  Created on: Oct 23, 2022
 *      Author: Martin Villasenor
 */


#include "stm32f407xx.h"

#define HIGH         1
#define LOW          0

#define BTN_PRESSED   HIGH

void delay(void)
{

	for (uint32_t i=0; i< 500000/2 ; i++);
}

int main(void)
{

	// create handle
	GPIO_Handle_t  GpioLed, GpioButton;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioLed.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIOPinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEDD_FAST ;


	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);


	GpioButton.pGPIOx = GPIOB;
	GpioButton.GPIOPinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GpioButton.GPIOPinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioButton.GPIOPinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;//GPIO_NO_PUPD;
	GpioButton.GPIOPinConfig.GPIO_PinSpeed = GPIO_SPEDD_FAST ;


	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioButton);



	while(1)
	{

		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_10) == BTN_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_10);
		}



	}

	return 0;
}

