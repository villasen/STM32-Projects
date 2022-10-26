/*
 * main.c
 *
 *  Created on: Oct 25, 2022
 *      Author: martin
 */

#include "stm32f407xx.h"



int main(void)
{



	return 0;

}



void EXTI0_IRQHandler(void){

	//handle the interrupt
	GPIO_IRQHandling(0);


}
