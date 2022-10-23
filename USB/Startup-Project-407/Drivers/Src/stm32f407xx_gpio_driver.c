/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Oct 22, 2022
 *      Author: Martin Villasenor
 */

#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

/******************************************************************************
 *
 * @fn         GPIO_PeriClockControl
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{

        if(pGPIOx == GPIOA)
        {
			GPIOA_PCLK_EN();
        }
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}


	}
	else
	{

        if(pGPIOx == GPIOA)
        {
        	GPIOA_PCLK_DI();
        }
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}// end of else
}
/******************************************************************************
 *
 * @fn         GPIO_Init
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

/* 1. Configure the mode of the GPIO pin
 */
	uint32_t temp = 0;  //temp register

	if(pGPIOHandle->GPIOPinConfig.GPIO_PinMode < GPIO_MODE_ANALOG)
	{
		// non-interrupts
		temp = (pGPIOHandle->GPIOPinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else   //interrupt modes
	{




	}

	temp = 0;

/* 2. Configure the speed
*/
	temp = (pGPIOHandle->GPIOPinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

/* 3. Configure the pullup pulldown (pupd) settings
*/
	temp = (pGPIOHandle->GPIOPinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIOPinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

/*  4. Configure the output type
 * */
	temp = (pGPIOHandle->GPIOPinConfig.GPIO_PinOPType << (pGPIOHandle->GPIOPinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;


/* Configure the alt functionality
 */
	if(pGPIOHandle->GPIOPinConfig.GPIO_PinAltFunMode == GPIO_MODE_ALTFN)
	{

		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIOPinConfig.GPIO_PinNumber / 8; //Gets the Low or High AFR
		temp2 = pGPIOHandle->GPIOPinConfig.GPIO_PinNumber % 8; //Gets the pin position

		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIOPinConfig.GPIO_PinAltFunMode << (4 * temp2);

	}


}


/******************************************************************************
 *
 * @fn         GPIO_DeInit
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){



}




/******************************************************************************
 *
 * @fn         GPIO_ReadFromInputPin
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	return 1;
}

/******************************************************************************
 *
 * @fn         GPIO_ReadFromInputPort
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){



	return 1;

}


/******************************************************************************
 *
 * @fn         GPIO_WriteFromOutpuPin
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void     GPIO_WriteFromOutpuPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value){



}

/******************************************************************************
 *
 * @fn         GPIO_WriteFromOutputPort
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void     GPIO_WriteFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value){




}


/******************************************************************************
 *
 * @fn         GPIO_ToggleOutputPin
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void     GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){





}


/******************************************************************************
 *
 * @fn         GPIO_IRQConfig
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDis){




}

/******************************************************************************
 *
 * @fn        GPIO_IRQHandling
 *
 * @brief
 *
 * @parma[i]
 *
 * @return
 *
 * @note
 *
 *
 *
 ********************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber){




}
