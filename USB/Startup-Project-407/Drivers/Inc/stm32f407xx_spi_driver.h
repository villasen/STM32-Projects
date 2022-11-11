/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Martin villasenor
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"



/* SPI Configuration Pin structure */
typedef struct
{
	/* Pins can hold values 0 to 15 */
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}SPI_PinConfig_t;



/* SPI Configuration structure */
typedef struct
{





}SPI_Config_t;


/* Handle structure for SPI Peripheral  */
typedef struct
{
	/*Pointer to hold the base address of GPIO */
	SPI_RegDef_t     *pSPIx;  /* This holds the base address of the SPI peripheral */
	SPI_Config_t      SPIConfig; /*This holds SPI pin configuration settings*/


}SPI_Handle_t;











#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
