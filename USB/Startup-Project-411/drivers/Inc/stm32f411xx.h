/*
 * stm32f411xx.h
 *
 *  Created on: Oct 21, 2022
 *      Author: MARTIN VILLASENOR
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_


/* Base address for flash and SRAM */

#define FLASH_BASEADDR      0X08000000U     /*Base address for flash area*/
#define SRAM_BASEADDR       0X20000000U    /*Base address for SRAM1 are.*/
#define SRAM                SRAM_BASEADDR   /* RAM alias */
#define SYSTEM_ADDR         0X1FFF0000U    /* Base address for System area or ROM area. */
#define ROM_BASEADDR        SYSTEM_ADDR    /* ROM alias */


/* Base addresses for the Bus domains */

#define PERIPH_BASEADDR     0X40000000U   /* Base address for Peripheral area */
#define APB1PERIPH_BASEADDR PERIPH_BASE   /* Base address APB1 bus domain */
#define APB2PERIPH_BASEADDR 0X40010000U  /* Base address for APB2 bus domain */
#define AHB1PERIPH_BASEADDR 0X40020000U  /* Base address for AHB1 bus domain */
#define AHB2PERIPH_BASEADDR 0X50000000U  /* Base address for AHB2 bus domain */


#endif /* INC_STM32F411XX_H_ */
