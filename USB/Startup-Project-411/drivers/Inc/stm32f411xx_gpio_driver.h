/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Oct 30, 2022
 *      Author: Martin Villasenor
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_


/* GPIO Configuration Pin structure */
typedef struct
{
	/* Pins can hold values 0 to 15 */
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;




/* Handle structure for a GPIO pin */
typedef struct
{

	/*Pointer to hold the base address of GPIO */
	GPIO_RegDef_t     *pGPIOx;  /* This holds the base address of the GPIO port to which this pin belongs to */
	GPIO_PinConfig_t  GPIOPinConfig; /*This holds GPIO pin configuration settings*/


}GPIO_Handle_t;



/* GPIO pin possible modes   */
#define GPIO_MODE_IN       0
#define GPIO_MODE_OUT      1
#define GPIO_MODE_ALTFN    2
#define GPIO_MODE_ANALOG   3
#define GPIO_MODE_IT_FT    4
#define GPIO_MODE_IT_RT    5
#define GPIO_MODE_IT_RFT   6


/* GPIO pin possible output types   */
#define GPIO_OP_TYPE_PP    0
#define GPIO_OP_TYPE_OD    1

/* GPIO pin possible output speeds   */
#define GPIO_SPEDD_LOW     0
#define GPIO_SPEDD_MEDIUM  1
#define GPIO_SPEDD_FAST    2
#define GPIO_SPEDD_HIGH    3

/* GPIO pin pull up and pull down configuration macros   */
#define GPIO_NO_PUPD       0
#define GPIO_PIN_PU        1
#define GPIO_PIN_PD        2


/* @GPIO_PIN_NUMBERS
 * GPIO Pin Numbers
 */
#define GPIO_PIN_NO_0      0
#define GPIO_PIN_NO_1      1
#define GPIO_PIN_NO_2      2
#define GPIO_PIN_NO_3      3
#define GPIO_PIN_NO_4      4
#define GPIO_PIN_NO_5      5
#define GPIO_PIN_NO_6      6
#define GPIO_PIN_NO_7      7
#define GPIO_PIN_NO_8      8
#define GPIO_PIN_NO_9      9
#define GPIO_PIN_NO_10     10
#define GPIO_PIN_NO_11     11
#define GPIO_PIN_NO_12     12
#define GPIO_PIN_NO_13     13
#define GPIO_PIN_NO_14     14
#define GPIO_PIN_NO_15     15







/************************************************************************************
*
*                      APIs supported by this driver
*           For more information about these APIs  check the Function Definitions
************************************************************************************/

/* Peripheral Clock setup  */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Init and DeInit GPIO  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/* data read and write GPIOs  */
uint8_t  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void     GPIO_WriteFromOutpuPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void     GPIO_WriteFromOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void     GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/* IRQ configuration and ISR handling  */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDis);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriotiy);





#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
