/*
 * stm32f74xxx_gpio_driver.h
 *
 *  Created on: Nov 1, 2022
 *      Author: Martin Villasenor
 */

#ifndef INC_STM32F74XXX_GPIO_DRIVER_H_
#define INC_STM32F74XXX_GPIO_DRIVER_H_

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




#endif /* INC_STM32F74XXX_GPIO_DRIVER_H_ */
