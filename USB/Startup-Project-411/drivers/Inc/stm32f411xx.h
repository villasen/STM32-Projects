/*
 * stm32f411xx.h
 *
 *  Created on: Oct 21, 2022
 *      Author: MARTIN VILLASENOR
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_


#include  <stdint.h>

#define __vo                 volatile


/*************START:  PROCESSOR SPECIFIC DETAILS******************************************
 *
 *
 */

/* 		 ARM CORTEX M4 Processor  NVIC ISERx Register Addresses  */
#define  NVIC_ISER0         ( (__vo uint32_t*)0xE000E100 )
#define  NVIC_ISER1         ( (__vo uint32_t*)0XE000E104 )
#define  NVIC_ISER2         ( (__vo uint32_t*)0XE000E108 )
#define  NVIC_ISER3         ( (__vo uint32_t*)0XE000E10C )

/* 		 ARM CORTEX M4 Processor  NVIC ICERx Register Addresses */
#define  NVIC_ICER0         ( (__vo uint32_t*)0xE000E180 )
#define  NVIC_ICER1         ( (__vo uint32_t*)0XE000E184 )
#define  NVIC_ICER2         ( (__vo uint32_t*)0XE000E188 )
#define  NVIC_ICER3         ( (__vo uint32_t*)0XE000E18C )

/*    ARM Cortex M4 Processor Priority Address Calculation*/
#define  NVIC_PR_BASE_ADDRS  ( (__vo uint32_t*)0XE000E400 )

#define  NO_PR_BITS_IMPLEMENTED      4
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


/* Base addresses for AHB1 Peripherals */

#define GPIOA_BASEADDR      (AHB1PERIPH_BASEADDR + 0X0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASEADDR + 0X0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASEADDR + 0X0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASEADDR + 0X0C00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASEADDR + 0X1000)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASEADDR + 0X1400)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASEADDR + 0X1800)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASEADDR + 0X1C00)
#define GPIOI_BASEADDR      (AHB1PERIPH_BASEADDR + 0X2000)

/* Base address for RCC Peripheral */

#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0X3800)


/* Base addresses for APB1 Peripherals */

#define I2C1_BASEADDR       (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR       (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR       (APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR       (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR       (APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR     (APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR     (APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR      (APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR      (APB1PERIPH_BASEADDR + 0x5000)

/* Base addresses for APB2 Peripherals */

#define SPI1_BASEADDR       (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR       (APB2PERIPH_BASEADDR + 0x3400)
#define USART1_BASEADDR     (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR     (APB2PERIPH_BASEADDR + 0x1400)
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800)


/* peripheral register definition structure for GPIO */

typedef struct
{
	__vo uint32_t  MODER;      /* GPIO port mode register. Address offset: 0x00 */
	__vo uint32_t  OTYPER;     /* GPIO port output type register.  Address offset: 0x04*/
	__vo uint32_t  OSPEEDR;   /* GPIO port output speed register. Address offset: 0x08 */
	__vo uint32_t  PUPDR;     /* GPIO port pull-up/pull-down register. Address offset: 0x0C */
	__vo uint32_t  IDR;       /* GPIO port input data register. Address offset: 0x10 */
	__vo uint32_t  ODR;      /* GPIO port output data register . Address offset: 0x14 */
	__vo uint32_t  BSRR;     /* GPIO port bit set/reset register. Address offset: 0x18 */
	__vo uint32_t  LCKR;     /* GPIO port configuration lock register. Address offset: 0x1C */
	__vo uint32_t  AFR[2];  /* GPIO alternate function low register . Address offset: 0x20 */

} GPIO_RegDef_t;


/* peripheral register definition structure for EXTI */

typedef struct
{
	__vo uint32_t  IMR;     /* Interrupt mask register. Address offset: 0x00 */
	__vo uint32_t  EMR;     /* Event mask register.  Address offset: 0x04*/
	__vo uint32_t  RTSR;    /* Rising trigger selection register. Address offset: 0x08 */
	__vo uint32_t  FTSR;    /* Falling trigger selection register. Address offset: 0x0C */
	__vo uint32_t  SWIER;   /* Software interrupt event register. Address offset: 0x10 */
	__vo uint32_t  PR;      /* Pending register. Address offset: 0x14 */

}EXTI_RedDef_t;


/* peripheral register definition structure for SYSCGF */

typedef struct
{

	__vo uint32_t MEMRMP;        /* memory remap register. Offset: 0x00 */
	__vo uint32_t PCM;           /* peripheral mode configuration register. Offset: 0x04 */
	__vo uint32_t EXTICR[4];     /* external interrupt configuration register 1,2,3,4. Offset: 0x08 - 0x14 */
	     uint32_t Reserved1[2];    /* Offset: 0x18 - 0x1C */
	__vo uint32_t CMPCR;          /* Compensation cell control register. Offset: 0x20 */
         uint32_t Reserved2[2];    /* Offset: 0x24 - 0x28 */
  //  __vo uint32_t CFGR;           /* Offset: 0x2C */

}SYSCFG_RedDef_t;


/* peripheral register definition structure for RCC */

typedef struct
{
	__vo uint32_t  CR;      /* clock control register. Address offset: 0x00 */
	__vo uint32_t  PLLCFGR;     /* PLL configuration register .  Address offset: 0x04*/
	__vo uint32_t  CFGR;   /* clock configuration register. Address offset: 0x08 */
	__vo uint32_t  CIR;         /*  clock interrupt register. 0X0C */
	__vo uint32_t  AHB1RSTR;     /* AHB1 peripheral reset register. Address offset: 0x10 */
	__vo uint32_t  AHB2RSTR;       /* AHB2 peripheral reset register. Address offset: 0x14 */
	__vo uint32_t  AHB3RSTR;      /* AHB3 peripheral reset register . Address offset: 0x18 */
	     uint32_t  Reserved0;     /* Address offset: 0x1C */
	__vo uint32_t  APB1RSTR;     /* APB1 peripheral reset register. Address offset: 0x20 */
	__vo uint32_t  APB2RSTR;    /* APB2 peripheral reset register . Address offset: 0x24 */
	     uint32_t  Reserved1;     /* Address offset: 0X28 */
	     uint32_t  Reserved2;    /* Address offset: 0X2C */
	__vo uint32_t  AHB1ENR;     /* AHB1 peripheral clock register. Address offset: 0X30.  */
	__vo uint32_t  AHB2ENR;      /* AHB2 peripheral clock enable register. Address offset: 0X34 */
	__vo uint32_t  AHB3ENR;     /* AHB3 peripheral clock enable register. Address offset:  0X38 */
	     uint32_t  Reserved3;       /* 0X3c */
	__vo uint32_t  APB1ENR;     /* APB1 peripheral clock enable register . Address offset:  Address offset: 0X40 */
	__vo uint32_t  APB2ENR;      /* APB2 peripheral clock enable register. Address offset: Address offset:  0X44 */
         uint32_t  Reserved4;     /* Address offset: Address offset:  0X48 */
         uint32_t  Reserved5;    /* Address offset: Address offset: 0X4C */
	__vo uint32_t  AHB1LPENR;     /* AHB1 peripheral clock enable in low power mode register. Address offset: 0X50 */
	__vo uint32_t  AHB2LPENR;	  /* AHB2 peripheral clock enable in low power mode register. Address offset: 0x54 */
	__vo uint32_t  AHB3LPENR;     /* AHB3 peripheral clock enable in low power mode register. Address offset: 0X58 */
         uint32_t  Reserved6;       /*Address offset:  0X5C */
    __vo uint32_t  APB1LPENR;     /* APB1 peripheral clock enable in low power mode register. Address offset: 0X60 */
    __vo uint32_t  APB2LPENR;	  /* APB2 peripheral clock enable in low power mode register. Address offset: 0x64 */
         uint32_t  Reserved7;       /* Address offset: 0X68 */
         uint32_t  Reserved8;       /* Address offset: 0X6C */
    __vo uint32_t  BDCR;     /* Backup domain control register. Address offset: 0X70 */
    __vo uint32_t  CSR;     /*  clock control & status register . Address offset:  0X74 */
         uint32_t  Reserved9;       /* Address offset: 0X78 */
         uint32_t  Reserved10;       /* Address offset: 0X7C */
    __vo uint32_t  SSCGR;     /* spread spectrum clock generation register. Address offset: 0X80 */
    __vo uint32_t  PLLI2SCFGR;     /* PLLI2S configuration register. Address offset: 0X84 */


} RCC_RegDef_t;


/* GPIO Peripheral accesses addresses */

#define GPIOA    ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB    ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC    ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD    ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE    ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF    ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG    ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH    ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI    ((GPIO_RegDef_t*)GPIOI_BASEADDR)


/* RCC Peripheral access address */

#define RCC      ((RCC_RegDef_t*)RCC_BASEADDR)

/* EXTI access address */
#define EXTI     ((EXTI_RedDef_t*)EXTI_BASEADDR)

/* EXTI access address */
#define SYSCFG   ((SYSCFG_RedDef_t*)SYSCFG_BASEADDR)

/* Clock Enable Macros for GPIOx peripherals */

#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1<<7))
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |= (1<<8))


/* Clock Enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1<<23))

/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1<<15))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1<<14))

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN()  (RCC->APB2ENR) |= (1<<4))
#define USART2_PCLK_EN()  (RCC->APB1ENR  |= (1<<17))
#define USART3_PCLK_EN()  (RCC->APB1ENR  |= (1<<18))
#define UART4_PCLK_EN()   (RCC->APB1ENR  |= (1<<19))
#define UART5_PCLK_EN()   (RCC->APB1ENR  |= (1<<20))
#define USART6_PCLK_EN()  (RCC->APB2ENR) |= (1<<5))


/* Clock Enable Macros for SYSCFG peripheral */
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR |= (1<<14))


/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<7))
#define GPIOI_PCLK_DI()  (RCC->AHB1ENR &= ~(1<<8))


/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<23))

/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<15))
#define SPI3_PCLK_DI()   (RCC->APB1ENR &= ~(1<<14))

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()  (RCC->APB2ENR) &= ~(1<<4))
#define USART2_PCLK_DI()  (RCC->APB1ENR  &= ~(1<<17))
#define USART3_PCLK_DI()  (RCC->APB1ENR  &= ~(1<<18))
#define UART4_PCLK_DI()   (RCC->APB1ENR  &= ~(1<<19))
#define UART5_PCLK_DI()   (RCC->APB1ENR  &= ~(1<<20))
#define USART6_PCLK_DI()  (RCC->APB2ENR) &= ~(1<<5))


/* Clock Disable Macros for SYSCFG peripheral */
#define SYSCFG_PCLK_DI()  (RCC->APB2ENR) &= ~(1<<14))

/* Macros to reset GPIOx peripherals */  // (RCC->AHB1ENR &= ~(1<<0))
#define GPIOA_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<0));  (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<1));  (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<2));  (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<3));  (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<4));  (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<5));  (RCC->AHB1RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<6));  (RCC->AHB1RSTR &= ~(1<<6));}while(0)
#define GPIOH_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<7));  (RCC->AHB1RSTR &= ~(1<<7));}while(0)
#define GPIOI_REG_RESET()   do{(RCC->AHB1RSTR |= (1<<8));  (RCC->AHB1RSTR &= ~(1<<8));}while(0)



#define GPIO_BASEADDR_TO_CODE(x)        ((x == GPIOA)?0 :\
		                                 (x == GPIOB)?1 :\
		                                 (x == GPIOC)?2 :\
                                         (x == GPIOD)?3 :\
                                         (x == GPIOE)?4 :\
                                         (x == GPIOF)?5 :\
                                         (x == GPIOG)?6 :\
                                         (x == GPIOH)?7 :0)


/*
 *  IRQ(Interrupt Request) Number or STM32F407x
 *  Positions in NVIC table
 */
#define IRQ_NO_EXTI0       6
#define IRQ_NO_EXTI1       7
#define IRQ_NO_EXTI2       8
#define IRQ_NO_EXTI3       9
#define IRQ_NO_EXTI4       10
#define IRQ_NO_EXTI9_5     23
#define IRQ_NO_EXTI15_10   40


/*
 *  IRQ Priority levels
 */
#define NVIC_IRQ_PRI0        0
#define NVIC_IRQ_PRI1        1
#define NVIC_IRQ_PRI2        2
#define NVIC_IRQ_PRI3        3
#define NVIC_IRQ_PRI4        4
#define NVIC_IRQ_PRI5        5
#define NVIC_IRQ_PRI6        6
#define NVIC_IRQ_PRI7        7
#define NVIC_IRQ_PRI8        8
#define NVIC_IRQ_PRI9        9
#define NVIC_IRQ_PRI10       10
#define NVIC_IRQ_PRI11       11
#define NVIC_IRQ_PRI12       12
#define NVIC_IRQ_PRI13       13
#define NVIC_IRQ_PRI14       14
#define NVIC_IRQ_PRI15       15



/* Generic Macros  */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET





#include "stm32f411xx_gpio_driver.h"

#endif /* INC_STM32F411XX_H_ */
