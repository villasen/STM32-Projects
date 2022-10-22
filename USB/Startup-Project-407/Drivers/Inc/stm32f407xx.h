/*
 * stm32f407xx.h
 *
 *  Created on: Oct 21, 2022
 *      Author: martin
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include  <stdint.h>

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
	volatile uint32_t  MODER;      /* GPIO port mode register. Address offset: 0x00 */
	volatile uint32_t  OTYPER;     /* GPIO port output type register.  Address offset: 0x04*/
	volatile uint32_t  OSPEEDR;   /* GPIO port output speed register. Address offset: 0x08 */
	volatile uint32_t  PUPDR;     /* GPIO port pull-up/pull-down register. Address offset: 0x0C */
	volatile uint32_t  IDR;       /* GPIO port input data register. Address offset: 0x10 */
	volatile uint32_t  ODR;      /* GPIO port output data register . Address offset: 0x14 */
	volatile uint32_t  BSRR;     /* GPIO port bit set/reset register. Address offset: 0x18 */
	volatile uint32_t  LCKR;     /* GPIO port configuration lock register. Address offset: 0x1C */
	volatile uint32_t  AFR[2];  /* GPIO alternate function low register . Address offset: 0x20 */

} GPIO_RegDef_t;


/* peripheral register definition structure for RCC */

typedef struct
{
	volatile uint32_t  CR;      /* GPIO port mode register. Address offset: 0x00 */
	volatile uint32_t  PLLCFGR;     /* GPIO port output type register.  Address offset: 0x04*/
	volatile uint32_t  CFGR;   /* GPIO port output speed register. Address offset: 0x08 */
	volatile uint32_t  CIR;         /*   0X0C */
	volatile uint32_t  AHB1RSTR;     /* GPIO port pull-up/pull-down register. Address offset: 0x10 */
	volatile uint32_t  AHB2RSTR;       /* GPIO port input data register. Address offset: 0x14 */
	volatile uint32_t  AHB3RSTR;      /* GPIO port output data register . Address offset: 0x18 */
	         uint32_t  Reserved0;     /* GPIO port bit set/reset register. Address offset: 0x1C */
	volatile uint32_t  APB1RSTR;     /* GPIO port configuration lock register. Address offset: 0x20 */
	volatile uint32_t  APB2RSTR;  /* GPIO alternate function low register . Address offset: 0x24 */
	         uint32_t  Reserved1;     /* 0X28 */
	         uint32_t  Reserved2;    /* 0X2C */
	volatile uint32_t  AHB1ENR;     /* 0X30 */
	volatile uint32_t  AHB2ENR;      /* 0X34 */
	volatile uint32_t  AHB3ENR;     /* 0X38 */
	         uint32_t  Reserved3;       /* 0X3c */
	volatile uint32_t  APB1ENR;     /* 0X40 */
	volatile uint32_t  APB2ENR;      /* 0X44 */
             uint32_t  Reserved4;     /* 0X48 */
             uint32_t  Reserved5;    /* 0X4C */
	volatile uint32_t  AHB1LPENR;     /* 0X50 */
	volatile uint32_t  AHB2LPENR;	  /*0x54 */
	volatile uint32_t  AHB3LPENR;     /* 0X58 */
             uint32_t  Reserved6;       /* 0X5C */
    volatile uint32_t  APB1LPENR;     /* 0X60 */
    volatile uint32_t  APB2LPENR;	  /*0x64 */
             uint32_t  Reserved7;       /* 0X68 */
             uint32_t  Reserved8;       /* 0X6C */
    volatile uint32_t  BDCR;     /* 0X70 */
    volatile uint32_t  CSR;     /* 0X74 */
             uint32_t  Reserved9;       /* 0X78 */
             uint32_t  Reserved10;       /* 0X7C */
    volatile uint32_t  SSCGR;     /* 0X80 */
    volatile uint32_t  PLLI2SCFGR;     /* 0X84 */

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
#define SYSCFG_PCLK_EN()  (RCC->APB2ENR) |= (1<<14))


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



/* Generic Macros  */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET













#endif /* INC_STM32F407XX_H_ */
