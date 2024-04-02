/*
 * stm32f070rbx.h
 *
 *  Created on: Feb 9, 2024
 *      Author: User
 */

#ifndef INC_STM32F070RBX_H_
#define INC_STM32F070RBX_H_


#include <stdint.h>

/****************Processor Specific Details***************************
 *
 * ARM Cortex M0 Processor NVIC Register Address
 */
#define NVIC_ISER               ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ICER               ( (volatile uint32_t*)0xE000E180 )
#define NVIC_ISPR               ( (volatile uint32_t*)0xE000E200 )
#define NVIC_ICPR               ( (volatile uint32_t*)0xE000E280 )

#define NVIC_IPR_BASEADDR       ( (volatile uint32_t*)0xE000E400 )

/*
 * ARM Cortex M0 Processor no.of priority bits implemented in priority register
 */
#define NO_PR_BITS_IMPLEMENTED          4




//base address of Flash and SRAM memories

#define FLASH_BASEADDR              0x08000000U
#define SRAM_BASEADDR               0x20000000U
#define ROM_BASEADDR                0x1FFFC800U


//base address of AHBs and APBs

#define APB1_BASEADDR               0x40000000U
#define APB2_BASEADDR               0x40010000U
#define AHB1_BASEADDR               0x40020000U
#define AHB2_BASEADDR               0x48000000U


/*base address of peripherals hanging on AHB1 Bus*/

#define DMA_BASEADDR                (AHB1_BASEADDR + 0x0000)
#define RCC_BASEADDR                (AHB1_BASEADDR + 0x1000)
#define FLASH_Interface_BASEADDR    (AHB1_BASEADDR + 0x2000)
#define CRC_BASEADDR                (AHB1_BASEADDR + 0x3000)

//base address of peripherals hanging on AHB2 bus

#define GPIOA_BASEADDR              (AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR              (AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR              (AHB2_BASEADDR + 0x0800)
#define GPIOD_BASEADDR              (AHB2_BASEADDR + 0x0C00)
#define GPIOF_BASEADDR              (AHB2_BASEADDR + 0x1400)


//base address of peripherals hanging on APB1 bus

#define TIM3_BASEADDR               (APB1_BASEADDR + 0x0000)
#define TIM6_BASEADDR               (APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR               (APB1_BASEADDR + 0x1400)
#define TIM14_BASEADDR              (APB1_BASEADDR + 0x2000)

#define RTC_BASEADDR                (APB1_BASEADDR + 0x2800)
#define WWDG_BASEADDR               (APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR               (APB1_BASEADDR + 0x3000)
#define SPI2_BASEADDR               (APB1_BASEADDR + 0x3800)

#define USART2_BASEADDR             (APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR             (APB1_BASEADDR + 0x4800)
#define USART4_BASEADDR             (APB1_BASEADDR + 0x4C00)

#define I2C1_BASEADDR               (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR               (APB1_BASEADDR + 0x5800)

#define USB_RAM_BASEADDR            (APB1_BASEADDR + 0x6000)
#define PWR_BASEADDR                (APB1_BASEADDR + 0x7000)



//base address of peripherals hanging on APB2 bus

#define SYSCFG_BASEADDR           (APB2_BASEADDR + 0x0000)
#define EXTI_BASEADDR             (APB2_BASEADDR + 0x0400)
#define ADC_BASEADDR              (APB2_BASEADDR + 0x2400)
#define TIM1_BASEADDR             (APB2_BASEADDR + 0x2C00)
#define SPI1_BASEADDR             (APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR           (APB2_BASEADDR + 0x3800)

#define TIM15_BASEADDR            (APB2_BASEADDR + 0x4000)
#define TIM16_BASEADDR            (APB2_BASEADDR + 0x4400)
#define TIM17_BASEADDR            (APB2_BASEADDR + 0x4800)

#define DBGMCU_BASEADDR           (APB2_BASEADDR + 0x5800)



//peripheral register definitions: SYSCFG

typedef struct{
	volatile uint32_t CFGR1;        //Configuration register 1                         address offset 0x00
	volatile uint32_t EXTICR[3];      //External interrupt configuration registers     address offset 0x08 - 0x14
	volatile uint32_t CFGR2;        //Configuration register 2                         address offset 0x18
}SYSCFG_RegDef_t;



//peripheral register definitions: RCC

typedef struct{
	volatile uint32_t CR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t APB1RSTR;
	volatile uint32_t AHBENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t AHBRSTR;
	volatile uint32_t CFGR2;
	volatile uint32_t CFGR3;
	volatile uint32_t CR2;
}RCC_RegDef_t;

//peripheral register definitions: GPIOx


typedef struct{
	volatile uint32_t MODER;    // GPIO port mode register
	volatile uint32_t OTYPER;   // GPIO port output type register
	volatile uint32_t OSPEEDR;  // GPIO port output speed register
	volatile uint32_t PUPDR;    // GPIO port pull-up/pull-down register
	volatile uint32_t IDR;      // GPIO port input data register
	volatile uint32_t ODR;      // GPIO port output data register
	volatile uint32_t BSRR;     // GPIO port bit set/reset register
	volatile uint32_t LCKR;     // GPIO port configuration lock register
	volatile uint32_t AFR[2];   // GPIO alternate function register;  AFR[0] : Low AFR[1] : High
	volatile uint32_t BRR;      // GPIO port bit reset register
}GPIO_RegDef_t;


//peripheral register definitions: I2C

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t TIMINGR;
	volatile uint32_t TIMEOUTR;
	volatile uint32_t ISR;
	volatile uint32_t ICR;
	volatile uint32_t PECR;
	volatile uint32_t RXDR;
	volatile uint32_t TXDR;
}I2C_RegDef_t;


//peripheral register definitions: SPI

typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
}SPI_RegDef_t;



//peripheral register definitions: EXTI

typedef struct{
	volatile uint32_t IMR;           //Interrupt mask register                 address offset 0x00
	volatile uint32_t EMR;           //Event mask register                     address offset 0x04
	volatile uint32_t RTSR;          //Rising trigger selection register       address offset 0x08
	volatile uint32_t FTSR;          //Falling trigger selection register      address offset 0x0C
	volatile uint32_t SWIER;         //Software interrupt event register       address offset 0x10
	volatile uint32_t PR;            //Pending register                        address offset 0x14
}EXTI_RegDef_t;




// peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)


#define GPIOA  ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB  ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC  ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD  ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOF  ((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define I2C1   ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2   ((I2C_RegDef_t*)I2C2_BASEADDR)

#define SPI1   ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2   ((SPI_RegDef_t*)SPI2_BASEADDR)

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI   ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*Clock enable macros for SYSCFG Peripherals*/

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1<<0))

/*Clock enable macros for GPIOx Peripherals*/

#define GPIOA_PCLK_EN()  (RCC->AHBENR |= (1<<17))
#define GPIOB_PCLK_EN()  (RCC->AHBENR |= (1<<18))
#define GPIOC_PCLK_EN()  (RCC->AHBENR |= (1<<19))
#define GPIOD_PCLK_EN()  (RCC->AHBENR |= (1<<20))
#define GPIOF_PCLK_EN()  (RCC->AHBENR |= (1<<22))

/*Clock disable macros for GPIOx Peripherals*/

#define GPIOA_PCLK_DI()  (RCC->AHBENR &= ~(1<<17))
#define GPIOB_PCLK_DI()  (RCC->AHBENR &= ~(1<<18))
#define GPIOC_PCLK_DI()  (RCC->AHBENR &= ~(1<<19))
#define GPIOD_PCLK_DI()  (RCC->AHBENR &= ~(1<<20))
#define GPIOF_PCLK_DI()  (RCC->AHBENR &= ~(1<<22))

/* Clock enable macros for I2Cx Peripherals*/

#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1<<22))

/* Clock disable macros for I2Cx Peripherals*/

#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<22))

/* Clock enable macros for SPIx Peripherals*/

#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1<<14))

/* Clock disable macros for SPIx Peripherals*/

#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~(1<<14))

/* Clock enable macros for USARTx Peripherals*/

#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN()   (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN()   (RCC->APB1ENR |= (1<<18))
#define USART4_PCLK_EN()   (RCC->APB1ENR |= (1<<19))
#define USART5_PCLK_EN()   (RCC->APB1ENR |= (1<<20))
#define USART6_PCLK_EN()   (RCC->APB2ENR |= (1<<5))


// Register reset macros for GPIOx Peripherals
#define GPIOA_REG_RESET()       do{(RCC->AHBRSTR |= (1<<17));        (RCC->AHBRSTR &= ~(1<<17)); } while(0)
#define GPIOB_REG_RESET()       do{(RCC->AHBRSTR |= (1<<18));        (RCC->AHBRSTR &= ~(1<<18)); } while(0)
#define GPIOC_REG_RESET()       do{(RCC->AHBRSTR |= (1<<19));        (RCC->AHBRSTR &= ~(1<<19)); } while(0)
#define GPIOD_REG_RESET()       do{(RCC->AHBRSTR |= (1<<20));        (RCC->AHBRSTR &= ~(1<<20)); } while(0)
#define GPIOF_REG_RESET()       do{(RCC->AHBRSTR |= (1<<22));        (RCC->AHBRSTR &= ~(1<<22)); } while(0)


// Register reset macros for I2Cx Peripherals
#define I2C1_RESET()           do{(RCC->APB1RSTR |= (1<<21));        (RCC->APB1RSTR &= ~(1<<21)); } while(0);
#define I2C2_RESET()           do{(RCC->APB1RSTR |= (1<<22));        (RCC->APB1RSTR &= ~(1<<22)); } while(0);

// Register reset macros for SPIx Peripherals
#define SPI1_RESET()           do{(RCC->APB2RSTR |= (1<<12));        (RCC->APB2RSTR &= ~(1<<12)); } while(0);
#define SPI2_RESET()           do{(RCC->APB1RSTR |= (1<<14));        (RCC->APB2RSTR &= ~(1<<14)); } while(0);


//IRQ(Interrupt Request) Number of STM32F070xx FAMILY

#define IRQ_NO_EXTI0_1     5
#define IRQ_NO_EXTI2_3     6
#define IRQ_NO_EXTI4_15    7



// Priority level macros

#define PRIORITY_LEVEL_0     0
#define PRIORITY_LEVEL_1     1
#define PRIORITY_LEVEL_2     2
#define PRIORITY_LEVEL_3     3
#define PRIORITY_LEVEL_4     4
#define PRIORITY_LEVEL_5     5
#define PRIORITY_LEVEL_6     6
#define PRIORITY_LEVEL_7     7
#define PRIORITY_LEVEL_8     8
#define PRIORITY_LEVEL_9     9
#define PRIORITY_LEVEL_10    10
#define PRIORITY_LEVEL_11    11
#define PRIORITY_LEVEL_12    12
#define PRIORITY_LEVEL_13    13
#define PRIORITY_LEVEL_14    14
#define PRIORITY_LEVEL_15    15


//some generic macros

#define ENABLE               1
#define DISABLE              0
#define SET                  ENABLE
#define RESET                DISABLE
#define GPIO_PIN_SET         SET
#define GPIO_PIN_RESET       RESET
#define FLAG_SET             SET
#define FLAG_RESET           RESET

/*********************************************************
 * SPI Peripheral CR1 Register bit position declaration
 *********************************************************/
#define SPI_CR1_CPHA         		0
#define SPI_CR1_CPOL         		1
#define SPI_CR1_MSTR		 		2
#define SPI_CR1_BR		 	 		3
#define SPI_CR1_SPE			 		6
#define SPI_CR1_LSB_FIRST			7
#define SPI_CR1_SSI			 		8
#define SPI_CR1_SSM			 		9
#define SPI_CR1_RXONLY			 	10
#define SPI_CR1_CRCL			 	11
#define SPI_CR1_CRCN_EXT			12
#define SPI_CR1_CRCN_EN			 	13
#define SPI_CR1_BIDIOE			 	14
#define SPI_CR1_BIDIMODE			15

/*********************************************************
 * SPI Peripheral CR2 Register bit position declaration
 *********************************************************/
#define SPI_CR2_DS                  8


/*********************************************************
 * SPI Peripheral SR Register bit position declaration
 *********************************************************/
#define SPI_SR_RXNE                0
#define SPI_SR_TXE                 1
#define SPI_SR_BSY                 7


#endif /* INC_STM32F070RBX_H_ */

