/*
 * stm32f070xx_rcc.h
 *
 *  Created on: Jun 1, 2024
 *      Author: User
 */

#ifndef INC_STM32F070XX_RCC_H_
#define INC_STM32F070XX_RCC_H_

#include "stm32f070rbx.h"


/*
 * RCC Internal or external Oscillator HSE, HSI, LSI and LSE configuration structure definition
 */
typedef struct{

	uint32_t OscillatorType; // type of oscillator 					/*!<possible values from @OscillatorType>*/
	uint32_t HSEState; //state of HSE 								/*!<possible values from @HSEState>*/
	uint32_t LSEState; //state of LSE								/*!<possible values from @LSEState>*/
	uint32_t HSIState; //state of HSI								/*!<possible values from @HSIState>*/
	uint32_t HSICalibrationValue; //The HSI calibration trimming value
	uint32_t LSIState; //state of LSI								/*!<possible values from @LSIState>*/

}RCC_OscInitTypeDef;


/*
 * RCC System AHB, APB bus clock configuration structure definition
 */
typedef struct{

	uint32_t Clocktype;  //type of clock (HSE, HSI, PLL)			/*!<possible values from @Clocktype>*/
	uint32_t SYSCLKSource; //clock source used in system			/*!<possible values from @SYSCLKSource>*/
	uint32_t AHBCLKDivider; //AHB clock divider						/*!<possible values from @AHBCLKDivider>*/
	uint32_t APB1CLKDivider; //APB clock divider					/*!<possible values from @APB1CLKDivider>*/

}RCC_ClkInitTypeDef;


/*
 * @OscillatorType
 */
#define RCC_OSCILLATORTYPE_NONE				0
#define RCC_OSCILLATORTYPE_HSE              1	//0b0001
#define RCC_OSCILLATORTYPE_HSI              2	//0b0010
#define RCC_OSCILLATORTYPE_LSE              4	//0b0100
#define RCC_OSCILLATORTYPE_LSI              8	//0b1000

/*
 * @HSEState
 */
#define RCC_HSE_OFF							0
#define RCC_HSE_ON							1
#define RCC_HSE_BYPASS						5	//0b0101 to enable both HSEON and HSEBYP bits

/*
 *@LSEState
 */
#define RCC_LSE_OFF							0
#define RCC_LSE_ON							1
#define RCC_LSE_BYPASS						5	//0b0101 to enable both LSEON and LSEBYP bits

/*
 * @HSIState
 */
#define RCC_HSI_OFF							0
#define RCC_HSI_ON							(1 << RCC_CR_HSION)

#define RCC_CALIBRATION_DEFAULT				(0x10U)


/*
 * @LSIState
 */
#define RCC_LSI_OFF							0
#define RCC_LSI_ON							(1 << RCC_CSR_LSION)

/*
 * @Clocktype
 */
#define RCC_CLOCKTYPE_SYSCLK             	1
#define RCC_CLOCKTYPE_HCLK               	2
#define RCC_CLOCKTYPE_PCLK1              	4

/*
 * @SYSCLKSource
 */
#define RCC_SYSCLKSOURCE_HSI				(RCC_CFGR_SW_HSI)
#define RCC_SYSCLKSOURCE_HSE				(RCC_CFGR_SW_HSE)
#define RCC_SYSCLKSOURCE_PLL				(RCC_CFGR_SW_PLL)

/*
 * SYSCLK Source status
 */
#define RCC_SYSCLKSOURCE_STATUS_HSI			(RCC_CFGR_SWS_HSI)
#define RCC_SYSCLKSOURCE_STATUS_HSE			(RCC_CFGR_SWS_HSE)
#define RCC_SYSCLKSOURCE_STATUS_PLL			(RCC_CFGR_SWS_PLL)

/*
 * @AHBCLKDivider
 */
#define RCC_SYSCLK_DIV1						RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2						RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4						RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8						RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16					RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64					RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128					RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256					RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512					RCC_CFGR_HPRE_DIV512

/*
 * @AHBCLKDivider
 */
#define RCC_HCLK_DIV1						RCC_CFGR_PPRE_DIV1
#define RCC_HCLK_DIV2						RCC_CFGR_PPRE_DIV2
#define RCC_HCLK_DIV4						RCC_CFGR_PPRE_DIV4
#define RCC_HCLK_DIV8						RCC_CFGR_PPRE_DIV8
#define RCC_HCLK_DIV16						RCC_CFGR_PPRE_DIV16

/*
 * RCC USART1 clock source
 */
#define RCC_USART1CLKSOURCE_PCLK			RCC_CFGR3_USART1SW_PCLK
#define RCC_USART1CLKSOURCE_SYSCLK			RCC_CFGR3_USART1SW_SYSCLK
#define RCC_USART1CLKSOURCE_LSI				RCC_CFGR3_USART1SW_LSI
#define RCC_USART1CLKSOURCE_HSI				RCC_CFGR3_USART1SW_HSI

/*
 * RCC I2C1 clock source
 */
#define RCC_I2C1CLKSOURCE_HSI				RCC_CFGR3_I2C1SW_HSI
#define RCC_I2C1CLKSOURCE_SYSCLK			RCC_CFGR3_I2C1SW_SYSCLK


uint32_t RCC_GetPCLKValue(void);

#endif /* INC_STM32F070XX_RCC_H_ */
