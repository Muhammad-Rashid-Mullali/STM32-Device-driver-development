/*
 * stm32f070xx_rcc.c
 *
 *  Created on: Jun 1, 2024
 *      Author: User
 */

#include "stm32f070xx_rcc.h"


uint16_t AHB_Prescaler[9] = {2,4,8,16,32,64,128,256,512};
uint8_t APB_Prescaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLKValue(void){

	uint32_t sysclk, pclk; //sysclk - system clock and pclk - peripheral clock for APB1 & APB2
	uint8_t clksrc, temp, ahbp, apbp;

	clksrc = ((RCC->CFGR >> 2) & 0x3); //clksrc - clocksource - HSI,HSE or PLL (SW)

	if(clksrc == 0) //HSI
	{
		RCC->CR |= (1 << RCC_CR_HSION);
		while(!(RCC->CR & RCC_CR_HSIRDY)){};
		RCC->CFGR |= RCC_CFGR_SW_HSI;

		sysclk = 8000000;
	}
	else if(clksrc == 1) //HSE
	{
		sysclk = 4000000;  //4-32 MHz
	}
	else if(clksrc == 2) //PLL
	{
		//sysclk = RCC_GetPLLOutputClock();
	}

	//AHB Prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahbp = 1;
	}
	else
	{
		ahbp = AHB_Prescaler[temp - 8];
	}

	//APB Prescaler
	temp = ((RCC->CFGR >> 8) & 0x7);
		if(temp < 4)
		{
			apbp = 1;
		}
		else
		{
			ahbp = APB_Prescaler[temp - 4];
		}

		pclk = (sysclk / ahbp) /apbp;
		return pclk;

}

/*uint32_t SystemClockConfig(void){

	uint32_t SysClock;

	SysClock = RCC->CR |= (1 << RCC_CR_HSEON) | (1 << RCC_CR_CSSON) | (1 << RCC_CR_HSEBYP);
	while( ! (RCC->CR & RCC_CR_HSERDY) ){};

	clksrc = ((RCC->CFGR >> 0) & 0x3);



}*/
