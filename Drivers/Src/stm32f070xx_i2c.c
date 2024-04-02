/*
 * stm32f070xx_i2c.c
 *
 *  Created on: Mar 14, 2024
 *      Author: User
 */


#include "stm32f070xx_i2c.h"

uint16_t AHB_Prescaler[9] = {2,4,8,16,32,64,128,256,512};
uint8_t APB_Prescaler[4] = {2,4,8,16};

/**************************************************************
 * APIs supported by this driver
 */


/*****************************************************************
 * @fn                     - I2C_PeripheralControl
 *
 * @breif                  -This function enables or disables I2C peripheral
 *
 * @param[in]              - base address of I2C Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1<<0);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<0);
	}
}


/*****************************************************************
 * @fn                     - I2C_PCLKControl
 *
 * @breif                  -This function enables or disables peripheral clock for given I2C port
 *
 * @param[in]              - base address of I2C Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void I2C_PCLKControl(I2C_RegDef_t *pI2Cx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else{
			I2C2_PCLK_EN();
		}
	}
	else{
		if(pI2Cx == I2C1)
				{
					I2C1_PCLK_DI();
				}
				else
				{
					I2C2_PCLK_DI();
				}
}
}


//peripheral Init and De-Init

uint32_t RCC_GetPLLOutputClock(){

	return 0;
}

//Low speed clocks for APB peripherals but not I2C1
uint32_t RCC_GetPCLK1Value(void){

	uint32_t sysclk, pclk1;
	uint8_t clksrc, temp, ahbp, apbp;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0)
	{
		sysclk = 8000000;
	}
	else if(clksrc == 1)
	{
		sysclk = 4000000;
	}
	else if(clksrc == 2)
	{
		sysclk = RCC_GetPLLOutputClock();
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

		pclk1 = (sysclk / ahbp) /apbp;
		return pclk1;

}

//High speed clock for I2C1 Peripheral


/*****************************************************************
 * @fn                     - I2C_Init
 *
 * @breif                  -This function initialize I2C peripheral
 *
 * @param[in]              - base address of I2C Handle
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg, sysclk;

	//clear PE bit in I2C CR1 register
	pI2CHandle->pI2Cx->CR1 &= ~(1<<0);

	//configure ANFOFF bit or DNF bit
	pI2CHandle->pI2Cx->CR1 &= ~(1<<12);  //enabled analog filter

	//configure TIMINGR register
	tempreg = 0;
	if(sysclk == 8000000 && pI2CHandle->I2C_Config.I2C_SCLSpeed == 100000)
	{
		tempreg = (uint32_t) RCC_GetPCLK1Value / 1000000;
		pI2CHandle->pI2Cx->TIMINGR |= tempreg & 0xF0000000;
	}



}

/*****************************************************************
 * @fn                     - I2C_DeInit
 *
 * @breif                  -This function di-initialize I2C peripheral
 *
 * @param[in]              - base address of I2C Peripheral
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx){
	if(pI2Cx == I2C1)
	{
		I2C1_RESET();
	}
	else
	{
		I2C2_RESET();
	}
}




















