/*
 * stm32f070xx_spi.c
 *
 *  Created on: Mar 26, 2024
 *      Author: User
 */


#include "stm32f070xx_spi.h"


/*****************************************************************
 * @fn                     - SPI_PeripheralControl
 *
 * @breif                  -This function enables or disables SPI peripheral
 *
 * @param[in]              - base address of SPI Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1<<6);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<6);
	}
}

/*****************************************************************
 * @fn                     - SPI_PCLKControl
 *
 * @breif                  -This function enables or disables clock for SPI peripheral
 *
 * @param[in]              - base address of SPI Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else
		{
			SPI2_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else
		{
			SPI2_PCLK_DI();
		}
	}
}

//SPI Init and De-Init
/*****************************************************************
 * @fn                     - SPI_Init
 *
 * @breif                  -This function initiates SPI peripheral
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	uint32_t tempreg = 0;

	//1.configure device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.configure bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		pSPIHandle->pSPIx->CR1 &= ~(1<<15);  //clear BIDIMODE bit in CR1 register to enable full duplex
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		pSPIHandle->pSPIx->CR1 |= (1<<15);  //set BIDIMODE bit in CR1 register to enable half duplex
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_RXONLY)
	{
		pSPIHandle->pSPIx->CR1 &= ~(1<<15);
		pSPIHandle->pSPIx->CR1 |= (1<<10);

	}
	//3.configure sclk speed
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	//4.configure the data size
	uint8_t temp = 0;
	temp |= pSPIHandle->SPI_Config.SPI_DS << SPI_CR2_DS;
	pSPIHandle->pSPIx->CR2 = temp;

	//5.configure CPOL bit of CR1 register
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6.configure CPHA bit of CR1 register
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*****************************************************************
 * @fn                     - SPI_DeInit
 *
 * @breif                  -This function De-Initialize SPI peripheral
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1)
	{
		RCC->APB2RSTR |= (1<<12);
	}
	else
	{
		RCC->APB1RSTR |= (1<<14);
	}
}

uint8_t GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){

	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}

//Data send and recieve
void Data_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0)
	{
		//1.wait until TXE is set
		 while(GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);          //while ( ! (pSPIx->SR & (1 << 1) ) )
	}
}
void Data_Recieve(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

