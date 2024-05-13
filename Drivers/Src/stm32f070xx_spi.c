/*
 * stm32f070xx_spi.c
 *
 *  Created on: Mar 26, 2024
 *      Author: User
 */


#include "stm32f070xx_spi.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*****************************************************************
 * @fn                     - GetFlagStatus
 *
 * @breif                  -This check the status of specific flags in the status register (SR) of an SPI peripheral
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              - Flagname in status register to check
 * @param[in]              -
 *
 * @return                 -uint8_t
 *
 * @Note                   -
 */
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
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		//wait until FTLVL is 00
		while(GetFlagStatus(pSPIx, SPI_SR_FTLVL) == FLAG_SET);

		//wait until BSY is reset
		while(GetFlagStatus(pSPIx, SPI_SR_BSY) == FLAG_SET);

		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*****************************************************************
 * @fn                     - SPI_SSIConfig
 *
 * @breif                  -This function enables or disables SPI peripheral SSI configuration
 *
 * @param[in]              - base address of SPI Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
		{
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}

/*****************************************************************
 * @fn                     - SPI_SSOEConfig
 *
 * @breif                  -This function enables or disables SPI peripheral SSOE configuration
 *
 * @param[in]              - base address of SPI Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
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

	uint16_t tempreg = 0;

	//spi peripheral clock control
	SPI_PCLKControl(pSPIHandle->pSPIx, ENABLE);

	//enable spi peripheral


	//1.configure device mode
	tempreg &= ~(1 << SPI_CR1_MSTR); //clear mstr bit
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2.configure bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);  //clear BIDIMODE bit in CR1 register to enable full duplex
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_BIDIMODE);  //set BIDIMODE bit in CR1 register to enable half duplex
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_RXONLY)
	{
		pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_BIDIMODE);
		pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_RXONLY);

	}
	//3.configure sclk speed
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	//4.configure DFF data frame format
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_LSB_FIRST;

	//5.configure the data size
	uint8_t temp = 0;
	temp |= pSPIHandle->SPI_Config.SPI_DS << SPI_CR2_DS;


	//6.configure CPOL bit of CR1 register
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//7.configure CPHA bit of CR1 register
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 |= tempreg;
	pSPIHandle->pSPIx->CR2 |= temp;

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

//Data send and recieve
/*****************************************************************
 * @fn                     - SPI_Data_Send
 *
 * @breif                  -This function transmits the message
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              - message content
 * @param[in]              - lenght of the message
 *
 * @return                 -none
 *
 * @Note                   - this is a blocking call or polling based
 */
void SPI_Data_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0)
	{
		//1.wait until TXE is set
		 while(GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);          //while ( ! (pSPIx->SR & (1 << 1) ) )

		 //2.check DS bit in CR2
		 if(pSPIx->CR2 & (1 << SPI_CR2_DS))
		 {
			 //16 bit data size
			 //1.load data into the DR
			 pSPIx->DR = *((uint16_t*)pTxBuffer);
			 Len--;
			 Len--;
			 (uint16_t*)pTxBuffer++;
		 }
		 else
		 {
			 //8 bit data size
			 pSPIx->DR = *pTxBuffer;
			 Len--;
			 pTxBuffer++;
		 }
	}
}

/*****************************************************************
 * @fn                     - SPI_Data_Recieve
 *
 * @breif                  -This function recieves the message
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              - message content
 * @param[in]              - lenght of the message
 *
 * @return                 -none
 *
 * @Note                   - this is a blocking call or polling based
 */
void SPI_Data_Recieve(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){

	while(Len > 0)
	{
		//wait until RXNE is set
		while(GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//2.check DS bit in CR2
		if(pSPIx->CR2 & (1 << SPI_CR2_DS))
				 {
					 //16 bit data size
					 //1.load data from DR to RX buffer
					 *((uint16_t*)pRxBuffer) = pSPIx->DR;
					 Len--;
					 Len--;
					 (uint16_t*)pRxBuffer++;
				 }
		else
		{
			//8 bit data size
			*pRxBuffer = pSPIx->DR;
		    Len--;
			pRxBuffer++;
		}
	}
}

//IRQ Configuration and ISR Handling
/*****************************************************************
 * @fn                     - SPI_IRQInterruptConfig
 *
 * @breif                  -This function configure the interrupt for a pin of a SPI peripheral
 *
 * @param[in]              - IRQ number
 * @param[in]              - enable or disable macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   - this is a blocking call or polling based
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE)
		{
			*NVIC_ISER |= (1 << IRQNumber);
		}
		else
		{
			*NVIC_ICER |= (1 << IRQNumber);
		}
}

/*****************************************************************
 * @fn                     - SPI_IRQPriorityConfig
 *
 * @breif                  -This function configures the priority of interrupt for a pin of a SPI peripheral
 *
 * @param[in]              - IRQ number
 * @param[in]              - IRQ Priority level
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   - none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

		uint8_t irpx = IRQNumber >> 2; // bitwise operation for division using power of 2, ie 2^2 is 4
		uint8_t irpx_sec = IRQNumber & 0x03;  // mode operation using bitwise instead of % operator

		uint8_t shift_amount = (8 * irpx_sec) + (8 - NO_PR_BITS_IMPLEMENTED);

		*(NVIC_IPR_BASEADDR + (irpx * 4 )) |= ( IRQPriority << shift_amount);
}

/*****************************************************************
 * @fn                     - SPI_IT_Data_Send
 *
 * @breif                  -This function transmits data in interrupt mode
 *
 * @param[in]              - SPI Peripheral handle structure address
 * @param[in]              - Transmission buffer address
 * @param[in]              - lengh of message
 *
 * @return                 - uint8_t
 *
 * @Note                   - none
 */
uint8_t SPI_IT_Data_Send(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->TxState;

	if( state != SPI_BUSY_IN_TX)
	{
			//save tx buffer address and len info in same global variables
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxState = Len;

			/*mark the SPI state as busy in transmission so no other
			transmission can take over in same SPI peripheral until transmission is over*/
			pSPIHandle->TxState = SPI_BUSY_IN_TX;

			//enable TXEIE bit to get interrupt when TXE flag is set in SR register
			pSPIHandle->pSPIx->CR2 = (1 << SPI_CR2_TXEIE );
	}

	return state;

}

/*****************************************************************
 * @fn                     - SPI_IT_Data_Recieve
 *
 * @breif                  -This function recieves the data in interrupt mode
 *
 * @param[in]              - SPI Peripheral handle structure address
 * @param[in]              - Reception buffer address
 * @param[in]              - lengh of message
 *
 * @return                 - uint8_t
 *
 * @Note                   - none
 */
uint8_t SPI_IT_Data_Recieve(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//save RX buffer address and len info in same global variables
		pSPIHandle->pRxBuffer =  pRxBuffer;
		pSPIHandle->RxLen = Len;

		/*mark the SPI state as busy in transmission so no other
		transmission can take over in same SPI peripheral until transmission is over*/
		pSPIHandle->TxState = SPI_BUSY_IN_RX;

		//enable RXNEIE bit to get interrupt when RXNE flag is set in SR register
		pSPIHandle->pSPIx->CR2 = (1 << SPI_CR2_RXNEIE );
	}

	return state;
}

/*****************************************************************
 * @fn                     - SPI_IRQHandling
 *
 * @breif                  -This function handles the ISR of interrupt for a pin of a SPI peripheral
 *
 * @param[in]              - SPI Peripheral handle structure address
 * @param[in]              - 
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   - none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1, temp2;
	//check for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	//check for RXNE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
		{
			//handle RXNE
			spi_rxne_interrupt_handle(pHandle);
		}

	//check for OverRun flag
		temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
		temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

		if( temp1 && temp2)
			{
				//handle RXNE
				spi_ovr_interrupt_handle(pHandle);
			}
}

//ISR handling helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_DS))
			 {
				 //16 bit data size
				 //1.load data into the DR
				 pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				 pSPIHandle->TxLen--;
				 pSPIHandle->TxLen--;
				 (uint16_t*)pSPIHandle->pTxBuffer++;
			 }
			 else
			 {
				 //8 bit data size
				 pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
				 pSPIHandle->TxLen--;
				 pSPIHandle->pTxBuffer++;

			 }

	if( ! pSPIHandle->TxLen)
	{
		CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_DS))
			 {
				 //16 bit data size
				 //1.load data from DR to RX buffer
				 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
				 pSPIHandle->RxLen--;
				 pSPIHandle->RxLen--;
				 (uint16_t*)pSPIHandle->pRxBuffer++;
			 }
			else
			{
				//8 bit data size
				*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;
			}
	if(! pSPIHandle->RxLen)
	{
		CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	//clear OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->pTxBuffer = 0;
	pSPIHandle->TxState = SPI_READY;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

}
void CloseReception(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->pRxBuffer = 0;
	pSPIHandle->TxState = SPI_READY;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
}

void ClearOVRFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;

	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventName)
{

	/*
	 * this is weak implementation. The application may override this function.
	 */
}
