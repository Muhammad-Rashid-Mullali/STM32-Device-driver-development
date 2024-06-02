/*
 * stm32f070xx_uart.c
 *
 *  Created on: May 17, 2024
 *      Author: User
 */


#include "stm32f070xx_usart.h"


/*****************************************************************
 * @fn                     - GetFlagStatus
 *
 * @breif                  -This check the status of specific flags in the status register (SR) of an USART peripheral
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              - Flagname in status register to check
 * @param[in]              -
 *
 * @return                 -uint8_t
 *
 * @Note                   -
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName){

	if(pUSARTx->ISR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}


/*****************************************************************
 * @fn                     - USART_SetBaudRate
 *
 * @breif                  -This function generates baudrate for USART peripheral
 *
 * @param[in]              - base address of USART Peripheral
 * @param[in]              - BaudRate to generate
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){

	uint32_t PCLKx; //APB clock
	uint32_t usartdiv;

	//to hold value of mantissa and fraction values
	uint32_t M_part, F_part;

	//uint32_t tempreg = 0;

	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		PCLKx = RCC_GetPCLKValue();
	}
	else
	{
		PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		//oversampling by 8
		usartdiv = (2 * PCLKx) / BaudRate;

		//right shift the last bit to one
		M_part = usartdiv/10;
		F_part = (usartdiv%10) >> 1;
		pUSARTx->BRR = (M_part << 4) | (F_part & 0x0F);
	}
	else
	{
		//oversampling by 16
		usartdiv = PCLKx/BaudRate;
		pUSARTx->BRR = usartdiv;
	}

}
/*****************************************************************
 * @fn                     - USART_PCLKControl
 *
 * @breif                  -This function enables or disables clock for USART peripheral
 *
 * @param[in]              - base address of USART Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void USART_PCLKControl(USART_RegDef_t *pUSARTx, uint32_t EnorDi){

	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == USART4)
		{
			USART4_PCLK_EN();
		}
		else if(pUSARTx == USART5)
		{
			USART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == USART4)
		{
			USART4_PCLK_DI();
		}
		else if(pUSARTx == USART5)
		{
			USART5_PCLK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}

/*****************************************************************
 * @fn                     - USART_Init
 *
 * @breif                  -This function initiates USART peripheral
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void USART_Init(USART_Handle_t *pUSARTHandle){

	/*******************************CR1 Register*****************************************/
	uint32_t tempreg = 0;

	USART_PCLKControl(pUSARTHandle->pUSARTx, ENABLE);

	//configure usart mode
	if(pUSARTHandle->USART_Config->USART_Mode == USART_MODE_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config->USART_Mode == USART_MODE_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config->USART_Mode == USART_MODE_TXRX)
	{
		 tempreg |= ( (1 << USART_CR1_TE) | (1 << USART_CR1_RE) );
	}

	//configure the wordlenght
	//default wordlenght is 8bit so don't need to configure it
	if( pUSARTHandle->USART_Config->USART_WordLenght == USART_WORDLEN_9BITS)
	{
		tempreg |= (1 << USART_CR1_M0);
	}

	//configure the parity control
	if(pUSARTHandle->USART_Config->USART_ParityControl == USART_PARITY_EVEN)
	{
		tempreg |= (1 << USART_CR1_PCE);
		//default value is even parity once parity control is enabled
	}
	else if(pUSARTHandle->USART_Config->USART_ParityControl == USART_PARITY_ODD)
	{
		tempreg |= ( (1 << USART_CR1_PCE) | (1 << USART_CR1_PS) );
	}

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/*********************CR2 Register*******************************************************/
	tempreg = 0;

	//configure stopbits
	tempreg |= pUSARTHandle->USART_Config->USART_NoOfStopBits << USART_CR2_STOP;
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/*************************CR3 Register****************************************************/
	tempreg = 0;

	//configure HW Flow control
	if(pUSARTHandle->USART_Config->USART_HWFlowControl == USART_HWFLOWCONTROL_CTS)
	{
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if(pUSARTHandle->USART_Config->USART_HWFlowControl == USART_HWFLOWCONTROL_RTS)
	{
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if(pUSARTHandle->USART_Config->USART_HWFlowControl == USART_HWFLOWCONTROL_CTS_RTS)
	{
		tempreg |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) );
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config->USART_Baud);

}

/*****************************************************************
 * @fn                     - USART_DeInit
 *
 * @breif                  -This function De-Initialize USART peripheral
 *
 * @param[in]              - base address of SPI Handle structure
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void USART_DeInit(USART_RegDef_t *pUSARTx){

	if(pUSARTx == USART1)
	{
		USART1_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_RESET();
	}
	else if(pUSARTx == USART3)
		{
			USART3_RESET();
		}
	else if(pUSARTx == USART4)
		{
			USART4_RESET();
		}
	else if(pUSARTx == USART5)
		{
			USART5_RESET();
		}
	else if(pUSARTx == USART6)
		{
			USART6_RESET();
		}
}

/*****************************************************************
 * @fn                     - USART_DataSend
 *
 * @breif                  -This function transmits the message
 *
 * @param[in]              - base address of USART Handle structure
 * @param[in]              - message content
 * @param[in]              - lenght of the message
 *
 * @return                 -none
 *
 * @Note                   - this is a blocking call or polling based
 */
void USART_DataSend(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len){

	uint16_t *pdata;

	for(uint32_t i=0; i<Len; i++)
	{
		//wait until TXE is set
		while(  USART_GetFlagStatus(pUSARTx, USART_TXE_FLAG) == FLAG_RESET);

		//define wordlenght
		if(pUSARTx->CR1 & (1 << USART_CR1_M0))
		{
			//wordlenght is 9bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTx->TDR = (*pdata & (uint16_t)0x01FF);

			if(pUSARTx->CR1 & (1 << USART_CR1_PCE))
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
			else
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
		}
		else
		{
			//wordlenght is 8bits
			pUSARTx->TDR = (*pTxBuffer & (uint8_t) 0xFF);
			pTxBuffer++;
		}

		while(! USART_GetFlagStatus(pUSARTx, USART_TC_FLAG));

	}
}

/*****************************************************************
 * @fn                     - USART_DataRecieve
 *
 * @breif                  -This function recieves the message
 *
 * @param[in]              - base address of USART Handle structure
 * @param[in]              - message content
 * @param[in]              - lenght of the message
 *
 * @return                 -none
 *
 * @Note                   - this is a blocking call or polling based
 */
void USART_DataRecieve(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);


void USART_DataSend_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_DataRecieve_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
