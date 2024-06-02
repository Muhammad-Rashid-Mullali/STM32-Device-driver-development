/*
 * stm32f070xx_uart.h
 *
 *  Created on: May 17, 2024
 *      Author: User
 */

#ifndef INC_STM32F070XX_USART_H_
#define INC_STM32F070XX_USART_H_

#include "stm32f070rbx.h"


/*
 * Configure structure for USART peripheral
 */
typedef struct{
	uint8_t USART_Mode;
	uint8_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLenght;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USART peripheral
 */
typedef struct{
	USART_RegDef_t *pUSARTx;
	USART_Config_t *USART_Config;
}USART_Handle_t;

/*
 * @USART_Mode
 */
#define USART_MODE_TX					0
#define USART_MODE_RX					1
#define USART_MODE_TXRX					2

/*
 * @USART_Baud
 */
#define USART_BAUD_2400					2400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200				19200
#define USART_BAUD_38400				38400
#define USART_BAUD_57600				57600
#define USART_BAUD_115200				115200
#define USART_BAUD_230400				230400
#define USART_BAUD_460800				460800
#define USART_BAUD_921600				921600
#define USART_BAUD_2M					2000000
#define USART_BAUD_3M					3000000
#define USART_BAUD_4M					4000000
#define USART_BAUD_5M					5000000
#define USART_BAUD_6M					6000000

/*
 * @USART_NoOfStopBits
 */
#define USART_STOPBIT_1					0
#define USART_STOPBIT_2					1

/*
 * @USART_WordLenght
 */
#define USART_WORDLEN_8BITS				0
#define USART_WORDLEN_9BITS				1

/*
 * @USART_ParityControl
 */
#define USART_PARITY_ODD				2
#define USART_PARITY_EVEN				1
#define USART_PARITY_DI					0

/*
 * @USART_HWFlowControl
 */
#define USART_NO_HWFLOWCONTROL			0
#define USART_HWFLOWCONTROL_CTS			1
#define USART_HWFLOWCONTROL_RTS			2
#define USART_HWFLOWCONTROL_CTS_RTS		3

/*
 * USART status flag definition
 */
#define USART_TXE_FLAG			(1 << USART_ISR_TXE)
#define USART_RXNE_FLAG			(1 << USART_ISR_RXNE)
#define USART_TC_FLAG			(1 << USART_ISR_TC)
#define USART_BSY_FLAG			(1 << USART_ISR_BSY)

/*
 * USART1 clock source selection definition
 */
#define USART1SW_PCLK					0
#define USART1SW_SYSCLK					1
#define USART1SW_LSE					2
#define USART1SW_HSE					3

/************************************************
*         APIs Supported by this driver
*************************************************/
//peripheral clock setup
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint32_t EnorDi);
void USART_PCLKControl(USART_RegDef_t *pUSARTx, uint32_t EnorDi);

//USART Init and De-Init
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

//Data send and recieve
void USART_DataSend(USART_RegDef_t *pUSARTx, uint8_t *pTxBuffer, uint32_t Len);
void USART_DataRecieve(USART_RegDef_t *pUSARTx, uint8_t *pRxBuffer, uint32_t Len);
void USART_DataSend_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_DataRecieve_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

//IRQ Configuration and ISR Handling
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void USART_IRQHandling(USART_Handle_t *pHandle);

#endif /* INC_STM32F070XX_USART_H_ */
