/*
 * stm32f070xx_spi.h
 *
 *  Created on: Mar 26, 2024
 *      Author: User
 */

#ifndef INC_STM32F070XX_SPI_H_
#define INC_STM32F070XX_SPI_H_

#include "stm32f070rbx.h"


/*
 * Configure structure for SPI peripheral
 */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_DS;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 * Handle structure for SPI peripheral
 */
typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_MASTER_MODE               1
#define SPI_SLAVE_MODE                0

/*
 * @BusConfig
 */
#define SPI_BUS_CONFIG_FD             1
#define SPI_BUS_CONFIG_HD             2
#define SPI_BUS_CONFIG_RXONLY         3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2           0
#define SPI_SCLK_SPEED_DIV4           1
#define SPI_SCLK_SPEED_DIV8           2
#define SPI_SCLK_SPEED_DIV16          3
#define SPI_SCLK_SPEED_DIV32          4
#define SPI_SCLK_SPEED_DIV64          5
#define SPI_SCLK_SPEED_DIV128         6
#define SPI_SCLK_SPEED_DIV256         7

/*
 * @SPI_DFF
 */
#define SPI_DFF_MSBFIRST              0
#define SPI_DFF_LSBFIRST              1

/*
 * @SPI_DS
 */
#define SPI_DS_4BITS                  3
#define SPI_DS_5BITS                  4
#define SPI_DS_6BITS                  5
#define SPI_DS_7BITS                  6
#define SPI_DS_8BITS                  7
#define SPI_DS_9BITS                  8
#define SPI_DS_10BITS                 9
#define SPI_DS_11BITS                 10
#define SPI_DS_12BITS                 11
#define SPI_DS_13BITS                 12
#define SPI_DS_14BITS                 13
#define SPI_DS_15BITS                 14
#define SPI_DS_16BITS                 15
/*
 * Bit 1 to 3 is not used values so if software attempts to write one of the
 *  “Not used” values, they are forced to the value "0111"(8 bits)
 */
#define SPI_DS_1BITS                    SPI_DS_8BITS
#define SPI_DS_2BITS			SPI_DS_8BITS
#define SPI_DS_3BITS			SPI_DS_8BITS

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH                 1
#define SPI_CPOL_LOW                  0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH                 1
#define SPI_CPHA_LOW                  0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN                    1
#define SPI_SSM_DI                    0


/*
 * SPI Application states
 */
#define SPI_READY				  0
#define SPI_BUSY_IN_RX				  1
#define SPI_BUSY_IN_TX				  2

/*
 * SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT			  1
#define SPI_EVENT_RX_CMPLT			  2
#define SPI_EVENT_OVR_ERR			  3
#define SPI_EVENT_CRC_ERR			  4

/*
 * SPI status flags definitions
 */
#define SPI_TXE_FLAG       (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG      (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG       (1 << SPI_SR_BSY)


/************************************************
*         APIs Supported by this driver
*************************************************/
//peripheral clock setup
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint32_t EnorDi);
void SPI_PCLKControl(SPI_RegDef_t *pSPIx, uint32_t EnorDi);


//SPI Init and De-Init
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data send and recieve
//non-interrupt mode
void SPI_Data_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_Data_Recieve(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//interrupt mode
uint8_t SPI_IT_Data_Send(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_IT_Data_Recieve(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

//NSS Management
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint32_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint32_t EnorDi);


void ClearOVRFlag(SPI_RegDef_t *pSPIx);
void CloseTransmission(SPI_Handle_t *pSPIHandle);
void CloseReception(SPI_Handle_t *pSPIHandle);

//IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);


/*
 * Application Callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t EventName);



#endif /* INC_STM32F070XX_SPI_H_ */
