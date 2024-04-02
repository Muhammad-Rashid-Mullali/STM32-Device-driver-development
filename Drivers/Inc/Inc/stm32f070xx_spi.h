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
 * @SPI_DS
 */
#define SPI_DS_1BITS                  0
#define SPI_DS_2BITS                  1
#define SPI_DS_3BITS                  2
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
void Data_Send(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void Data_Recieve(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//IRQ Configuration and ISR Handling
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);




#endif /* INC_STM32F070XX_SPI_H_ */
