/*
 * stm32f070xx_i2c.h
 *
 *  Created on: Mar 14, 2024
 *      Author: User
 */

#ifndef INC_STM32F070XX_I2C_H_
#define INC_STM32F070XX_I2C_H_

#include "stm32f070rbx.h"


/*
 * configuration structure for I2C Peripheral
 */
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_PeriControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2C peripheral
 */
typedef struct{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;


/*
 * I2C SCL Speed config macros
 */
#define I2C_SCL_SPEED_SM      	100000
#define I2C_SCL_SPEED_FM2K    	200000
#define I2C_SCL_SPEED_FM4K    	400000
#define I2C_SCL_SPEED_FM7K      700000
#define I2C_SCL_SPEED_FM10K     1000000


//macros for DNF(Digital Noise Filter) bit
#define DNF0     0x0000   	 //Digital filter disabled
#define DNF1     0x0001   	 //Digital filter enabled and filtering capability up to one tI2CCLK
#define DNF2     0x0010  	 //Digital filter enabled and filtering capability up to two tI2CCLK
#define DNF3     0x0011  	 //Digital filter enabled and filtering capability up to three tI2CCLK
#define DNF4     0x0100  	 //Digital filter enabled and filtering capability up to four tI2CCLK
#define DNF5     0x0101    	 //Digital filter enabled and filtering capability up to five tI2CCLK
#define DNF6     0x0110   	 //Digital filter enabled and filtering capability up to six tI2CCLK
#define DNF7     0x0111    	 //Digital filter enabled and filtering capability up to seven tI2CCLK
#define DNF8     0x1000      //Digital filter enabled and filtering capability up to eight tI2CCLK
#define DNF9     0x1001      //Digital filter enabled and filtering capability up to nine tI2CCLK
#define DNF10    0x1010      //Digital filter enabled and filtering capability up to ten tI2CCLK
#define DNF11    0x1011      //Digital filter enabled and filtering capability up to eleven tI2CCLK
#define DNF12    0x1100      //Digital filter enabled and filtering capability up to twelve tI2CCLK
#define DNF13    0x1101      //Digital filter enabled and filtering capability up to thirteen tI2CCLK
#define DNF14    0x1110      //Digital filter enabled and filtering capability up to fourteen tI2CCLK
#define DNF15    0x1111      //Digital filter enabled and filtering capability up to fifteen tI2CCLK
//APIs supported by this driver
//**************************************************************


//peripheral control
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint32_t EnorDi);

//peripheral clock setup
void I2C_PCLKControl(I2C_RegDef_t *pI2Cx, uint32_t EnorDi);

//I2C Init and DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

//Data send and recieve



//Interrupt config and handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint32_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
#endif /* INC_STM32F070XX_I2C_H_ */
