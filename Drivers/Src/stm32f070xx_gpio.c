/*
 * stm32f070xx_gpio.c
 *
 *  Created on: Feb 22, 2024
 *      Author: User
 */

#include "stm32f070xx_gpio.h"




//
/**************************************************************
 * APIs supported by this driver
 */

//peripheral clock setup

/*****************************************************************
 * @fn                     - GPIO_PCLKControl
 *
 * @breif                  -This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              - ENABLE or DISABLE macros
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */

void GPIO_PCLKControl(GPIO_RegDef_t *pGPIOx, uint32_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
        }
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA){
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIOB){
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIOC){
					GPIOC_PCLK_DI();
		        }
				else if(pGPIOx == GPIOD){
					GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIOF){
					GPIOF_PCLK_DI();
				}
	}
}

//GPIO Init & De-Init

/*****************************************************************
 * @fn                     - GPIO_Init
 *
 * @breif                  -This function enables for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint16_t temp=0;

	if(pGPIOHandle->GPIO_PinConfig.PinMode <= GPIO_MODE_ANALOG)
	{

		//non interrupt modes
		//1. configure gpio mode
		temp = ((pGPIOHandle->GPIO_PinConfig.PinMode) << (2 * pGPIOHandle->GPIO_PinConfig.PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.PinNumber); //clearing the bit fields
		pGPIOHandle->pGPIOx->MODER |= temp; // setting the bit fields
		temp=0;


		//2.configure the speed
		temp=0;
		temp = (pGPIOHandle->GPIO_PinConfig.PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.PinNumber); //clearing the bit fields
		pGPIOHandle->pGPIOx->OSPEEDR |= temp; // setting the bit fields
		temp=0;


		//3.configure pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.PinNumber); //clearing the bit fields
		pGPIOHandle->pGPIOx->PUPDR |= temp; //setting the bit fields
		temp=0;

		//4.configure the otype
		temp = (pGPIOHandle->GPIO_PinConfig.PinOPType << pGPIOHandle->GPIO_PinConfig.PinNumber);
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.PinNumber); //clearing the bit fields
		pGPIOHandle->pGPIOx->OTYPER |= temp; //setting the bit fields
		temp=0;

		//5. configure alternate functionality
		if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_ALTFN)
		{
		uint32_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.PinNumber >> 3;    // bitwise operation for division using power of 2, ie 2^3 is 8
		temp2 = pGPIOHandle->GPIO_PinConfig.PinNumber & 0x07;  // mode operation using bitwise instead of % operator;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0XF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.PinAltFunMode << (4 * temp2));
		}

	}
		else
		{
			if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_FT)
			{
				//Configure FTSR
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
				//clear corresponding rtsr bit
				EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RT)
			{
				//Configure RTSR
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
				//clear corresponding ftsr bit
				EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			}
			else if(pGPIOHandle->GPIO_PinConfig.PinMode == GPIO_MODE_IT_RFT)
			{
				//1.CONFIGURE BOTH RTSR AND FTSR
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);
			}

			//2.CONFIGURE GPIO PORT SELECTION IN SYSCFG_EXTICR
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.PinNumber >> 2;    // bitwise operation for division using power of 2, ie 2^2 is 4
			uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.PinNumber & 0x03;  // mode operation using bitwise instead of % operator;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1+1] = portcode << (temp2 * 4);


			//3.ENABLE EXTI INTERRUPT DELIVERY USING IMR
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.PinNumber);

		}
}

/*****************************************************************
 * @fn                     - GPIO_DeInit
 *
 * @breif                  -This function disables for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB){
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC){
				GPIOC_REG_RESET();
	        }
			else if(pGPIOx == GPIOD){
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOF){
				GPIOF_REG_RESET();
			}
}

//GPIO data write and read

/*****************************************************************
 * @fn                     - GPIO_ReadFromInputPin
 *
 * @breif                  -This function reads the pin for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              - Pin Number
 * @param[in]              -
 *
 * @return                 - 0 or 1
 *
 * @Note                   -none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*****************************************************************
 * @fn                     - GPIO_ReadFromInputPort
 *
 * @breif                  -This function reads for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 - 0 or 1
 *
 * @Note                   -none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

/*****************************************************************
 * @fn                     - GPIO_WriteToOutputPin
 *
 * @breif                  -This function writes the pin for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              - Pin Number
 * @param[in]              - Value to be write
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET)
	{
		//write 1 to output data register at bit field corresponding to the pin number
		pGPIOx->ODR |= (1<< PinNumber);
	}
	else
	{
		//write 0 to output data register at bit field corresponding to pin number
		pGPIOx->ODR &= ~(1<< PinNumber);
	}
}

/*****************************************************************
 * @fn                     - GPIO_WriteToOutputPin
 *
 * @breif                  -This function writes the pin for given GPIO port
 *
 * @param[in]              - base address of GPIO Peripheral
 * @param[in]              -  Value to be write
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

		pGPIOx->ODR = Value;
}

/*****************************************************************
 * @fn                     - GPIO_TogglePin
 *
 * @breif                  -This function toggles the pin for given GPIO port
 *
 * @param[in]              - Base address of GPIO Peripheral
 * @param[in]              - Pin Number
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

//GPIO Interrupt config and handling

/*****************************************************************
 * @fn                     - GPIO_IRQConfig
 *
 * @breif                  -This function configure the interrupt for a pin of a GPIO port
 *
 * @param[in]              - IRQ Number
 * @param[in]              - Enable or Disable
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
 * @fn                     - GPIO_IRQPriorityConfig
 *
 * @breif                  -This function configures the priority of interrupt for a pin of a GPIO port
 *
 * @param[in]              - IRQ Number
 * @param[in]              - IRQ Priority level
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){

	uint8_t irpx = IRQNumber >> 2; // bitwise operation for division using power of 2, ie 2^2 is 4
	uint8_t irpx_sec = IRQNumber & 0x03;  // mode operation using bitwise instead of % operator

	uint8_t shift_amount = (8 * irpx_sec) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (irpx * 4 )) |= ( IRQPriority << shift_amount);


}

/*****************************************************************
 * @fn                     - GPIO_IRQHandling
 *
 * @breif                  -This function handles the interrupt for given pin of a GPIO port
 *
 * @param[in]              - Pin Number
 * @param[in]              -
 * @param[in]              -
 *
 * @return                 -none
 *
 * @Note                   -none
 */

void GPIO_IRQHandling(uint8_t PinNumber){

	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);
	}
}
