/*
 * 002_button_interrupt.c
 *
 *  Created on: Mar 10, 2024
 *      Author: User
 */

#include "stm32f070rbx.h"
#include "stm32f070xx_gpio.h"


int main(void){

	GPIO_Handle_t Gpioled, Gpiobtn;

	//configure led
	Gpioled.pGPIOx = GPIOA;
	Gpioled.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_5;
	Gpioled.GPIO_PinConfig.PinMode = GPIO_MODE_OUT;
	Gpioled.GPIO_PinConfig.PinSpeed = GPIO_LOW_SPEED;
	Gpioled.GPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	Gpioled.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;

	//enable the clock
	GPIO_PCLKControl(GPIOA, ENABLE);
	GPIO_Init(&Gpioled);

	//configure the button
	Gpiobtn.pGPIOx = GPIOC;
	Gpiobtn.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_13;
	Gpiobtn.GPIO_PinConfig.PinMode = GPIO_MODE_IT_FT;
	Gpiobtn.GPIO_PinConfig.PinSpeed = GPIO_HIGH_SPEED;
	Gpiobtn.GPIO_PinConfig.PinPuPdControl = GPIO_PIN_PU;

	//enable the clock
	GPIO_PCLKControl(GPIOC, ENABLE);
	GPIO_Init(&Gpiobtn);


	//IRQ Configurations
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI4_15, ENABLE);
	//GPIO_IRQPriorityConfig(IRQ_NO_EXTI4_15, PRIORITY_LEVEL_15);


	while(1){

	}
}

void EXTI4_15_IRQHandler(void){

	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_TogglePin(GPIOA, GPIO_PIN_NO_5);
	//delay();

	}
