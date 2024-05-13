/*
 * 003_spi_tx_testing.c
 *
 *  Created on: Apr 7, 2024
 *      Author: User
 */
#include <string.h>
#include "stm32f070rbx.h"
#include "stm32f070xx_gpio.h"
#include "stm32f070xx_spi.h"
//PA7 SPI2_MOSI
//PA6 SPI2_MISO
//PA5 SPI2_SCK
//PA4 SPI2_NSS
//ALT FN MODE AF0 GPIOB

void SPI_GPIOInits(){

	GPIO_Handle_t  SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.PinAltFunMode = 0;
	SPIPins.GPIO_PinConfig.PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.PinSpeed = GPIO_HIGH_SPEED;

	//SCK
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	//MISO
	/*SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);*/

	//NSS
	SPIPins.GPIO_PinConfig.PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void){

	SPI_Handle_t  SPIhandle;
	SPIhandle.pSPIx = SPI1;
	SPIhandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIhandle.SPI_Config.SPI_DeviceMode = SPI_MASTER_MODE;
	SPIhandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //4MHz
	SPIhandle.SPI_Config.SPI_DS = SPI_DS_8BITS;
	SPIhandle.SPI_Config.SPI_DFF = SPI_DFF_MSBFIRST;
	SPIhandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPIhandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPIhandle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPIhandle);
}
int main(void){

	char user_data[] = "Hello world";


	//a function to initialize the gpio pins to behave as SPI2 pins
	SPI_GPIOInits();

	SPI2_Init();
	SPI_SSIConfig(SPI1,ENABLE); //this makes nss signal internally high and avoid MODF error
	SPI_PeripheralControl(SPI1, ENABLE);


	SPI_Data_Send(SPI1, (uint8_t*)user_data, strlen(user_data));
	while(1);

	return 0;
}
