/*
 * @filename	    : lpm.c
 * @description  	: This file contains the source code
 * @author 	    	: Sarayu Managoli
 * @course          : Internet of Things Embedded Firmware
 * @code leverage   : Lecture slides for Internet of Things Embedded Firmware
 * 					  https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */


#include "lpm.h"

void LPM_On()
{
	//Enables all the pins to turn ON Load Power
//	GPIO_PinOutSet(I2C0_ENABLE_PORT,I2C0_ENABLE_PIN);
	GPIO_PinOutSet(I2C0_SCL_PORT, I2C0_SCL_PIN);
	GPIO_PinOutSet(I2C0_SDA_PORT, I2C0_SDA_PIN);

	timerWaitMs(80); //Time needed for power to stabilize = 80ms
}

void LPM_Off()
{
	//Disables all the pins to turn OFF Load Power
//	GPIO_PinOutClear(I2C0_ENABLE_PORT, I2C0_ENABLE_PIN);
	GPIO_PinOutClear(I2C0_SCL_PORT, I2C0_SCL_PIN);
	GPIO_PinOutClear(I2C0_SDA_PORT, I2C0_SDA_PIN);
}
