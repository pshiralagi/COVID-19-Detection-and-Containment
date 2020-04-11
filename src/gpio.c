/*
 * gpio.c
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 *      Co-author :Pshiralagi
 */
#include "gpio.h"
#include "em_gpio.h"
#include <string.h>




/*
 * @brief : Function to initialize required GPIO pins in required mode
 *
 */

void gpioInit()
{
	/*	PB0 Button initialization */
	GPIO_PinModeSet(PB0_Port, PB0_Pin, gpioModeInputPull, true);
	GPIO_IntConfig(PB0_Port, PB0_Pin, true, true, true);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}
void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}
void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}
void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

/*	@brief : Function to toggle LED based on flag set in interrupt (Assignment 2)	*/
void toggleLed(void)
{
	  if (irq_flg == 0)
	  {
		  gpioLed0SetOn();
	  }
	  if (irq_flg == 1)
	  {
		  gpioLed0SetOff();
	  }
}


void gpioEnableDisplay(void)
{
	GPIO_PinOutSet(LCD_Port,LCD_ENABLE);
}
void gpioSetDisplayExtcomin(bool state)
{
	if(state == true)
		GPIO_PinOutSet(LCD_Port,LCD_EXTCOMIN);
	else
		GPIO_PinOutClear(LCD_Port,LCD_EXTCOMIN);
}

void GPIO_EVEN_IRQHandler()
{
	uint32_t flag = GPIO_IntGet();
	GPIO_IntClear(flag);
	if(flag & 0x40)
		gecko_external_signal(button_press);
}
