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
	/*	I2C enables	*/
	GPIO_DriveStrengthSet(SCL_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(SCL_port, SCL_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(SDA_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(SDA_port, SDA_pin, gpioModePushPull, false);
	GPIO_DriveStrengthSet(enable_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(enable_port, enable_pin, gpioModePushPull, false);
	GPIO_PinOutSet(enable_port,enable_pin);
	/*	PB0 Button initialization */
	GPIO_PinModeSet(PB0_Port, PB0_Pin, gpioModeInputPull, true);
	/*	PB0 Button initialization */
	GPIO_PinModeSet(PB1_Port, PB1_Pin, gpioModeInputPull, true);
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
	static uint8_t irq_flg = 0;
	  if (irq_flg == 0)
	  {
		  gpioLed0SetOn();
		  irq_flg = 1;
	  }
	  else if (irq_flg == 1)
	  {
		  gpioLed0SetOff();
		  irq_flg = 0;
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

/***************************************************************************//**
 * This is a callback function that is invoked each time a GPIO interrupt
 * in one of the pushbutton inputs occurs. Pin number is passed as parameter.
 *
 * @param[in] pin  Pin number where interrupt occurs
 *
 * @note This function is called from ISR context and therefore it is
 *       not possible to call any BGAPI functions directly. The button state
 *       change is signaled to the application using gecko_external_signal()
 *       that will generate an event gecko_evt_system_external_signal_id
 *       which is then handled in the main loop.
 ******************************************************************************/
void gpioint(uint8_t pin)
{
  if (pin == PB0_Pin)
  {
    gecko_external_signal(0x40);
  }
}

/***************************************************************************//**
 * Enable button interrupts for PB0. Both GPIOs are configured to trigger
 * an interrupt on the rising edge (button released).
 ******************************************************************************/
void enable_button_interrupts(void)
{
  GPIOINT_Init();

  /* configure interrupt for PB0 and PB1, both falling and rising edges */
  GPIO_ExtIntConfig(PB0_Port, PB0_Pin, PB0_Pin, true, true, true);

  /* register the callback function that is invoked when interrupt occurs */
  GPIOINT_CallbackRegister(PB0_Pin, gpioint);
}
