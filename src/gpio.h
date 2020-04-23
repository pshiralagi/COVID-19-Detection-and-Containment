/*
 * gpio.h
 *
 *  Created on: Dec 12, 2018
 *      Author: Dan Walkes
 *      Co author: Pshiralai
 */

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include "letimer.h"
#include "gpiointerrupt.h"

/*
 * @brief : Function to initialize required GPIO pins in required mode
 *
 */
void gpioInit();
void gpioLed0SetOn();	//Sets LED 0
void gpioLed0SetOff();	//Clears LED 0
void gpioLed1SetOn();	//Sets LED 1
void gpioLed1SetOff();	//Clears LED 1
void redAlert(void);
void clearAlert(void);
/*
 * @brief : Function to deinitialize required GPIO pins
 *
 */
void tempGpioDeInit(void);

/*
 * @brief : Function to reinitialize required GPIO pins after deinitializing before sleep
 *
 */
void tempGpioReInit(void);		//Resets required GPIO for LPM to restart peripherals
void toggleLed(void);	//Toggles LEDs

/*	@brief : Re-initialize GPIO pins for LPM OFF	*/
void lpm_off(void);

/*	@brief : Re-initialize GPIO pins for LPM ON	*/
void lpm_on(void);

/*	@brief : Function to enable display	*/
void gpioEnableDisplay(void);

/*	@brief : Refreshes the screen	*/
void gpioSetDisplayExtcomin(bool state);

void pirInit(void);

void motionDetected(uint8_t pin);


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
void gpioint(uint8_t pin);

/***************************************************************************//**
 * Enable button interrupts for PB0. Both GPIOs are configured to trigger
 * an interrupt on the rising edge (button released).
 ******************************************************************************/
void enable_button_interrupts(void);

#define	LED0_port gpioPortF
#define LED0_pin 4
#define LED1_port gpioPortF
#define LED1_pin 5
#define SCL_port gpioPortC
#define SCL_pin 10
#define SDA_port gpioPortC
#define SDA_pin 11
#define enable_port gpioPortD
#define enable_pin 15
#define LCD_Port gpioPortD
#define LCD_EXTCOMIN 13
#define LCD_ENABLE 15
#define PB0_Port gpioPortF
#define PB0_Pin 6
#define PB1_Port gpioPortF
#define PB1_Pin 7

#define MOTION_PORT         gpioPortD
#define MOTION_PIN          13

#define GPIO_SET_DISPLAY_EXT_COMIN_IMPLEMENTED 	1
#define GPIO_DISPLAY_SUPPORT_IMPLEMENTED		1

#endif /* SRC_GPIO_H_ */
