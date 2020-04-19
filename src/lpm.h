/*
 * @filename	  : lpm.h
 * @description	  : This file contains header files for lpm.c
 * @author 		  : Sarayu Managoli
 * @course        : Internet of Things Embedded Firmware
 * @code leverage : Lecture slides for Internet of Things Embedded Firmware
 * 					https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */

#ifndef SRC_LPM_H_
#define SRC_LPM_H_

#include "main.h"

/*******************************************************************************
 ********************************  DEFINES  ************************************
 ******************************************************************************/

#define I2C0_SCL_PIN 10 //I2C0 SCL Pin
#define I2C0_SDA_PIN 11 //I2C0 SDA Pin
#define I2C0_ENABLE_PIN 15 //Enable Pin for I2C0
#define I2C0_SCL_PORT gpioPortC //I2C0 SCL Port
#define I2C0_SDA_PORT gpioPortC //I2C0 SDA Port
#define I2C0_ENABLE_PORT gpioPortD //Enable Port for I2C0

/*******************************************************************************
 **************************    FUNCTION PROTOTYPES    **************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief   Load Power ON
 *
 * @detail  Turns on the required GPIO pins for transaction
 *
 * @return  void
  *****************************************************************************/

void LPM_On(void);

/**************************************************************************//**
 * @brief   Load Power OFF
 *
 * @detail  Turns off the required GPIO pins for transaction
 *
 * @return  void
  *****************************************************************************/
void LPM_Off(void);

#endif /* SRC_LPM_H_ */
