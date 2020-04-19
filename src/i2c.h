/*
 * @filename	  : i2c.h
 * @description	  : This file contains header files for i2c.c
 * @author 		  : Sarayu Managoli
 * @course        : Internet of Things Embedded Firmware
 * @code leverage : Lecture slides for Internet of Things Embedded Firmware
 *					https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */


#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "em_i2c.h"
#include <i2cspm.h>
#include "main.h"

/*******************************************************************************
 ********************************  DEFINES  ************************************
 ******************************************************************************/

#define SLAVE_ADDRESS 0x40
#define I2C_COMPLETE 2
#define I2C_FAIL 1

extern float Received_Data;

/*******************************************************************************
 **************************    FUNCTION PROTOTYPES    **************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief   Initialize the I2C
 *
 * @detail  Default configurations for I2C are set
 *
 * @return  Void
 *****************************************************************************/

void I2C_Initialize(void);

/**************************************************************************//**
 * @brief   Write function for I2C
 *
 * @detail  Writes the No Master Hold Mode command
 *
 * @return  Void
 *****************************************************************************/
void I2C_Write(void);

/**************************************************************************//**
 * @brief   Read function for I2C
 *
 * @detail  Reads temperature data from I2C
 *
 * @return  Float value of temperature
 *****************************************************************************/
void I2C_Read(void);

/**************************************************************************//**
 * @brief  Event Handler for I2C
 *
 * @detail
 *
 * @return  Void
 *****************************************************************************/
void I2C0_IRQHandler(void);

/**************************************************************************//**
 * @brief   Calculates the temperature
 *
 * @detail  This function initializes read from I2C and calculates the temperature
 *
 * @return  Void
 *****************************************************************************/
void Get_Temp(void);

void Temp_Buffer(void);

#endif /* SRC_I2C_H_ */
