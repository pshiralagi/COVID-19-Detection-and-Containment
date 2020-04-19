/*
 * @filename	: Sleep_mode.h
 * @description	: This file contains the header files for Sleep_mode.c
 * @author 		: Sarayu Managoli
 * @course      : Internet of Things Embedded Firmware
 * 				  https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */

#ifndef SRC_SLEEP_MODE_H_
#define SRC_SLEEP_MODE_H_

#include "gecko_configuration.h"
#include "native_gecko.h"
#include "sleep.h"
#include "log.h"
#include "cmu.h"
#include "em_emu.h"



/*******************************************************************************
 **************************    FUNCTION PROTOTYPES    **************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief   Initializes sleep
 *
 * @detail  This function initializes sleep functionality and sets value for sleep block
 *
 * @return  Void
 *****************************************************************************/
void sleep_initialize(void);

/**************************************************************************//**
 * @brief   Puts to sleep
 *
 * @detail  This function puts the board to deep sleep based on the energy mode
 *
 * @return  Void
 *****************************************************************************/
void sleep_mode_on(void);

/**************************************************************************//**
 * @brief   Selects sleep mode
 *
 * @detail  This function sets the sleep mode
 *
 * @return  Void
 *****************************************************************************/
void mode_Select(void);

#endif /* SRC_SLEEP_MODE_H_ */
