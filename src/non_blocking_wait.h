/*
 * @filename	  : non_blocking_wait.h
 * @description	  : This file contains header files for non_blocking_wait.c
 * @author 		  : Sarayu Managoli
 * @course        : Internet of Things Embedded Firmware
 * @code leverage : Lecture slides for Internet of Things Embedded Firmware
 *					https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */

#ifndef SRC_NON_BLOCKING_WAIT_H_
#define SRC_NON_BLOCKING_WAIT_H_

#include "src/letimer.h"
#include "stdint.h"
#include "stdbool.h"
#include "em_timer.h"
#include "em_letimer.h"
#include "em_cmu.h"

/*******************************************************************************
 **************************    FUNCTION PROTOTYPES    **************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief   Wait function in microseconds
 *
 * @detail  Generates an interrupt when the counter counts upto the time input
 *
 * @return  Void
 *****************************************************************************/

void timerWaitMs(uint32_t ms_wait);

#endif /* SRC_NON_BLOCKING_WAIT_H_ */
