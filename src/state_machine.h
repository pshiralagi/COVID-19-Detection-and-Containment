/*
 * @filename	: state_machine.h
 * @description	: This file contains header files for state_machine.c
 * @author 		: Sarayu Managoli
 * @course      : Internet of Things Embedded Firmware
 * 				  https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <stdbool.h>
#include "em_core.h"
#include "main.h"

#define SCHEDULER_SUPPORTS_DISPLAY_UPDATE_EVENT 1

typedef enum
{
	POWER_OFF=1,
	POWER_UP,
	WRITE_START,
	WRITE_COMPLETE,
	READ_START,
	READ_COMPLETE
}eState;
extern eState eNextState;

/*******************************************************************************
 **************************    FUNCTION PROTOTYPES    **************************
 ******************************************************************************/

/**************************************************************************//**
 * @brief   State machine implementation
 *
 * @detail  Different states for I2C transfer of Temperature
 *
 * @return  Void
 *****************************************************************************/
void state(void);

#endif /* SRC_STATE_MACHINE_H_ */
