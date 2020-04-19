/*
 * @filename	: Sleep_mode.c
 * @description	: This file contains the source code
 * @author 		: Sarayu Managoli
 * @course      : Internet of Things Embedded Firmware
 * 				  https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */


#include "Sleep_mode.h"

SLEEP_EnergyMode_t EnergyMode = sleepEM1;


void sleep_initialize()
{
	  const SLEEP_Init_t init_sleep = {0}; //NULL because no callbacks are required
	  SLEEP_InitEx(&init_sleep);
}

void sleep_mode_on()
{
	mode_Select();
	  if(EnergyMode == sleepEM1 || EnergyMode == sleepEM2)
	  {
		  logFlush();
		  SLEEP_Sleep(); //Command applicable only for EM1 and EM2
	  }
	  else if(EnergyMode == sleepEM3)
	  {
		  logFlush();
		  EMU_EnterEM3(true); //Command applicable for EM3 only
	  }
}

void mode_Select(void)
{
	if(eNextState == POWER_OFF)
		EnergyMode = sleepEM3; //Keep in EM3 except when in I2C transfer
	else
	{
		EnergyMode = sleepEM1;
		const SLEEP_EnergyMode_t sleep_mode_blocked=(EnergyMode+1); //Sleep mode is blocked from EM2 and EM3 for EM1 and EM2 respectively
		SLEEP_SleepBlockBegin(sleep_mode_blocked);
	}
}
