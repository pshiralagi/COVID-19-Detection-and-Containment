/*
 * @filename	: non_blocking_wait.c
 * @description	: This file contains the source code
 * @author 		: Sarayu Managoli
 * @course      : Internet of Things Embedded Firmware
 * 				  https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */

#include "non_blocking_wait.h"

void timerWaitMs(uint32_t ms_wait)
{
	uint32_t ticks = 0, delay = 0;
	uint32_t delay_s = 0;

	//To get the current number of ticks (in seconds)
	ticks=LETIMER_CounterGet(LETIMER0);

	//Converts the given ms_wait value to seconds
	delay_s = (ms_wait*CMU_ClockFreqGet(cmuClock_LETIMER0))/1000;

	if(ticks >= delay_s)
	{
		delay = (ticks - delay_s); //Wait until the current ticks is equal to the difference between the ticks and delays
	}
	else
	{
		delay = (LETIMER_CompareGet(LETIMER0,0) - (delay_s - ticks)); //Calculation for when the tick values is lesser than the delay mentioned
	}

	LETIMER_CompareSet(LETIMER0, 1, delay);
	LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
}
