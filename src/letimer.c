/*
 * letimer.c
 *
 *  Created on: Jan 29, 2020
 *      Author: Gitanjali Suresh
 *   Reference: https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/group__LETIMER.html
 */

#include "letimer.h"
extern bool interrupt_flag;
/* Global variable declarations */
uint16_t On_val;
uint32_t overflow_count = 0;

/* Function to initialize LETIMER0 */
void letimer_Init(void)
{
	LETIMER_Init_TypeDef Init;

	Init.bufTop = false;
	Init.comp0Top = true;
	Init.debugRun = false;
	Init.enable = false;
	Init.out0Pol = 0;
	Init.out1Pol = 0;
	Init.repMode = letimerRepeatFree;
	Init.ufoa0 = letimerUFOANone;
	Init.ufoa1 = letimerUFOANone;
	Init.topValue = 0;

	LETIMER_Init(LETIMER0,&Init);
	compute_CompVal();
	LETIMER_CompareSet(LETIMER0,0,On_val);
	//	LETIMER_CompareSet(LETIMER0,1,0xFFFF);
	LETIMER_IntEnable(LETIMER0,LETIMER_IEN_UF);
	LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
	NVIC_EnableIRQ(LETIMER0_IRQn);
}

/* Function to compute the COMP0 register values for ON and OFF times */
void compute_CompVal(void)
{
	uint32_t clock_freq;
	clock_freq = CMU_ClockFreqGet(cmuClock_LETIMER0);
	On_val = ((On_Time)*clock_freq)/1000;
}

/* LETIMER0 Interrupt Handler */
void LETIMER0_IRQHandler(void)
{
	CORE_DECLARE_IRQ_STATE;
	uint32_t interrupt = LETIMER_IntGet(LETIMER0);
	if(interrupt & LETIMER_IF_COMP1)
	{
		LETIMER_CompareSet(LETIMER0, 1, 0xFFFF); //Load values to COMP1
		LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1); //Disable COMP1 interrupt

		if(eNextState == WRITE_START)
		{
			gecko_external_signal(0x03);
		}
		else if(eNextState == READ_START)
		{
			gecko_external_signal(0x05);
		}

	}
	if(interrupt & LETIMER_IF_UF)
	{
		overflow_count++;
		CORE_ENTER_CRITICAL(); //Critical section starts
		interrupt_flag = true; //Set the event
		eNextState = POWER_UP; //Select the next state after POWER_OFF
		gecko_external_signal(0x02);
		CORE_EXIT_CRITICAL(); //Critical section ends
		LETIMER_CompareSet(LETIMER0, 0, On_val);
	}
	LETIMER_IntClear(LETIMER0, interrupt); //Clear LETIMER0 interrupt
}

/* Function to get the run time in milliseconds */
uint32_t timerGetRunTimeMilliseconds(void)
{
	volatile uint32_t milli_sec, curr_ticks, tot_ticks;
	uint32_t clock_freq;
	curr_ticks = LETIMER_CounterGet(LETIMER0);
	tot_ticks = LETIMER_CompareGet(LETIMER0,0);
	clock_freq = CMU_ClockFreqGet(cmuClock_LETIMER0);
	milli_sec = ((tot_ticks - curr_ticks)*1000)/clock_freq;
	milli_sec += (overflow_count*tot_ticks);
	return milli_sec;
}

///* Function to wait for a given millisecond */
//void timerSetEventInMs(uint32_t ms_until_wakeup)
//{
//	uint32_t wait_val, curr_val, tot_val, new_val;
//	uint32_t clock_freq = CMU_ClockFreqGet(cmuClock_LETIMER0);
//	wait_val = (ms_until_wakeup * clock_freq)/1000;
//	curr_val = LETIMER_CounterGet(LETIMER0);
//	tot_val = LETIMER_CompareGet(LETIMER0,0);
//	if(curr_val >= wait_val)
//	{
//		new_val = curr_val - wait_val;
//	}
//	else
//	{
//		new_val = tot_val - (wait_val - curr_val);
//	}
//	LETIMER_CompareSet(LETIMER0,1,new_val);
//	LETIMER_IntEnable(LETIMER0,LETIMER_IEN_COMP1);
//}
