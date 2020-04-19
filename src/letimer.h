/*
 * letimer.h
 *
 *  Created on: Jan 29, 2020
 *      Author: Gitanjali Suresh
 */

#ifndef SRC_LETIMER_H_
#define SRC_LETIMER_H_

/******** Board Includes *******/
#include <em_letimer.h>
#include <em_cmu.h>
#include <stdint.h>
#include "main.h"
#include <native_gecko.h>


#define On_Time 3000
#define TIMER_SUPPORTS_1HZ_TIMER_EVENT	1

extern bool flag;
/********* Function Prototypes *******/
void letimer_Init(void);							/* Function to initialize LETIMER0 */
void compute_CompVal(void);							/* Function to compute the COMP0 register values for ON and OFF times */
void timerWaitUs(uint32_t wait_us);					/* Function to wait for a given microseconds */
uint32_t timerGetRunTimeMilliseconds(void);			/* Function to get the run time in milliseconds */
//void timerSetEventInMs(uint32_t ms_until_wakeup);	/* Function to wait for a given millisecond */

#endif /* SRC_LETIMER_H_ */
