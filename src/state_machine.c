/*
 * @filename	  : state_machine.c
 * @description	  : This file contains source code
 * @author 		  : Sarayu Managoli
 * @course        : Internet of Things Embedded Firmware
 * @code leverage : Lecture slides for Internet of Things Embedded Firmware
 *					https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */
#include "state_machine.h"

bool interrupt_flag;


void state(void)
{

		switch(eNextState)
		{

		case POWER_OFF: //LOG_INFO("POWER_OFF STATE");
			eNextState = POWER_UP;
			LPM_Off();  //Turn off GPIO pins for I2C
			SLEEP_SleepBlockEnd(sleepEM2);
			CORE_DECLARE_IRQ_STATE;
			CORE_ENTER_CRITICAL(); //Critical section starts
			interrupt_flag = false;
			CORE_EXIT_CRITICAL(); //Critical section ends
		break;

		case POWER_UP:
//			LOG_INFO("POWER_UP STATE");
			eNextState = WRITE_START;
			SLEEP_SleepBlockBegin(sleepEM2);
			LPM_On(); //Turn on GPIO pins for I2C
		break;

		case WRITE_START:
//			LOG_INFO("WRITE_START STATE");
			eNextState = WRITE_COMPLETE;
			I2C_Write(); //Initiate I2C write
		break;

		case WRITE_COMPLETE:
//			LOG_INFO("WRITE_COMPLETE STATE");
			eNextState = READ_START;
			timerWaitMs(10); //Wait for write complete
		break;

		case READ_START:
//			LOG_INFO("READ_START STATE");
			eNextState = READ_COMPLETE;
			I2C_Read(); //Initiate I2C read
		break;

		case READ_COMPLETE:
//			LOG_INFO("READ_COMPLETE STATE");
			eNextState = POWER_OFF;
			Get_Humidity(); //Calculate temperature read
			gecko_external_signal(0x01); //Setting signal event for next state
		break;
		}
}
