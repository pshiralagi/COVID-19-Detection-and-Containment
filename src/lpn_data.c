/*
 * @filename lpn_data.c
 * @author	Pavan Shiralagi
 * @brief	File containing functions to receive data from LPNs
 */

#include "lpn_data.h"

static bool rec_temp = 0, rec_acc = 0;

void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
	switch(client_addr)
	{
	case 3:
		if(request->on_off == 1)
		{
	    	if(authorized_personnel)
	    	{
	    		GPIO_ExtIntConfig(MOTION_PORT, MOTION_PIN, MOTION_PIN, true, true, true);
	    		authorized_personnel = 0;
	    		LOG_INFO("Authorized personnel leaving");
	    		displayPrintf(DISPLAY_ROW_AUTHORITY, "Authority Left");
	    	}
	    	else
	    	{
	    		LOG_INFO("Authorized personnel entered");
	    		GPIO_ExtIntConfig(MOTION_PORT, MOTION_PIN, MOTION_PIN, true, true, false);
	    		authorized_personnel = 1;
	    		displayPrintf(DISPLAY_ROW_AUTHORITY, "Authority Present");
	    	}
	    	psDataSave(AUTHORIZED_PERSONNEL, &authorized_personnel, sizeof(authorized_personnel));
		}

		break;
	case 2:
		if(request->on_off == 1)
		{
			rec_temp = 0;
			rec_acc = 1;
		}
		else
		{
			rec_acc = 0;
			rec_temp = 1;
		}
		break;
	}
}

void level_request(uint16_t model_id,
        		   uint16_t element_index,
				   uint16_t client_addr,
				   uint16_t server_addr,
				   uint16_t appkey_index,
				   const struct mesh_generic_request *request,
				   uint32_t transition_ms,
				   uint16_t delay_ms,
				   uint8_t request_flags)
{
	static float data = 0;
	switch(client_addr)
	{
	case 3:
		data = (float)(request->level)/100;
		LOG_INFO("Ultrasonic Data ----- %f", data);
		displayPrintf(DISPLAY_ROW_ULTRASONIC, "%.2f",data);
		break;
	case 2:
		if (rec_temp)
		{
			data = (float)(request->level)/100;
			LOG_INFO("Temperature Data ----- %f", data);
			displayPrintf(DISPLAY_ROW_TEMPERATURE, "%.2f", data);
			psDataLoad(MAX_TEMP, &high_temp, sizeof(high_temp));
			if (data>high_temp)
			{
				psDataSave(MAX_TEMP, &high_temp, sizeof(high_temp));
			}
			if (data>34)
			{
				redAlert();
				displayPrintf(DISPLAY_ROW_ALERT_PATIENT, "High temperature");
			}


		}
		else if (rec_acc)
		{
			LOG_INFO("Accelerometer Data ----- %d", request->level);
			if ((request->level)>2900)
			{
				redAlert();
				displayPrintf(DISPLAY_ROW_ALERT_PATIENT, "Patient Fainted");
			}
			displayPrintf(DISPLAY_ROW_ACCELEROMETER, "%d", request->level);
		}
		break;
	}
}
