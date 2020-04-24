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
	LOG_INFO("Client address is %d",client_addr);
	LOG_INFO("request_on_off - %d",request->on_off);
	LOG_INFO("transition_ms = %d",transition_ms);

	switch(client_addr)
	{
	case 1:
		if(request->on_off == 1)
		{
	    	displayPrintf(DISPLAY_ROW_TEMPVALUE, "Button Pressed");
	    	psDataSave(AUTHORIZED_PERSONNEL, &authorized_personnel, sizeof(authorized_personnel));
	    	if(authorized_personnel)
	    	{
	    		GPIO_ExtIntConfig(MOTION_PORT, MOTION_PIN, MOTION_PIN, true, true, true);
	    		authorized_personnel = 0;
	    	}
	    	else
	    	{
	    		GPIO_ExtIntConfig(MOTION_PORT, MOTION_PIN, MOTION_PIN, true, true, false);
	    		authorized_personnel = 1;
	    	}
		}

		break;
	case 2:
		LOG_INFO("SSSSSSSSSSSAAAAAAAAAAAARRRRRRRRRRRIIIIIIIIIIIYYYYYYYYYUUUUUUUUUU");
		if(request->on_off == 1)
		{
			rec_temp = 1;
			rec_acc = 0;
		}
		else
		{
			rec_acc = 1;
			rec_temp = 0;
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
	LOG_INFO("In level_request function");
	LOG_INFO("Level value - %d",request->level);
	LOG_INFO("Client addr= %d", client_addr);
	switch(client_addr)
	{
	case 1:
		LOG_INFO("Ultrasonic Data - %d", request->level);
		break;
	case 2:
//		update_level_state(2, request->level);
		break;
	}
}
