/*
 * @filename lpn_data.h
 * @author	Pavan Shiralagi
 * @brief	Header file to receive data from LPNs
 */

#ifndef LPN_DATA_H_
#define LPN_DATA_H_

#include "main.h"
/* Bluetooth stack headers */
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
//Persistent storage keys
#define MAX_TEMP (0xa000)
#define AUTHORIZED_PERSONNEL (0xb000)
#define BUTTON_COUNT (0xc000)

/*
 * @brief	Callback function to handle onoff data received by publishers
 */
void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags);


/*
 * @brief	Callback function to handle level data received by publishers
 */
void level_request(uint16_t model_id,
        		   uint16_t element_index,
				   uint16_t client_addr,
				   uint16_t server_addr,
				   uint16_t appkey_index,
				   const struct mesh_generic_request *request,
				   uint32_t transition_ms,
				   uint16_t delay_ms,
				   uint8_t request_flags);

extern uint8_t authorized_personnel;
extern float high_temp;

#endif
