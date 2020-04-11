/*
 * @filename : main.h
 *
 *  @date : Jan 29, 2020
 *  @description : main header file
 *
 *    	@author : pshiralagi
 *    	@reference : https://siliconlabs.github.io/Gecko_SDK_Doc/efr32bg13/html/index.html
 */
#ifndef main_h
#define main_h

enum states {
	button_press,
	num_states
};//External states

#include "gecko_configuration.h"
#include "gpio.h"
#include "native_gecko.h"
#include "letimer.h"
#include "cmu.h"
#include "energy.h"
#include <em_emu.h>
#include <sleep.h>
#include <em_core.h>
#include<gatt_db.h>
//#include<infrastructure.h>
#include "gecko_ble_errors.h"
#include "display.h"
#include "ble_mesh_device_type.h"
#include "app.h"

#endif



