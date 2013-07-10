/*
 * defines.h
 *
 *  Created on: May 21, 2013
 *      Author: Avionics Lab-3
 */

#ifndef DEFINES_H_
#define DEFINES_H_


#include <avr32/io.h>
#include "board.h"
#include "print_funcs.h"
#include "gpio.h"
#include "pm.h"
#include "intc.h"
#include "twi.h"
#include "delay.h"
#include <stdio.h>
#include "usart.h"
#include "power_clocks_lib.h"
#include "spi.h"
#include "dip204.h"
#include "tc.h"
#include <stdbool.h>
#include "math.h"
#include "myMath.h"
#include "kalmann.h"
#include "imuData.h"
#include "motorControler.h"

#define ACC_EEPROM_ADDR 	0x53		//for ACC
#define ITG_EEPROM_ADDR     0x68        // EEPROM's TWI address for ITG
#define EEPROM_ADDR_LGT       1         // Address length of the EEPROM memory
#define VIRTUALMEM_ADDR_START 0x68    	// Address of the virtual mem in the EEPROM
#define TWI_SPEED             50000     // Speed of TWI


#define BUTTON0            	GPIO_PUSH_BUTTON_0
#define BUTTON1    			GPIO_PUSH_BUTTON_1
#define BUTTON2			    GPIO_PUSH_BUTTON_2






#endif /* DEFINES_H_ */
