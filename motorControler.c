/*
 * motorControler.c
 *
 *  Created on: Aug 4, 2013
 *      Author: Avionics Lab PC13-G4
 */

#include "defines.h"




void initMotorControler(){

	static const gpio_map_t TWI_GPIO_MAP =
	  {
	#if BOARD == EVK1100
	    {AVR32_TWI_SDA_0_0_PIN, AVR32_TWI_SDA_0_0_FUNCTION},
	    {AVR32_TWI_SCL_0_0_PIN, AVR32_TWI_SCL_0_0_FUNCTION}
	#elif BOARD == EVK1101
	    {AVR32_TWI_SDA_0_0_PIN, AVR32_TWI_SDA_0_0_FUNCTION},
	    {AVR32_TWI_SCL_0_0_PIN, AVR32_TWI_SCL_0_0_FUNCTION}
	#elif BOARD == STK1000
	    {AVR32_TWI_SDA_0_PIN, AVR32_TWI_SDA_0_FUNCTION},
	    {AVR32_TWI_SCL_0_PIN, AVR32_TWI_SCL_0_FUNCTION}
	#else
	#  error The TWI configuration to use in this example is missing.
	#endif
	  };



#if BOARD == EVK1100 || BOARD == EVK1101

  // Switch to oscillator 0
  pm_switch_to_osc0(&AVR32_PM, FOSC0, OSC0_STARTUP);

#endif

  // Init debug serial line
  init_dbg_rs232(FOSC0);

//----------------------------------TWI INITIALISATION--------------------------------//
  // TWI gpio pins configuration
  gpio_enable_module(TWI_GPIO_MAP, sizeof(TWI_GPIO_MAP) / sizeof(TWI_GPIO_MAP[0]));

  // options settings
  opt.pba_hz = FOSC0;
  opt.speed = TWI_SPEED;
  opt.chip = ITG_EEPROM_ADDR;

  // initialize TWI driver with options
  status = twi_master_init(&AVR32_TWI, &opt);
  // check init result
  if (status == TWI_SUCCESS)
  {
    // display test result to user
    print_dbg("MotorController:\tPASS\r\n");
  }
  else
  {
    // display test result to user
    print_dbg("MotorController:\tFAIL\r\n");
  }



}


void writeMotorValue(int motor, int speed){
	twi_package_t readTwi;
	char twi_buffer[20];

	 status = 0;

	  // TWI chip address to communicate with
	 switch(motor){
		case 1:		readTwi.chip = ADDMOTOR1;
		break;
		case 2:		readTwi.chip = ADDMOTOR2;
		break;
		case 3:		readTwi.chip = ADDMOTOR3;
		break;
		case 4:		readTwi.chip = ADDMOTOR4;
		break;
		default:	status = 1; //If wrong motor-index, set to 1
	 }

	  //register -> Es gibt nur ein Register, darum egal
	  readTwi.addr = speed;
	  // Length of the TWI data address segment (1-3 bytes)
	  readTwi.addr_length = 1;
	  // Where to find the data to be written
	  readTwi.buffer = (void*)twi_buffer;
	  // How many bytes do we want to write
	  readTwi.length = 1;
	  // perform a write access
	  twi_buffer[0] = speed;

	  //Only write if all parameters are ok
	  if(status !=1 && speed >= 0 && speed <=255){
	 	status = twi_master_write(&AVR32_TWI, &readTwi);
	  }



}


void stopMotor(int Motor){

	writeMotorValue(Motor, 0);

}


void stopAllMotors(){

	writeMotorValue(1, 0);
	writeMotorValue(2, 0);
	writeMotorValue(3, 0);
	writeMotorValue(4, 0);

}



