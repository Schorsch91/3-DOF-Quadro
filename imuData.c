/*
 * imuData.c
 *
 *  Created on: May 21, 2013
 *      Author: Avionics Lab-3
 */

#include "defines.h"

void imu_init(){

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
    print_dbg("Probe test:\tPASS\r\n");
  }
  else
  {
    // display test result to user
    print_dbg("Probe test:\tFAIL\r\n");
  }
  print_dbg("test2");

	 //---------------------------------IMU-INITIALISATION------------------------------------------//
  //ITG1 BESCHREIBEN
  char twi_buffer[20];
  // TWI chip address to communicate with
  ITG1.chip = ITG_EEPROM_ADDR;
  // TWI address/commands to issue to the other chip (node)
  ITG1.addr = 0x16;
  // Length of the TWI data address segment (1-3 bytes)
  ITG1.addr_length = 1;
  // Where to find the data to be writte
  ITG1.buffer = (void*)twi_buffer;
  // How many bytes do we want to write
  ITG1.length = 1;
  // perform a write access

  twi_buffer[0] = 0x18;
  status = twi_master_write(&AVR32_TWI, &ITG1);

  // check write result
  if (status == TWI_SUCCESS)
  {
    // display test result to user
    print_dbg("ITG1 WriteTest");
  }
  else
  {
    // display test result to user
    print_dbg("ITG1 WriteTest_FAIL");
  }
//ITG2 BESCHREIBEN

  // TWI chip address to communicate with
  ITG2.chip = ITG_EEPROM_ADDR;
  // TWI address/commands to issue to the other chip (node)
  ITG2.addr = 0x3E;
  // Length of the TWI data address segment (1-3 bytes)
  ITG2.addr_length = EEPROM_ADDR_LGT;
  // Where to find the data to be written
  ITG2.buffer = (void*)twi_buffer;
  // How many bytes do we want to write
  ITG2.length = 1;
  twi_buffer[0] = 0x01;
  // perform a write access
  status = twi_master_write(&AVR32_TWI, &ITG2);

  // check write result
  if (status == TWI_SUCCESS)
  {
    // display test result to user
    print_dbg("ITG2 WriteTest");
  }
  else
  {
    // display test result to user
    print_dbg("ITG2 WriteTest_FAIL");
  }

  //ADXL1 BESCHREIBEN

  // TWI chip address to communicate with
  ADXL1.chip = ACC_EEPROM_ADDR;
  // TWI address/commands to issue to the other chip (node)
  ADXL1.addr = 0x2D;
  // Length of the TWI data address segment (1-3 bytes)
  ADXL1.addr_length = EEPROM_ADDR_LGT;
  // Where to find the data to be written
  ADXL1.buffer = (void*)twi_buffer;
  // How many bytes do we want to write
  ADXL1.length = 1;
  twi_buffer[0] = 0x08;
  // perform a write access
  status = twi_master_write(&AVR32_TWI, &ADXL1);

  // check write result
  if (status == TWI_SUCCESS)
  {
    // display test result to user
    print_dbg("ADXL1 WriteTest");
  }
  else
  {
    // display test result to user
    print_dbg("ADXL1 WriteTest_FAIL");
  }
  //ADXL2 BESCHREIBEN

  // TWI chip address to communicate with
  ADXL2.chip = ACC_EEPROM_ADDR;
  // TWI address/commands to issue to the other chip (node)
  ADXL2.addr = 0x31;
  // Length of the TWI data address segment (1-3 bytes)
  ADXL2.addr_length = EEPROM_ADDR_LGT;
  // Where to find the data to be written
  ADXL2.buffer = (void*)twi_buffer;
  // How many bytes do we want to write
  ADXL2.length = 1;
  twi_buffer[0] = 0x09;
  // perform a write access
  status = twi_master_write(&AVR32_TWI, &ADXL2);

  // check write result
  if (status == TWI_SUCCESS)
  {
    // display test result to user
    print_dbg("ADXL2 WriteTest");
  }
  else
  {
    // display test result to user
    print_dbg("ADXL2 WriteTest_FAIL");
  }

}

void readRegister(unsigned int reg_addr, unsigned short chip_addr, char result[6]){

	char readData[6];
	int succes;

	twi_package_t temp;
	temp.chip = chip_addr;
	temp.addr_length = EEPROM_ADDR_LGT;
	temp.length = 6;
	temp.addr = reg_addr;
	temp.buffer = (void*)readData;

	succes = twi_master_read(&AVR32_TWI, &temp);

	result[0] = readData[0];
	result[1] = readData[1];
	result[2] = readData[2];
	result[3] = readData[3];
	result[4] = readData[4];
	result[5] = readData[5];
}



void read_sensor(sensorDaten* read){

	  char GhighX,GhighY,GhighZ, AhighX, AhighY, AhighZ;	 //Hight Data
	  short GlowX,GlowY,GlowZ, AlowX, AlowY, AlowZ;			//low Data
	  short GX,GY,GZ, AX, AY, AZ, offX, offY, offZ;			//shifted comlete Data
	  char rohAcc[6], rohGyro[6];

	//------------------GYRO DATA-----------------------//


			  readRegister(0x1D,ITG_EEPROM_ADDR,rohGyro);

			  GhighX = rohGyro[0];
			  GlowX = rohGyro[1];
			  GhighY = rohGyro[2];
			  GlowY = rohGyro[3];
			  GhighZ = rohGyro[4];
			  GlowZ = rohGyro[5];


		  	 //shifting high and low
		  	 GX = (GhighX << 8) |GlowX;
		  	 GY = (GhighY << 8) |GlowY;
		  	 GZ = (GhighZ << 8) |GlowZ;

	//----------------ACCL-DATA-------------------------//

			  readRegister(0x32,ACC_EEPROM_ADDR,rohAcc);

			  AlowX = rohAcc[0];
			  AhighX = rohAcc[1];
			  AlowY = rohAcc[2];
			  AhighY = rohAcc[3];
			  AlowZ = rohAcc[4];
			  AhighZ = rohAcc[5];


		  	 //shifting high and low
		  	 AX = (AhighX << 8) |AlowX;
		  	 AY = (AhighY << 8) |AlowY;
		  	 AZ = (AhighZ << 8) |AlowZ;
	//---------------OFFSET ACCL DATA ----------------//
//		  	 offX = readRegister(0x1E,ACC_EEPROM_ADDR);
//		  	 offY = readRegister(0x1F,ACC_EEPROM_ADDR);
//		  	 offZ = readRegister(0x20,ACC_EEPROM_ADDR);


		  	 read->acc_x = AX;
		  	 read->acc_y = AY;
		  	 read->acc_z = AZ;
		  	 read->gyro_x = GX;
		  	 read->gyro_y = GY;
		  	 read->gyro_z = GZ;
//		  	 read->aoffX = offX;
//		  	 read->aoffY = offY;
//		  	 read->aoffZ = offZ;

}



void calibrate_all(sensorDaten *data){


	char disp[30];

	int counter = 0;
	int calibAX = 0;
	int calibAY = 0;
	int calibAZ = 0;
	int calibGX = 0;
	int calibGY = 0;
	int calibGZ = 0;
	int dispCounter = 0;

	 while(counter < 1000){
			 dispCounter++;

			 if(dispCounter % 50){
				   sprintf(disp,"Calibration...%.0f",(counter/10.0));
				   dip204_clear_display();
				   dip204_set_cursor_position(1,1);
				   dip204_write_string(disp);
			 }


		 sensorDaten temp = {0};

		  read_sensor(&temp);
		  delay_ms(2);

		  calibAX += temp.acc_x;
		  calibAY += temp.acc_y;
		  calibAZ += temp.acc_z;

		  calibGX += temp.gyro_x;
		  calibGY += temp.gyro_y;
		  calibGZ += temp.gyro_z;

		  counter++;
	 }



	 data->acc_x_bias = -(calibAX/counter);
	 data->acc_y_bias = -(calibAY/counter);
	 data->acc_z_bias = -(calibAZ/counter)+265;//265 = 1000/4

	 data->gyro_x_bias = -(calibGX/counter);
	 data->gyro_y_bias = -(calibGY/counter);
	 data->gyro_z_bias = -(calibGZ/counter);


	   dip204_clear_display();
}

void RPYtoQuat(Quaternion *quat, RPY *rpy){
	quat->q0 = (cosf((rpy->roll / 2) * (M_PI / 180.0)) * cosf((rpy->pitch / 2) * (M_PI / 180.0)) * cosf((rpy->yaw / 2)* (M_PI / 180.0))) + (sinf((rpy->roll / 2) * (M_PI / 180.0)) * sinf((rpy->pitch / 2) * (M_PI / 180.0)) * sinf((rpy->yaw / 2) * (M_PI / 180.0)));
	quat->q1 = (sinf((rpy->roll / 2) * (M_PI / 180.0)) * cosf((rpy->pitch / 2) * (M_PI / 180.0)) * cosf((rpy->yaw / 2)* (M_PI / 180.0))) - (cosf((rpy->roll / 2) * (M_PI / 180.0)) * sinf((rpy->pitch / 2) * (M_PI / 180.0)) * sinf((rpy->yaw / 2) * (M_PI / 180.0)));
	quat->q2 = (cosf((rpy->roll / 2) * (M_PI / 180.0)) * sinf((rpy->pitch / 2) * (M_PI / 180.0)) * cosf((rpy->yaw / 2)* (M_PI / 180.0))) + (sinf((rpy->roll / 2) * (M_PI / 180.0)) * cosf((rpy->pitch / 2) * (M_PI / 180.0)) * sinf((rpy->yaw / 2) * (M_PI / 180.0)));
	quat->q3 = (cosf((rpy->roll / 2) * (M_PI / 180.0)) * cosf((rpy->pitch / 2) * (M_PI / 180.0)) * sinf((rpy->yaw / 2)* (M_PI / 180.0))) - (sinf((rpy->roll / 2) * (M_PI / 180.0)) * sinf((rpy->pitch / 2) * (M_PI / 180.0)) * cosf((rpy->yaw / 2) * (M_PI / 180.0)));
}

double quatAbs(Quaternion *quat){
	double q0,q1,q2,q3;
	q0 = quat->q0;
	q1 = quat->q1;
	q2 = quat->q2;
	q3 = quat->q3;

	double abs = sqrt(pow(q0,2.0) + pow(q1,2.0) + pow(q2,2.0) + pow(q3,2.0));
	return abs;

}

void normQuat(Quaternion *quat){
	Quaternion temp;
	temp.q0 = quat->q0;
	temp.q1 = quat->q1;
	temp.q2 = quat->q2;
	temp.q3 = quat->q3;

	double abs = quatAbs(&temp);

	quat->q0 = temp.q0 / abs;
	quat->q1 = temp.q1 / abs;
	quat->q2 = temp.q2 / abs;
	quat->q3 = temp.q3 / abs;

}

void quatMultiplication(Quaternion *quatx, Quaternion *quaty, Quaternion *resT){
	Quaternion quat1, quat2, res;

	quat1.q0 = quatx->q0;
	quat1.q1 = quatx->q1;
	quat1.q2 = quatx->q2;
	quat1.q3 = quatx->q3;

	quat2.q0 = quaty->q0;
	quat2.q1 = quaty->q1;
	quat2.q2 = quaty->q2;
	quat2.q3 = quaty->q3;


//	  char test2[20];
//	  sprintf(test2, "q: %f,%f,%f,%f", quat2.q0, quat2.q1, quat2.q2, quat2.q3);
//	  print_dbg(test2);



	res.q0 = (quat1.q0 * quat2.q0) - (quat1.q1 * quat2.q1) - (quat1.q2 * quat2.q2) - (quat1.q3 * quat2.q3);
	res.q1 = (quat1.q0 * quat2.q1) + (quat1.q1 * quat2.q0) - (quat1.q2 * quat2.q3) + (quat1.q3 * quat2.q2);
	res.q2 = (quat1.q0 * quat2.q2) + (quat1.q1 * quat2.q3) + (quat1.q2 * quat2.q0) - (quat1.q3 * quat2.q1);
	res.q3 = (quat1.q0 * quat2.q3) - (quat1.q1 * quat2.q2) + (quat1.q2 * quat2.q1) + (quat1.q3 * quat2.q0);



		resT->q0 = res.q0;
		resT->q1 = res.q1;
		resT->q2 = res.q2;
		resT->q3 = res.q3;
}

void quatToRotMatrix(Quaternion *quatT, RotMatrix *rotT){
	Quaternion quat;
	RotMatrix rot;

	quat.q0 = quatT->q0;
	quat.q1 = quatT->q1;
	quat.q2 = quatT->q2;
	quat.q3 = quatT->q3;

	rot.a1 = pow(quat.q0,2.0) + pow(quat.q1, 2.0) - pow(quat.q2, 2.0) - pow(quat.q3, 2.0);
	rot.a2 = 2*((quat.q1 * quat.q2) + (quat.q0 * quat.q3));
	rot.a3 = 2*((quat.q1 * quat.q3) - (quat.q0 * quat.q2));

	rot.b1 = 2*((quat.q1 * quat.q2) - (quat.q0 + quat.q3));
	rot.b2 = pow(quat.q0, 2.0) - pow(quat.q1, 2.0) + pow(quat.q2, 2.0) - pow(quat.q3, 2.0);
	rot.b3 = 2*((quat.q2 * quat.q3) + (quat.q0 * quat.q1));

	rot.c1 = 2*((quat.q1 * quat.q3) + (quat.q0 * quat.q2));
	rot.c2 = 2*((quat.q2 + quat.q3) - (quat.q0 * quat.q1));
	rot.c3 = pow(quat.q0, 2.0) - pow(quat.q1, 2.0) - pow(quat.q2, 2.0) - pow(quat.q3, 2.0);


	rotT->a1 = rot.a1;
	rotT->a2 = rot.a2;
	rotT->a3 = rot.a3;

	rotT->b1 = rot.b1;
	rotT->b2 = rot.b2;
	rotT->b3 = rot.b3;

	rotT->c1 = rot.c1;
	rotT->c2 = rot.c2;
	rotT->c3 = rot.c3;


}





