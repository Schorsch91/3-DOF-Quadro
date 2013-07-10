/*
 * imuData.h
 *
 *  Created on: May 21, 2013
 *      Author: Avionics Lab-3
 */


#ifndef IMUDATA_H_
#define IMUDATA_H_



twi_options_t opt;
 twi_package_t ITG1, ITG2, ADXL1, ADXL2, ITG_read, ADXL_read;
 int status, i;

 typedef struct {
 int gyro_x; int gyro_y; int gyro_z;
 int acc_x; int acc_y; int acc_z;
 int gyro_x_bias; int gyro_y_bias; int gyro_z_bias;
 int acc_x_bias; int acc_y_bias; int acc_z_bias;
 int aoffX; int aoffY; int aoffZ;
 } sensorDaten;		// sensorDaten ist nun ein Typ

 typedef struct {
	 double roll;
	 double pitch;
	 double yaw;
 } RPY;

 typedef struct {
	 double q0;
	 double q1;
	 double q2;
	 double q3;
 } Quaternion;

 typedef struct {
	 double a1;
	 double a2;
	 double a3;
	 double b1;
	 double b2;
	 double b3;
	 double c1;
	 double c2;
	 double c3;
 } RotMatrix;

 void imu_init();
 void read_sensor(sensorDaten* read);
 void read_sensor(sensorDaten *read);
 void calibrate_all(sensorDaten *data);
 void RPYtoQuat(Quaternion *quat, RPY *rpy);
 double quatAbs(Quaternion *quat);
 void normQuat(Quaternion *quat);
 void quatMultiplication(Quaternion *quatx, Quaternion *quaty, Quaternion *resT);
 void quatToRotMatrix(Quaternion *quatT, RotMatrix *rotT);
 void RotMatrixToRPY(RotMatrix *rotT, RPY *rpy);




#endif /* IMUDATA_H_ */
