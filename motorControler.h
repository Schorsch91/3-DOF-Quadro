/*
 * motorControler.h
 *
 *  Created on: Aug 4, 2013
 *      Author: Avionics Lab PC13-G4
 */

#ifndef MOTORCONTROLER_H_
#define MOTORCONTROLER_H_


//Werte 0-255 einstellbar
#define ADDMOTOR1	0x29
#define ADDMOTOR2	0x2A
#define ADDMOTOR3	0x2B
#define ADDMOTOR4	0x2C



void initMotorControler();
void writeMotorValue(int motor, int speed);
void stopMotor(int Motor);
void stopAllMotors();

#endif /* MOTORCONTROLER_H_ */
