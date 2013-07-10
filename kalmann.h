/*
 * kalmann.h
 *
 *  Created on: Jun 26, 2013
 *      Author: Avionics Lab-3
 */

#ifndef KALMANN_H_
#define KALMANN_H_




typedef struct {
	double A[2][2], C[2][2], Q[2][2], R[2][2];
	double Pd[2][2], Plast[2][2], K[2][2], P[2][2], I[2][2];
	double Xd[2], Xlast[2], X[2], Y[2];
} kalmannData;



void initializeKalmann(kalmannData *KD);
void kalmannFilter(kalmannData *KD, double y[2]);



#endif /* KALMANN_H_ */
