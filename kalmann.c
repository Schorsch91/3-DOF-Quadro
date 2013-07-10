/*
 * kalmann.c
 *
 *  Created on: Jun 26, 2013
 *      Author: Avionics Lab-3
 */
#include "defines.h"


//Kalman Things



void initializeKalmann(kalmannData *KD){


	//initialize Kalmann
		//sampletime
		double Ts = 0.01;
	double Pacc = 0.01;
	double Pgyro = 500.0;
	double Macc = 5000.0;
	double Mgyro = 1.0;


	  KD->A[0][0] = 1.0;
	  KD->A[0][1] = Ts;
	  KD->A[1][0] = 0.0;
	  KD->A[1][1] = 1.0;

	  KD->C[0][0] = 1.0;
	  KD->C[0][1] = 0.0;
	  KD->C[1][0] = 0.0;
	  KD->C[1][1] = 1.0;

	  KD->Q[0][0] = Pacc;
	  KD->Q[0][1] = 0.0;
	  KD->Q[1][0] = 0.0;
	  KD->Q[1][1] = Pgyro;

	  KD->R[0][0] = Macc;
	  KD->R[0][1] = 0.0;
	  KD->R[1][0] = 0.0;
	  KD->R[1][1] = Mgyro;


	  KD->Pd[0][0] = 0.0;
	  KD->Pd[0][1] = 0.0;
	  KD->Pd[1][0] = 0.0;
	  KD->Pd[1][1] = 0.0;

	  KD->Plast[0][0] = 1.0;
	  KD->Plast[0][1] = 0.0;
	  KD->Plast[1][0] = 0.0;
	  KD->Plast[1][1] = 1.0;

	  KD->K[0][0] = 0.0;
	  KD->K[0][1] = 0.0;
	  KD->K[1][0] = 0.0;
	  KD->K[1][1] = 0.0;

	  //Einheitsmatrix
	  KD->I[0][0] = 1.0;
	  KD->I[0][1] = 0.0;
	  KD->I[1][0] = 0.0;
	  KD->I[1][1] = 1.0;



	  KD->Xd[0] = 0.0;
	  KD->Xd[1] = 0.0;

	  KD->Xlast[0] = 0.0;
	  KD->Xlast[1] = 0.0;

	  KD->X[0] = 0.0;
	  KD->X[1] = 0.0;

	  KD->Y[0] = 0.0;
	  KD->Y[1] = 0.0;

}

void kalmannFilter(kalmannData *KD, double y[2]){
	double temp1[2][2];
	double temp2[2][2];
	double temp3[2][2];
	double tempVect1[2];
	double tempVect2[2];

	//calculate Xd
	Matrix_mal_Vektor_2D(KD->A,KD->Xlast, KD->Xd);

	//calculate Pd
	double transA[2][2];
	Matrixmultiplikation2D(KD->A,KD->Plast,temp1); // KEINEN RESULT POINTER???? FRAGEN!!
	Transponiert2D(KD->A, transA);
	Matrixmultiplikation2D(temp1,transA,temp2);
	Matrixaddition2D(temp2, KD->Q, KD->Pd);

	//Calculate K

	//klammer berechnen
	double transC[2][2];
	Transponiert2D(KD->C, transC);
	Matrixmultiplikation2D(KD->C,KD->Pd,temp1);
	Matrixmultiplikation2D(temp1, transC, temp2);
	Matrixaddition2D(temp2, KD->R, temp3);
	double inverseKlammer[2][2];
	Matrix_Inverse(temp3, inverseKlammer);

	//rest
	Matrixmultiplikation2D(KD->Pd,transC, temp1);
	Matrixmultiplikation2D(temp1, inverseKlammer, KD->K);

	//calculate X
	Matrix_mal_Vektor_2D(KD->C,KD->Xd,tempVect1);
	Skalar_mal_Vektor(-1,tempVect1);
	Vektoraddition2D(y,tempVect1,tempVect2);
	Matrix_mal_Vektor_2D(KD->K,tempVect2, tempVect1);
	Vektoraddition2D(KD->Xd,tempVect1,KD->X);

	//calculate P

	//klammer
	Matrixmultiplikation2D(KD->K, KD->C, temp1);
	Skalar_mal_Matrix(-1,temp1);
	Matrixaddition2D(KD->I,temp1, temp2);
	Matrixmultiplikation2D(temp2, KD->Pd,KD->P);


	//"Aus neu mach alt"
	KD->Xlast[0] = 	KD->X[0];
	KD->Xlast[1] = 	KD->X[1];

	KD->Plast[0][0] = 	KD->P[0][0];
	KD->Plast[0][1] = 	KD->P[0][1];
	KD->Plast[1][0] = 	KD->P[1][0];
	KD->Plast[1][1] = 	KD->P[1][1];



}
