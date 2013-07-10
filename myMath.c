/*
 * myMath.c
 *
 *  Created on: 05.08.2011
 *      Author: gageik
 */
// This Quadcopter Software Framework was developed at
// University Wuerzburg
// Chair Computer Science 8
// Aerospace Information Technology
// Copyright (c) 2011 Nils Gageik


double euclidean_norm (double a, double b){

	double c;
	c = (a-b)*(a-b);
	return c;
}

short round_s (double zahl) {
// Rounds a double and converts to short

	short out;
	short sig = 1;

	if (zahl<0)
		sig = -1;

	zahl = zahl + 0.5f * sig;

	out = (short) zahl;

	return out;
}

// TODO: überholen!!!
double betrag(double z){

	if (z < 0.0f)
		return -z;
	else
		return z;
}

int betrag_i(int a){

	if (a < 0)
		return -a;
	else
		return a;
}

double limit_max(double x, double max){
// Gibt Wert bis maximaler Grenze wieder

	if (x>max)
			return max;
		else
			return x;
}

double limit_min(double x, double min){
// Gibt Wert bis minimaler Grenze wieder

	if (x<min)
			return min;
		else
			return x;
}


double saturate(double x, double grenze){
	// magnitude of grenze limits x to both sides (+ and -)


	if (x > grenze)
		return grenze;

	if (x < -grenze)
			return -grenze;

	return x;
}

double maximum(double a, double b){
// Gibt das Maximum aus zwei Werten wieder

	if (a>b)
		return a;
	else
		return b;
}

double minimum(double a, double b){
// Gibt das Minimum aus zwei Werten wieder

	if (a<b)
		return a;
	else
		return b;
}

unsigned int minimum_ui(unsigned int a, unsigned int b){
// Gibt das Minimum aus zwei Werten wieder

	if (a<b)
		return a;
	else
		return b;
}

double sign(double a){

	if (a>0)
		return 1.0f;
	else
		return -1.0f;
}

void Matrixmultiplikation (double * M1, double * M2, double *c){
	 // 3x3

	 int i,j,k;

	 for(i=0;i< 3;i++)
	  {
	   for(j=0;j< 3;j++)
	   {
	    c[3*i+j]=0;
	    for(k=0;k< 3;k++)
	    {
	     c[3*i+j] += M1[i+3*k] * M2[k+3*j];
	    }
	  }
	 }
}


void Transponiert2D(double Matrix[2][2], double R[2][2]){
	// Matrix Pointer to 2x2 double Array

	R[0][0] = Matrix[0][0];
	R[0][1] = Matrix[1][0];
	R[1][0] = Matrix[0][1];
	R[1][1] = Matrix[1][1];
}

void Matrixmultiplikation2D (double M1[2][2], double M2[2][2], double R[2][2]){
	// M1, M2, R Pointer to 2x2 double Array

	R[0][0] = M1[0][0]*M2[0][0]+M1[0][1]*M2[1][0];
	R[0][1] = M1[0][0]*M2[0][1]+M1[0][1]*M2[1][1];
	R[1][0] = M1[1][0]*M2[0][0]+M1[1][1]*M2[1][0];
	R[1][1] = M1[1][0]*M2[0][1]+M1[1][1]*M2[1][1];
}


void Matrixaddition2D(double M1[2][2], double M2[2][2], double R[2][2]){
	R[0][0] = M1[0][0]+M2[0][0];
	R[0][1] = M1[0][1]+M2[0][1];
	R[1][0] = M1[1][0]+M2[1][0];
	R[1][1] = M1[1][1]+M2[1][1];
}

void Vektoraddition2D(double* V1, double* V2, double* V){
	V[0] = V1[0]+V2[0];
	V[1] = V1[1]+V2[1];
}

void Skalar_mal_Matrix (double s, double R[2][2]){
	R[0][0] = R[0][0]*s;
	R[0][1] = R[0][1]*s;
	R[1][0] = R[1][0]*s;
	R[1][1] = R[1][1]*s;
}

void Skalar_mal_Vektor(double s, double* R){
	R[0] = R[0]*s;
	R[1] = R[1]*s;
}

void Matrix_mal_Vektor_2D (double M[2][2], double* V, double * Vr){
	// M 2x2 double Array, V 1x2 Vector Array
	// V2 1x2 Vector contains results at the end
	Vr[0] = M[0][0]*V[0]+M[0][1]*V[1];
	Vr[1] = M[1][0]*V[0]+M[1][1]*V[1];
}

double Determinante_2D (double M[2][2]){
	// M 2x2 double Array
	double d;
	d = M[0][0]*M[1][1]-M[1][0]*M[0][1];

	return d;
}

void Matrix_Inverse (double M[2][2], double R[2][2]){
	// M,R 2x2 double Array
	// Computes Inverse of M and saves it to R
	double s = Determinante_2D(M);

	if (betrag(s) < 0.001)
		s = sign(s) * 0.001;

	R[0][0] = M[1][1]/s;
	R[0][1] = -M[0][1]/s;
	R[1][0] = -M[1][0]/s;
	R[1][1] = M[0][0]/s;
}

double number_potent (double value, unsigned int exponent){

	double r = 1.0f;

	if (exponent == 0)
		return 1;

	for (;exponent > 0;exponent--)
		r = r * value;

	return r;
}


