/*
 * myMath.h
 *
 *  Created on: 05.08.2011
 *      Author: gageik
 */

#ifndef MYMATH_H_
#define MYMATH_H_

double euclidean_norm (double a, double b);
short round_s (double zahl);
double maximum(double a, double b);
double minimum(double a, double b);
unsigned int minimum_ui(unsigned int a, unsigned int b);
double limit_max(double x, double max);
double limit_min(double x, double min);
double sign(double a);
double betrag(double z);
int betrag_i(int a);
double saturate(double x, double grenze);
void Matrixmultiplikation (double * M1, double * M2, double *c);
void Transponiert2D (double Matrix[2][2], double R[2][2]);
void Matrixmultiplikation2D (double M1[2][2], double M2[2][2], double R[2][2]);
void Matrixaddition2D (double M1[2][2], double M2[2][2], double R[2][2]);
void Skalar_mal_Matrix (double s, double R[2][2]);
void Skalar_mal_Vektor (double s, double* R);
void Matrix_mal_Vektor_2D (double M[2][2], double* V, double * Vr);
double Determinante_2D (double M[2][2]);
void Matrix_Inverse (double M[2][2], double R[2][2]);
double number_potent (double value, unsigned int exponent);
void Vektoraddition2D(double* V1, double* V2, double* V);


#endif /* MYMATH_H_ */


