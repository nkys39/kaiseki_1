/*
*  malloc.c
*  Cluster
*
*  Created by Naoyuki Kubota on 03/07/14 and modified by Wei Hong on 12/07/17.
*  Copyright 2014 首都大学東京. All rights reserved.
*
*/

#include "malloc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void free2d_double(double ** a)	//2次元配列の開放
{
	free(a[0]);
	free(a);
}

double **malloc2d_double(int x, int y)	//2次元配列の初期化
{ 
	double **a;
	int i;
	a = (double **)malloc(sizeof(double *)*(x + 1));
	a[0] = (double *)malloc(sizeof(double)*(y + 1)*(x + 1));
	for (i = 1; i<(x + 1); i++) a[i] = a[0] + i*(y + 1);
	memset(a[0], 0, sizeof(a[0]));
	return a;
}

void free2d_int(int ** a)	//2次元配列の開放
{
	free(a[0]);
	free(a);
}

int **malloc2d_int(int x, int y)	//2次元配列の初期化
{
	int **a;
	int i;
	a = (int **)malloc(sizeof(int *)*(x + 1));
	a[0] = (int *)malloc(sizeof(int)*(y + 1)*(x + 1));
	for (i = 1; i<(x + 1); i++) a[i] = a[0] + i*(y + 1);
	memset(a[0], 0, sizeof(a[0]));
	return a;
}

void free3d_double(double *** a)	//2次元配列の開放
{
	free(a[0]);
	free(a);
}

double ***malloc3d_double(int x, int y, int z)	//2次元配列の初期化
{
	double ***array;
	int i, j, k; 

	array = (double ***)malloc(sizeof(double **) * x);

	for (i = 0; i < x; i++) {
		array[i] = (double **)malloc(sizeof(double *) * y);

		for (j = 0; j < y; j++) {
			array[i][j] = (double *)malloc(sizeof(double) * z);

			for (k = 0; k < z; k++)
				array[i][j][k] = i * j * k;
		}
	}
	return array;
}

