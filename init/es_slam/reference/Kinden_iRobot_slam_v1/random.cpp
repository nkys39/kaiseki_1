/*
*  random.c
*  slam
*
*  Created by Naoyuki Kubota on 12/04/10.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/

#include "random.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#if defined(M_PI)
#else
#define M_PI 3.14159265358979323846
#endif

double rnd()
{						// uniform randomnumber 0<= rnd <= 1.0
	return ((double)(rand() % RAND_MAX) / (double)(RAND_MAX - 1));
}

double rndn()		//normal random number
{
	double r = rnd() + rnd() + rnd() + rnd() + rnd() + rnd()
			 + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 6.0;
	
	return r;
}

double rndn_10()		//normal random number
{
	double r = rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd()
			 + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 10.0;

	return r;
}

double cal_probability(int k)
{
	double e = tanh((double)k / 10.0);

	return e;
}

//角度計算
double angle(double x1, double y1) 
{
	double kakudo = atan(y1 / x1);//180/PI;

	if (x1 == 0 && y1 == 0)
	{
		kakudo = 0;
	}

	if (x1 < 0 && 0 <= y1)	// 90〜180
	{
		kakudo += M_PI;
	}

	if (x1 < 0 && y1 < 0)	//180〜270
	{
		kakudo += M_PI;
	}

	if (0 <= x1 && y1 < 0)	//270〜360
	{
		kakudo += 2 * M_PI;
	}

	return kakudo;
}

//角度差
double ang_lr(int it, double dx, double dy, double r_theta) 
{
	double dt = angle(dx, dy) - r_theta;

	if (dt < -M_PI)
	{
		dt += M_PI * 2.0;
	}
	else if (dt > M_PI)
	{
		dt -= M_PI * 2.0;
	}

	return dt;
}

//2点間距離
double dt_tg2(double tgx, double tgy, double mex, double mey) 
{
	double dt2x = tgx - mex;
	double dt2y = tgy - mey;

	return sqrt(dt2x*dt2x + dt2y*dt2y);
}