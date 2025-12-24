#pragma once
/*
*  random.h
*  slam
*
*  Created by Naoyuki Kubota on 10/04/12 and modified by Wei Hong on 12/7/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#if !defined(RANDOM_HEADER)
#define RANDOM_HEADER

double rnd();
double rndn();
double rndn_10();
double cal_probability(int k);

double angle(double x1, double y1);
//角度差
double ang_lr(int it, double dx, double dy, double r_theta);
//2点間距離
double dt_tg2(double tgx, double tgy, double mex, double mey);

#endif
