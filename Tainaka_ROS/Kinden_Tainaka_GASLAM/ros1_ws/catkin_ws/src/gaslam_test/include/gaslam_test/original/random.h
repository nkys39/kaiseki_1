/*
 *  random.h
 *  slam
 *
 *  Created by Naoyuki Kubota on 12/04/10.
 *  Copyright 2012 首都大学東京. All rights reserved.
 *
 */

double rnd();
double rndn();
double rndn_10();
double cal_probability(int k);

double angle(double x1, double y1);
//角度差
double ang_lr(int it, double dx, double dy, double r_theta);
//2点間距離
double dt_tg2(double tgx, double tgy, double mex, double mey);

double calc_inner(double *a, double *b, int dim);