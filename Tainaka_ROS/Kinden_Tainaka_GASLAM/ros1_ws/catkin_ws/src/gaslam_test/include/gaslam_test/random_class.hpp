#ifndef RANDOM_CLASS_HPP
#define RANDOM_CLASS_HPP

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
class Random
{
private:

private:

public:
    Random();//コンストラクタ
    ~Random();//デストラクタ

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

public:
    
};
#endif