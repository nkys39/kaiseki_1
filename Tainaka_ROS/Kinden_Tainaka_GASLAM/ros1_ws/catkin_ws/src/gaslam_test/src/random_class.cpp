#include "random_class.hpp"

Random::Random()
{
}

Random::~Random()
{

}
// uniform randomnumber 0<= rnd <= 1.0
double Random::rnd(){
    return ((double)(rand()%RAND_MAX)/(double)(RAND_MAX-1));
}
//normal random number
double Random::rndn(){
    double r;
	r = rnd() + rnd() + rnd() + rnd() + rnd() + rnd()
	+ rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 6.0;
	return r;
}
//normal random number
double Random::rndn_10(){
    double r;
	r = rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd() + rnd()
	+ rnd() + rnd() + rnd() + rnd() + rnd() + rnd()  + rnd() + rnd() + rnd() + rnd() - 10.0;
	return r;
}
double Random::cal_probability(int k){
    double e;
	e = tanh((double)k/5.0);
	return e;
}

double Random::angle(double x1, double y1){
    double kakudo=atan(y1/x1);//180/PI;
	if(x1==0&&y1==0)kakudo =0;
	if( x1<0&&0<=y1)kakudo+=M_PI;// 90〜180
	if( x1<0&&y1<0 )kakudo+=M_PI;//180〜270
	if(0<=x1&&y1<0 )kakudo+=2*M_PI;//270〜360
	return kakudo;
}
//角度差
double Random::ang_lr(int it, double dx, double dy, double r_theta){
    double dt;
	dt = angle(dx,dy)-r_theta;
	if	   (dt<-M_PI)	dt += M_PI*2.0;
	else if(dt> M_PI)	dt -= M_PI*2.0;
	return dt;
}
//2点間距離
double Random::dt_tg2(double tgx, double tgy, double mex, double mey){
    double dt2x=tgx-mex, dt2y=tgy-mey;
	return sqrt( dt2x*dt2x + dt2y*dt2y );
}

double Random::calc_inner(double *a, double *b, int dim){
    int i;
    double result = 0.0;
    
    for(i=0;i<dim;i++)
        result += a[i]*b[i];
    
    return result;
}