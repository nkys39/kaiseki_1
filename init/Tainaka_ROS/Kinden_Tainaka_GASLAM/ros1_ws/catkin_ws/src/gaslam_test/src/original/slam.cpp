/*
 *  slam.c
 *  slam
 *
 *  Created by Yuichiro Toda on 12/04/10.
 *  Copyright 2012 TMU. All rights reserved.
 *
 */
#include "random.h"
#include "slam.h"


#pragma mark GA
//#define T2 500	//700
#define PI M_PI

const int T2 = 500;
float x[1080], y[1080];
float theta[1080];

void transformation_Rmatrix(struct robot_position *sa1, struct robot_position *sa2, double R[][3], double tv[])
{
    double roll = sa1->rangle/180.0*M_PI;
    double pitch = 0.0;
    double yaw = sa1->rangle/180.0*M_PI;
    
    
    tv[0] = sa1->real_x/1000.0;
    tv[1] = sa2->real_x/1000.0;
    tv[2] = sa1->real_y/1000.0;
    
    R[0][0] = cos(roll)*cos(pitch), R[0][1] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*sin(yaw), R[0][2] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
    R[1][0] = sin(roll)*cos(pitch), R[1][1] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), R[1][2] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
    R[2][0] = -sin(pitch),          R[2][1] = cos(pitch)*sin(yaw),                                R[2][2] = cos(pitch)*cos(yaw);
    
}

#pragma mark -
#pragma mark Initialize the input robot position. Created by Yuichiro Toda on 12/04/19.
void initialize_RobotPosition(struct robot_position *robot, int dflag)
{
	if(dflag == 0){
		robot->real_x = 0;
		robot->real_y = 0;
		robot->rangle = 0;
		robot->dangle = 0;
		robot->longitude = 0.0;
		robot->latitude = 0.0;
		robot->map_x = CENTER;
		robot->map_y = CENTER;
		robot->slam_count = 0;
	}else{
		robot->real_x = ERR;
		robot->real_y = ERR;
		robot->rangle = 0;
		robot->dangle = 0;
		robot->map_x = 0;
		robot->map_y = 0;
		robot->longitude = 0.0;
		robot->latitude = 0.0;
		robot->slam_count = 0;
	}
}


#pragma mark -
#pragma mark Caluculate localization of the robot by using SSGA. Created by Yuichiro Toda.
double fitcal_m(int i, struct robot_position *robot,int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE])
{
	int k;
	double h,h2;
	int hit=0,err=0;
	int num = 0;
	double x_1,y_1,x_3,y_3;
	int pt1_x,pt1_y;
	int hit2=0,err2=0,unce=0;
	double p=0.0,p2, p4 = 0.0;
	double x_c2,y_c2;
	double s1,c1;
	s1 = sin((robot->dangle+robot->gen_m[i][2])*PI/180.0);
	c1 = cos((robot->dangle+robot->gen_m[i][2])*PI/180.0);
	x_c2 = robot->real_x;
	y_c2 = robot->real_y;
	x_c2 += robot->gen_m[i][0]*c1 - robot->gen_m[i][1]*s1;
	y_c2 += robot->gen_m[i][1]*c1 + robot->gen_m[i][0]*s1;
	
	int ks,ke,skip;
    ks = 0;
    ke = 1080;
    skip = 2;
    p = 0.0;
	
	if(i>=0){
		for(k=ks;k<ke;k+=skip){
			
			x_1 = x[k],y_1 = y[k];
			if(x_1 != ERR && y_1 != ERR){
				x_3 = (double)x_1*c1 - (double)y_1*s1 + x_c2;
				y_3 = (double)x_1*s1 + (double)y_1*c1 + y_c2;
				pt1_x = CENTER - (int)x_3/MAPRATE;
				pt1_y = CENTER - (int)y_3/MAPRATE;
				
				if(pt1_x < 0 || pt1_x >= MAPSIZE || pt1_y < 0 ||pt1_y > MAPSIZE) continue;
				h = omap[pt1_x][pt1_y];
				h2 = amap[pt1_x][pt1_y];
				
				if( h2 > 0)
					hit++;
				else
					err++;
				
				if( cal_probability(h) > 0.3)
					hit2++;
				else if(cal_probability(h) < -0.3)
					err2++;
				else
					unce++;
				
				p4 += cal_probability(h);
				num++;
			}
		}
	}
	p2 = ((double)hit)/(double)(hit+err);
	p = ((double)hit2)/((double)hit2+(double)err2+(double)unce);
	p4 = (p*p2);
	return p4;
}

int g_max(struct robot_position *robot, double *best_fitness)
{
	int j;
	double max;
	for(int i=0;i<GANM;i++){
		if(i == 0){
			max = robot->fit_m[i];
			j = i;
		}else if(max < robot->fit_m[i]){
			max = robot->fit_m[i];
			j = i;
		}
	}
	*best_fitness = max;
	return j;
}

int g_min(struct robot_position *robot, double *worst_fitness)
{
	int j;
	double min;
	for(int i=0;i<GANM;i++){
		if(i == 0){
			min = robot->fit_m[i];
			j = i;
		}else if(min > robot->fit_m[i]){
			min = robot->fit_m[i];
			j = i;
		}
	}
	*worst_fitness = min;
	return j;
}

void mga_init(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE])
{
	
	for (int i = 0; i<GANM; i++)
	{
		if (i == 0) {
			robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
			robot->gen_m[i][1] = robot->gen_m[robot->best_gene][1];
			robot->gen_m[i][2] = robot->gen_m[robot->best_gene][2];
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			continue;
		}
		else if (i == 1) {
			robot->gen_m[i][0] = 0;
			robot->gen_m[i][1] = 0;
			robot->gen_m[i][2] = 0;
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			continue;
		}
		else if (i == 2) {
			robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
			robot->gen_m[i][1] = -robot->gen_m[robot->best_gene][1];
			robot->gen_m[i][2] = -robot->gen_m[robot->best_gene][2];
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			continue;
		}
		else {
			robot->gen_m[i][0] = 40.0*rndn();
			robot->gen_m[i][1] = 40.0*rndn();
			robot->gen_m[i][2] = rndn();
		}
		robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
	}
}


void map_mating(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE]){
	
	int t=0,g,g2;
	int worst_gene;
 	double worst_fitness;
	double best_fitness;
	
	mga_init(robot, omap, amap);
	
	while (t < T2){
		double r,r2;
		robot->best_gene = g_max(robot, &best_fitness);
		worst_gene = g_min(robot, &worst_fitness);
		
		g = (int)(GANM*rnd());
		while(g == robot->best_gene)
			g = (int)(GANM*rnd());
		r2 = (10.0*(best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);
		g2 = robot->best_gene;
		for (int j = 0; j<GALM; j++) {
			r = rnd();
			//-------crossover-------------------------
			if (r < 0.7)
				robot->gen_m[worst_gene][j] = robot->gen_m[g2][j];
			else
				robot->gen_m[worst_gene][j] = (robot->gen_m[g][j] + robot->gen_m[g2][j])*rnd();

			//------mutation---------------------------
			if (rnd() < 0.7) {
				if (j != 2)
					robot->gen_m[worst_gene][j] += 10 * rndn_10()*(0.01 + r2*(double)(T2 - t) / (double)T2);
				else
					robot->gen_m[worst_gene][j] += rndn()*(0.01 + r2*(double)(T2 - t) / (double)T2);
			}
			else {
				if (j != 2)
					robot->gen_m[worst_gene][j] += 10.0*rndn_10()*rndn()*(0.01 + r2*(double)(T2 - t) / (double)T2);
				else
					robot->gen_m[worst_gene][j] += rndn()*rndn()*(0.01 + r2*(double)(T2 - t) / (double)T2);
			}
		}
		
		robot->fit_m[worst_gene] = fitcal_m(worst_gene, robot, omap, amap);
		t++;
	}
	
	robot->best_gene = g_max(robot, &best_fitness);
//    printf("Best individual[%d] is %lf\n", robot->slam_count,best_fitness);
}

void calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE])
{
	if(sflag == 1){
		if(k >= pt_x)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}else if(sflag == 2){
		if(k <= pt_x)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}else if(sflag == 3){
		if(l >= pt_y)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}else{
		if(l <= pt_y)
			omap[k][l]+=2;
		else
			omap[k][l]--;
	}
	
	if(cal_probability(amap[k][l]) > 0.85)
		if(omap[k][l] < 0 && amap[k][l] + omap[k][l] > 0)
			omap[k][l] += amap[k][l];
	
}

void map_building2(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE])
{
	int i;
	int pt_x,pt_y,ptc_x,ptc_y;
	double x_3,y_3;
	
	int dx,dy;
	int j,e,k,l;
	double s1,c1;
	s1 = sin((double)robot->dangle*PI/180.0);
	c1 = cos((double)robot->dangle*PI/180.0);
	ptc_x = CENTER - (int)robot->real_x/MAPRATE;
	ptc_y = CENTER - (int)robot->real_y/MAPRATE;

	int ks,ke,skip;
    ks = 0;
    ke = 1080;
    skip = 2;
	
	for(j=ks;j<ke;j++){
		if(x[j] != ERR && y[j] != ERR){
			x_3 = (double)x[j]*c1 - (double)y[j]*s1 + (double)robot->real_x;
			y_3 = (double)x[j]*s1 + (double)y[j]*c1 + (double)robot->real_y;
			pt_x = CENTER - (int)x_3/MAPRATE;
			pt_y = CENTER - (int)y_3/MAPRATE;
            if(pt_x < 0 || pt_x > MAPSIZE || pt_y < 0 || pt_y > MAPSIZE) continue;
			amap[pt_x][pt_y]++;
			dx= (pt_x - ptc_x);
			dy= (pt_y - ptc_y);
			
			e=0;
			if(dx>=0&&dy>=0){
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k<=pt_x;k++){
						e+=dy;
						if(e > dx){
							e-=dx;
							l++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l<=pt_y;l++){
						e+=dx;
						if(e > dy){
							e-=dy;
							k++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap);
					}
				}
			}else if(dx>=0&&dy<=0){
				dy *= -1;
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k<=pt_x;k++){
						e+=dy;
						if(e > dx){
							e-=dx;
							l--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l>=pt_y;l--){
						e+=dx;
						if(e > dy){
							e-=dy;
							k++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap);
					}
				}
			}else if(dx<=0&&dy>=0){
				dx *= -1;
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k>=pt_x;k--){
						e+=dy;
						if(e > dx){
							e-=dx;
							l++;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l<=pt_y;l++){
						e+=dx;
						if(e > dy){
							e-=dy;
							k--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap);
					}
				}
			}else{
				dx *= -1;
				dy *= -1;
				if(dx>dy){
					for(l=ptc_y,k=ptc_x;k>=pt_x;k--){
						e+=dy;
						if(e > dx){
							e-=dx;
							l--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap);
					}
				}else{
					for(l=ptc_y,k=ptc_x;l>=pt_y;l--){
						e+=dx;
						if(e > dy){
							e-=dy;
							k--;
						}
						
						calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap);
					}
				}
			}
		}	
	}
}

#pragma mark -
#pragma mark Transform the angle between -PI/2 and PI/2 . Created by Yuichiro Toda on 12/04/19.
double transformation_angle(double theta1)
{
	double err_a;
	
	if(fabs(theta1) > 180){
		err_a = fabs(theta1) - 180.0; 
		if(theta1 > 0){
			theta1 = -180.0;
			theta1 += err_a;
		}else{
			theta1 = 180.0;
			theta1 -= err_a;
		}
	}
	return theta1;
}

#pragma mark -
void slam(struct robot_position *robot, long *lrf_data, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE])
{
	int i,j;
	static double m_x, m_y;
	double s_x,s_y,s_r;
	static double pre_angle;
	
	pre_angle = robot->dangle;
	if(robot->slam_count==0){
        for(i=0;i<1080;i++)
            theta[i]=(((double)i-540.0)*(0.25)*M_PI)/180.0;
		
		for(i=0;i<GANM+1;i++){
			robot->fit_m[i] = 0;
			for(j=0;j<GALM+1;j++)
				robot->gen_m[i][j] = 0;
		}
		robot->best_gene = 0;
	}
	
    for(i=0;i<1080;i++){
        if(lrf_data[i] < 500 || lrf_data[i] > 20000){
            x[i] = ERR;
            y[i] = ERR;
        }else{
            
            x[i] = lrf_data[i]*sin(theta[i]);
            y[i] = lrf_data[i]*cos(theta[i]);
        }
    }
	
	if(robot->slam_count > 30){
		map_mating(robot, omap, amap);
		
		m_x += robot->gen_m[robot->best_gene][0];
		m_y += robot->gen_m[robot->best_gene][1];
		robot->dangle += robot->gen_m[robot->best_gene][2];
		s_x = 0,s_y = 0,s_r=0;
		s_x = robot->gen_m[robot->best_gene][0];
		s_y = robot->gen_m[robot->best_gene][1];
		s_r = robot->gen_m[robot->best_gene][2];
		
        robot->real_x += s_x*cos(robot->dangle*PI/180) - s_y*sin(robot->dangle*PI/180);
        robot->real_y += s_y*cos(robot->dangle*PI/180) + s_x*sin(robot->dangle*PI/180);
        
        robot->rangle += robot->dangle - pre_angle;
        
        robot->map_x = CENTER - (int)robot->real_x/MAPRATE; 
        robot->map_y = CENTER - (int)robot->real_y/MAPRATE;
        
        double err_a;
        if(fabs(robot->rangle) > 180){
            err_a = fabs(robot->rangle) - 180.0;
            if(robot->rangle > 0){
                robot->rangle = -180.0;
                robot->rangle += err_a;
            }else{
                robot->rangle = 180.0;
                robot->rangle -= err_a;
            }
        }
	}
    
	robot->slam_count++;
}

void slam_ang(struct robot_position *robot, long *lrf_data, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], double ang)
{
	int i,j;
	static double m_x, m_y;
	double s_x,s_y,s_r;
	static double pre_angle;
	
	pre_angle = robot->dangle;
	if(robot->slam_count==0){
        for(i=0;i<1080;i++)
            theta[i]=(((double)i-540.0)*(0.25)*M_PI)/180.0;
		
		for(i=0;i<GANM+1;i++){
			robot->fit_m[i] = 0;
			for(j=0;j<GALM+1;j++)
				robot->gen_m[i][j] = 0;
		}
		robot->best_gene = 0;
	}
	
    for(i=0;i<1080;i++){
        if(lrf_data[i] < 500 || lrf_data[i] > 20000){
            x[i] = ERR;
            y[i] = ERR;
        }else{
            
            x[i] = lrf_data[i]*cos(ang)*sin(theta[i]);
            y[i] = lrf_data[i]*cos(ang)*cos(theta[i]);
        }
    }
	
	if(robot->slam_count > 30){
		map_mating(robot, omap, amap);
		
		m_x += robot->gen_m[robot->best_gene][0];
		m_y += robot->gen_m[robot->best_gene][1];
		robot->dangle += robot->gen_m[robot->best_gene][2];
		s_x = 0,s_y = 0,s_r=0;
		s_x = robot->gen_m[robot->best_gene][0];
		s_y = robot->gen_m[robot->best_gene][1];
		s_r = robot->gen_m[robot->best_gene][2];
		
        robot->real_x += s_x*cos(robot->dangle*PI/180) - s_y*sin(robot->dangle*PI/180);
        robot->real_y += s_y*cos(robot->dangle*PI/180) + s_x*sin(robot->dangle*PI/180);
        
        robot->rangle += robot->dangle - pre_angle;
        
        robot->map_x = CENTER - (int)robot->real_x/MAPRATE; 
        robot->map_y = CENTER - (int)robot->real_y/MAPRATE;
        
        double err_a;
        if(fabs(robot->rangle) > 180){
            err_a = fabs(robot->rangle) - 180.0;
            if(robot->rangle > 0){
                robot->rangle = -180.0;
                robot->rangle += err_a;
            }else{
                robot->rangle = 180.0;
                robot->rangle -= err_a;
            }
        }
	}
    
	robot->slam_count++;
}


void mga_init_sa(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], double dis, int gno)
{
	
	for (int i = 0; i<GANM; i++)
	{
		if (i == 0) {
			robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
			robot->gen_m[i][1] = robot->gen_m[robot->best_gene][1];
			robot->gen_m[i][2] = robot->gen_m[robot->best_gene][2];
			
			robot->gen_m[i][gno] = dis;
			
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			continue;
		}
		else if (i == 1) {
			robot->gen_m[i][0] = 0;
			robot->gen_m[i][1] = 0;
			robot->gen_m[i][2] = 0;
			
			robot->gen_m[i][gno] = dis;
			
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			continue;
		}
		else if (i == 2) {
			robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
			robot->gen_m[i][1] = -robot->gen_m[robot->best_gene][1];
			robot->gen_m[i][2] = -robot->gen_m[robot->best_gene][2];
			
			robot->gen_m[i][gno] = dis;
			
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
			continue;
		}
		else {
			robot->gen_m[i][0] = 40.0*rndn();
			robot->gen_m[i][1] = 40.0*rndn();
			robot->gen_m[i][2] = rndn();
			
			robot->gen_m[i][gno] = dis;
		}
		robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
	}
}


void map_mating_sa(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], double dis, int gno){
	
	int t=0,g,g2;
	int worst_gene;
 	double worst_fitness;
	double best_fitness;
	
	mga_init_sa(robot, omap, amap, dis, gno);
	
	while (t < T2){
		double r,r2;
		robot->best_gene = g_max(robot, &best_fitness);
		worst_gene = g_min(robot, &worst_fitness);
		
		g = (int)(GANM*rnd());
		while(g == robot->best_gene)
			g = (int)(GANM*rnd());
		r2 = (10.0*(best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);
		g2 = robot->best_gene;
		for (int j = 0; j<GALM; j++) {
			r = rnd();
			//-------crossover-------------------------
			if (r < 0.7)
				robot->gen_m[worst_gene][j] = robot->gen_m[g2][j];
			else
				robot->gen_m[worst_gene][j] = (robot->gen_m[g][j] + robot->gen_m[g2][j])*rnd();

			//------mutation---------------------------
			if (rnd() < 0.7) {
				if (j != 2)
					robot->gen_m[worst_gene][j] += 10 * rndn_10()*(0.01 + r2*(double)(T2 - t) / (double)T2);
				else
					robot->gen_m[worst_gene][j] += rndn()*(0.01 + r2*(double)(T2 - t) / (double)T2);
			}
			else {
				if (j != 2)
					robot->gen_m[worst_gene][j] += 10.0*rndn_10()*rndn()*(0.01 + r2*(double)(T2 - t) / (double)T2);
				else
					robot->gen_m[worst_gene][j] += rndn()*rndn()*(0.01 + r2*(double)(T2 - t) / (double)T2);
			}
		}
		robot->gen_m[worst_gene][gno] = dis;
		robot->fit_m[worst_gene] = fitcal_m(worst_gene, robot, omap, amap);
		t++;
	}
	
	robot->best_gene = g_max(robot, &best_fitness);
//    printf("Best individual[%d] is %lf\n", robot->slam_count,best_fitness);
}


void slam_sa(struct robot_position *robot, long *lrf_data, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], double dis, int gno, double ang)
{
	int i,j;
	static double m_x, m_y;
	double s_x,s_y,s_r;
	static double pre_angle;
	
	pre_angle = robot->dangle;
	if(robot->slam_count==0){
        for(i=0;i<1080;i++)
            theta[i]=(((double)i-540.0)*(0.25)*M_PI)/180.0;
		
		for(i=0;i<GANM+1;i++){
			robot->fit_m[i] = 0;
			for(j=0;j<GALM+1;j++)
				robot->gen_m[i][j] = 0;
		}
		robot->best_gene = 0;
	}
	
    for(i=0;i<1080;i++){
        if(lrf_data[i] < 500 || lrf_data[i] > 20000){
            x[i] = ERR;
            y[i] = ERR;
        }else{
            
            x[i] = lrf_data[i]*cos(ang)*sin(theta[i]);
            y[i] = lrf_data[i]*cos(ang)*cos(theta[i]);
        }
    }
	
	if(robot->slam_count > 30){
		map_mating_sa(robot, omap, amap, dis , gno);
		
		m_x += robot->gen_m[robot->best_gene][0];
		m_y += robot->gen_m[robot->best_gene][1];
		robot->dangle += robot->gen_m[robot->best_gene][2];
		s_x = 0,s_y = 0,s_r=0;
		s_x = robot->gen_m[robot->best_gene][0];
		s_y = robot->gen_m[robot->best_gene][1];
		s_r = robot->gen_m[robot->best_gene][2];
		
        robot->real_x += s_x*cos(robot->dangle*PI/180) - s_y*sin(robot->dangle*PI/180);
        robot->real_y += s_y*cos(robot->dangle*PI/180) + s_x*sin(robot->dangle*PI/180);
        
        robot->rangle += robot->dangle - pre_angle;
        
        robot->map_x = CENTER - (int)robot->real_x/MAPRATE; 
        robot->map_y = CENTER - (int)robot->real_y/MAPRATE;
        
        double err_a;
        if(fabs(robot->rangle) > 180){
            err_a = fabs(robot->rangle) - 180.0;
            if(robot->rangle > 0){
                robot->rangle = -180.0;
                robot->rangle += err_a;
            }else{
                robot->rangle = 180.0;
                robot->rangle -= err_a;
            }
        }
	}
    
	robot->slam_count++;
}


