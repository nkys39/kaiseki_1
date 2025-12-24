/*
 *  slam.h
 *  slam
 *
 *  Created by Yuichiro Toda on 12/04/10.
 *  Copyright 2012 TMU. All rights reserved.
 *
 */
#ifndef SLAM_H
#define SLAM_H
 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define MAPSIZE 1000 //1500
#define ERR -999999999
#define MAPRATE 50.0	//25.0 kijun
#define CENTER MAPSIZE/2
#define GALM 3
#define GANM 100

struct robot_position{
	int map_x;
	int map_y;
	double real_x;
	double real_y;
	double rangle;
	double dangle;
	
	double longitude;
	double latitude;
	
	double gen_m[GANM+1][GALM+1];
	double fit_m[GANM+1];
	int best_gene;
	int slam_count;
};


void slam(struct robot_position *robot, long *lrf_data, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE]);
void map_building2(struct robot_position *robot, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE]);
double transformation_angle(double theta1);
void initialize_RobotPosition(struct robot_position *robot, int dflag);

void slam_ang(struct robot_position *robot, long *lrf_data, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], double ang);
void slam_sa(struct robot_position *robot, long *lrf_data, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], double dis, int gno, double ang);

#endif
