/*
*  slam.h
*  slam
*
*  Created by Yuichiro Toda on 10/04/12 and modified by Wei Hong on 12/7/17.
*  Copyright 2012 TMU. All rights reserved.
*
*/
#if !defined(SLAM_HEADER)
#define SLAM_HEADER

//#include <opencv/cv.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "Def.h"
#include "draw.h"  //henkoModel

void slam(struct robot_position *robot, long *lrf_data, int **omap, int **amap, double *outp);  //henkoModel
void map_building2(struct robot_position *robot, int **omap, int **amap, IplImage *omap_Image, IplImage *amap_Image);
double transformation_angle(double theta1);
void initialize_RobotPosition(struct robot_position *robot, int dflag);

int g_min(struct robot_position *robot, double *worst_fitness);  //henkoModel
void calcRobotModel(double *m);

//RS-LiDAR—pSLAM
void slam2(struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, int** omap, int** amap, double* outp);

#endif
