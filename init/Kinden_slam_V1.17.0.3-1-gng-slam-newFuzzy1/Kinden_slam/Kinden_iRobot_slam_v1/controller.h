/*
*  controller.h
*  slam
*
*  Created by Naoyuki Kubota on 16/07/12 and modified by Wei Hong on 12/7/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#pragma once
#if !defined(CONTROLLER_HEADER)
#define CONTROLLER_HEADER

#include "slam.h"
#include "random.h" //TMU20190820 正面の障害物への対処
#include "gngPlanner.h"

int decision_making(struct robot_position *robot, double *foutp, long *lrfdata, int path[][2], int *sflag, int pct, int mode);

//RS-LiDAR用
int decision_making2(struct robot_position* robot, double* foutp, long* lrf_dataD, float* lrf_dataA, int path[][2], int* sflag, int pct, int mode);

//gng path planning(HOKUYO)
int movement_controller(struct robot_position* robot, double* foutp, long* lrfdata, int path[][2], int* sflag, int pct, int mode, FastAstar* pp);

#endif