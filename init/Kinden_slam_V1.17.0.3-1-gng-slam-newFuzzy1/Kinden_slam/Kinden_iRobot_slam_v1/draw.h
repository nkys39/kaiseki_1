
/*
*  draw.h
*  slam
*
*  Created by Yuichiro Toda on 12/04/17.
*  Copyright 2012 TMU. All rights reserved.
*
*/
#pragma once
#if !defined(DRAW_HEADER)
#define DRAW_HEADER

#include <windows.h>  //henkoModel
#include "slam.h"
#include "Def.h"

void draw_robot(struct robot_position *robot, double robot_size, IplImage *draw_Image, int R, int G, int B);
void draw_LRFdata(int mode, struct robot_position *robot, long *lrf_data, IplImage *drawImage, int R, int G, int B);
//RS-LiDAR—p
void draw_LRFdata2(int mode, struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, IplImage* drawImage, int R, int G, int B);

void draw_globalMap(int **omap, int **amap, IplImage *omapImage, IplImage *amapImage, IplImage *mmapImage[]);
void draw_node(struct robot_position *robot, IplImage *draw_Image, int R, int G, int B);
void readLRF2();//henkoIP

void calc_arrow(double base[3], double gen[3], double start[2], double end[2]);  //henkoModel
void draw_gen(int mode, struct robot_position *robot, int step, double *outp);

#endif