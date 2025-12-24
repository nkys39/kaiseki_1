
/*
*  localization.h
*  slam
*
*  Created by Naoyuki Kubota on 12/04/12 and modified by Wei Hong on 12/7/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#pragma once
#if !defined(LOCALIZATION_HEADER)
#define LOCALIZATION_HEADER

#include "slam.h"
#include "random.h"
#include "multi_resolution.h"
#include "Def.h"

struct es_gene 
{
	double gene[EGAN][4];
	int best_gene;
	double es_fit[EGAN];
	double es_wei[EGAN];
	double wei_ave;
	double init_pos[3]; //TMU20190821 向きの追加
	int lo_count;
};

struct multi_map *initial_Localization(struct es_gene *gene, struct robot_position *robot, long *lrf_data, int **omap, struct multi_map *multi_Map, IplImage *mImage, int *end_flag);
void addjust_scale_multi(struct robot_position *robot, long *lrf_data, struct multi_map *multi_Map);
void readLRF();//henkoIP

//RS-LiDAR用
double evolutionary_strategy2(struct es_gene* Es_gene, struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, int** omap, struct multi_map* multi_Map, IplImage* mImage, int* ini_flag);
struct multi_map* initial_Localization2(struct es_gene* gene, struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, int** omap, struct multi_map* multi_Map, IplImage* mImage, int* end_flag);
#endif