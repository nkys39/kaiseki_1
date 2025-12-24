/*
 * slam.cpp
 * Created by Yuichiro Toda on 10/04/12. 
 * Modified by Wei Hong CHin on 12/07/17.
 * Copyright 2012 TMU. All rights reserved.
 *
 */

#include "slam.h"
#include "random.h"
#include "malloc.h"

 //局所的知推定と地図構築用関数
//RS-LiDAR用
double x[SCAN_MARKS_RS16], y[SCAN_MARKS_RS16];
double theta[SCAN_MARKS_RS16];

//double x[SCAN_MARKS], y[SCAN_MARKS];
//double theta[SCAN_MARKS];

double foutp[2];  //henkoModel

int fitness_start_time = 0;
int fitness_end_time = 0;
int fitness_count = 0;
int fitness_flag = 0;

int fitness_mail_start_time = 0;
int fitness_mail_end_time = 0;

//
int **omap_CAD;	//CAD map
int **amap_CAD; //CAD map
int **omap_CAD_for_fit;
int **amap_CAD_for_fit;
double map_match_rate = 1;
bool mapUpdate = false;
bool apply = true;
//

void initialize_RobotPosition(struct robot_position *robot, int dflag)
{
	if (dflag == 0) 
	{
		robot->real_x = 0;
		robot->real_y = 0;
		robot->rangle = 0;
		robot->dangle = 0;
		robot->longitude = 0.0;
		robot->latitude = 0.0;
		robot->map_x = map_center_width;
		robot->map_y = map_center_height;
		robot->slam_count = 0;
	}
	else 
	{
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

	if (run_difficult_notice)
	{
		fitness_count = 0;
		fitness_flag = 0;
	}
}

double fitcal_m(int i, struct robot_position *robot, int **omap, int **amap)
{
	//int k;
	int hit = 0, err = 0;
	int num = 0;
	int pt1_x, pt1_y;
	int hit2 = 0, err2 = 0, unce = 0;
	double x_1, y_1, x_3, y_3;
	double h, h2;
	double p = 0.0, p2, p4 = 0.0;
	double x_c2, y_c2;
	double s1, c1;

	s1 = sin((robot->dangle + robot->gen_m[i][2]) * PI / 180.0);
	c1 = cos((robot->dangle + robot->gen_m[i][2]) * PI / 180.0);
	x_c2 = robot->real_x;
	y_c2 = robot->real_y;
	x_c2 += robot->gen_m[i][0] * c1 - robot->gen_m[i][1] * s1;
	y_c2 += robot->gen_m[i][1] * c1 + robot->gen_m[i][0] * s1;

	if (i >= 0) 
	{
		//RS-LiDAR用
		int numX;
		if (SENSOR_MODE == 0) {
			numX = SCAN_MARKS;
		}
		else {
			numX = rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM;
		}

		for (int k = 0; k < numX; k += 2)
		{

			x_1 = x[k], y_1 = y[k];
			if (x_1 != ERR && y_1 != ERR) 
			{
				x_3 = (double)x_1 * c1 - (double)y_1 * s1 + x_c2;
				y_3 = (double)x_1 * s1 + (double)y_1 * c1 + y_c2;
				pt1_x = (int)(map_center_width - (int)x_3 / map_rate);
				pt1_y = (int)(map_center_height - (int)y_3 / map_rate);

				//point out side map
				if ((pt1_x < 0) || (pt1_x >= map_size_width) || (pt1_y < 0) || (pt1_y >= map_size_height))
				{
					err++;
					err2++;
					continue;
				}

				//refer to real time map
				h = omap[pt1_x][pt1_y];
				h2 = amap[pt1_x][pt1_y];
				
				if (h2 > 0)
				{
					hit++;
				}
				else
				{
					err++;
				}

				if (cal_probability((int)h) > 0.3)
				{
					hit2++;
				}
				else if (cal_probability((int)h) < -0.3)
				{
					err2++;
				}
				else
				{
					unce++;
				}

				p4 += cal_probability((int)h);
				num++;
			}
		}
	}
	p2 = ((double)hit) / (double)(hit + err);
	p = ((double)hit2) / ((double)hit2 + (double)err2 + (double)unce);
	p4 = (p * p2);

	return p4;
}

//matching with environment map and CAD map
double fitcal_MIX(int i, struct robot_position* robot, int** omap, int** amap) {

	double fit_env = 0;
	double fit_cad = 0;
	double fit_mix = 0;
	double weight = 0;
	double model_ratio = 0;
	double dis_m, dis_g;

	fit_env = fitcal_m(i, robot, omap, amap);
	fit_cad = fitcal_m(i, robot, omap_CAD, amap_CAD);

	//printf("fit_env = %4f, fit_cad = %4f\n", fit_env, fit_cad);

	//model priority
	dis_m = sqrt((robot->model[0] * robot->model[0]) + (robot->model[1] * robot->model[1]));
	dis_g = sqrt((robot->gen_m[i][0] * robot->gen_m[i][0]) + (robot->gen_m[i][1] * robot->gen_m[i][1]));
	model_ratio = abs(dis_m - dis_g) / dis_m;
	//

	weight = map_match_rate * (1 - model_ratio);
	fit_mix = weight * fit_env + (1 - weight) * fit_cad;

	return fit_mix;
}

//
// 配列内の最大値位置を返す
//
int g_max(struct robot_position *robot, double *best_fitness)
{
	int arr_count;
	double max;
	for (int i = 0; i < GANM; i++) 
	{
		if (i == 0) 
		{
			max = robot->fit_m[i];
			arr_count = i;
		}
		else if (max < robot->fit_m[i]) 
		{
			max = robot->fit_m[i];
			arr_count = i;
		}
	}
	*best_fitness = max;
	return arr_count;
}

//
//配列内の最小値位置を返す
//
int g_min(struct robot_position *robot, double *worst_fitness)
{
	int arr_count;
	double min;
	for (int i = 0; i < GANM; i++) 
	{
		if (i == 0) 
		{
			min = robot->fit_m[i];
			arr_count = i;
		}
		else if (min > robot->fit_m[i]) 
		{
			min = robot->fit_m[i];
			arr_count = i;
		}
	}
	*worst_fitness = min;
	return arr_count;
}

void mga_init(struct robot_position *robot, int **omap, int **amap)
{

	//ロボットモデルを用いた予測 henkoModel
	double model[3];
	calcRobotModel(model);
    robot->model[0] = model[0];  //2022.04.28追加
    robot->model[1] = model[1];  //2022.04.28追加
    robot->model[2] = model[2];  //2022.04.28追加
    
	for (int i = 0; i < GANM; i++)
	{
		if (ga_search_range == 0)
		{
			// GA初期個体配置改善なし
			if (i == 0)
			{
				robot->gen_m[i][0] = robot->gen_m[robot->best_gene][0];
				robot->gen_m[i][1] = robot->gen_m[robot->best_gene][1];
				robot->gen_m[i][2] = robot->gen_m[robot->best_gene][2];
				if (apply) {
					robot->fit_m[i] = fitcal_MIX(i, robot, omap, amap);
				}
				else {
					robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
				}
				continue;
			}
			else if (i == 1)
			{
				robot->gen_m[i][0] = 0;
				robot->gen_m[i][1] = 0;
				robot->gen_m[i][2] = 0;
				if (apply) {
					robot->fit_m[i] = fitcal_MIX(i, robot, omap, amap);
				}
				else {
					robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
				}
				continue;
			}
			else if (i == 2)
			{
				robot->gen_m[i][0] = -robot->gen_m[robot->best_gene][0];
				robot->gen_m[i][1] = -robot->gen_m[robot->best_gene][1];
				robot->gen_m[i][2] = -robot->gen_m[robot->best_gene][2];
				if (apply) {
					robot->fit_m[i] = fitcal_MIX(i, robot, omap, amap);
				}
				else {
					robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
				}
				continue;
			}
			else
			{
				if (modelMode == 0) {  //henkoModel
					robot->gen_m[i][0] = 40.0 * rndn();
					robot->gen_m[i][1] = 40.0 * rndn();
					robot->gen_m[i][2] = 1.0*rndn();
				}
				else {
					//ロボットモデルを用いた予測結果に基づく初期化 henkoModel
					robot->gen_m[i][0] = model[0] + 40.0 * rndn();
					robot->gen_m[i][1] = model[1] + 40.0 * rndn();
					robot->gen_m[i][2] = model[2] + 1.0*rndn();
				}
			}
		}
		else
		{
			// GA初期個体配置改善あり
			if (modelMode == 0) {  //henkoModel
				robot->gen_m[i][0] = 40.0 * rndn();
				robot->gen_m[i][1] = 40.0 * rndn();
				robot->gen_m[i][2] = 1.0*rndn();
			}
			else {
				//ロボットモデルを用いた予測結果に基づく初期化 henkoModel
				robot->gen_m[i][0] = model[0] + 40.0 * rndn() * ga_search_range;
				robot->gen_m[i][1] = model[1] + 40.0 * rndn() * ga_search_range;
				robot->gen_m[i][2] = model[2] + 1.0*rndn();
			}
		}

		if (apply) {
			robot->fit_m[i] = fitcal_MIX(i, robot, omap, amap);
		}
		else {
			robot->fit_m[i] = fitcal_m(i, robot, omap, amap);
		}
	}
}


void map_mating(struct robot_position *robot, int **omap, int **amap) {

	int t = 0, g, g2;
	int worst_gene;
	double worst_fitness;
	double best_fitness;

	//
	if (apply) {
		if (map_match_rate < 0.98) {//////////////////////////////////////////////////////////////////////////

			//printf("reset environment map\n");

			for (int i = 0; i < width; i++) {
				for (int j = 0; j < height; j++) {

					omap[i][j] = omap_CAD[i][j];
					amap[i][j] = amap_CAD[i][j];

				}
			}
			mapUpdate = true;
		}
	}
	//

	if (miniMapMode == true) {  //henkoModel
		draw_gen(0, robot, t, foutp);
	}

	mga_init(robot, omap, amap);

	if (miniMapMode == true) {  //henkoModel
		draw_gen(1, robot, t, foutp);
	}

	while (t < T2)
	{
		double r, r2;
		robot->best_gene = g_max(robot, &best_fitness);
		worst_gene = g_min(robot, &worst_fitness);

		g = (int)(GANM * rnd());
		while (g == robot->best_gene)
		{
			g = (int)(GANM * rnd());
		}

		r2 = 0.5;//(10.0*(best_fitness - robot->fit_m[g])) / (best_fitness - worst_fitness + 0.01);
		g2 = robot->best_gene;
		for (int j = 0; j < GALM; j++)
		{
			r = rnd();
			//-------crossover-------------------------
			if (r < 0.7)
			{
				robot->gen_m[worst_gene][j] = robot->gen_m[g2][j];
			}
			else
			{
				robot->gen_m[worst_gene][j] = (robot->gen_m[g][j] + robot->gen_m[g2][j]) * rnd();
			}

			//------mutation---------------------------
			if (rnd() < 0.7)
			{
				if (j != 2)
				{
					robot->gen_m[worst_gene][j] += 10 * rndn_10() * (0.01 + r2 * (double)(T2 - t) / (double)T2);
				}

				else
				{
					robot->gen_m[worst_gene][j] += rndn() * (0.01 + r2 * (double)(T2 - t) / (double)T2);
				}
			}
			else
			{
				if (j != 2)
				{
					robot->gen_m[worst_gene][j] += 10.0 * rndn_10() * rndn() * (0.01 + r2 * (double)(T2 - t) / (double)T2);
				}
				else
				{
					robot->gen_m[worst_gene][j] += rndn() * rndn() * (0.01 + r2 * (double)(T2 - t) / (double)T2);
				}
			}
		}

		robot->fit_m[worst_gene] = fitcal_m(worst_gene, robot, omap, amap);
		if (miniMapMode == true) {  //henkoModel
			if (t % 5 == 0) {
				//draw_gen(1, robot, t, foutp);
			}
		}
		t++;
	}

	if (miniMapMode == true) {  //henkoModel
		draw_gen(2, robot, t, foutp);
	}

	robot->best_gene = g_max(robot, &best_fitness);
	//printf("best_fitness = %4f, slam_count = %d\n", best_fitness, robot->slam_count);

}

void draw_map(int k, int l, int map_value, IplImage *omap_Image)
{
	CvPoint pt;
	double p;

	if (map_value > 0) 
	{
		p = cal_probability(map_value);
		if (p >= 0.8) 
		{
			pt.x = k;
			pt.y = l;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 0;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 0;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 0;
		}
		else 
		{
			pt.x = k;
			pt.y = l;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 125 - (int)(125 * p);
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 125 - (int)(125 * p);
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 125 - (int)(125 * p);
		}
	}
	else 
	{
		p = cal_probability(abs(map_value));
		if (p >= 0.8) 
		{
			pt.x = k;
			pt.y = l;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = (char)255;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = (char)255;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = (char)255;
		}
		else 
		{
			pt.x = k;
			pt.y = l;
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3] = 125 + (int)(125 * p);
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 1] = 125 + (int)(125 * p);
			omap_Image->imageData[omap_Image->widthStep*pt.y + pt.x * 3 + 2] = 125 + (int)(125 * p);
		}
	}
}

void draw_updated_map(int** omap, IplImage* omap_Image) {

	CvPoint pt;
	double p;
	double map_value;

	for (int k = 0; k < width; k++) {
		for (int l = 0; l < height; l++) {

			map_value = omap[k][l];

			if (map_value > 0)
			{
				p = cal_probability(map_value);
				if (p >= 0.8)
				{
					pt.x = k;
					pt.y = l;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3] = 0;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 1] = 0;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 2] = 0;
				}
				else
				{
					pt.x = k;
					pt.y = l;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3] = 125 - (int)(125 * p);
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 1] = 125 - (int)(125 * p);
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 2] = 125 - (int)(125 * p);
				}
			}
			else
			{
				p = cal_probability(abs(map_value));
				if (p >= 0.8)
				{
					pt.x = k;
					pt.y = l;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3] = (char)255;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 1] = (char)255;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 2] = (char)255;
				}
				else
				{
					pt.x = k;
					pt.y = l;
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3] = 125 + (int)(125 * p);
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 1] = 125 + (int)(125 * p);
					omap_Image->imageData[omap_Image->widthStep * pt.y + pt.x * 3 + 2] = 125 + (int)(125 * p);
				}
			}
		}
	}

}

void calc_mapvalue(int k, int l, int pt_x, int pt_y, int sflag, int **omap, int **amap, IplImage *omap_Image)
{
	//if (omap[k][l] < 100) {
	if (sflag == 1)
	{
		if (k >= pt_x)
		{
			omap[k][l] += 2;
		}
		else
		{
			omap[k][l]--;
		}
	}
	else if (sflag == 2)
	{
		if (k <= pt_x)
		{
			omap[k][l] += 2;
		}
		else
		{
			omap[k][l]--;
		}
	}
	else if (sflag == 3)
	{
		if (l >= pt_y)
		{
			omap[k][l] += 2;
		}
		else
		{
			omap[k][l]--;
		}
	}
	else
	{
		if (l <= pt_y)
		{
			omap[k][l] += 2;
		}
		else
		{
			omap[k][l]--;
		}
	}
	//}

	if (cal_probability(amap[k][l]) > 0.85)
	{
		if (omap[k][l] < 0 && amap[k][l] + omap[k][l] > 0)
		{
			omap[k][l] += amap[k][l];
		}
	}
	draw_map(k, l, omap[k][l], omap_Image);

}

void map_building2(struct robot_position *robot, int **omap, int **amap, IplImage *omap_Image, IplImage *amap_Image)
{

	int pt_x, pt_y, ptc_x, ptc_y;
	int dx, dy;
	double x_3, y_3;
	double s1, c1;
	CvPoint pt, pt2;

	s1 = sin((double)robot->dangle * PI / 180.0);
	c1 = cos((double)robot->dangle * PI / 180.0);
	ptc_x = (int)(map_center_width - (int)robot->real_x / map_rate);
	ptc_y = (int)(map_center_height - (int)robot->real_y / map_rate);

	//RS-LiDAR用
	int num;
	if (SENSOR_MODE == 0) {
		num = SCAN_MARKS;
	}
	else {
		num = rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM;
	}

	if (apply) {
		if (mapUpdate) {
			//printf("map updated!\n");
			draw_updated_map(omap, omap_Image);
			mapUpdate = false;
		}
	}

	//update envmap
	for (int j = 0; j < num; j++)
	{
		if (x[j] != ERR && y[j] != ERR)
		{
			x_3 = (double)x[j] * c1 - (double)y[j] * s1 + (double)robot->real_x;
			y_3 = (double)x[j] * s1 + (double)y[j] * c1 + (double)robot->real_y;
			pt_x = (int)(map_center_width - (int)x_3 / map_rate);
			pt_y = (int)(map_center_height - (int)y_3 / map_rate);

			if ((pt_x < 0) || (pt_x >= map_size_width) || (pt_y < 0) || (pt_y >= map_size_height))
			{
				continue;
			}
			//if (pt_x < 0 || pt_y < 0)
			//{
			//	continue;
			//}

			amap[pt_x][pt_y]++;
			dx = (pt_x - ptc_x);
			dy = (pt_y - ptc_y);

			int e = 0;
			if (dx >= 0 && dy >= 0)
			{
				if (dx > dy)
				{
					for (int l = ptc_y, k = ptc_x; k <= pt_x; k++)
					{
						e += dy;
						if (e > dx)
						{
							e -= dx;
							l++;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap, omap_Image);
					}
				}
				else
				{
					for (int l = ptc_y, k = ptc_x; l <= pt_y; l++)
					{
						e += dx;
						if (e > dy)
						{
							e -= dy;
							k++;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap, omap_Image);
					}
				}
			}
			else if (dx >= 0 && dy <= 0)
			{
				dy *= -1;
				if (dx > dy)
				{
					for (int l = ptc_y, k = ptc_x; k <= pt_x; k++)
					{
						e += dy;
						if (e > dx)
						{
							e -= dx;
							l--;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 1, omap, amap, omap_Image);
					}
				}
				else
				{
					for (int l = ptc_y, k = ptc_x; l >= pt_y; l--)
					{
						e += dx;
						if (e > dy)
						{
							e -= dy;
							k++;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap, omap_Image);
					}
				}
			}
			else if (dx <= 0 && dy >= 0)
			{
				dx *= -1;
				if (dx > dy)
				{
					for (int l = ptc_y, k = ptc_x; k >= pt_x; k--)
					{
						e += dy;
						if (e > dx)
						{
							e -= dx;
							l++;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap, omap_Image);
					}
				}
				else
				{
					for (int l = ptc_y, k = ptc_x; l <= pt_y; l++)
					{
						e += dx;
						if (e > dy)
						{
							e -= dy;
							k--;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 3, omap, amap, omap_Image);
					}
				}
			}
			else
			{
				dx *= -1;
				dy *= -1;
				if (dx > dy)
				{
					for (int l = ptc_y, k = ptc_x; k >= pt_x; k--) {
						e += dy;
						if (e > dx)
						{
							e -= dx;
							l--;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 2, omap, amap, omap_Image);
					}
				}
				else
				{
					for (int l = ptc_y, k = ptc_x; l >= pt_y; l--)
					{
						e += dx;
						if (e > dy)
						{
							e -= dy;
							k--;
						}

						calc_mapvalue(k, l, pt_x, pt_y, 4, omap, amap, omap_Image);
					}
				}
			}

			if (amap[pt_x][pt_y] > 0 && amap_Image != NULL)
			{
				pt.x = (int)(map_center_width - (int)x_3 / map_rate);
				pt.y = (int)(map_center_height - (int)y_3 / map_rate);
				pt2.x = (int)(map_center_width - (int)x_3 / map_rate);
				pt2.y = (int)(map_center_height - (int)y_3 / map_rate);
				amap_Image->imageData[amap_Image->widthStep * pt.y + pt.x * 3] = 0;
				amap_Image->imageData[amap_Image->widthStep * pt.y + pt.x * 3 + 1] = 0;
				amap_Image->imageData[amap_Image->widthStep * pt.y + pt.x * 3 + 2] = 0;
			}

			draw_map(pt_x, pt_y, omap[pt_x][pt_y], omap_Image);
		}
	}
	//

	//save env info
	if (apply) {
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {

				if (cal_probability(omap_CAD[i][j]) < 0.8 && cal_probability(omap[i][j]) >= 0.8 && amap[i][j] > 100) {

					omap_CAD[i][j] = omap[i][j];
					amap_CAD[i][j] = amap[i][j];

				}

			}
		}
	}
	//

}


double transformation_angle(double theta1)
{
	double err_a;

	if (fabs(theta1) > 180) 
	{
		err_a = fabs(theta1) - 180.0;
		if (theta1 > 0) 
		{
			theta1 = -180.0;
			theta1 += err_a;
		}
		else 
		{
			theta1 = 180.0;
			theta1 -= err_a;
		}
	}

	return theta1;
}

int exception_followerdata(struct robot_position *robot, struct robot_position *frobot)
{
	double theta1, theta2, theta3;
	double lf_dis;
	const int SENSOR_RANGE = 3900;

	lf_dis = (frobot->real_x - robot->real_x) * (frobot->real_x - robot->real_x) + (frobot->real_y - robot->real_y) * (frobot->real_y - robot->real_y);

	if (lf_dis < SENSOR_RANGE * SENSOR_RANGE) 
	{
		theta1 = 180.0 * atan2(frobot->real_y - robot->real_y, frobot->real_x - robot->real_x) / M_PI - 90.0;

		theta1 = transformation_angle(theta1);

		theta2 = robot->rangle;
		theta3 = -theta1 + theta2;

		theta3 = transformation_angle(theta3);

		return (int)(theta3 * (1024.0 / 360.0)) + 384;
	}
	else
	{
		return 0;
	}
}

void compare_map(int** omap, int** amap) {

	int total = 0;
	int match = 0;
	double p_env = 0;
	double p_cad = 0;

	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			
			p_env = cal_probability(omap[i][j]);
			p_cad = cal_probability(omap_CAD[i][j]);

			if (p_env >= 0.8) {

				total++;
				if (p_cad >= 0.8) {
					match++;
				}

			}
		}
	}
	//printf("total = %d,  match = %d\n", total, match);

	map_match_rate = (double)match / (double)total;

	//printf("map_match_rate = %4f\n", map_match_rate);

}

void slam(struct robot_position *robot, long *lrf_data, int **omap, int **amap, double *outp)  //henkoModel
{
	//int i, j;
	static double pre_angle;
	static double m_x, m_y;
	double s_x, s_y, s_r;

	foutp[0] = outp[0];  //henkoModel
	foutp[1] = outp[1];	 //

	pre_angle = robot->dangle;
	if (robot->slam_count == 0) 
	{
		FILE *fp = fopen("C:\\kinden\\log\\fitnessSLAM.csv", "w");  //henkoModel
		fclose(fp);
		for (int i = 0; i < SCAN_MARKS; i++)
		{
			theta[i] = (((double)i - 540.0) * (360.0 / 1440.0) * M_PI) / 180.0;
		}

		for (int i = 0; i < GANM + 1; i++) 
		{
			robot->fit_m[i] = 0;
			for (int j = 0; j < GALM + 1; j++)
			{
				robot->gen_m[i][j] = 0;
			}
		}

		robot->best_gene = 0;

		//copy CAD map
		if (apply) {
			printf("save initial baseMap as CAD map\n");
			omap_CAD = malloc2d_int(width, height);
			amap_CAD = malloc2d_int(width, height);
			omap_CAD_for_fit = malloc2d_int(width, height);
			amap_CAD_for_fit = malloc2d_int(width, height);

			for (int i = 0; i < width; i++) {
				for (int j = 0; j < height; j++) {

					omap_CAD[i][j] = omap[i][j];
					amap_CAD[i][j] = amap[i][j];
					omap_CAD_for_fit[i][j] = omap[i][j];
					amap_CAD_for_fit[i][j] = amap[i][j];

				}
			}
		}
		//

	}

	for (int i = 0; i < SCAN_MARKS; i++)
	{
		//henko2
		//if (lrf_data[i] < 200 || lrf_data[i] > 40000 || i < 200 || i > 880) {
		//if (lrf_data[i] < 200 || lrf_data[i] > 40000) {
		if (lrf_data[i] < 100 || lrf_data[i] > 40000) {
			x[i] = ERR;
			y[i] = ERR;
		}
		else
		{

			x[i] = lrf_data[i] * sin(theta[i]);
			y[i] = lrf_data[i] * cos(theta[i]);
		}
	}

	if (robot->slam_count != 0) 
	{

		//compare environment map and CAD map
		if (apply) {
			compare_map(omap, amap);
		}
		//

		map_mating(robot, omap, amap);

		m_x += robot->gen_m[robot->best_gene][0];
		m_y += robot->gen_m[robot->best_gene][1];
		robot->dangle += robot->gen_m[robot->best_gene][2];
		s_x = 0, s_y = 0, s_r = 0;
		s_x = robot->gen_m[robot->best_gene][0];
		s_y = robot->gen_m[robot->best_gene][1];
		s_r = robot->gen_m[robot->best_gene][2];

		robot->real_x += s_x*cos(robot->dangle * PI / 180) - s_y*sin(robot->dangle * PI / 180);
		robot->real_y += s_y*cos(robot->dangle * PI / 180) + s_x*sin(robot->dangle * PI / 180);

		//printf("robot_rx: %f", robot->real_x); //TMU20190820 非表示化
		//printf("robot_ry: %f", robot->real_y);

		robot->rangle += robot->dangle - pre_angle;

		robot->map_x = (int)(map_center_width - (int)robot->real_x / map_rate);
		robot->map_y = (int)(map_center_height - (int)robot->real_y / map_rate);

		double err_a;
		if (fabs(robot->rangle) > 180) 
		{
			err_a = fabs(robot->rangle) - 180.0;
			if (robot->rangle > 0) 
			{
				robot->rangle = -180.0; 
				robot->rangle += err_a;
			}
			else 
			{
				robot->rangle = 180.0;
				robot->rangle -= err_a;
			}
		}
	}

	//SLAM地図崩壊判定、通知
	if (run_difficult_notice)
	{
		if (robot->fit_m[robot->best_gene] < fitness_th)	//fitness値が閾値を下回ったら
		{
			if (fitness_flag == 0)
			{
				fitness_flag = 1;
				fitness_start_time = clock();
				fitness_end_time = clock();
			}
			else
			{
				fitness_end_time = clock();
			}

			if ((fitness_end_time - fitness_start_time) > (fitness_time * 1000))	//fitness値が閾値を下回った時間が判定時間を経過したら
			{
				fitness_count++;

				fitness_start_time = clock();
				fitness_end_time = clock();

				if (fitness_count == fitness_cnt)
				{
					fitness_count = 0;
					fitness_flag = 0;

					fitness_mail_end_time = clock();
					if (fitness_mail_end_time - fitness_mail_start_time > 60 * 1000)
					{
						sendMail(RUN_DIFFICULT_MAIL);
						fitness_mail_start_time = clock();
					}
				}
			}
		}
		else
		{
			fitness_flag = 0;
		}
	}

	//適応度保存 henkoModel
	FILE *fp = fopen("C:\\kinden\\log\\fitnessSLAM.csv", "a");
	fprintf(fp, "%d,", robot->slam_count);
	fprintf(fp, "%f,", robot->fit_m[robot->best_gene]);
	fprintf(fp, "\n");
	fclose(fp);

	robot->slam_count++;
}

void slam2(struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, int** omap, int** amap, double* outp)
{
	//int i, j;
	static double pre_angle;
	static double m_x, m_y;
	double s_x, s_y, s_r;

	foutp[0] = outp[0];  //henkoModel
	foutp[1] = outp[1];

	pre_angle = robot->dangle;
	if (robot->slam_count == 0)
	{
		FILE* fp = fopen("C:\\kinden\\log\\fitnessSLAM.csv", "w");  //henkoModel
		fclose(fp);
		for (int i = 0; i < SCAN_MARKS; i++)
		{
			theta[i] = (((double)i - 540.0) * (360.0 / 1440.0) * M_PI) / 180.0;
		}

		for (int i = 0; i < GANM + 1; i++)
		{
			robot->fit_m[i] = 0;
			for (int j = 0; j < GALM + 1; j++)
			{
				robot->gen_m[i][j] = 0;
			}
		}

		robot->best_gene = 0;
	}

	//RS-LiDAR用
	for (int i = 0; i < rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM; i++) {
		if (lrf_dataD[i] < LASER_RANGE_MIN || lrf_dataD[i] > LASER_RANGE_MAX) {
			x[i] = ERR;
			y[i] = ERR;
		}
		else {
			x[i] = (double)lrf_dataD[i] * sin(lrf_dataA[i]);
			y[i] = (double)lrf_dataD[i] * cos(lrf_dataA[i]);
		}
	}

	if (robot->slam_count != 0)
	{
		map_mating(robot, omap, amap);


		//??????????map_mating()?????
		double min_fit = 0;
		if (robot->fit_m[robot->best_gene] < min_fit) {

		}


		m_x += robot->gen_m[robot->best_gene][0];
		m_y += robot->gen_m[robot->best_gene][1];
		robot->dangle += robot->gen_m[robot->best_gene][2];
		s_x = 0, s_y = 0, s_r = 0;
		s_x = robot->gen_m[robot->best_gene][0];
		s_y = robot->gen_m[robot->best_gene][1];
		s_r = robot->gen_m[robot->best_gene][2];

		robot->real_x += s_x * cos(robot->dangle * PI / 180) - s_y * sin(robot->dangle * PI / 180);
		robot->real_y += s_y * cos(robot->dangle * PI / 180) + s_x * sin(robot->dangle * PI / 180);

		//printf("robot_rx: %f", robot->real_x); //TMU20190820 非表示化
		//printf("robot_ry: %f", robot->real_y);

		robot->rangle += robot->dangle - pre_angle;

		robot->map_x = (int)(map_center_width - (int)robot->real_x / map_rate);
		robot->map_y = (int)(map_center_height - (int)robot->real_y / map_rate);

		double err_a;
		if (fabs(robot->rangle) > 180)
		{
			err_a = fabs(robot->rangle) - 180.0;
			if (robot->rangle > 0)
			{
				robot->rangle = -180.0;
				robot->rangle += err_a;
			}
			else
			{
				robot->rangle = 180.0;
				robot->rangle -= err_a;
			}
		}
	}

	//SLAM地図崩壊判定、通知
	if (run_difficult_notice)
	{
		if (robot->fit_m[robot->best_gene] < fitness_th)	//fitness値が閾値を下回ったら
		{
			if (fitness_flag == 0)
			{
				fitness_flag = 1;
				fitness_start_time = clock();
				fitness_end_time = clock();
			}
			else
			{
				fitness_end_time = clock();
			}

			if ((fitness_end_time - fitness_start_time) > (fitness_time * 1000))	//fitness値が閾値を下回った時間が判定時間を経過したら
			{
				fitness_count++;

				fitness_start_time = clock();
				fitness_end_time = clock();

				if (fitness_count == fitness_cnt)
				{
					fitness_count = 0;
					fitness_flag = 0;

					fitness_mail_end_time = clock();
					if (fitness_mail_end_time - fitness_mail_start_time > 60 * 1000)
					{
						sendMail(RUN_DIFFICULT_MAIL);
						fitness_mail_start_time = clock();
					}
				}
			}
		}
		else
		{
			fitness_flag = 0;
		}
	}

	//適応度保存 henkoModel
	FILE* fp = fopen("C:\\kinden\\log\\fitnessSLAM.csv", "a");
	fprintf(fp, "%d,", robot->slam_count);
	fprintf(fp, "%f,", robot->fit_m[robot->best_gene]);
	fprintf(fp, "\n");
	fclose(fp);

	robot->slam_count++;
}

void calcRobotModel(double* m) {  //henkoModel
	static double alpha[2] = { 0.2, 0.2 };  //変換倍率
	static double r = 125.0;  //車輪直径[mm]
	static double d = 350.0;  //車輪間隔[mm]
	static double dt = 0.020;  //微小時間[s]

	double wL, wR;  //車輪回転角速度[rad/s]
	double vL, vR, v;  //車輪移動速度[mm/s]
	double w, l;  //旋回角速度[rad/s], 旋回半径[mm]

	double dx, dy, dtheta;  //移動量[mm, deg]
	static int init_flag = 1;


	double h = 0.0;  //2022.04.13パラメータh追加

	//2022.04.13時間計測追加
	static time_t startTime, endTime;
	if (init_flag == 1) {
		startTime = clock();
	}
	endTime = clock();
	dt = (double)(endTime - startTime) / CLOCKS_PER_SEC;  //処理時間[s]
	//printf("%f\n", dt);


	if (init_flag == 1) {
		//char buf[255];
		//FILE *fp = fopen("robot_model.txt", "r");
		//fscanf(fp, "%s\t%lf", buf, r);
		//fscanf(fp, "%s\t%lf", buf, d);
		//fscanf(fp, "%s\t%lf", buf, dt);
		//fscanf(fp, "%s\t%lf", buf, alpha[0]);
		//fscanf(fp, "%s\t%lf", buf, alpha[1]);
		//fclose(fp);

		//FILE *fp1 = fopen("C:\\kinden\\log\\motorSpeed.csv", "w");
		//fclose(fp1);

		r = wheelDia;
		d = wheelWidth;
		h = lidarWheelDist;
		//dt = microTime;  //2022.04.13削除
		alpha[0] = outputCoefL;
		alpha[1] = outputCoefR;
		init_flag = 0;
	}

	FILE* fp2 = fopen("C:\\kinden\\log\\motorSpeed.csv", "a");
	fprintf(fp2, "%lf,%lf\n", foutp[0], foutp[1]);
	fclose(fp2);


	wL = alpha[0] * foutp[1];
	wR = alpha[1] * foutp[0];

	vL = (r / 2.0) * wL;
	vR = (r / 2.0) * wR;

	v = (vL + vR) / 2.0;
	w = (vR - vL) / d;
	l = v / w;

	if (abs(vR - vL) < 1e-10 * 1000.0) {
		dx = 0.0;
		dy = v * dt;
		dtheta = 0.0;
	}
	else {
		//2022.04.13パラメータh追加
		dx = l * cos(w * dt) - l * cos(0) + h * sin(w * dt);
		dy = l * sin(w * dt) - l * sin(0) + h * cos(w * dt);
		dtheta = w * dt * 180.0 / M_PI;
	}

	m[0] = -dx;
	m[1] = dy;
	m[2] = -dtheta;

	//2022.04.13時間計測追加
	startTime = clock();
}


