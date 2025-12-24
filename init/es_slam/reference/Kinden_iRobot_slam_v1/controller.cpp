/*
*  controller.cpp
*  slam
*
*  Created by Naoyuki Kubota on 16/07/12 and modified by Wei Hong on 12/7/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/

#include "controller.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "Def.h"

int cpath_ct = 0;

/*ファジィ推論IF-THENルール↓*/
double calmemf3(int r_num, double det, double t_theta) 
{
	double tlw = 1.0;//total weight 総合の重み(初期化)
	int i;
	double ptrg = 0.5, ptrg2 = 1.5;
	double t_rad;

	/*const int frif1[RULENUM1][MEMNUM1] = {
		{ 0,0 }, // rule 0	Negative & S(目標が後ろにある時)
				 //{0,1}, // rule 1	N & MS(Sより)
		{ 0,2 }, // rule 2	N & M
				 //{0,3}, // rule 3	N & ML
		{ 0,4 }, // rule 4	N & L(近くにある時)
		{ 1,0 }, // rule 5	P & S
				 //{1,1}, // rule 6	P & MS
		{ 1,2 }, // rule 7	P & M
				 //{1,3}, // rule 8	P & ML
		{ 1,4 }
	};//rule 9	P & L          fuzzy reference if文
	*/
	const int frif1[RULENUM1][MEMNUM1] = 
	{
		{ 0,0 }, // rule 0	Negative & S(目標が後ろにある時)
		//{ 0,1 }, // rule 1	N & MS(Sより)
		{ 0,2 }, // rule 2	N & M
		{ 0,3 }, // rule 3	N & ML
		{ 0,4 }, // rule 4	N & L(近くにある時)
		{ 1,0 }, // rule 5	P & S
		//{ 1,1}, // rule 6	P & MS
		{ 1,2 }, // rule 7	P & M
		{ 1,3 }, // rule 8	P & ML
		{ 1,4 }
	};//rule 9	P & L          fuzzy reference if文


	  // angle ------------------------------------------------------------
	  // Negative
	i = 0;
	t_rad = M_PI * t_theta / 180.0;
	if (frif1[r_num][i] == 0) 
	{
		if (t_rad < -M_PI / 2)
		{
			tlw *= 1.0;
		}
		else
		{
			tlw *= exp((-(t_rad + M_PI / 2) * (t_rad + M_PI / 2)) / 2.0);
		}
	}
	// Positive
	else if (frif1[r_num][i] == 1) 
	{
		if (t_rad > M_PI / 2)
		{
			tlw *= 1.0;
		}
		else
		{
			tlw *= exp((-(t_rad - M_PI / 2) * (t_rad - M_PI / 2)) / 2.0);
		}
	}


	// distance ---------------------------------------------------------
	// S
	i = 1;
	if (frif1[r_num][i] == 0) 
	{
		if (det > -ptrg2)
		{
			tlw *= exp(-(det + ptrg2) * (det + ptrg2) / 0.0045);	// mobir 0.0045
		}														//else if(det>0)
	}
	// MS
	else if (frif1[r_num][i] == 1) 
	{
		tlw *= exp(-(det + ptrg) * (det + ptrg) / 0.002);
	}
	// M
	else if (frif1[r_num][i] == 2) 
	{
		tlw *= exp(-det*det / 0.0002);
	}
	// ML
	else if (frif1[r_num][i] == 3) 
	{
		tlw *= exp(-(det - ptrg) * (det - ptrg) / 0.002);
	}
	// L
	else if (frif1[r_num][i] == 4) 
	{
		if (det < ptrg2)
		{
			tlw *= exp(-(det - ptrg2) * (det - ptrg2) / 0.0045);
		}
	}

	return tlw;
}

void fuzzyinf3(double det, double t_theta, double outp[])//ファジィ推論 OK
{
	double f[RULENUM1];
	//int i, j;

	const double frth1[RULENUM1][CTRLNUM] = {
		{ 0.4, -0.2 },	// rule 0	N & S
						//{-50,0}, // rule 1	N & MS
		{ 0,    0 },	// rule 2	N & M
		{ 0.6,  0.2},	// rule 3	N & ML
		{ 1.0,  0.6 },	// rule 4	N & L
		{ -0.2, 0.4 },	// rule 5	P & S
						//{0,-40},// rule 6	P & MS
		{ 0,    0 },	// rule 7	P & M
		{ 0.2,  0.6},	// rule 8	P & ML
		{ 0.6,  1.0 } };// rule 9	P & L  //fuzzy reference for文 {右車輪, 左車輪}

	for (int i = 0; i < CTRLNUM; i++)	//for (ic=0;ic<cntl;ic++) 
	{
		outp[i] = 0;//outp[it][ib][ic]=0;	// init outputs
	}

	double tlw = 0.0;

	//printf("TT");
	for (int j = 0; j < RULENUM1; j++)	//for (ir=0;ir<10;ir++) 
	{
		f[j] = calmemf3(j, det, t_theta);//各ルールの適応度 f[ir]= calmemf3(it,ib,ir);
		for (int i = 0; i < CTRLNUM; i++)
		{
			outp[i] += f[j] * frth1[j][i];
		}
		tlw += f[j];
		//if (f[j] * 100.0 > 10.0)
		//	printf("[%d]%2.0f ", j, f[j] * 100.0);
	}
	//printf("\n");

	for (int i = 0; i < CTRLNUM; i++)	//for (ic=0;ic<cntl;ic++) 
	{
		if (tlw > 0)
		{
			outp[i] = /*MAX_OUTPUT*/outp[i] / tlw;//total weightで正規化 outp[it][ib][ic] = outp[it][ib][ic]/tlw;
		}
		//printf("%lf,", outp[i]); TMU20190819 非表示化
	}
	//printf("\n"); TMU20190819
}

//===================================================================================
/*ファジィ推論IF-THENルール（障害物回避）↓*/
double calmemf(int m, double *sdt)   	//  Fuzzy inference of rule m 
{
	double	x[MEMF + 1],	// Fitness of membership function
			tlw;			// total weights

	const int frif[RULE + 1][MEMF + 1] = 	// Fuzzy Rules if parts (premise)
	{										//Collision Avoidance using　LRF
											//0,1,2,3,4
		{ 0, 0, 0, 0, 0 },	//Rule1:
		{ 1, 0, 0, 0, 0 },	//Rule2:
		{ 0, 1, 0, 0, 0 },	//Rule3:
		{ 0, 0, 1, 0, 0 },	//Rule4:
		{ 0, 0, 0, 1, 0 },	//Rule5:
		{ 0, 0, 0, 0, 1 },	//Rule6:
		{ 1, 1, 0, 0, 0 },    //Rule7:　障害物回避の改善のため、ルールの追加
		{ 0, 1, 1, 0, 0 },    //Rule8:
		{ 0, 0, 1, 1, 0 },    //Rule9:
		{ 0, 0, 0, 1, 1 },    //Rule10:
		{ 1, 1, 1, 0, 0 },    //Rule11:
		{ 0, 1, 1, 1, 0 },    //Rule12:
		{ 0, 0, 1, 1, 1 },    //Rule13:
		{ 1, 1, 1, 1, 0 },    //Rule14:
		{ 0, 1, 1, 1, 1 },    //Rule15:
		{ 1, 1, 1, 1, 1 },    //Rule16:
	};

	tlw = 1.0;
	for (int i = 0; i < MEMF; i++) 
	{
		x[i] = 0;
		if (frif[m][i] == 0)
		{
			//x[i] = sdt[i] / MAX_LRF_DISTANCE;
			if (sdt[i] < MIN_LRF_DISTANCE) //TMU20190823　最小距離の導入
				x[i] = 0.0;
			else
				x[i] = (sdt[i] - MIN_LRF_DISTANCE) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
		}
		else if (frif[m][i] == 1)
		{
			//x[i] = (MAX_LRF_DISTANCE - sdt[i]) / MAX_LRF_DISTANCE;
			if (sdt[i] < MIN_LRF_DISTANCE) //TMU20190823　最小距離の導入
				x[i] = 1.0;
			else
				x[i] = (MAX_LRF_DISTANCE - sdt[i]) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
		}
		tlw *= x[i];
	}
	return(tlw);
}

void fuzzyinf(double *sdt, double *outp)   	// Fuzzy inference  
{
	//int		i, j;		// h: behavior No.
	double	f[RULE],
		tlw;		// total output, total weight
	
	//TMU20190820 正面の障害物への対処
	static time_t start, end; 
	static int init_flag = 1;
	static double rnd0, rnd1;

	//TMU20190820 正面の障害物への対処
	if (init_flag == 1)
	{
		start = clock();
		rnd0 = rnd();
		rnd1 = rnd();
		init_flag = 0;
	}

	end = clock();
	if ((double)(end - start) / (double)CLOCKS_PER_SEC > 5.0) //5s毎に回避方向を変化
	{
		rnd0 = rnd();
		rnd1 = rnd();
		start = clock();
	}
	
	double frth[RULE + 1][CNTL + 1] =		// Fuzzy Rules then parts (conclusions)
	{
		//{ 0.6,    0.6 },	//Rule1:
		//{ 0.8,    0.2 },	//Rule2:
		//{ 0.5,   -0.5 },	//Rule3:
		//{ -0.5,  -0.5 },	//Rule4:
		//{ -0.5,   0.5 },	//Rule5:
		//{ 0.2,    0.8 },	//Rule6:
		{ 0.8,    0.8 },	//Rule1:
		{ 0.8,    0.2 },	//Rule2:
		{ 0.8,   -0.8 },	//Rule3:
		//{ -0.8,  -0.8 },	//Rule4:
		{ -0.8 + 0.8*rnd0,  -0.8 + 0.8*rnd1 },	//Rule4:  TMU20190820 正面の障害物への対処
		{ -0.8,   0.8 },	//Rule5:
		{ 0.2,    0.8 },	//Rule6:
		{ 0.8,   -0.8 },	//Rule7 = 3:　TMU20190826 障害物回避の改善のため、ルールの追加
		{ 0.8,   -0.8 },	//Rule8 = 3:
		{ -0.8,   0.8 },	//Rule9 = 5:
		{ -0.8,   0.8 },	//Rule10 = 5:
		{ 1.0,   -1.0 },	//Rule11 = 3:
		{ -1.0 + 1.0*rnd0,  -1.0 + 1.0*rnd1 },	//Rule12 = 4:
		{ -1.0,   1.0 },	//Rule13 = 5:
		{ 1.0,   -1.0 },	//Rule14 = 3:
		{ -1.0,   1.0 },	//Rule15 = 5:
		{ -1.0 + 0.6*rnd0,  -1.0 + 0.6*rnd1 },	//Rule16 = 4:
	};

	for (int j = 0; j < CNTL; j++)
	{
		outp[j] = 0;	// init outputs
	}

	tlw = 0;

	//printf("CA");
	for (int i = 0; i < RULE; i++)
	{
		f[i] = calmemf(i, sdt);
		tlw += f[i];
		//if (f[i] * 100.0 > 10.0)
		//	printf("[%d]%2.0f ", i, f[i] * 100.0);
	}
	//printf("\n");


	for (int j = 0; j < CNTL; j++)
	{
		for (int i = 0; i < RULE; i++) 
		{
			outp[j] += f[i] * frth[i][j];
		}
	}

	for (int j = 0; j < CNTL; j++) 
	{
		if (tlw>0) 
		{
			outp[j] /= tlw;
		}
		//outp[j] *= MAX_OUTPUT;
		if (outp[j] > maxSpeed)
		{
			outp[j] = maxSpeed;
		}
		//printf("%lf,", outp[j]); TMU20190819　非表示化
	}

	//printf("\n"); TMU20190819
}

//=========================================================================
// Multi-objective behaviour control------------------------------
#define BEHVNUM 2 // Number of Behaviour

//double	cdng;					// Current degree of danger
double	wgt[BEHVNUM];			// Weight
double  dw[BEHVNUM][BEHVNUM];	// Update Matrix
void init_mobc()
{
	//int ib, is;

	double init_dw[2][2] = 
	{
		// CA   TTT
		{ 0.9,  0.1 },  // si[0]
		{ 0.1,  0.9 }   // si[1]
	};

	for (int ib = 0; ib < BEHVNUM; ib++)
	{
		for (int is = 0; is < BEHVNUM; is++)
		{
			dw[ib][is] = init_dw[ib][is];
		}
	}

	for (int ib = 0; ib < BEHVNUM; ib++)
	{
		wgt[ib] = 1 / 2.0;
	}
}
//===================================================================================
void Update_wgt(double si[BEHVNUM])											//重みアップデート
{
	//int ib, is;
	double dwgt, wgtn;

	//更新式計算
	for (int ib = 0; ib < BEHVNUM; ib++) 
	{
		dwgt = 0.0;
		for (int is = 0; is < BEHVNUM; is++) 
		{
			dwgt += dw[ib][is] * si[is];
		}
		wgt[ib] = wgt[ib] * 1.0 + dwgt;		//係数 = dwgt反映比率調整
	}

	//忘却＋スケール調整

	wgtn = 0.0;
	for (int ib = 0; ib < BEHVNUM; ib++)
	{
		wgtn += wgt[ib];
	}
		
	for (int ib = 0; ib < BEHVNUM; ib++)
	{
		wgt[ib] /= wgtn;
	}
}
//===================================================================================
void cal_outp(double outp[CTRLNUM], double toutp[BEHVNUM][CTRLNUM])
{
	//int ib, ic;
	double Sw, Swb;

	Sw = 0.0;
	for (int ib = 0; ib < BEHVNUM; ib++)
	{
		Sw += wgt[ib];
	}

	for (int ic = 0; ic < CTRLNUM; ic++) 
	{
		Swb = 0.0;
		for (int ib = 0; ib < BEHVNUM; ib++)
		{
			Swb += wgt[ib] * toutp[ib][ic];
		}
		outp[ic] = Swb / Sw;
	}
	//printf("CA:%.0f TT:%.0f\n", wgt[0] / Sw*100.0, wgt[1] / Sw*100.0); //TMU20190819 障害物回避(CA)と目標追従(TA)の割合
}
double cal_length(double *obj, double *robot) // calculate distance between two targets
{
	double dist_length = 0;

	dist_length = sqrt((obj[0] - robot[0]) * (obj[0] - robot[0]) + (obj[1] - robot[1]) * (obj[1] - robot[1]));

	return dist_length;
}

double cal_angle(double dist_base, double dist_top, double dist_robot) // Calculate angle btw 2 robot locations with target point
{
	double angle = 0;
	angle = double(dist_top * dist_top + dist_base * dist_base - dist_robot * dist_robot) / double(2 * dist_top * dist_base);
	angle = acos(angle) * 180 / M_PI;

	return angle;
}

int decision_making(struct robot_position* robot, double* foutp, long* lrfdata, int path[][2], int* sflag, int pct, int mode)
{
	static int initflag = 1;
	double det, t_theta;
	double t_x, t_y;
	double toutp[2][CTRLNUM];
	double outp[CTRLNUM];
	double min_dis[MEMF];	//各方向の最小距離格納用変数
							//0:左, 1:左前方, 2:中央, 3:右前方, 4:右
	double outp2[CNTL];
	const int seg = (881) / MEMF;
	double si[2];
	static double robot_prev[2], robot_curr[2], dest[2];
	static double dist_base, dist_robot, dist_top, angle, sumAngle;
	static int firstEntry = 1, loop_detect = 0;
	static time_t start, end;
	int entry = 0;

	//初期化
	if (initflag == 1)
	{
		init_mobc();
		initflag = 0;
	}

	//計測点座標
	t_x = (double)(map_center_width - path[cpath_ct][0]) * map_rate;
	t_y = (double)(map_center_height - path[cpath_ct][1]) * map_rate;

	//現在位置・計測点の距離
	det = sqrt((t_x - robot->real_x) * (t_x - robot->real_x) + (t_y - robot->real_y) * (t_y - robot->real_y));

	if (det < 250 || loop_detect == 1)	// 2 conditions to go to next target point
	{
		//計測完了
		*sflag = 1;
		cpath_ct++;
		loop_detect = 0; //Reset loop detection
		firstEntry = 1;
		//printf("Target %d dis %lf\n", cpath_ct, det); TMU20190819 非表示化
	}

	if (det < 1000)	// scanning threshold to activate angle detection 
	{
		entry = 1;
		robot_curr[0] = robot->real_x;
		robot_curr[1] = robot->real_y;
		dest[0] = t_x;
		dest[1] = t_y;

		if (firstEntry == 1)
		{
			dist_base = cal_length(dest, robot_curr);
			firstEntry = 0;
			start = clock();
		}
		else
		{
			dist_robot = cal_length(robot_curr, robot_prev);
			dist_top = cal_length(dest, robot_curr);
			angle = cal_angle(dist_base, dist_top, dist_robot);
			sumAngle += angle;
			dist_base = dist_top;
		}

		robot_prev[0] = robot->real_x;
		robot_prev[1] = robot->real_y;
		end = clock();
		if (sumAngle >= 360 || (double)(end - start) / (double)CLOCKS_PER_SEC > 60.0)
		{
			loop_detect = 1; // loop detected
			sumAngle = 0;
		}
		else
		{
			loop_detect = 0;
		}
	}

	//目標追従
	//--入力の計算
	det /= 1000.0;
	t_theta = 180.0 * atan2(t_y - robot->real_y, t_x - robot->real_x) / M_PI - 90.0;
	t_theta = transformation_angle(t_theta);
	t_theta = -t_theta + robot->rangle;
	t_theta = transformation_angle(t_theta);
	if (fabs(t_theta) > 90)
	{
		det *= -1.0;
	}

	fuzzyinf3(det, t_theta, outp);	//目標物追従に関する出力の計算

	//障害物回避
	//--入力の計算
	for (int i = 0; i < MEMF; i++)
	{
		min_dis[i] = MAX_LRF_DISTANCE;//1.0E5; TMU20190823
	}

	for (int i = 100; i <= 981; i++)	//LRF前方範囲(-110～+110[deg]) 
	{
		//150mm～1000mmの検知範囲
		/*if (lrfdata[i] < 150 || lrfdata[i] > MAX_LRF_DISTANCE)
		{
			continue;
		}*/
		if (lrfdata[i] > MAX_LRF_DISTANCE)
		{
			continue;
		}
		if (lrfdata[i] < 150) {
			lrfdata[i] = 150;
		}

		//範囲内を5分割してID計算
		int j = (i - 100) / seg;
		j = abs(j-4);

		//各エリアの最小距離
		if (min_dis[j] > lrfdata[i])
		{
			min_dis[j] = lrfdata[i];
		}
	}

	double mmd = MAX_LRF_DISTANCE;	//全エリアの最小距離
	for (int i = 0; i < MEMF; i++)
	{
		if (mmd > min_dis[i])
		{
			mmd = min_dis[i];
		}
	}
	//--出力の計算
	/*for (int i = 0; i < 5; i++)
	{
		printf("[%d]%f, ", i, min_dis[i]);
	}
	printf("\n");*/
	fuzzyinf(min_dis, outp2);	//障害物回避に関する出力値の計算

	//多目的行動調停
	for (int i = 0; i < CTRLNUM; i++)
	{
		toutp[0][i] = outp2[1 - i];
		toutp[1][i] = outp[1 - i];
	}
	//--環境知覚情報計算
	if (mmd < MIN_LRF_DISTANCE)
	{
		si[0] = 1.0;
	}
	else
	{
		si[0] = (MAX_LRF_DISTANCE - mmd) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
	}
	si[0] = exp(-(si[0] - 1.0) * (si[0] - 1.0) * 4.0);
	si[1] = 1.0 - si[0];

	//--重みの更新
	Update_wgt(si);
	cal_outp(foutp, toutp);	//多目的行動調停による最終出力値の計算
	foutp[0] *= mmd / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT; TMU20190823
	foutp[1] *= mmd / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT;

	if (pct == cpath_ct) {
		foutp[0] = 0;
		foutp[2] = 0;
		foutp[1] = 0;
		foutp[3] = 0;
		//cpath_ct = 0;
		printf("Finish\n");
		return 9;
	}

	return mode;
}

int decision_making2(struct robot_position* robot, double* foutp, long* lrf_dataD, float* lrf_dataA, int path[][2], int* sflag, int pct, int mode)
{
	static int initflag = 1;
	double det, t_theta;
	double t_x, t_y;
	double toutp[2][CTRLNUM];
	double outp[CTRLNUM];
	double min_dis[MEMF];	//各方向の最小距離格納用変数
							//0:左, 1:左前方, 2:中央, 3:右前方, 4:右
	double outp2[CNTL];
	const int seg = (881) / MEMF;
	double si[2];
	static double robot_prev[2], robot_curr[2], dest[2];
	static double dist_base, dist_robot, dist_top, angle, sumAngle;
	static int firstEntry = 1, loop_detect = 0;
	static time_t start, end;
	int entry = 0;

	//初期化
	if (initflag == 1)
	{
		init_mobc();
		initflag = 0;
	}

	//計測点座標
	t_x = (double)(map_center_width - path[cpath_ct][0]) * map_rate;
	t_y = (double)(map_center_height - path[cpath_ct][1]) * map_rate;

	//現在位置・計測点の距離
	det = sqrt((t_x - robot->real_x) * (t_x - robot->real_x) + (t_y - robot->real_y) * (t_y - robot->real_y));

	if (det < 250 || loop_detect == 1)	// 2 conditions to go to next target point
	{
		//計測完了
		*sflag = 1;
		cpath_ct++;
		loop_detect = 0; //Reset loop detection
		firstEntry = 1;
		//printf("Target %d dis %lf\n", cpath_ct, det); TMU20190819 非表示化
	}

	if (det < 1000)	// scanning threshold to activate angle detection 
	{
		entry = 1;
		robot_curr[0] = robot->real_x;
		robot_curr[1] = robot->real_y;
		dest[0] = t_x;
		dest[1] = t_y;

		if (firstEntry == 1)
		{
			dist_base = cal_length(dest, robot_curr);
			firstEntry = 0;
			start = clock();
		}
		else
		{
			dist_robot = cal_length(robot_curr, robot_prev);
			dist_top = cal_length(dest, robot_curr);
			angle = cal_angle(dist_base, dist_top, dist_robot);
			sumAngle += angle;
			dist_base = dist_top;
		}

		robot_prev[0] = robot->real_x;
		robot_prev[1] = robot->real_y;
		end = clock();
		if (sumAngle >= 360 || (double)(end - start) / (double)CLOCKS_PER_SEC > 60.0)
		{
			loop_detect = 1; // loop detected
			sumAngle = 0;
		}
		else
		{
			loop_detect = 0;
		}
	}

	//目標追従
	//--入力の計算
	det /= 1000.0;
	t_theta = 180.0 * atan2(t_y - robot->real_y, t_x - robot->real_x) / M_PI - 90.0;
	t_theta = transformation_angle(t_theta);
	t_theta = -t_theta + robot->rangle;
	t_theta = transformation_angle(t_theta);
	if (fabs(t_theta) > 90)
	{
		det *= -1.0;
	}

	fuzzyinf3(det, t_theta, outp);	//目標物追従に関する出力の計算

	//障害物回避
	//--入力の計算
	for (int i = 0; i < MEMF; i++)
	{
		min_dis[i] = MAX_LRF_DISTANCE;//1.0E5; TMU20190823
	}

	int j = 0;

	//RS-LiDAR用
	for (int i = 0; i < rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM; i++) {
		//150mm～1000mmの検知範囲
		//printf("%ld,%f\n", lrf_dataD[i], -lrf_dataA[i] * 180.0 / M_PI);
		if (lrf_dataD[i] < 150 || lrf_dataD[i] > MAX_LRF_DISTANCE) {
			continue;
		}
		//LRF前方範囲(-110～+110[deg])
		if (-lrf_dataA[i] >= 250.0 / 180.0 * M_PI && -lrf_dataA[i] < 294.0 / 180.0 * M_PI) {
			j = 4;
		}
		else if (-lrf_dataA[i] >= 294.0 / 180.0 * M_PI && -lrf_dataA[i] < 338.0 / 180.0 * M_PI) {
			j = 3;
		}
		else if (-lrf_dataA[i] >= 338.0 / 180.0 * M_PI && -lrf_dataA[i] < 360.0 / 180.0 * M_PI || -lrf_dataA[i] >= 0.0 / 180.0 * M_PI && -lrf_dataA[i] < 22.0 / 180.0 * M_PI) {
			j = 2;
		}
		else if (-lrf_dataA[i] >= 22.0 / 180.0 * M_PI && -lrf_dataA[i] < 66.0 / 180.0 * M_PI) {
			j = 1;
		}
		else if (-lrf_dataA[i] >= 66.0 / 180.0 * M_PI && -lrf_dataA[i] < 110.0 / 180.0 * M_PI) {
			j = 0;
		}
		else {
			continue;
		}

		//各エリアの最小距離
		if (min_dis[j] > lrf_dataD[i]) {
			min_dis[j] = lrf_dataD[i];
		}
	}

	double mmd = MAX_LRF_DISTANCE;	//全エリアの最小距離
	for (int i = 0; i < MEMF; i++)
	{
		if (mmd > min_dis[i])
		{
			mmd = min_dis[i];
		}
	}
	//--出力の計算
	fuzzyinf(min_dis, outp2);	//障害物回避に関する出力値の計算

	//多目的行動調停
	for (int i = 0; i < CTRLNUM; i++)
	{
		toutp[0][i] = outp2[i];
		toutp[1][i] = outp[1 - i];
	}
	//--環境知覚情報計算
	if (mmd < MIN_LRF_DISTANCE)
	{
		si[0] = 1.0;
	}
	else
	{
		si[0] = (MAX_LRF_DISTANCE - mmd) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
	}
	si[0] = exp(-(si[0] - 1.0) * (si[0] - 1.0) * 4.0);
	si[1] = 1.0 - si[0];

	//--重みの更新
	Update_wgt(si);
	cal_outp(foutp, toutp);	//多目的行動調停による最終出力値の計算
	foutp[0] *= mmd / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT; TMU20190823
	foutp[1] *= mmd / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT;

	if (pct == cpath_ct) {
		foutp[0] = 0;
		foutp[2] = 0;
		foutp[1] = 0;
		foutp[3] = 0;
		//cpath_ct = 0;
		printf("Finish\n");
		return 9;
	}

	return mode;
}
