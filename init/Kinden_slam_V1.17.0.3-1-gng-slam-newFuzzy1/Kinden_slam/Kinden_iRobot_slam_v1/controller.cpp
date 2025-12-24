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
#include <algorithm>
#include "Def.h"

// *** Fuzzy Define *** //
#define INPUT_NUM_TT        1               // 目標追従入力数 << MEMNUM1
#define RULE_NUM_TT         5               // 目標追従ルール数 << RULENUM1

#define INPUT_NUM_CA        13              // 障害物回避入力数 << MEMNUM
#define RULE_NUM_CA         13              // 障害物回避ルール数 << RULENUM

#define BEHVNUM             2               // 多目的行動調停 行動数
#define OUTPUT_NUM          2               // 多目的行動調停 出力数(左右輪) << CTRLNUM

#define NUMOFLRFGROUP      15				//グループ数

// *** GNG plannning 関係 *** //
int cpath_ct = 0;
int gng_path_idx = 0; //gngプランニングの中継点ID（次の測定点に行くときは0に戻す）
int gng_path[MAX_MEASURE_POINT][2];// 中継点配列

// *** 移動制御 関係 *** //
#define BEHVNUM 2 // Number of Behaviour
double    wgt[BEHVNUM];            // Weight
double  dw[BEHVNUM][BEHVNUM];    // Update Matrix

//===================================================================================
// *** private 関数 *** //
//目標追従ファジィ(Target Trace)
double membershipTT(int rule_num, double angle);    // << calmemf3()
void fuzzyTT(double angle, double output[2]);       // << fuzzyinf3()
//障害物回避ファジィ(Collision Avoidance)
double membershipCA(int rule_num, double* min_len); // << calmemf()
void fuzzyCA(double* min_len, double* output);      // << fuzzyinf()
//多目的行動調停
void init_mobc();
void Update_wgt(double si[BEHVNUM]);
void cal_outp(double outp[OUTPUT_NUM], double toutp[BEHVNUM][OUTPUT_NUM]);

//===================================================================================
//目標追従ファジィ(Target Trace)
//各ルールの入力ファジィ集合
const int A_TT[RULE_NUM_TT][INPUT_NUM_TT] =
{
	{ 0 },  // rule 1    Negative(0)  & Close(0)
	{ 1 },  // rule 2    NegativeS(1) & Close(0)
	{ 2 },  // rule 3    Zero(2)      & Close(0)
	{ 3 },  // rule 4    PositiveS(3) & Close(0)
	{ 4 }   // rule 5    Positive(4)  & Close(0)
};
//各ルールの出力シングルトン
const double w_TT[RULE_NUM_TT][OUTPUT_NUM] =
{
	{ -0.2,  0.4  },  // rule 1    N  & C
	{  0.7,  0.8  },  // rule 3    NS & C
	{  0.8,  0.8  },  // rule 5    Z  & C
	{  0.8,  0.7  },  // rule 7    PS & C
	{  0.4, -0.2  }   // rule 9    P  & C
};

//障害物回避ファジィ(Collision Avoidance)
//各ルールの入力ファジィ集合
const int A_CA[RULE_NUM_CA][INPUT_NUM_CA] =
{
	{ 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  // rule 1 左2
	{ 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  // rule 2 左2
	{ 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },  // rule 3 左2
	{ 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 },  // rule 4 左2
	{ 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 },  // rule 5 左2
	{ 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0 },  // rule 6 中心2
	{ 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0 },  // rule 7 中心2
	{ 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0 },  // rule 8 右2
	{ 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0 },  // rule 9 右2
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0 },  // rule 10 右2
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0 },  // rule 11 右2
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1 },  // rule 12 右2
	{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }   // rule 13 障害物なし
};

//各ルールの出力シングルトン
const double w_CA[RULE_NUM_CA][OUTPUT_NUM] =
{

	{  0.8,  0.4  },  // rule 1
	{  0.8,  0.2  },  // rule 2
	{  0.8,  0.0  },  // rule 3

	{  0.8, -0.4  },  // rule 4
	{  0.8, -0.4  },  // rule 5

	{  0.8, -0.8  },  // rule 6
	{ -0.8,  0.8  },  // rule 7

	{ -0.4,  0.8  },  // rule 8
	{ -0.4,  0.8  },  // rule 9

	{  0.0,  0.8  },  // rule 10
	{  0.2,  0.8  },  // rule 11
	{  0.4,  0.8  },   // rule 12

	{  0.8,  0.8  }  // rule 13
};//{右, 左}

//多目的行動調停
const double init_dw[2][2] =
{ 
	// CA   TT
	{ 0.8,  0.2 },  // si[0]
	{ 0.2,  0.8 }   // si[1]
};

//===================================================================================
//目標追従ファジィ(TT) 発火度計算
double membershipTT(int rule_num, double angle) {
	double fire = 1.0;  //発火度(角度適合度×距離適合度)
	int input_id;
	double radian;
	static double mem_center_max = M_PI / 3; //N, Pメンバシップ関数の中心位置[rad]
	static double mem_center_s = M_PI / 12;  //NS, PSメンバシップ関数の中心位置[rad]
	static double mem_width_max = 0.2;     //N, Pメンバシップ関数の幅
	static double mem_width_s = 0.05;      //NS, PSメンバシップ関数の幅

	// 角度に関する適合度
	input_id = 0;
	radian = M_PI * angle / 180.0;
	// Negative
	if (A_TT[rule_num][input_id] == 0) {
		if (radian < -mem_center_max) {
			fire *= 1.0;
		}
		else {
			fire *= exp(-pow(radian + mem_center_max, 2) / mem_width_max);
		}
		// Negative-S
	}
	else if (A_TT[rule_num][input_id] == 1) {
		fire *= exp(-pow(radian + mem_center_s, 2) / mem_width_s);
		// Zero
	}
	else if (A_TT[rule_num][input_id] == 2) {
		fire *= exp(-pow(radian, 2) / mem_width_s);
		// Positive-S
	}
	else if (A_TT[rule_num][input_id] == 3) {
		fire *= exp(-pow(radian - mem_center_s, 2) / mem_width_s);
		// Positive
	}
	else if (A_TT[rule_num][input_id] == 4) {
		if (radian > mem_width_max) {
			fire *= 1.0;
		}
		else {
			fire *= exp(-pow(radian - mem_center_max, 2) / mem_width_max);
		}
	}

	return fire;
}

//目標追従ファジィ(TT) 出力計算
void fuzzyTT(double angle, double output[2]) {
	//出力値初期化
	for (int i = 0; i < OUTPUT_NUM; i++) {
		output[i] = 0.0;
	}
	//ファジィ推論
	double fire[RULE_NUM_TT];  //各ルールの発火度
	double totalFire = 0.0;  //発火度総和
	for (int j = 0; j < RULE_NUM_TT; j++) {
		//ルールj(Angle, Distance)に対する発火度計算
		fire[j] = membershipTT(j, angle);
		//左右輪出力値計算
		for (int i = 0; i < OUTPUT_NUM; i++) {
			output[i] += fire[j] * w_TT[j][i];
		}
		totalFire += fire[j];
	}
	//出力値正規化
	for (int i = 0; i < OUTPUT_NUM; i++) {
		if (totalFire > 0) {
			output[i] = output[i] / totalFire;
		}
	}
}

//===================================================================================
/*ファジィ推論IF-THENルール（障害物回避）↓*/
//障害物回避ファジィ(CA) 発火度計算
double membershipCA(int rule_num, double* min_len) {
	double fire = 1.0;  //発火度(全適合度の掛け算)

	//各エリアに対する適合度
	for (int i = 0; i < INPUT_NUM_CA; i++) {
		//障害物なし
		if (A_CA[rule_num][i] == 0) {
			if (min_len[i] < MIN_LRF_DISTANCE) {
				fire *= 0.0;
			}
			else {
				fire *= (min_len[i] - MIN_LRF_DISTANCE) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
			}
		//障害物あり
		}
		else if (A_CA[rule_num][i] == 1) {
			if (min_len[i] < MIN_LRF_DISTANCE) {
				fire *= 1.0;
			}
			else {
				fire *= (MAX_LRF_DISTANCE - min_len[i]) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
			}
		}
	}
	return fire;
}

//障害物回避ファジィ(CA) 出力計算
void fuzzyCA(double* min_len, double* output) {
	double fire[RULE_NUM_CA];
	double totalFire;
	static double pre_output[2];

	for (int j = 0; j < OUTPUT_NUM; j++) {
		output[j] = 0;
	}

	totalFire = 0;

	//ルールiに対する発火度計算
	for (int i = 0; i < RULE_NUM_CA; i++) {
		fire[i] = membershipCA(i, min_len);
		totalFire += fire[i];
	}

	//左右輪出力値計算
	for (int j = 0; j < OUTPUT_NUM; j++) {
		for (int i = 0; i < RULE_NUM_CA; i++) {
			output[j] += fire[i] * w_CA[i][j];
		}
	}

	//左右輪出力値正規化
	for (int j = 0; j < OUTPUT_NUM; j++) {
		if (totalFire > 0) {
			output[j] /= totalFire;
			pre_output[j] = output[j];
		}
		else {
			output[j] = pre_output[j];
		}
	}
}

//=========================================================================
// Multi-objective behaviour control------------------------------
//多目的行動調停 重み初期化
void init_mobc() {
	for (int k = 0; k < BEHVNUM; k++) {
		for (int m = 0; m < BEHVNUM; m++) {
			dw[k][m] = init_dw[k][m];
		}
	}

	for (int k = 0; k < BEHVNUM; k++) {
		wgt[k] = 1 / 2.0;
	}

	return;
}

//多目的行動調停 重み更新
void Update_wgt(double si[BEHVNUM])
{
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
	//重み正規化
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

void cal_outp(double outp[OUTPUT_NUM], double toutp[BEHVNUM][OUTPUT_NUM])
{
	//int ib, ic;
	double Sw, Swb;

	Sw = 0.0;
	for (int ib = 0; ib < BEHVNUM; ib++)
	{
		Sw += wgt[ib];
	}

	for (int ic = 0; ic < OUTPUT_NUM; ic++)
	{
		Swb = 0.0;
		for (int ib = 0; ib < BEHVNUM; ib++)
		{
			Swb += wgt[ib] * toutp[ib][ic];
		}
		outp[ic] = Swb / Sw;
	}
	printf("CA:%.04f TT:%.04f\n", wgt[0] / Sw*100.0, wgt[1] / Sw*100.0); //TMU20190819 障害物回避(CA)と目標追従(TA)の割合
}

double cal_length(double* obj, double* robot) // calculate distance between two targets
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

// HOKUYO LiDAR 行動計画
// robot: robot_position[2]
// foutp: 左右輪出力 これが結果
// lrfdata: LiDAR data[1080]
// path[個数][x,y]
// sflag
// pct: 最後のパスID
// mode:

double* fuzzy_control(struct robot_position* robot, int path_size, long* lrfdata) {

	double foutp[OUTPUT_NUM];	 //左右輪出力（output）
	double det, t_theta;
	double t_x, t_y;
	double toutp[2][OUTPUT_NUM]; //2つの出力の保存
	double outp[OUTPUT_NUM];     //目標追従の出力
	double min_dis[NUMOFLRFGROUP];	     //各方向の最小距離格納用変数 0:左
	double outp2[OUTPUT_NUM];    //衝突回避の出力
	double outpWF[OUTPUT_NUM];   // 壁追従の出力
	const int seg = (900) / NUMOFLRFGROUP; //LRF分割(900=範囲(225[deg])) /=15 = 60
	double si[2];

	//中継点座標
	t_x = (double)(map_center_width - gng_path[gng_path_idx][0]) * map_rate;
	t_y = (double)(map_center_height - gng_path[gng_path_idx][1]) * map_rate;

	//現在位置・中継点の距離
	det = sqrt((t_x - robot->real_x) * (t_x - robot->real_x) + (t_y - robot->real_y) * (t_y - robot->real_y));
	//printf("* gng goal[%d](%d,%d) det: %f\n", gng_path_idx, gng_path[gng_path_idx][0], gng_path[gng_path_idx][1], det);
	if (det < 800)
	{
		// 到達
		printf("gng path ID: %d, size: %d\n", gng_path_idx, path_size);
		if (gng_path_idx < path_size - 1) {
			gng_path_idx++; //次の中継点へ
		}
		else {
			//printf("gng path END\n");
		}
		printf("* gng Target %d dis %lf\n", gng_path_idx, det); //TMU20190819 非表示化
	}

	// --- 目標追従 --- //
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
	//printf("t_theta: %f\n", t_theta);
	fuzzyTT(t_theta, outp);	//目標物追従に関する出力の計算

	// --- 障害物回避 --- //
	//--初期化
	for (int i = 0; i < NUMOFLRFGROUP; i++)
	{
		min_dis[i] = MAX_LRF_DISTANCE; //1.0E5; TMU20190823
	}
	//--入力の計算 LRF[mm] 1080(270[deg]) -> 900(225[deg])
	for (int i = 90; i <= 990; i++)
	{
		//150mm～1000mmの検知範囲
		if (lrfdata[i] < MIN_LRF_DISTANCE) {
			lrfdata[i] = MIN_LRF_DISTANCE;
		}
		else if (lrfdata[i] > MAX_LRF_DISTANCE)
		{
			lrfdata[i] = MAX_LRF_DISTANCE;
		}

		//範囲内を5分割してID計算->変更15分割
		int j = (i - 90) / seg;
		j = abs(j - (NUMOFLRFGROUP-1));

		//各エリアの最小距離
		if (min_dis[j] > lrfdata[i])
		{
			min_dis[j] = lrfdata[i];
		}
	}

	double mmd = MAX_LRF_DISTANCE;	//全エリアの最小距離
	double min_dis_average = 0;
	for (int i = 0; i < NUMOFLRFGROUP; i++)
	{
		if (mmd > min_dis[i])
		{
			mmd = min_dis[i];
		}
		/*min_dis_average += min_dis[i];*/
	}
	for (int i = 3; i < NUMOFLRFGROUP - 3; i++)
	{
		min_dis_average += min_dis[i];
	}
	min_dis_average /= (double)(NUMOFLRFGROUP-6);
	//障害物回避に関する出力値の計算
	double CA_min_dis[NUMOFLRFGROUP - 2]; //前方13 //障害物回避に使う分割数の調整: 20240226
	for (int i = 1; i < NUMOFLRFGROUP - 1; i++) {
		CA_min_dis[i - 1] = min_dis[i];
		printf("[%d] %f, ", i - 1, CA_min_dis[i - 1]);
	}
	printf("\n");
	fuzzyCA(CA_min_dis, outp2);

	// --- 多目的行動調停 --- //
	for (int i = 0; i < OUTPUT_NUM; i++)
	{
		toutp[0][i] = outp2[1 - i];
		toutp[1][i] = outp[i];
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
	for (int i = 0; i < NUMOFLRFGROUP; i++)
		printf("[%d] %f, ", i, min_dis[i]);
	printf("\n");
	//--目標方向の安全度評価
	double group_min[3]; // 目標方向のレーザーグループ
	int numofuse = 0;
	double dgvalue = 0; // 危険度 (危険->1)
	t_theta *= -1;
	if (abs(t_theta) > 225.0 * 0.5) {
		if (t_theta >= 0) {
			group_min[0] = min_dis[NUMOFLRFGROUP - 1];
			printf(" * group_min: [%d] %f << min_dis[14], t_theta = %f\n", numofuse, group_min[numofuse], t_theta);
		}
		else {
			group_min[0] = min_dis[0];
			printf(" * group_min: [%d] %f << min_dis[0], t_theta = %f\n", numofuse, group_min[numofuse], t_theta);
		}
		numofuse++;
	}
	else {
		for (int i = 0; i < 3; i++) {
			int index = (int)(abs(t_theta + 225.0 * 0.5) / (225.0 / NUMOFLRFGROUP)) + i - (int)(3.0 * 0.5);
			if (index < 0 || index > NUMOFLRFGROUP-3) continue;
			group_min[numofuse] = min_dis[index];
			printf(" * group_min: [%d] %f << min_dis[%d], t_theta = %f\n", numofuse, group_min[numofuse], index, t_theta);
			numofuse++;
		}
	}
	for (int i = 0; i < numofuse; i++) {
		//printf(" * group_min: [%d] %f\n", i, group_min[i]);
		if (group_min[i] < MIN_LRF_DISTANCE) dgvalue += 1.0;
		else if (group_min[i] == MAX_LRF_DISTANCE);
		else {
			dgvalue += (MAX_LRF_DISTANCE - group_min[i]) / (MAX_LRF_DISTANCE - MIN_LRF_DISTANCE);
		}
	}
	dgvalue /= numofuse; // 危険度 0~1
	double safeThread = 0.3; // 安全と判断できる閾値 大きいと早くTTを優先できる
	if (dgvalue < safeThread) { // 危険度＜安全と判断できる閾値
		si[0] -= 0.4; //OAを下げたい
		if (si[0] < 0) si[0] = 0.0;
		si[1] = 1 - si[0];
	}
	printf("Degree of danger: %f, si[0]: %f, si[1]: %f\n\n", dgvalue, si[0],si[1]);
	//--重みの更新
	Update_wgt(si);
	cal_outp(foutp, toutp);	//多目的行動調停による最終出力値の計算

	/*foutp[0] = outp2[1];
	foutp[1] = outp2[0];*/

	//mmd += 100.0;
	//if (mmd > MAX_LRF_DISTANCE) mmd = MAX_LRF_DISTANCE;
	//foutp[0] *= mmd / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT; TMU20190823
	//foutp[1] *= mmd / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT;
	printf("OA:(%f,%f), TT:(%f,%f)\n", toutp[0][0], toutp[0][1], toutp[1][0], toutp[1][1]);
	//foutp[0] *= 0.9 * maxSpeed;//MAX_OUTPUT; TMU20190823
	//foutp[1] *= 0.9 * maxSpeed;//MAX_OUTPUT;
	foutp[0] *= min_dis_average / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT; TMU20190823
	foutp[1] *= min_dis_average / MAX_LRF_DISTANCE * maxSpeed;//MAX_OUTPUT;

	return foutp;
}

int movement_controller(struct robot_position* robot, double* foutp, long* lrfdata, int path[][2], int* sflag, int pct, int mode, FastAstar* pp) {

	//初期化
	static int initflag = 1;
	static int firstEntry = 1, loop_detect = 0;
	static double robot_prev[2], robot_curr[2], dest[2];
	static double dist_base, dist_robot, dist_top, angle, sumAngle;
	static time_t start, end;
	int entry = 0;
	static int gng_path_size = 0;

	if (initflag == 1)
	{
		init_mobc();
		initflag = 0;
		//最初の測定点への経路探索
		pp->Intialization(robot->map_x, robot->map_y, path[cpath_ct][0], path[cpath_ct][1]);
		pp->AstarPathPlanningProcess();
		gng_path_size = 0;
		for (int i = 0; i < (int)pp->result_path.size() - 1; i++) {
			gng_path[i][0] = pp->nodes[pp->result_path[i]].pos_wide;//pixel
			gng_path[i][1] = pp->nodes[pp->result_path[i]].pos_hight;
			gng_path_size++;
		}
		gng_path[gng_path_size][0] = path[cpath_ct][0]; //最後に測定点位置を格納
		gng_path[gng_path_size][1] = path[cpath_ct][1];
		gng_path_size++;
		gng_path_idx = 1;

		printf("gng path: (size: %d)\n", gng_path_size);
		for (int i = 0; i < gng_path_size; i++) {
			printf(" [%d] (%d, %d)\n", i, gng_path[i][0], gng_path[i][1]);
		}
	}
	
	// 現在の測定点との距離
	// 計測点座標(cpath_ct<<グローバル変数)
	double t_x, t_y;
	t_x = (double)(map_center_width - path[cpath_ct][0]) * map_rate;
	t_y = (double)(map_center_height - path[cpath_ct][1]) * map_rate;
	//現在位置・計測点の距離
	double distance = sqrt((t_x - robot->real_x) * (t_x - robot->real_x) + (t_y - robot->real_y) * (t_y - robot->real_y));
	//printf("point[%d](%d,%d) distance: %f\n", cpath_ct, path[cpath_ct][0], path[cpath_ct][1], distance);
	// 到達したら次の測定点へ
	//if (distance < 250 || loop_detect == 1)	// 2 conditions to go to next target point
	if (distance < 250)	// 2 conditions to go to next target point
	{
		//計測完了
		*sflag = 1;
		cpath_ct++;
		loop_detect = 0; //Reset loop detection
		firstEntry = 1;
		printf("Main Target %d dis %lf\n", cpath_ct, distance); //TMU20190819 非表示化

		// gng planner
		if (pct != cpath_ct) {
			gng_path_idx = 1; //中継点添え字リセット
			pp->Intialization(robot->map_x, robot->map_y, path[cpath_ct][0], path[cpath_ct][1]);
			pp->AstarPathPlanningProcess();
			gng_path_size = 0;
			for (int i = 0; i < (int)pp->result_path.size() - 1; i++) {
				gng_path[i][0] = pp->nodes[pp->result_path[i]].pos_wide;//pixel
				gng_path[i][1] = pp->nodes[pp->result_path[i]].pos_hight;
				gng_path_size++;
			}
			gng_path[gng_path_size][0] = path[cpath_ct][0]; //最後に測定点位置を格納
			gng_path[gng_path_size][1] = path[cpath_ct][1];
			gng_path_size++;

		}
	}
	// 測定点に接近した時の処理
	if (distance < 1000)	// scanning threshold to activate angle detection 
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

	// 全ての測定点に到達した終了
	if (pct == cpath_ct) {
		foutp[0] = 0;
		foutp[2] = 0;
		foutp[1] = 0;
		foutp[3] = 0;
		//cpath_ct = 0;
		printf("Finish\n");
		pp->draw_flag = false;
		return 9;
	}
	//
	pp->current_target_index = gng_path_idx;

	double v_rate = 1.0;
	if (distance < 1000)
	{
		v_rate = 0.3 + 0.7 * ((distance - 250.0) / 750.0);
	}

	// GNG Plannerによる中継点への移動
	//fuzzy_control(ロボットの位置、中継点配列、配列サイズ、スキャン情報) output:double* foutp
	double* wheel_output = fuzzy_control(robot, gng_path_size, lrfdata);
	foutp[0] = v_rate * wheel_output[0];
	foutp[1] = v_rate * wheel_output[1];
	//printf("mv output: %f,%f\n", foutp[0], foutp[1]);
	return mode;
}

// --- 以前の関数 --- //

int decision_making(struct robot_position* robot, double* foutp, long* lrfdata, int path[][2], int* sflag, int pct, int mode)
{
	static int initflag = 1;
	double det, t_theta;
	double t_x, t_y;
	double toutp[2][CTRLNUM];
	double outp[CTRLNUM];
	//double min_dis[MEMF];	//各方向の最小距離格納用変数
							//0:左, 1:左前方, 2:中央, 3:右前方, 4:右
	double min_dis[15];    //20230818変更　各方向の最小距離格納用変数 (左から0~14, 225deg)
	double outp2[CNTL];
	//const int seg = (881) / MEMF;
	const int seg = (900) / 15; //20230818変更
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

	//fuzzyinf3(det, t_theta, outp);	//目標物追従に関する出力の計算

	//障害物回避
	//--入力の計算 //20230818変更
	for (int i = 0; i < 15; i++)
	{
		min_dis[i] = MAX_LRF_DISTANCE;//1.0E5; TMU20190823
	}

	for (int i = 90; i <= 990; i++)	//LRF前方範囲(-110～+110[deg])
	{
		//150mm～1000mmの検知範囲
		if (lrfdata[i] < 150 || lrfdata[i] > MAX_LRF_DISTANCE)
		{
			continue;
		}

		//範囲内を15分割してID計算
		int j = (i - 90) / seg;
		j = abs(j - 14);

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

	//障害物回避に関する出力値の計算 //20230818変更
	double OA_min_dis[15 - 2]; //前方13 障害物回避用最小距離配列
	for (int i = 1; i < 15 - 1; i++) {
		OA_min_dis[i - 1] = min_dis[i];
		printf("[%d]%f,", i - 1, OA_min_dis[i - 1]);
	}
	//--出力の計算
	printf("\n");
	//fuzzyinf(OA_min_dis, outp2);	//障害物回避に関する出力値の計算
	//printf("OA outp2: {%f, %f}\n", outp2[0], outp2[1]);
	//多目的行動調停 //{right, left}
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
	//foutp {right, left}
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

	//fuzzyinf3(det, t_theta, outp);	//目標物追従に関する出力の計算

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
	//fuzzyinf(min_dis, outp2);	//障害物回避に関する出力値の計算

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
