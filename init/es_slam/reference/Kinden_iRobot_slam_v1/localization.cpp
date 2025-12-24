/*
*  localization.c
*  slam
*
*  Created by Naoyuki Kubota on 12/04/12.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#include "localization.h"

//double xL[SCAN_MARKS], yL[SCAN_MARKS];
//double thetaL[SCAN_MARKS];
double xL[SCAN_MARKS_RS16], yL[SCAN_MARKS_RS16];
double thetaL[SCAN_MARKS_RS16];

//double area_length = AREA_LENGTH; //TMU20190820 探索領域(単位：mm)を示す変数
//henkoIP------------
//int *lrf = NULL;//henkoIPz
int lrf[10] = { 0 };//henkoIPz
int devide = 10;//henkoIPz
void readLRF()
{
	char lrf_area[255];
	sprintf(lrf_area, "C:\\kinden\\cmd\\lrf_area.txt");

	FILE *mfp4 = fopen(lrf_area, "r");
	if (mfp4 == NULL)
	{
		printf("LRF AREA1 file open error1!\n");
		return;
	}
	int l;
	int ct = 0;
	int i = 0;
	while (fscanf(mfp4, "%d\n", &l) != EOF)
	{
		//if (ct == 0)
		//{
		//	devide = 10;//l;//henkoIPz
		//	//lrf = (int*)malloc(sizeof(int) * l);
		//}
		//else
		//{
			lrf[i] = l;
			i++;
		//}
		ct++;
	}
	FILE *mfp5 = fopen("C:\\kinden\\log\\LRFdevide.txt", "a");//ファイルへの保存
	printf("devide = %d\n", devide);
	fprintf(mfp5, "devide = %d\n", devide);
	for (i = 0;i < devide;i++)
	{
		printf("%d,", lrf[i]);
		fprintf(mfp5, "%d,", lrf[i]);
	}
	fprintf(mfp5, "\n");
	fclose(mfp5);
	printf("\n");
}
//*/
//henkoIP------------

//評価関数の計算
double fitcal_p(int fx, int fy, double fr, struct multi_map *multi_Map)
{
	int num = 0;
	int pt1_x, pt1_y;
	int hit2 = 0, err2 = 0;
	double h;
	double x_1, y_1, x_3, y_3;
	double p = 0.0;
	double x_c2, y_c2;
	double s1, c1;

	x_c2 = (double)(map_center_width - fx) * map_rate;
	y_c2 = (double)(map_center_height - fy) * map_rate;
	s1 = sin((fr) * M_PI / 180.0);
	c1 = cos((fr) * M_PI / 180.0);

	//RS-LiDAR用
	int numX;
	if (SENSOR_MODE == 0) {
		numX = SCAN_MARKS;
	}
	else {
		numX = rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM;
	}

	for (int k = 0; k < numX; k++)
	{
		x_1 = xL[k], y_1 = yL[k];
		if (x_1 != ERR && y_1 != ERR) 
		{
			x_3 = (double)x_1 * c1 - (double)y_1 * s1 + x_c2;
			y_3 = (double)x_1 * s1 + (double)y_1 * c1 + y_c2;
			pt1_x = (int)((map_center_width - (int)x_3 / map_rate) / multi_Map->rate);
			pt1_y = (int)((map_center_height - (int)y_3 / map_rate) / multi_Map->rate);
			if (pt1_x < 0 || pt1_x >= MAPSIZE_WIDTH / multi_Map->rate || pt1_y < 0 || pt1_y >= MAPSIZE_HIGHT / multi_Map->rate)
			{
				err2++;
				num++;
				continue;
			}

			h = -multi_Map->anorm[pt1_x][pt1_y] - multi_Map->norm[pt1_x][pt1_y];

			if (h != 0)
			{
				hit2++;
			}
			else
			{
				err2++;
			}
			num++;
		}
	}

	p = ((double)hit2) / ((double)hit2 + (double)err2);
	return p;

}

int p_max(struct es_gene *Es_gene, int parn)
{
	int max_count;
	double max;
	for (int i = 0; i < parn; i++) 
	{

		if (i == 0) 
		{
			max = Es_gene->es_fit[i];
			max_count = i;
		}
		else if (max < Es_gene->es_fit[i]) 
		{
			max = Es_gene->es_fit[i];
			max_count = i;
		}
	}

	return max_count;
}

int p_min(struct es_gene *Es_gene, int parn)
{
	int min_count;
	double min;
	for (int i = 0; i < parn; i++) 
	{
		if (i == 0) 
		{
			min = Es_gene->es_fit[i];
			min_count = i;
		}
		else if (min > Es_gene->es_fit[i]) 
		{
			min = Es_gene->es_fit[i];
			min_count = i;
		}
	}

	return min_count;
}

void initial_gene(double *npar, double init_pos[])
{
	/*
	if (rnd() < 0.5)
	{
		npar[0] = (int)(init_pos[0] + (double)60 * rnd());
	}
	else
	{
		npar[0] = (int)(init_pos[0] - (double)60 * rnd());
	}

	if (rnd() < 0.5)
	{
		npar[1] = (int)(init_pos[1] + (double)60 * rnd());
	}
	else
	{
		npar[1] = (int)(init_pos[1] - (double)60 * rnd());
	}
	npar[2] = 360.0 * rnd();
	*/

	//TMU20190820 初期位置推定の探索領域の指定をピクセルからメートルに変更
	if (rnd() < 0.5)
	{
		npar[0] = (int)(init_pos[0] + (double)(area_length / (2 * map_rate)) * rnd());
	}
	else
	{
		npar[0] = (int)(init_pos[0] - (double)(area_length / (2 * map_rate)) * rnd());
	}

	if (rnd() < 0.5)
	{
		npar[1] = (int)(init_pos[1] + (double)(area_length / (2 * map_rate)) * rnd());
	}
	else
	{
		npar[1] = (int)(init_pos[1] - (double)(area_length / (2 * map_rate)) * rnd());
	}

	if (start_deg == 1)
	{
		npar[2] = init_pos[2] + 90.0 * (rnd() - 0.5); //指定角度から+-45度の範囲を探索
		if (npar[2] < 0)
		{
			npar[2] += 360.0;
		}
	}
	else
	{
		npar[2] = 360.0 * rnd();
	}
}

void genetic_operator(double *npar, double *ppar, int **omap, double init_pos[])
{
	npar[0] = ppar[0] + (int)(30.0 * (rndn() + rndn()));
	npar[1] = ppar[1] + (int)(30.0 * (rndn() + rndn()));

	if (DEF_AL == 0)
	{
		if (npar[0] < init_pos[0] - 120 || npar[0] > init_pos[0] + 120)
		{
			if (rnd() < 0.5)
			{
				npar[0] = (int)(init_pos[0] + (double)60 * rnd());
			}
			else
			{
				npar[0] = (int)(init_pos[0] - (double)60 * rnd());
			}
		}

		if (npar[1] < init_pos[1] - 120 || npar[1] > init_pos[1] + 120)
		{
			if (rnd() < 0.5)
			{
				npar[1] = (int)(init_pos[1] + (double)60 * rnd());
			}
			else
			{
				npar[1] = (int)(init_pos[1] - (double)60 * rnd());
			}
		}

		npar[2] = ppar[2] + rndn() + rndn();
	}
	else
	{
		//TMU20190820 初期位置探索領域をmmで指定
		if (npar[0] < init_pos[0] - area_length / map_rate || npar[0] > init_pos[0] + area_length / map_rate)
		{
			if (rnd() < 0.5)
			{
				npar[0] = (int)(init_pos[0] + (double)(area_length / (2 * map_rate)) * rnd());
			}
			else
			{
				npar[0] = (int)(init_pos[0] - (double)(area_length / (2 * map_rate)) * rnd());
			}
		}

		if (npar[1] < init_pos[1] - area_length / map_rate || npar[1] > init_pos[1] + area_length / map_rate)
		{
			if (rnd() < 0.5)
			{
				npar[1] = (int)(init_pos[1] + (double)(area_length / (2 * map_rate)) * rnd());
			}
			else
			{
				npar[1] = (int)(init_pos[1] - (double)(area_length / (2 * map_rate)) * rnd());
			}
		}

		npar[2] = ppar[2] + rndn() + rndn();
	}
}

//#pragma mark Evorutionary Starategy for Initial Localization. Created by Yuichiro Toda on 12/04/22.
double evolutionary_strategy(struct es_gene *Es_gene, struct robot_position *robot, long *lrf_data, 
	                         int **omap, struct multi_map *multi_Map, IplImage *mImage, int *ini_flag) 
{
	double weight = 0.0;
	int flag1 = 0;
	int num = 0;
	int k = 0;
	int n_lrf;//henkoIP2
	//int i, j, k;

	int m_num;
	double min;
	double fit1;
	double npar[EGAN][3];
	if (Es_gene->lo_count == 0)
	{
		for (int i = 0; i < SCAN_MARKS; i++)
		{
			thetaL[i] = (((double)i - 540.0) * (360.0 / 1440.0) * M_PI) / 180.0;
		}
	}


	FILE* mfp5 = fopen("C:\\kinden\\log\\LRFarea.log", "w");	

	for (int i = 0; i < SCAN_MARKS; i++)
	{
		if (lrf_data[i] < 100 || lrf_data[i] > 40000) 
		{
			xL[i] = ERR;
			yL[i] = ERR;

			fprintf(mfp5,"%04d, %d\n", i, 0);

		}
		else 
		{
			//henkoIPz---------------------------
			
			double n = (double)(((double)(i*devide)) / SCAN_MARKS);
			n_lrf = (int)n;
			if (lrf[n_lrf] == 0) {//chui
				xL[i] = ERR;
				yL[i] = ERR;

				fprintf(mfp5, "%04d, %d\n", i, 0);
			}
			else {
				xL[i] = lrf_data[i] * sin(thetaL[i]);
				yL[i] = lrf_data[i] * cos(thetaL[i]);

				fprintf(mfp5, "%04d, %d\n", i, 1);
			}
			//henkoIPz---------------------------
		}
	}

	fclose(mfp5);

	for (int i = 0; i < EGAN; i++)
	{
		if (Es_gene->lo_count == 0 || Es_gene->es_fit[Es_gene->best_gene] < 0.3 || *ini_flag == 1) 
		{
			if (i == 0) 
			{
				Es_gene->gene[i][0] = Es_gene->init_pos[0];
				Es_gene->gene[i][1] = Es_gene->init_pos[1];
//#if defined(_DEG_ON)
				//Es_gene->gene[i][2] = Es_gene->init_pos[2]; // TMU20190821 向きの追加
//#else
				Es_gene->gene[i][2] = Es_gene->init_pos[2];//henkoIPz
//#endif
				Es_gene->es_fit[i] = fitcal_p((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1], Es_gene->gene[i][2], multi_Map);
				continue;
			}
			else
			{
				initial_gene(Es_gene->gene[i], Es_gene->init_pos);
			}
			Es_gene->es_fit[i] = fitcal_p((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1], Es_gene->gene[i][2], multi_Map);
			weight += Es_gene->es_fit[i];
			if (mImage != NULL)
			{
				cvCircle(mImage, cvPoint((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1]), 2, CV_RGB(255, 0, 255), -1, 8, 0);
			}
			flag1 = 0;
		}
		else 
		{
			if ((num = (int)(round(Es_gene->es_wei[i] * EGAN))) > 0) 
			{
				for (int j = 0; j < num; j++) 
				{
					genetic_operator(npar[k], Es_gene->gene[i], omap, Es_gene->init_pos);
					k++;
					if (k >= EGAN)
					{
						break;
					}
				}

				if (k >= EGAN)
				{
					break;
				}
			}
			flag1 = 1;
		}
	}

	if (flag1 == 1) 
	{
		while (k < EGAN) 
		{
			initial_gene(npar[k], Es_gene->init_pos);
			k++;
		}

		for (int i = 0; i < EGAN; i++) 
		{
			for (int j = 0; j < 3; j++)
			{
				Es_gene->es_fit[i] = fitcal_p((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1], Es_gene->gene[i][2], multi_Map);
			}
			
			if (mImage != NULL)
			{
				cvCircle(mImage, cvPoint((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1]), 2, CV_RGB(255, 0, 255), -1, 8, 0);
			}
		}

		m_num = p_min(Es_gene, EGAN);
		min = Es_gene->es_fit[m_num];
		for (int i = 0; i < EGAN; i++) 
		{
			fit1 = fitcal_p((int)npar[i][0], (int)npar[i][1], npar[i][2], multi_Map);
			if (fit1 > min) 
			{
				for (int j = 0; j < 3; j++)
				{
					Es_gene->gene[m_num][j] = npar[i][j];
				}
				Es_gene->es_fit[m_num] = fit1;
				m_num = p_min(Es_gene, EGAN);
				min = Es_gene->es_fit[m_num];
			}

			if (mImage != NULL)
			{
				cvCircle(mImage, cvPoint((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1]), 2, CV_RGB(255, 0, 255), -1, 8, 0);
			}
		}

		for (int i = 0; i < EGAN; i++)
		{
			weight += Es_gene->es_fit[i];
		}
	}

	Es_gene->best_gene = p_max(Es_gene, EGAN);
	Es_gene->wei_ave = (double)weight / (double)EGAN;

	if (weight != 0)
	{
		for (int i = 0; i < EGAN; i++)
		{
			Es_gene->es_wei[i] = Es_gene->es_fit[i] / weight;
		}
	}

	robot->map_x = (int)Es_gene->gene[Es_gene->best_gene][0];
	robot->map_y = (int)Es_gene->gene[Es_gene->best_gene][1];
	robot->real_x = (double)(map_center_width - robot->map_x) * map_rate;
	robot->real_y = (double)(map_center_height - robot->map_y) * map_rate;
	robot->rangle = Es_gene->gene[Es_gene->best_gene][2];
	robot->dangle = Es_gene->gene[Es_gene->best_gene][2];

	if (*ini_flag == 1)
	{
		*ini_flag = 0;
	}

	return Es_gene->es_fit[Es_gene->best_gene];

}

//#pragma mark -
//#pragma mark Algorithm Initial Localization Method. Created by Yuichiro Toda on 12/04/22.
//大域的自己位置推定用関数
struct multi_map *initial_Localization(struct es_gene *gene, struct robot_position *robot, long *lrf_data, int **omap, 
	                                   struct multi_map *multi_Map, IplImage *mImage, int *end_flag)
{
	static int es_count;
	static int ini_flag;

	if (gene->es_fit[gene->best_gene] > 0.4 && es_count > 10 && gene->lo_count > 35) 
	{
		if (multi_Map->parent == NULL) 
		{
			*end_flag = 1;
			robot->rangle = (int)robot->rangle % 360;
			transformation_angle(robot->rangle);
		}
		else 
		{
			multi_Map = multi_Map->parent;
		}
		es_count = 0;

	}
	else if (es_count == 100) 
	{
		if (multi_Map->child != NULL) 
		{
			multi_Map = multi_Map->child;
			if (multi_Map->child != NULL)
			{
				multi_Map = multi_Map->child;
			}
		}
		ini_flag = 1;
		es_count = 0;
	}

	//	printf("Best gene[%d]:%lf\n", gene->best_gene,gene->es_fit[gene->best_gene]);

	if (*end_flag != 1)
	{
		gene->es_fit[gene->best_gene] = evolutionary_strategy(gene, robot, lrf_data, omap, multi_Map, mImage, &ini_flag);
	}
	//henkoIP2------------
	FILE *mfp6 = fopen("C:\\kinden\\log\\position.txt", "a");//ファイルへの保存
	fprintf(mfp6, "%d\t%d\t%f\t%f\t%f\t%f", gene->lo_count,multi_Map->reso, gene->es_fit[gene->best_gene], gene->gene[gene->best_gene][0], gene->gene[gene->best_gene][1], gene->gene[gene->best_gene][2]);
	fprintf(mfp6, "\n");
	fclose(mfp6);
	//henkoIP2------------
	es_count++;
	gene->lo_count++;

	return multi_Map;
}

double evolutionary_strategy2(struct es_gene* Es_gene, struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, int** omap, struct multi_map* multi_Map, IplImage* mImage, int* ini_flag)
{
	double weight = 0.0;
	int flag1 = 0;
	int num = 0;
	int k = 0;
	int n_lrf;//henkoIP2
	//int i, j, k;

	int m_num;
	double min;
	double fit1;
	double npar[EGAN][3];
	if (Es_gene->lo_count == 0)
	{
		for (int i = 0; i < SCAN_MARKS; i++)
		{
			thetaL[i] = (((double)i - 540.0) * (360.0 / 1440.0) * M_PI) / 180.0;
		}
	}


	FILE* mfp5 = fopen("C:\\kinden\\log\\LRFarea.log", "w");

	//RS-LiDAR用
	for (int i = 0; i < rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM; i++)
	{
		if (lrf_dataD[i] < LASER_RANGE_MIN || lrf_dataD[i] > LASER_RANGE_MAX)
		{
			xL[i] = ERR;
			yL[i] = ERR;

			fprintf(mfp5, "%04d, %d\n", i, 0);

		}
		else
		{
			//henkoIPz---------------------------
			double n = (double)(((double)(i * devide)) / (rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM));
			n_lrf = (int)n;
			if (lrf[n_lrf] == 0) {//chui
				xL[i] = ERR;
				yL[i] = ERR;

				fprintf(mfp5, "%04d, %d\n", i, 0);
			}
			else {
				xL[i] = (double)lrf_dataD[i] * sin(lrf_dataA[i]);
				yL[i] = (double)lrf_dataD[i] * cos(lrf_dataA[i]);

				fprintf(mfp5, "%04d, %d\n", i, 1);
			}
			//henkoIPz---------------------------
		}
	}

	fclose(mfp5);

	for (int i = 0; i < EGAN; i++)
	{
		if (Es_gene->lo_count == 0 || Es_gene->es_fit[Es_gene->best_gene] < 0.3 || *ini_flag == 1)
		{
			if (i == 0)
			{
				Es_gene->gene[i][0] = Es_gene->init_pos[0];
				Es_gene->gene[i][1] = Es_gene->init_pos[1];
				//#if defined(_DEG_ON)
								//Es_gene->gene[i][2] = Es_gene->init_pos[2]; // TMU20190821 向きの追加
				//#else
				Es_gene->gene[i][2] = Es_gene->init_pos[2];//henkoIPz
//#endif
				Es_gene->es_fit[i] = fitcal_p((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1], Es_gene->gene[i][2], multi_Map);
				continue;
			}
			else
			{
				initial_gene(Es_gene->gene[i], Es_gene->init_pos);
			}
			Es_gene->es_fit[i] = fitcal_p((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1], Es_gene->gene[i][2], multi_Map);
			weight += Es_gene->es_fit[i];
			if (mImage != NULL)
			{
				cvCircle(mImage, cvPoint((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1]), 2, CV_RGB(255, 0, 255), -1, 8, 0);
			}
			flag1 = 0;
		}
		else
		{
			if ((num = (int)(round(Es_gene->es_wei[i] * EGAN))) > 0)
			{
				for (int j = 0; j < num; j++)
				{
					genetic_operator(npar[k], Es_gene->gene[i], omap, Es_gene->init_pos);
					k++;
					if (k >= EGAN)
					{
						break;
					}
				}

				if (k >= EGAN)
				{
					break;
				}
			}
			flag1 = 1;
		}
	}

	if (flag1 == 1)
	{
		while (k < EGAN)
		{
			initial_gene(npar[k], Es_gene->init_pos);
			k++;
		}

		for (int i = 0; i < EGAN; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Es_gene->es_fit[i] = fitcal_p((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1], Es_gene->gene[i][2], multi_Map);
			}

			if (mImage != NULL)
			{
				cvCircle(mImage, cvPoint((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1]), 2, CV_RGB(255, 0, 255), -1, 8, 0);
			}
		}

		m_num = p_min(Es_gene, EGAN);
		min = Es_gene->es_fit[m_num];
		for (int i = 0; i < EGAN; i++)
		{
			fit1 = fitcal_p((int)npar[i][0], (int)npar[i][1], npar[i][2], multi_Map);
			if (fit1 > min)
			{
				for (int j = 0; j < 3; j++)
				{
					Es_gene->gene[m_num][j] = npar[i][j];
				}
				Es_gene->es_fit[m_num] = fit1;
				m_num = p_min(Es_gene, EGAN);
				min = Es_gene->es_fit[m_num];
			}

			if (mImage != NULL)
			{
				cvCircle(mImage, cvPoint((int)Es_gene->gene[i][0], (int)Es_gene->gene[i][1]), 2, CV_RGB(255, 0, 255), -1, 8, 0);
			}
		}

		for (int i = 0; i < EGAN; i++)
		{
			weight += Es_gene->es_fit[i];
		}
	}

	Es_gene->best_gene = p_max(Es_gene, EGAN);
	Es_gene->wei_ave = (double)weight / (double)EGAN;

	if (weight != 0)
	{
		for (int i = 0; i < EGAN; i++)
		{
			Es_gene->es_wei[i] = Es_gene->es_fit[i] / weight;
		}
	}

	robot->map_x = (int)Es_gene->gene[Es_gene->best_gene][0];
	robot->map_y = (int)Es_gene->gene[Es_gene->best_gene][1];
	robot->real_x = (double)(map_center_width - robot->map_x) * map_rate;
	robot->real_y = (double)(map_center_height - robot->map_y) * map_rate;
	robot->rangle = Es_gene->gene[Es_gene->best_gene][2];
	robot->dangle = Es_gene->gene[Es_gene->best_gene][2];

	if (*ini_flag == 1)
	{
		*ini_flag = 0;
	}

	return Es_gene->es_fit[Es_gene->best_gene];

}

//大域的自己位置推定用関数
struct multi_map* initial_Localization2(struct es_gene* gene, struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, int** omap,	struct multi_map* multi_Map, IplImage* mImage, int* end_flag)
{
	static int es_count;
	static int ini_flag;

	if (gene->es_fit[gene->best_gene] > 0.4 && es_count > 10 && gene->lo_count > 35)
	{
		if (multi_Map->parent == NULL)
		{
			*end_flag = 1;
			robot->rangle = (int)robot->rangle % 360;
			transformation_angle(robot->rangle);
		}
		else
		{
			multi_Map = multi_Map->parent;
		}
		es_count = 0;

	}
	else if (es_count == 100)
	{
		if (multi_Map->child != NULL)
		{
			multi_Map = multi_Map->child;
			if (multi_Map->child != NULL)
			{
				multi_Map = multi_Map->child;
			}
		}
		ini_flag = 1;
		es_count = 0;
	}

	//	printf("Best gene[%d]:%lf\n", gene->best_gene,gene->es_fit[gene->best_gene]);

	if (*end_flag != 1)
	{
		//RS-LiDAR用
		gene->es_fit[gene->best_gene] = evolutionary_strategy2(gene, robot, lrf_dataD, lrf_dataA, omap, multi_Map, mImage, &ini_flag);
	}
	//henkoIP2------------
	FILE* mfp6 = fopen("C:\\kinden\\log\\position.txt", "a");//ファイルへの保存
	fprintf(mfp6, "%d\t%d\t%f\t%f\t%f\t%f", gene->lo_count, multi_Map->reso, gene->es_fit[gene->best_gene], gene->gene[gene->best_gene][0], gene->gene[gene->best_gene][1], gene->gene[gene->best_gene][2]);
	fprintf(mfp6, "\n");
	fclose(mfp6);
	//henkoIP2------------
	es_count++;
	gene->lo_count++;

	return multi_Map;
}

int sg_max(struct robot_position *robot, double *best_fitness)
{
	int max_count;
	double max;
	for (int i = 0; i < GANS; i++) 
	{
		if (i == 0) 
		{
			max = robot->fit_s[i];
			max_count = i;
		}
		else if (max < robot->fit_s[i]) 
		{
			max = robot->fit_s[i];
			max_count = i;
		}
	}
	*best_fitness = max;
	return max_count;
}

int sg_min(struct robot_position *robot, double *worst_fitness)
{
	int min_count;
	double min;
	for (int i = 0; i < GANS; i++) 
	{
		if (i == 0) 
		{
			min = robot->fit_s[i];
			min_count = i;
		}
		else if (min > robot->fit_s[i]) 
		{
			min = robot->fit_s[i];
			min_count = i;
		}
	}
	*worst_fitness = min;
	return min_count;
}

//適合度関数の計算（スケール調整）
double fitcal_s_multi(int i, struct robot_position *robot, struct multi_map *multi_Map)
{
	int num = 0;
	int pt1_x, pt1_y;
	int hit2 = 0, err2 = 0;
	int pt2_x, pt2_y;
	double h;
	double x_1, y_1, x_3, y_3;
	double p = 0.0;
	double x_c2, y_c2;
	double s1, c1;
	double max = 0.0;

	s1 = sin((robot->dangle) * M_PI / 180.0);
	c1 = cos((robot->dangle) * M_PI / 180.0);
	x_c2 = (double)(map_center_width - (robot->map_x)) * map_rate;
	y_c2 = (double)(map_center_height - (robot->map_y)) * map_rate;

	pt2_x = (int)(map_center_width - (int)x_c2 / map_rate);
	pt2_y = (int)(map_center_height - (int)y_c2 / map_rate);

	if (i >= 0) 
	{
		for (int l = -2; l <= 2; l++) 
		{
			for (int n = -2; n <= 2; n++) 
			{
				hit2 = 0;
				err2 = 0;
				for (int k = 0; k < SCAN_MARKS; k += 1)
				{
					x_1 = xL[k], y_1 = yL[k];
					if (x_1 != ERR && y_1 != ERR) 
					{

						x_1 *= robot->gen_s[i][0];
						y_1 *= robot->gen_s[i][0];
						x_3 = (double)x_1 * c1 - (double)y_1 * s1 + x_c2;
						y_3 = (double)x_1 * s1 + (double)y_1 * c1 + y_c2;
						pt1_x = (int)((map_center_width - (int)x_3 / map_rate) / multi_Map->rate + l);
						pt1_y = (int)((map_center_height - (int)y_3 / map_rate) / multi_Map->rate + n);

						if (pt1_x < 0 || pt1_x >= MAPSIZE_WIDTH / multi_Map->rate || pt1_y < 0 || pt1_y > MAPSIZE_HIGHT / multi_Map->rate)
						{
							err2++;
							num++;
							continue;
						}

						h = -multi_Map->anorm[pt1_x][pt1_y] - multi_Map->norm[pt1_x][pt1_y];

						if (h != 0)
						{
							hit2++;
						}
						else
						{
							err2++;
						}

						num++;
						pt1_x *= (int)multi_Map->rate;
						pt1_y *= (int)multi_Map->rate;

					}
				}

				p = ((double)hit2) / ((double)hit2 + (double)err2);
				if (p > max) max = p;
			}
		}
	}

	return max;
}

void sga_init_multi(struct robot_position *robot, struct multi_map *multi_Map)
{
	for (int i = 0; i < GANS; i++)
	{
		if (i == 0) 
		{
			robot->gen_s[i][0] = 1.0;
		}
		else 
		{
			if (rnd() > 0.5)
			{
				robot->gen_s[i][0] = 1.0 + 0.5 * rnd();
			}
			else
			{
				robot->gen_s[i][0] = 1.0 - 0.5 * rnd();
			}
		}

		robot->fit_s[i] = fitcal_s_multi(i, robot, multi_Map);
	}
}

//スケール調整用関数
void addjust_scale_multi(struct robot_position *robot, long *lrf_data, struct multi_map *multi_Map)
{
	double t = 0;
	double best_fitness, worst_fitness;
	int worst_gene;
	int g, g2;
	int n_lrf;//henkoIP2

	for (int i = 0; i < SCAN_MARKS; i++)
	{
		if (lrf_data[i] < 100 || lrf_data[i] > 40000) 
		{
			xL[i] = ERR;
			yL[i] = ERR;
		}
		else 
		{
			//henkoIP2---------------------------
			n_lrf = (int)(i * devide / SCAN_MARKS);
			if (lrf[n_lrf] == 0) {
				xL[i] = ERR;
				yL[i] = ERR;
			}
			else {
				xL[i] = lrf_data[i] * sin(thetaL[i]);
				yL[i] = lrf_data[i] * cos(thetaL[i]);
			}
			//henkoIP2---------------------------
		}
	}

	sga_init_multi(robot, multi_Map);
	FILE *mfp7 = fopen("C:\\kinden\\log\\scale.txt", "a");//henkoIP2
	while (t < TS) 
	{
		double r, r2;
		robot->best_gene = sg_max(robot, &best_fitness);
		worst_gene = sg_min(robot, &worst_fitness);

		g = (int)(GANS*rnd());
		while (g == robot->best_gene || g == GANS)
		{
			g = (int)(GANS*rnd());
		}

		r2 = 0.5;//(10.0*(best_fitness - robot->fit_s[g]))/(best_fitness - worst_fitness + 0.01);
		g2 = robot->best_gene;
		r = rnd();
		//-------crossover-------------------------
		if (r < 0.7)
		{
			robot->gen_s[worst_gene][0] = robot->gen_s[g2][0];
		}
		else 
		{
			r = rnd();
			robot->gen_s[worst_gene][0] = (r*robot->gen_s[g][0] + (1.0 - r) * robot->gen_s[g2][0]);
		}

		//------mutation---------------------------
		if (rnd() < 0.7) 
		{
			robot->gen_s[worst_gene][0] += rndn() * (0.01 + r2 * (double)(TS - t) / (double)TS);
		}
		else 
		{
			robot->gen_s[worst_gene][0] += rndn() * rndn() * (0.01 + r2 * (double)(TS - t) / (double)TS);
		}


		if (robot->gen_s[worst_gene][0] <= 0.8) 
		{
			if (rnd() > 0.5)
			{
				robot->gen_s[worst_gene][0] = 1.0 + 0.5*rnd();
			}
			else
			{
				robot->gen_s[worst_gene][0] = 1.0 - 0.5*rnd();
			}
		}

		robot->fit_s[worst_gene] = fitcal_s_multi(worst_gene, robot, multi_Map);
		if (scaleMode == 0)
		{
			fprintf(mfp7, "%f\t%f\n", robot->fit_s[robot->best_gene], robot->gen_s[robot->best_gene][0]);//henkoIP2
		}
		else
		{
			fprintf(mfp7, "%f\t%d\t%f\t%f\n", t, multi_Map->reso, robot->fit_s[robot->best_gene], robot->gen_s[robot->best_gene][0]);//henkoIP2
		}
		t++;
	}
	fclose(mfp7);//henkoIP2
	robot->best_gene = sg_max(robot, &best_fitness);
	printf("[%lf]:%lf\n", robot->fit_s[robot->best_gene], robot->gen_s[robot->best_gene][0]);
}