/*
*  draw.c
*  slam
*
*  Created by Yuichiro Toda on 12/04/17.
*  Copyright 2012 TMU. All rights reserved.
*
*/

#include "draw.h"

//henkoIP------------
int lrf2[10] = { 0 };//henkoIPz
int devide2 = 10;;//henkoIPz

void readLRF2()
{
	char lrf_area[255];
	sprintf(lrf_area, "C:\\kinden\\cmd\\lrf_area.txt");
	FILE *mfp4 = fopen(lrf_area, "r");
	if (mfp4 == NULL)
	{
		printf("LRF AREA2 file open error2!\n");
		return;
	}
	int l;
	int ct = 0;
	int i = 0;
	while (fscanf(mfp4, "%d\n", &l) != EOF)
	{
		//if (ct == 0)
		//{
		//	if (scaleMode == 0)
		//	{
		//		devide2 = 10;//henkoIPz
		//	}
		//	else
		//	{
		//		devide2 = l;//henkoIPz
		//	}
		//	//lrf2 = (int*)malloc(sizeof(int) * l);//henkoIPz
		//}
		//else
		//{
			lrf2[i] = l;
			i++;
		//}
		//ct++;
	}
	fclose(mfp4);
	printf("devide = %d\n", devide2);
	for (i = 0;i < devide2;i++)
	{
		printf("%d,", lrf2[i]);
	}
	printf("\n");
}

//henkoIP---------------

void draw_node(struct robot_position *robot, IplImage *draw_Image, int R, int G, int B)
{
	int i, j;
	for (i = 0; i < robot->node_n; i++)
	{
		robot->node_x = (int)(map_center_width - (int)robot->node[i][0] * 1000 / map_rate);
		robot->node_y = (int)(map_center_height - (int)robot->node[i][1] * 1000 / map_rate);


		cvCircle(draw_Image, cvPoint(robot->node_x, robot->node_y), 4, CV_RGB(B, G, R), -1, 8, 0);
		cvCircle(draw_Image, cvPoint(robot->node_x, robot->node_y), 4, CV_RGB(B, G, R), -1, 8, 0);
		for (j = 0; j<robot->node_n; j++)
		{
			if (i != j)
			{
				robot->node_x2 = (int)(map_center_width - (int)robot->node[j][0] * 1000 / map_rate);
				robot->node_y2 = (int)(map_center_height - (int)robot->node[j][1] * 1000 / map_rate);


				if (robot->edge[i][j] == 1 && robot->edge[j][i] == 1)
				{
					cvLine(draw_Image, cvPoint(robot->node_x, robot->node_y), cvPoint(robot->node_x2, robot->node_y2), CV_RGB(255, 0, 255), 3, 8, 0);
				}
			}
		}
	}

}

void draw_robot(struct robot_position *robot, double robot_size, IplImage *draw_Image, int R, int G, int B)
{
	CvPoint rpt, rpt2, rpt3;
	rpt.x = robot->map_x;
	rpt.y = robot->map_y;
	rpt2.x = (int)(robot->map_x + (-robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) - (robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
	rpt2.y = (int)(robot->map_y + (robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) + (-robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
	rpt3.x = (int)(robot->map_x + (robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) - (robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
	rpt3.y = (int)(robot->map_y + (robot_size)*cos(((double)robot->rangle / 180.0)*M_PI) + (robot_size)*sin(((double)robot->rangle / 180.0)*M_PI));
	cvLine(draw_Image, rpt, rpt2, CV_RGB(B, G, R), 3, 8, 0);
	cvLine(draw_Image, rpt, rpt3, CV_RGB(B, G, R), 3, 8, 0);
}


void draw_LRFdata(int mode, struct robot_position *robot, long *lrf_data, IplImage *drawImage, int R, int G, int B)
{
	int i;
	double s1, c1;
	double x_1, y_1, theta;
	double x_2, y_2;
	CvPoint pt1;
	bool err = false;
	int n_lrf;//henkoIP2

	for (i = 0; i < SCAN_MARKS; i++) {
		if (mode == 0) {  //初期位置推定時
			err = lrf_data[i] < 100 || lrf_data[i] > 40000;
		}
		//henkoIP2-------
		/*
		else {  //SLAM時
		//err = lrf_data[i] < 200 || lrf_data[i] > 40000 || i < 200 || i > 880;
		}
		*/
		//henkoIP2-------
		if (!err) {
			theta = (((double)i - 540.0) * (360.0 / 1440.0) * M_PI) / 180.0;
			x_1 = lrf_data[i] * sin(theta);
			y_1 = lrf_data[i] * cos(theta);
			s1 = sin((robot->rangle)* M_PI / 180.0);
			c1 = cos((robot->rangle)* M_PI / 180.0);
			x_2 = (double)x_1 * c1 - (double)y_1 * s1 + (double)(map_center_width - (robot->map_x)) * MAPRATE;
			y_2 = (double)x_1 * s1 + (double)y_1 * c1 + (double)(map_center_height - (robot->map_y)) * MAPRATE;
			pt1.x = (int)(map_center_width - (int)x_2 / MAPRATE);
			pt1.y = (int)(map_center_height - (int)y_2 / MAPRATE);
			//henkoIP2--------
			if (mode == 0)
			{
				n_lrf = (int)(i * devide2 / SCAN_MARKS);
				if (lrf2[n_lrf] == 0) {
					cvCircle(drawImage, cvPoint(pt1.x, pt1.y), 1, CV_RGB(B, G, R), -1, 8, 0);
				}
				else {
					cvCircle(drawImage, cvPoint(pt1.x, pt1.y), 1, CV_RGB(R, G, B), -1, 8, 0);
				}
			}
			else
				cvCircle(drawImage, cvPoint(pt1.x, pt1.y), 1, CV_RGB(R, G, B), -1, 8, 0);
			//henkoIP2--------
		}
	}
}

void draw_LRFdata2(int mode, struct robot_position* robot, long* lrf_dataD, float* lrf_dataA, IplImage* drawImage, int R, int G, int B)
{
	int i;
	double s1, c1;
	double x_1, y_1, theta;
	double x_2, y_2;
	CvPoint pt1;
	bool err = false;
	int n_lrf;//henkoIP2

	for (i = 0; i < rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM; i++) {
		if (mode == 0) {  //初期位置推定時
			err = lrf_dataD[i] < LASER_RANGE_MIN || lrf_dataD[i] > LASER_RANGE_MAX;
		}
		if (!err) {
			theta = (((double)i - 540.0) * (360.0 / 1440.0) * M_PI) / 180.0;
			x_1 = (double)lrf_dataD[i] * sin(lrf_dataA[i]);
			y_1 = (double)lrf_dataD[i] * cos(lrf_dataA[i]);
			s1 = sin((robot->rangle) * M_PI / 180.0);
			c1 = cos((robot->rangle) * M_PI / 180.0);
			x_2 = (double)x_1 * c1 - (double)y_1 * s1 + (double)(map_center_width - (robot->map_x)) * MAPRATE;
			y_2 = (double)x_1 * s1 + (double)y_1 * c1 + (double)(map_center_height - (robot->map_y)) * MAPRATE;
			pt1.x = (int)(map_center_width - (int)x_2 / MAPRATE);
			pt1.y = (int)(map_center_height - (int)y_2 / MAPRATE);
			//henkoIP2--------
			if (mode == 0)
			{
				n_lrf = (int)(i * devide2 / (rs16_NumOfPackage * BLOCKS_PER_PACKET * DATA_NUM));
				if (lrf2[n_lrf] == 0) {
					cvCircle(drawImage, cvPoint(pt1.x, pt1.y), 1, CV_RGB(B, G, R), -1, 8, 0);
				}
				else {
					cvCircle(drawImage, cvPoint(pt1.x, pt1.y), 1, CV_RGB(R, G, B), -1, 8, 0);
				}
			}
			else
				cvCircle(drawImage, cvPoint(pt1.x, pt1.y), 1, CV_RGB(R, G, B), -1, 8, 0);
			//henkoIP2--------
		}
	}
}

void draw_globalMap(int **omap, int **amap, IplImage *omapImage, IplImage *amapImage, IplImage *mmapImage[])
{
	int i, j, l;

	for (i = 0; i<MAPSIZE_WIDTH; i++)
		for (j = 0; j<MAPSIZE_HIGHT; j++) {
			if (omap[i][j] < -1) {
				omapImage->imageData[omapImage->widthStep*j + i * 3] = (char)255;
				omapImage->imageData[omapImage->widthStep*j + i * 3 + 1] = (char)255;
				omapImage->imageData[omapImage->widthStep*j + i * 3 + 2] = (char)255;

				if (mmapImage[0] != NULL)
					for (l = 0; l<10; l++) {
						mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3] = (char)255;
						mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 1] = (char)255;
						mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 2] = (char)255;
					}
			}
			else if (omap[i][j] > 1 && omap[i][j]) {
				omapImage->imageData[omapImage->widthStep*j + i * 3] = 0;
				omapImage->imageData[omapImage->widthStep*j + i * 3 + 1] = 0;
				omapImage->imageData[omapImage->widthStep*j + i * 3 + 2] = 0;

				if (mmapImage[0] != NULL)
					for (l = 0; l<10; l++) {
						mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3] = 0;
						mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 1] = 0;
						mmapImage[l]->imageData[mmapImage[l]->widthStep*j + i * 3 + 2] = 0;
					}
			}

			if (amap[i][j] > 0 && amap != NULL) {
				amapImage->imageData[amapImage->widthStep*j + i * 3] = 0;
				amapImage->imageData[amapImage->widthStep*j + i * 3 + 1] = 0;
				amapImage->imageData[amapImage->widthStep*j + i * 3 + 2] = 0;
			}
		}
}

void calc_arrow(double base[3], double gen[3], double start[2], double end[2]) {  //henkoModel
	double len = 20.0;

	double s_x, s_y, s_r;
	double map_xyz[3];

	//[mm]
	s_x = gen[0];
	s_y = gen[1];
	s_r = gen[2];
	//[pixel]
	map_xyz[0] = base[0] + (s_x * cos((base[2] + s_r) * M_PI / 180.0) - s_y * sin((base[2] + s_r) * M_PI / 180.0)) / TR;
	map_xyz[1] = base[1] + (s_y * cos((base[2] + s_r) * M_PI / 180.0) + s_x * sin((base[2] + s_r) * M_PI / 180.0)) / TR;
	map_xyz[2] = base[2] + s_r;

	//始点, 終点計算
	start[0] = -miniMapRate * (map_xyz[0] - base[0]) + (double)miniMapSize / 2.0 - len / 2.0 * sin(base[2] * M_PI / 180.0);
	start[1] = -miniMapRate * (map_xyz[1] - base[1]) + (double)miniMapSize / 2.0 + len / 2.0 * cos(base[2] * M_PI / 180.0);
	end[0] = start[0] + len * sin(map_xyz[2] * M_PI / 180.0);
	end[1] = start[1] - len * cos(map_xyz[2] * M_PI / 180.0);
}

void draw_gen(int mode, struct robot_position *robot, int step, double *outp) {  //henkoModel
	static double base[3];
	double start[2], end[2];
	char text[50];
	double zero[3] = { 0.0, 0.0, 0.0 };
	int x_opt, y_opt;

	//画像の初期化
	static cv::Mat miniMap = cv::Mat(cv::Size(miniMapSize, miniMapSize), CV_8UC3);

	//基準値(前回の最良個体)の保存＋過去の最良個体の保存
	if (mode == 0) {
		base[0] = robot->real_x / TR;
		base[1] = robot->real_y / TR;
		base[2] = robot->dangle;
	}

	//GA結果の描画
	if (mode == 1) {
		//画像のリセット
		cv::rectangle(miniMap, cv::Point(0, 0), cv::Point(miniMapSize, miniMapSize), cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);

		//グリッド描画
		for (double i = 0; i < (double)miniMapSize; i += (double)miniMapSize / (double)gridNum) {
			cv::line(miniMap, cv::Point((int)i, 0), cv::Point((int)i, miniMapSize), cv::Scalar(230, 230, 230), 1, 8, 0);
			memset(text, 0, sizeof(text));
			sprintf(text, "%d", (int)((i - (double)miniMapSize / 2.0) * TR / miniMapRate));
			cv::putText(miniMap, text, cv::Point((int)i + 2, miniMapSize / 2 - 2), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(230, 230, 230), 1, CV_AA);

			cv::line(miniMap, cv::Point(0, (int)i), cv::Point(miniMapSize, (int)i), cv::Scalar(230, 230, 230), 1, 8, 0);
			memset(text, 0, sizeof(text));
			sprintf(text, "%d", (int)(-(i - (double)miniMapSize / 2.0) * TR / miniMapRate));
			cv::putText(miniMap, text, cv::Point(miniMapSize / 2 + 2, (int)i - 2), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(230, 230, 230), 1, CV_AA);
		}

		//解候補の描画ループ
		for (int i = 0; i < GANM; i++) {
			calc_arrow(base, robot->gen_m[i], start, end);
			if (i != robot->best_gene) {
				cv::arrowedLine(miniMap, cv::Point((int)start[0], (int)start[1]), cv::Point((int)end[0], (int)end[1]), cv::Scalar(200, 200, 200), 2, 8, 0, 0.3);
			}
		}
	}

	if (mode == 2) {
		//基準値(前回の最良個体)の描画
		calc_arrow(base, zero, start, end);
		cv::arrowedLine(miniMap, cv::Point((int)start[0], (int)start[1]), cv::Point((int)end[0], (int)end[1]), cv::Scalar(0, 255, 0), 2, 8, 0, 0.3);

		//現在の最悪個体の描画
		double a;
		calc_arrow(base, robot->gen_m[g_min(robot, &a)], start, end);
		cv::arrowedLine(miniMap, cv::Point((int)start[0], (int)start[1]), cv::Point((int)end[0], (int)end[1]), cv::Scalar(255, 0, 0), 2, 8, 0, 0.3);

		//現在の最良個体の描画
		calc_arrow(base, robot->gen_m[robot->best_gene], start, end);
		cv::arrowedLine(miniMap, cv::Point((int)start[0], (int)start[1]), cv::Point((int)end[0], (int)end[1]), cv::Scalar(0, 0, 255), 2, 8, 0, 0.3);

		//ロボットモデル計算結果の描画
//		double temp[3];  //2022.04.28コメントアウト
//		calcRobotModel(temp);  //2022.04.28コメントアウト
		calc_arrow(base, robot->model, start, end);  //2022.04.28変更
		cv::arrowedLine(miniMap, cv::Point((int)start[0], (int)start[1]), cv::Point((int)end[0], (int)end[1]), cv::Scalar(255, 0, 255), 2, 8, 0, 0.3);

		//ステップ数表示
		memset(text, 0, sizeof(text));
		sprintf(text, "t=%d", step);
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize / 30.0), (int)((double)miniMapSize / 30.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(0, 0, 0), (int)((double)miniMapSize / 500.0), CV_AA);

		//単位系表示
		memset(text, 0, sizeof(text));
		sprintf(text, "[mm]");
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize / 1.1), (int)((double)miniMapSize / 1.05)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(230, 230, 230), (int)((double)miniMapSize / 500.0), CV_AA);

		//解の表示
		memset(text, 0, sizeof(text));
		x_opt = (int)(-(robot->gen_m[robot->best_gene][0] * cos((robot->dangle + robot->gen_m[robot->best_gene][2]) * M_PI / 180.0) - robot->gen_m[robot->best_gene][1] * sin((robot->dangle + robot->gen_m[robot->best_gene][2]) * M_PI / 180.0)));
		y_opt = (int)(robot->gen_m[robot->best_gene][1] * cos((robot->dangle + robot->gen_m[robot->best_gene][2]) * M_PI / 180.0) + robot->gen_m[robot->best_gene][0] * sin((robot->dangle + robot->gen_m[robot->best_gene][2]) * M_PI / 180.0));
		sprintf(text, "gen=(%d, %d)", x_opt, y_opt);
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize / 30.0), (int)((double)miniMapSize / 15.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(0, 0, 255), (int)((double)miniMapSize / 500.0), CV_AA);

		memset(text, 0, sizeof(text));
		sprintf(text, "fitness=%.2lf", robot->fit_m[robot->best_gene]);
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize / 30.0), (int)((double)miniMapSize / 10.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(0, 0, 255), (int)((double)miniMapSize / 500.0), CV_AA);

		//モータ出力値表示
		memset(text, 0, sizeof(text));
		sprintf(text, "output=(%d, %d)", (int)outp[1], (int)outp[0]);
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize / 30.0), (int)((double)miniMapSize / 7.5)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(255, 0, 255), (int)((double)miniMapSize / 500.0), CV_AA);

		//描画情報表示
		memset(text, 0, sizeof(text));
		sprintf(text, "Pos_best(t-1)");
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize * 0.75), (int)((double)miniMapSize / 30.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(0, 255, 0), (int)((double)miniMapSize / 500.0), CV_AA);
		memset(text, 0, sizeof(text));
		sprintf(text, "Pos_cand(t)");
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize * 0.75), (int)((double)miniMapSize / 15.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(200, 200, 200), (int)((double)miniMapSize / 500.0), CV_AA);
		memset(text, 0, sizeof(text));
		sprintf(text, "Pos_best(t)");
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize * 0.75), (int)((double)miniMapSize / 10.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(0, 0, 255), (int)((double)miniMapSize / 500.0), CV_AA);
		memset(text, 0, sizeof(text));
		sprintf(text, "Pos_worst(t)");
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize * 0.75), (int)((double)miniMapSize / 7.5)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(255, 0, 0), (int)((double)miniMapSize / 500.0), CV_AA);
		memset(text, 0, sizeof(text));
		sprintf(text, "Pos_model(t)");
		cv::putText(miniMap, text, cv::Point((int)((double)miniMapSize * 0.75), (int)((double)miniMapSize / 6.0)), cv::FONT_HERSHEY_SIMPLEX, (double)miniMapSize / 1000.0, cv::Scalar(255, 0, 255), (int)((double)miniMapSize / 500.0), CV_AA);

		//ミニマップの表示
		cv::imshow("miniMap", miniMap);
		cv::waitKey(1);
		if (step < 100) {
			Sleep(100);
		}

	}
}

