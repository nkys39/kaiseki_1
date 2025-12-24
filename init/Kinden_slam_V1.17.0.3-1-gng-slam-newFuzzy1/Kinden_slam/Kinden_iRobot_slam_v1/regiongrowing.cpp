/*
*  regiongrowing.cpp
*  slam
*
*  Created by Kohei Oshio on 20/09/16.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/

#include "regiongrowing.h"

char cad_path[255];  //CAD図のファイル名(入力)
char edge_cad_path[255];  //エッジ抽出後CAD図のファイル名(出力)

cv::Mat image;  //CAD図面画像
cv::Mat cpImage;  //CAD図面画像
cv::Mat sImage;  //縮小画像
cv::Mat room;  //部屋画像
cv::Mat edge;  //輪郭画像

int xsize, ysize;  //画像サイズ
double rate[2];
int selfPos[2] = { 1000, 1000 };  //自己位置[px]
bool start_flag;

//gng用
int** hit_data = NULL;
int** err_data = NULL;
int hit_data_size = 0;
int err_data_size = 0;
//#pragma mark -
#pragma mark getMousePos()
//マウス操作コールバック関数
void getMousePos(int event, int x, int y, int flags, void *userdata) {
	//左クリック
	if (event == cv::EVENT_LBUTTONDOWN) {
		selfPos[0] = (int)(x / rate[0]);
		selfPos[1] = (int)(y / rate[1]);
	}
	//右クリック
	if (event == cv::EVENT_RBUTTONDOWN) {
		start_flag = true;
	}
}

#pragma mark -
#pragma mark deleteBlue()
//青色画素削除関数
int deleteBlue() {
	bool isBlue;
	cv::Mat hsvImage;
	cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);  //BGRをHSVへ変換
													   //青色削除
	for (int x = 0; x < xsize; x++) {
		for (int y = 0; y < ysize; y++) {
			//色相が150°〜250°を青色と判定
			isBlue = true;
//			isBlue &= hsvImage.at<cv::Vec3b>(y, x)[0] * 2 > 150;
//			isBlue &= hsvImage.at<cv::Vec3b>(y, x)[0] * 2 < 250;
			isBlue = image.at<cv::Vec3b>(y, x)[0] >= THcolor;
			if (isBlue == true) {
				image.at<cv::Vec3b>(y, x)[0] = 255;
				image.at<cv::Vec3b>(y, x)[1] = 255;
				image.at<cv::Vec3b>(y, x)[2] = 255;
			}

			// エッジCAD図のサイズが大きいと作成に時間がかかるため、途中でループを抜ける処理を追加
			std::string rcvBuff = std::string(PipeReceive);
			size_t command = rcvBuff.find("Stop");
			if (command != std::string::npos)
			{
				return -1;
			}
		}
	}
	return 0;
}

#pragma mark -
#pragma mark initImage()
//画像初期化関数
int initImage() {
	room = cv::Mat(cv::Size(xsize, ysize), CV_8UC3);
	for (int x = 0; x < xsize; x++) {
		for (int y = 0; y < ysize; y++) {
			room.at<cv::Vec3b>(y, x)[0] = 0;
			room.at<cv::Vec3b>(y, x)[1] = 0;
			room.at<cv::Vec3b>(y, x)[2] = 0;

			// エッジCAD図のサイズが大きいと作成に時間がかかるため、途中でループを抜ける処理を追加
			std::string rcvBuff = std::string(PipeReceive);
			size_t command = rcvBuff.find("Stop");
			if (command != std::string::npos)
			{
				return -1;
			}
		}
	}
	edge = cv::Mat(cv::Size(xsize, ysize), CV_8UC3);
	for (int x = 0; x < xsize; x++) {
		for (int y = 0; y < ysize; y++) {
			edge.at<cv::Vec3b>(y, x)[0] = 0;
			edge.at<cv::Vec3b>(y, x)[1] = 0;
			edge.at<cv::Vec3b>(y, x)[2] = 0;

			// エッジCAD図のサイズが大きいと作成に時間がかかるため、途中でループを抜ける処理を追加
			std::string rcvBuff = std::string(PipeReceive);
			size_t command = rcvBuff.find("Stop");
			if (command != std::string::npos)
			{
				return -1;
			}
		}
	}
	cpImage = image.clone();
	sImage = cv::Mat(cv::Size(MAP_DISP_SIZE, MAP_DISP_SIZE), CV_8UC3);

	return 0;
}

#pragma mark -
#pragma mark selectSelfPos()
//初期位置決定関数
void selectSelfPos() {
	start_flag = false;

	while (true) {
		cv::setMouseCallback("Map", getMousePos);  //マウスのコールバック関数設定
		cpImage = image.clone();
		cv::circle(cpImage, cv::Point(selfPos[0], selfPos[1]), 10, cv::Scalar(0, 0, 255));

		cv::resize(cpImage, sImage, cv::Size(), rate[0], rate[1]);
		cv::imshow("Map", sImage);
		cv::waitKey(1);

		if (start_flag == true) {
			break;
		}
	}
}

#pragma mark -
#pragma mark showImage()
//画像表示関数
void showImage() {
	int val;

	for (int x = 0; x < xsize; x++) {
		for (int y = 0; y < ysize; y++) {
			val = 0;
			val += image.at<cv::Vec3b>(y, x)[0];
			val += image.at<cv::Vec3b>(y, x)[1];
			val += image.at<cv::Vec3b>(y, x)[2];
			val /= 3;
			if (val > THcolor) {
				cpImage.at<cv::Vec3b>(y, x)[0] = 255;
				cpImage.at<cv::Vec3b>(y, x)[1] = 255;
				cpImage.at<cv::Vec3b>(y, x)[2] = 255;
			}
			else {
				cpImage.at<cv::Vec3b>(y, x)[0] = 0;
				cpImage.at<cv::Vec3b>(y, x)[1] = 0;
				cpImage.at<cv::Vec3b>(y, x)[2] = 0;
			}
			if (room.at<cv::Vec3b>(y, x)[0] == 255) {
				cpImage.at<cv::Vec3b>(y, x)[0] = 255;
				cpImage.at<cv::Vec3b>(y, x)[1] = 0;
				cpImage.at<cv::Vec3b>(y, x)[2] = 0;
			}
			if (edge.at<cv::Vec3b>(y, x)[2] == 255) {
				cpImage.at<cv::Vec3b>(y, x)[0] = 0;
				cpImage.at<cv::Vec3b>(y, x)[1] = 0;
				cpImage.at<cv::Vec3b>(y, x)[2] = 255;
			}
		}
	}
	cv::circle(cpImage, cv::Point(selfPos[0], selfPos[1]), 10, cv::Scalar(0, 0, 255));

	cv::resize(cpImage, sImage, cv::Size(), rate[0], rate[1]);
	cv::imshow("Map", sImage);
	cv::waitKey(1);
}

#pragma mark -
#pragma mark searchRoom()
//周囲マス探索関数
int searchRoom(int px, int py) {
	std::stack<int> stack_x, stack_y;  //スタック
	bool skip = false;
	int val;

	//初期値反映
	room.at<cv::Vec3b>(py, px)[0] = 255;
	stack_x.push(px);
	stack_y.push(py);

	while (true) {
		int x = stack_x.top();
		int y = stack_y.top();
		//各方向について
		for (int d = 0; d < 4; d++) {
			//探索セルが端の場合
			if (x == 0 || x == xsize - 1 || y == 0 || y == ysize - 1) {
				edge.at<cv::Vec3b>(y, x)[1] = 255;  //輪郭ラベリング
													//探索セルが端でない場合
			}
			else {
				val = 0;
				val += image.at<cv::Vec3b>(y + vec[d][1], x + vec[d][0])[0];
				val += image.at<cv::Vec3b>(y + vec[d][1], x + vec[d][0])[1];
				val += image.at<cv::Vec3b>(y + vec[d][1], x + vec[d][0])[2];
				val /= 3;
				//探索方向セルが黒色でない場合
				if (val > THcolor) {
					//探索済みセルは無視
					if (room.at<cv::Vec3b>(y + vec[d][1], x + vec[d][0])[0] == 0) {
						//部屋ラベリング
						room.at<cv::Vec3b>(y + vec[d][1], x + vec[d][0])[0] = 255;
						//次回探索候補へ追加
						stack_x.push(x + vec[d][0]);
						stack_y.push(y + vec[d][1]);
						skip = true;  //スキップ
					}
					//探索方向セルが黒色の場合
				}
				else {
					//輪郭ラベリング
					edge.at<cv::Vec3b>(y + vec[d][1], x + vec[d][0])[2] = 255;
				}
			}

			if (skip == true) {
				break;
			}
		}

		if (skip == false) {
			//削除処理
			stack_x.pop();
			stack_y.pop();
			if (stack_x.empty()) {
				break;
			}
		}

		skip = false;

		// エッジCAD図のサイズが大きいと作成に時間がかかるため、途中でループを抜ける処理を追加
		std::string rcvBuff = std::string(PipeReceive);
		size_t command = rcvBuff.find("Stop");
		if (command != std::string::npos)
		{
			return -1;
		}

	}
	return 0;
}

#pragma mark -
#pragma mark createImage()
//画像作成関数
void createImage() {
	for (int x = 0; x < xsize; x++) {
		for (int y = 0; y < ysize; y++) {
			if (edge.at<cv::Vec3b>(y, x)[2] == 255) {
				edge.at<cv::Vec3b>(y, x)[0] = 0;
				edge.at<cv::Vec3b>(y, x)[1] = 0;
				edge.at<cv::Vec3b>(y, x)[2] = 255;
			}
			else {
				/*edge.at<cv::Vec3b>(y, x)[0] = 255;
				edge.at<cv::Vec3b>(y, x)[1] = 255;
				edge.at<cv::Vec3b>(y, x)[2] = 255;*/
				edge.at<cv::Vec3b>(y, x)[0] = 0;
				edge.at<cv::Vec3b>(y, x)[1] = 0;
				edge.at<cv::Vec3b>(y, x)[2] = 0;
			}
			if (room.at<cv::Vec3b>(y, x)[0] == 255) {
				room.at<cv::Vec3b>(y, x)[0] = 255;
				room.at<cv::Vec3b>(y, x)[1] = 255;
				room.at<cv::Vec3b>(y, x)[2] = 255;
			}
			else {
				room.at<cv::Vec3b>(y, x)[0] = 0;
				room.at<cv::Vec3b>(y, x)[1] = 0;
				room.at<cv::Vec3b>(y, x)[2] = 0;
			}
		}
	}
}

#pragma mark -
#pragma mark getEdge()
//エッジ抽出関数
int getEdge(char* boxPath, char *cadPath, char *edgePath, int *init_pos) {
	//ファイル名設定
	strcpy(cad_path, boxPath);
	sprintf(cad_path, "%s\\kinden\\map\\%s", boxPath, cadPath);
//	sprintf(cad_path, "C:\\kinden\\map\\%s", cadPath);
	sprintf(edge_cad_path, "C:\\kinden\\log\\%s", edgePath);
	printf("%s\n", cad_path);
	printf("%s\n", edge_cad_path);

	//画像読み込み
	image = cv::imread(cad_path, cv::IMREAD_COLOR);
	xsize = image.cols;
	ysize = image.rows;
	rate[0] = (double)MAP_DISP_SIZE / (double)xsize;
	rate[1] = (double)MAP_DISP_SIZE / (double)xsize;

	//初期位置の選択
	if (RG_MODE == 0) {
		selectSelfPos();
	}
	else {
		selfPos[0] = init_pos[0];
		selfPos[1] = init_pos[1];
	}

	int ret;
	//青色削除
	ret = deleteBlue();
	if (ret == -1) return ret;

	//画像初期化
	ret = initImage();
	if (ret == -1) return ret;

	//探索
	ret = searchRoom(selfPos[0], selfPos[1]);
	if (ret == -1) return ret;

	//結果表示
	showImage();
	cv::waitKey(1);
	Sleep(1000);

	//保存
	createImage();
	cv::imwrite(edge_cad_path, edge);

	cv::destroyAllWindows();

	return 0;
}

void updateImage(cv::Mat cadMat) {
	for (int x = 0; x < xsize; x++) {
		for (int y = 0; y < ysize; y++) {
			if (room.at<cv::Vec3b>(y, x)[0] == 255) {
				cadMat.at<cv::Vec3b>(y, x)[0] = 255;
				cadMat.at<cv::Vec3b>(y, x)[1] = 255;
				cadMat.at<cv::Vec3b>(y, x)[2] = 255;
			}
			if (edge.at<cv::Vec3b>(y, x)[2] == 255) {
				cadMat.at<cv::Vec3b>(y, x)[0] = 0;
				cadMat.at<cv::Vec3b>(y, x)[1] = 0;
				cadMat.at<cv::Vec3b>(y, x)[2] = 255;
			}
		}
	}
}

//gng用エッジ抽出
void getInputForGNG(cv::Mat cadMat, int init_x, int init_y) {
	//画像読み込み
	image = cadMat;
	xsize = image.cols;
	ysize = image.rows;

	// 初期化
	int ret;
	ret = initImage();
	
	// 領域拡張
	ret = searchRoom(init_x, init_y);
	createImage();
	updateImage(cadMat);

	//　入力準備
	int use_data_num = 0, hit = 0, err = 0;
	for (int y = 1; y < ysize-1; y++) {
		for (int x = 1; x < xsize-1; x++) {
			// 走行領域 >> err
			if (room.at<cv::Vec3b>(y, x)[0] == 255) {
				use_data_num++;
				err++;
			}
			// 輪郭 >> edge
			if (edge.at<cv::Vec3b>(y, x)[2] == 255) {
				use_data_num++;
				hit++;
			}
		}
	}
	hit_data = (int**)malloc(sizeof(int*) * hit);
	for (size_t i = 0; i < hit; i++) {
		hit_data[i] = (int*)malloc(sizeof(int) * 2);
	}
	err_data = (int**)malloc(sizeof(int*) * err);
	for (size_t i = 0; i < err; i++) {
		err_data[i] = (int*)malloc(sizeof(int) * 2);
	}
	int a = 0, hc = 0, ec = 0;
	for (int y = 1; y < ysize-1; y++) {
		for (int x = 1; x < xsize-1; x++) {
			// 走行領域 >> err
			if (room.at<cv::Vec3b>(y, x)[0] == 255) {
				err_data[ec][0] = x; //x
				err_data[ec][1] = y; //y
				ec++;
			}
			// 輪郭 >> edge
			if (edge.at<cv::Vec3b>(y, x)[2] == 255) {
				hit_data[hc][0] = x; //x
				hit_data[hc][1] = y; //y
				hc++;
			}
		}
	}
	hit_data_size = hc;
	err_data_size = ec;
}

int** getHitInput() {
	return hit_data;
}

int** getErrorInput() {
	return err_data;
}

int getHitDataSize() {
	return hit_data_size;
}

int getErrorDataSize() {
	return err_data_size;
}
