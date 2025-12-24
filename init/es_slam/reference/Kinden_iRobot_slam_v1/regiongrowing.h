/*
*  regiongrowing.h
*  slam
*
*  Created by Kohei Oshio on 20/09/16.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#pragma once
#if !defined(REGIONGROWING_HEADER)
#define REGIONGROWING_HEADER

#include <stack>
#include <windows.h>
#include "Def.h"

#define THcolor 200  //白黒判別の閾値
//#define DISP_SIZE 700  //表示サイズ

const int vec[4][2] = {  //優先探索方向（4近傍）
	{ +0, +1 },  //下
	{ +1, +0 },  //右
	{ +0, -1 },  //上
	{ -1, +0 }   //左
};
//マウス操作コールバック関数
void getMousePos(int event, int x, int y, int flags, void *userdata);
//青色画素削除関数
int deleteBlue();
//画像初期化関数
int initImage();
//初期位置決定関数
void selectSelfPos();
//画像表示関数
void showImage();
//周囲マス探索関数
int searchRoom(int px, int py);
//画像作成関数
void createImage();
//エッジ抽出関数
int getEdge(char *boxPath, char *cadPath, char *edgePath, int *init_data);

#endif
