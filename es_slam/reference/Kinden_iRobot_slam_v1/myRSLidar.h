#if !defined(MYRSLIDAR_HEADER)
#define MYRSLIDAR_HEADER

#define HAVE_STRUCT_TIMESPEC

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <pthread.h>

#include "Def.h"
#include "urg_sensor.h"  //URG
#include "urg_utils.h"

//パケットデータ
struct UDP_Package
{
	char Head[42];  //ヘッダ
	char FiringData[1200];  //データ
	char Tail[6];  //末尾
};

//共用体 受信パケット分類のため
union UDP_Data
{
	char Recvchars[1248];
	UDP_Package package;
};

//共用体 2バイト結合のため
union TwoCharsInt
{
	char datain[2];
	unsigned short dataout;
};


//RS-LiDAR-16接続
bool RS_Initial();
//スレッド処理開始
void startThread();
//スレッド処理停止
void endThread();

//RS-LiDAR-16からデータ受信
void *RSRecvThread(void *arg);

//反射強度関連(使ってない)
float computeTemperature(unsigned char bit1, unsigned char bit2);
int estimateTemperature(float Temper);
float CalibrateIntensity(float intensity, int calIdx, int distance);

//パケットから距離と角度に変換
void *RSProcess(void *arg);

#endif