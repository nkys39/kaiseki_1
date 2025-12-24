#pragma warning(disable : 4996)
#pragma comment(lib, "Shlwapi.lib")

#include "main.h" 
#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_connection.h"
#include "urg_serial_utils.h"
#include "urg_detect_os.h"
#include "urg_debug.h"
#include "errno.h"
#include "urg_ring_buffer.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <time.h>
#include <io.h>
#include <sys/types.h>

#include "myRSLidar.h"
#include "localization.h"
#include "slam.h"
#include "random.h"
#include "multi_resolution.h"
#include "draw.h"
#include "illuminance.h"
#include "controller.h"
#include "regiongrowing.h"  //henko2 領域拡張追加
#include "GNG.h"			//gng
#include "gngPlanner.h"		//astar-planner
#include <iostream>
#include <conio.h>
#include <tchar.h>
#include <locale.h>
#include <thread>
#include <string.h>
#include <wininet.h>	// Win32インターネット拡張機能
#include <windows.h>

#define EXTERN_REF
#include "Def.h"
#include "malloc.h"
#pragma comment(lib,"Winmm.lib")//この行の代わりに「追加の依存ファイル」に追加しても良い
#include <mmsystem.h>
#pragma comment(lib, "wininet.lib")

//double TR = DEF_TR;	//mm/pixel henkoModel
//double TR = 60.0;	//mm/pixel
double TR_o = TR;			//mm/pixel
int data_max;
long *data = NULL;
long time_stamp;
urg_t urg;
urg_connection_type_t connection_type = URG_SERIAL;
long baudrate_or_port = CBR_115200;
int ill_data;
int uc_flag = -1;
char idfname[255];

double illmData;
char bumper[3];
int stratPosition[3];
double tr;							// TR値
char mapFileName[255];
char mapFileNameEdge[255];  //追加 henko2
char PipeReceive[255];
char RobotStatus;
int falling[2];
char engineering_mode;

char resultViewflie[255];	//iPad表示用のtmp結果ファイルurl

int illmPort;
int illmPort2;
int tcpPort;
int lrfPort;
int ardnPort;
int roombaPort;
int maxSpeed;
int constantSpeed;
int lowSpeed;
int lowLimitSpeed;

int init_time ;
int judge_range;
double area_length;
int judge_range_tmp;
double area_length_tmp;
double tr_threshold;

char scaleMode;
char slamMode;
char modelMode;

double ga_search_range;

double wheelDia;
double wheelWidth;
double lidarWheelDist;
double microTime;
double outputCoefL;
double outputCoefR;

std::string rcvBuff;
std::string::size_type command;

char BoxPath[255] = "";
char nameBuf[255];


TCHAR strPcName[MAX_PATH];
TCHAR strConstructionSite[MAX_PATH];

int rs16_thread_mode;	// RS-LiDAR-16 スレッドモード(0:スレッド処理なし,1:スレッド処理あり)
int rs16_rpm;			// RS-LiDAR-16 回転速度[rpm]
int rs16_NumOfPackage;	// RS-LiDAR-16 パケットデータ数

//Parameter.ini関連
int edge_cad;
int fall_sensor;
int fallThresHold;
int run_finish_notice;
int fall_sensor_notice;
int run_difficult_notice;
int running_notice;
int running_notice_time;
TCHAR strMail1[MAX_PATH];
TCHAR strMail2[MAX_PATH];
TCHAR strMail3[MAX_PATH];
TCHAR strMail4[MAX_PATH];
TCHAR strMail5[MAX_PATH];
TCHAR strSMTPServer[MAX_PATH];
TCHAR strSMTPPassword[MAX_PATH];
TCHAR strSMTPMailAddress[MAX_PATH];
int SMTPPort;

int runningStartTime;
int runningEndTime;

double fitness_th;
int fitness_time;
int fitness_cnt;

// ---- Initial ----

bool restartFlg = false;
char slam_eror_count;
//--------------------mapping----------------------------
IplImage *viewImage2;
IplImage *mapImage;
IplImage *multimapImage[10];
IplImage *limage;
IplImage *limage2;  //追加 henko2

//int map[MAPSIZE][MAPSIZE];
//int map2[MAPSIZE][MAPSIZE];
//int cad_map[MAPSIZE][MAPSIZE];  //追加 henko2
//int cad_map2[MAPSIZE][MAPSIZE];
//int edge_map[MAPSIZE][MAPSIZE];  //追加 henko2
//int edge_map2[MAPSIZE][MAPSIZE];
struct org_map baseMap;
int width;
int height;
int map_size_width;
int map_size_height;
int map_center_width;
int map_center_height;

struct multi_map *root = NULL; 
struct multi_map *cmap = NULL;
struct robot_position *robot_pos;
struct es_gene *initial_Position;
int lflag = 0;
int flag = 5;

measurement_position measurementPosition[255];	// 計測地点

static int measurementId[255];			//計測地点ID番号

time_t InitPosiSerchStartTime;

//------------------------RS-Lidar------------------------
long dataD[RS16_NUM_OF_PACKAGE_MAX * BLOCKS_PER_PACKET * DATA_NUM];  //距離データ
float dataA[RS16_NUM_OF_PACKAGE_MAX * BLOCKS_PER_PACKET * DATA_NUM];  //角度データ

//------------robot movement-------------
int moveMode=11;
double poutp[4];
int fpath[MAX_MEASURE_POINT][2];// 測定点格納バッファ
int path_ct;

//------------ gng planning -------------
GNG *gng;
FastAstar *pp;

//------------ setting -------------
bool initPosiJugde = false;

int SoundPlayMode = 0;
TCHAR strSoundPath[MAX_PATH];
bool isOnline = false;

int colflag = 0;

static void print_data(urg_t *urg, long data[], int data_n, long time_stamp);
static void urg_exit(urg_t *urg, const char *message);
bool search_data();
int SLAM(void);
void rtoc_robotPosition(struct robot_position *robot, char *buf, int nmode, int robotID, int fType);
bool Save_Settings();
void restartSet();
void DispVersionInfo();
bool checkInternet();
void PathPlannerSetting(IplImage* Image);

#if defined(TABLET_NON)
// エレベーターホール
//int InitX = 800;//730
//int InitY = 570;//700
//int InitAngle = 0;//135
//int InitX = 830;//730, 900
//int InitY = 210;//700, 160
//int InitAngle = 90;//135, -90
// 1号館本実験
int InitX = 900;//730, 900
int InitY = 160;//700, 160
int InitAngle = -90;//135, -90
// 1号館test
//int InitX = 900;//730, 900
//int InitY = 215;//700, 160
//int InitAngle = 90;//135, -90
// 1goukan 830,210,90
// 7f 770,570,180
#endif
// 1号館本実験
#define MAPFILENAME "1F01_new.png"
#define PATHFILENAME "C:\\kinden\\map\\path2-Building1-1to2mtg.txt"
// エレベーターホール
//#define MAPFILENAME "elevhall-pp.png"
//#define PATHFILENAME "C:\\kinden\\map\\path-hino7F2.txt" //4点:path-hino7F, 仮想扉:path-hino7F2

void ini_Read()
{
	static TCHAR strPath[MAX_PATH];
	TCHAR* last;

//	GetCurrentDirectory(MAX_PATH, strPath);

	GetModuleFileName(NULL, strPath, MAX_PATH);
	last = _tcsrchr(strPath, '\\');
	*last = '\0';

	wsprintf(strPath, TEXT("%s\\%s"), strPath, TEXT(INI_FILE_NAME));

	if (PathFileExists(strPath))
	{
#if defined(_WIN64)
		tcpPort = ::GetPrivateProfileInt("NETWORK", "TCPPORT", TCP_PORT, strPath);
		maxSpeed = ::GetPrivateProfileInt("ROBOT", "MAXSPEED", MAX_OUTPUT, strPath);
		constantSpeed = ::GetPrivateProfileInt("ROBOT", "CONSTANTSPEED", CONSTANT_SPEED, strPath);
		lowSpeed = ::GetPrivateProfileInt("ROBOT", "LOWSPEED", LOW_SPEED, strPath);
		lowLimitSpeed = ::GetPrivateProfileInt("ROBOT", "LOWLIMITSPEED", LOW_LIMIT_SPEED, strPath);
		//fallThresHold = ::GetPrivateProfileInt("ROBOT", "FALLTHRESHOLD", FALL_THRESHOLD, strPath);
		illmPort = ::GetPrivateProfileInt("ROBOT", "ILLUMPORT", ILLUM_PORT, strPath);
		illmPort2 = ::GetPrivateProfileInt("ROBOT", "ILLUMPORT2", ILLUM_PORT2, strPath);
		ardnPort = ::GetPrivateProfileInt("ROBOT", "ARDNPORT", ARDN_PORT, strPath);
		roombaPort = ::GetPrivateProfileInt("ROBOT", "ROOMBAPORT", ROOMBA_PORT, strPath);
		lrfPort = ::GetPrivateProfileInt("ROBOT", "LRFPORT", LRF_PORT, strPath);
		engineering_mode = ::GetPrivateProfileInt("ROBOT", "ENGINEERINGMODE", ENGINEERING_MODE, strPath);
		save_settings = ::GetPrivateProfileInt("ROBOT", "SAVESETTINGS", SAVE_SETTINGS, strPath);
		start_deg = ::GetPrivateProfileInt("ROBOT", "STARTDEG", START_DEG, strPath);
		//TR = ::GetPrivateProfileInt("ROBOT", "TR", (int)(DEF_TR), strPath);
		//TR_o = TR;

		rs16_thread_mode = ::GetPrivateProfileInt("ROBOT", "RS16_THREAD_MODE", RS16_THREAD_MODE, strPath);
		rs16_rpm = ::GetPrivateProfileInt("ROBOT", "RS16_ROTATION_SPEED", RS16_ROTATION_SPEED, strPath);
		switch (rs16_rpm)
		{
		case 300: rs16_NumOfPackage = 125; break;
		case 600: rs16_NumOfPackage = 63; break;
		case 1200: rs16_NumOfPackage = 32; break;
		default: rs16_rpm = RS16_ROTATION_SPEED; rs16_NumOfPackage = 63; break;
		}

		init_time = ::GetPrivateProfileInt("ROBOT", "INITTIME", INIT_TIME, strPath);
		area_length = ::GetPrivateProfileInt("ROBOT", "SEARCHRANGE", AREA_LENGTH, strPath);
		judge_range = ::GetPrivateProfileInt("ROBOT", "JUDGERANGE", JUDGE_RANGE, strPath);
		tr_threshold = ::GetPrivateProfileInt("ROBOT", "TRTHRESHOLD", (int)TR_THRESHOLD_MAX, strPath);

		scaleMode = ::GetPrivateProfileInt("ROBOT", "SCALEMODE", SCALE_MODE, strPath);
		slamMode = ::GetPrivateProfileInt("ROBOT", "SLAMMODE", SLAM_MODE, strPath);
		modelMode = ::GetPrivateProfileInt("ROBOT", "MODELMODE", MODEL_MODE, strPath);

		::GetPrivateProfileString("ROBOT", "SOUND_PATH", SOUND_PATH, strSoundPath, MAX_PATH, strPath);

		char s[32];
		TCHAR strBuf[260];

		snprintf(s, sizeof(s), "%f", GA_SEARCH_RANGE);
		::GetPrivateProfileString("ROBOT", "GA_SEARCH_RANGE", s, strBuf, MAX_PATH, strPath);
		ga_search_range = atof(strBuf);

		//printf("rs16_thread_mode=%d\n", rs16_thread_mode);
		//printf("rs16_rpm=%d\n", rs16_rpm);
		//printf("rs16_NumOfPackage=%d\n", rs16_NumOfPackage);
		//printf("ga_search_range=%f\n", ga_search_range);

		snprintf(s, sizeof(s), "%f", WHEEL_DIA);
		::GetPrivateProfileString("ROBOT", "WHEELDIA", s, strBuf, MAX_PATH, strPath);
		wheelDia = atof(strBuf);

		memset(strBuf, 0, sizeof(strBuf));
		snprintf(s, sizeof(s), "%f", WHEEL_WIDTH);
		::GetPrivateProfileString("ROBOT", "WHEELWIDTH", s, strBuf, MAX_PATH, strPath);
		wheelWidth = atof(strBuf);

		memset(strBuf, 0, sizeof(strBuf));
		snprintf(s, sizeof(s), "%f", LIDAR_WHEEL_DIST);
		::GetPrivateProfileString("ROBOT", "LIDAR_WHEEL_DIST", s, strBuf, MAX_PATH, strPath);
		lidarWheelDist = atof(strBuf);

		//memset(strBuf, 0, sizeof(strBuf));
		//snprintf(s, sizeof(s), "%f", MICRO_TIME);
		//::GetPrivateProfileString("ROBOT", "MICROTIME", s, strBuf, MAX_PATH, strPath);
		//microTime = atof(strBuf);

		memset(strBuf, 0, sizeof(strBuf));
		snprintf(s, sizeof(s), "%f", OUTPUT_COEF_L);
		::GetPrivateProfileString("ROBOT", "OUTPUTCOEFL", s, strBuf, MAX_PATH, strPath);
		outputCoefL = atof(strBuf);

		memset(strBuf, 0, sizeof(strBuf));
		snprintf(s, sizeof(s), "%f", OUTPUT_COEF_R);
		::GetPrivateProfileString("ROBOT", "OUTPUTCOEFR", s, strBuf, MAX_PATH, strPath);
		outputCoefR = atof(strBuf);


		memset(strBuf, 0, sizeof(strBuf));
		snprintf(s, sizeof(s), "%f", FITNESS_TH);
		::GetPrivateProfileString("ROBOT", "FITNESS_TH", s, strBuf, MAX_PATH, strPath);
		fitness_th = atof(strBuf);
		fitness_time = ::GetPrivateProfileInt("ROBOT", "FITNESS_TIME", FITNESS_TIME, strPath);
		fitness_cnt = ::GetPrivateProfileInt("ROBOT", "FITNESS_CNT", FITNESS_CNT, strPath);


		::GetPrivateProfileString("ROBOT", "PC_NAME", PC_NAME, strPcName, MAX_PATH, strPath);

#else 
		tcpPort = ::GetPrivateProfileInt(L"NETWORK", L"TCP_PORT", TCP_PORT, strPath);
		maxSpeed = ::GetPrivateProfileInt(L"ROBOT", L"MAX_SPEED", MAX_OUTPUT, strPath);
		constantSpeed = ::GetPrivateProfileInt(L"ROBOT", L"CONSTANT_SPEED", CONSTANT_SPEED, strPath);
		lowSpeed = ::GetPrivateProfileInt(L"ROBOT", L"LOW_SPEED", LOW_SPEED, strPath);
#endif
	}
	else
	{
#if defined(_WIN64)
		char *tcpPortStr = NULL;
		char *maxSpeedtStr = NULL;
		char *constantSpeedStr = NULL;
		char *lowSpeedStr = NULL;
		char *illuminancePortStr = NULL;
		char *illuminancePort2Str = NULL;
		char *ArduinoPortStr = NULL;
		char *RoombaPortStr = NULL;
		char *LrfPortStr = NULL;

		sprintf(tcpPortStr, "%d", TCP_PORT);
		sprintf(maxSpeedtStr, "%d", MAX_OUTPUT);
		sprintf(constantSpeedStr, "%d", CONSTANT_SPEED);
		sprintf(lowSpeedStr, "%d", LOW_SPEED);
		sprintf(illuminancePortStr, "%d", ILLUM_PORT);
		sprintf(illuminancePort2Str, "%d", ILLUM_PORT2);
		sprintf(ArduinoPortStr, "%d", ARDN_PORT);
		sprintf(RoombaPortStr, "%d", ROOMBA_PORT);
		sprintf(LrfPortStr, "%d", LRF_PORT);

		WritePrivateProfileString("NETWORK", "TCPPORT", tcpPortStr, strPath);
		WritePrivateProfileString("ROBOT", "MAXSPEED", maxSpeedtStr, strPath);
		WritePrivateProfileString("ROBOT", "CONSTANTSPEED", constantSpeedStr, strPath);
		WritePrivateProfileString("ROBOT", "LOWSPEED", lowSpeedStr, strPath);
		WritePrivateProfileString("ROBOT", "ILLUMPORT", illuminancePortStr, strPath);
		WritePrivateProfileString("ROBOT", "ILLUMPORT2", illuminancePort2Str, strPath);
		WritePrivateProfileString("ROBOT", "ARDNPORT", ArduinoPortStr, strPath);
		WritePrivateProfileString("ROBOT", "ROOMBAPORT", RoombaPortStr, strPath);
		WritePrivateProfileString("ROBOT", "LRFPORT", LrfPortStr, strPath);
#else
		LPWSTR tcpPortStr = NULL;
		LPWSTR maxSpeedtStr = NULL;
		LPWSTR constantSpeedStr = NULL;
		LPWSTR lowSpeedStr = NULL;

		wsprintf(tcpPortStr, L"%d", TCP_PORT);
		wsprintf(maxSpeedtStr, L"%d", MAX_OUTPUT);
		wsprintf(constantSpeedStr, L"%d", CONSTANT_SPEED);
		wsprintf(lowSpeedStr, L"%d", LOW_SPEED);

		WritePrivateProfileString(L"NETWORK", L"TCPPORT", tcpPortStr, strPath);
		WritePrivateProfileString(L"ROBOT", L"MAXSPEED", maxSpeedtStr, strPath);
		WritePrivateProfileString(L"ROBOT", L"CONSTANTSPEED", constantSpeedStr, strPath);
		WritePrivateProfileString(L"ROBOT", L"LOWSPEED", lowSpeedStr, strPath);
#endif
	}
}

//
// Parameter.ini読み込み
//
void readParameter()
{
		static TCHAR strPath[MAX_PATH];
		TCHAR* last;

		GetModuleFileName(NULL, strPath, MAX_PATH);
		last = _tcsrchr(strPath, '\\');
		*last = '\0';

		wsprintf(strPath, TEXT("%s\\%s"), strPath, TEXT(FILE_NAME));

		if (PathFileExists(strPath))
		{
			::GetPrivateProfileString("USER", "CONSTRUCTION_SITE", CONSTRUCTION_SITE, strConstructionSite, MAX_PATH, strPath);
			edge_cad = ::GetPrivateProfileInt("USER", "EDGE_CAD", EDGE_CAD, strPath);
			fall_sensor = ::GetPrivateProfileInt("USER", "FALL_SENSOR", FALL_SENSOR_ENABLE, strPath);
			fallThresHold = ::GetPrivateProfileInt("USER", "FALL_SENSOR_TH", FALL_SENSOR_TH, strPath);
			run_finish_notice = ::GetPrivateProfileInt("USER", "RUN_FINISH_NOTICE", RUN_FINISH_NOTICE, strPath);
			fall_sensor_notice = ::GetPrivateProfileInt("USER", "FALL_SENSOR_NOTICE", FALL_SENSOR_NOTICE, strPath);
			run_difficult_notice = ::GetPrivateProfileInt("USER", "RUN_DIFFICULT_NOTICE", RUN_DIFFICULT_NOTICE, strPath);
			running_notice = ::GetPrivateProfileInt("USER", "RUNNNING_NOTICE", RUNNNING_NOTICE, strPath);
			running_notice_time = ::GetPrivateProfileInt("USER", "RUNNNING_NOTICE_TIME", RUNNNING_NOTICE_TIME, strPath);

			::GetPrivateProfileString("USER", "MAIL_1", MAIL1, strMail1, MAX_PATH, strPath);
			::GetPrivateProfileString("USER", "MAIL_2", MAIL1, strMail2, MAX_PATH, strPath);
			::GetPrivateProfileString("USER", "MAIL_3", MAIL1, strMail3, MAX_PATH, strPath);
			::GetPrivateProfileString("USER", "MAIL_4", MAIL1, strMail4, MAX_PATH, strPath);
			::GetPrivateProfileString("USER", "MAIL_5", MAIL1, strMail5, MAX_PATH, strPath);

			::GetPrivateProfileString("MAIL", "SMTP_SERVER", MAIL1, strSMTPServer, MAX_PATH, strPath);
			::GetPrivateProfileString("MAIL", "SMTP_PASSWORD", MAIL1, strSMTPPassword, MAX_PATH, strPath);
			::GetPrivateProfileString("MAIL", "SEND_MAIL_ADDRESS", MAIL1, strSMTPMailAddress, MAX_PATH, strPath);
			SMTPPort = ::GetPrivateProfileInt("MAIL", "SMTP_PORT", SMTP_PORT, strPath);
		}

		//BOXディレクトリパス作成
		char dirBuf1[255] = "\\Box\\";

		strcpy(BoxPath, "C:\\Users\\");
		strcat(BoxPath, strPcName);
		strcat(BoxPath, dirBuf1);
		strcat(BoxPath, strConstructionSite);

}

//メール送信処理
bool sendMail(MAIL_STATUS kind)
{
	// モバイル回線未接続時は処理しない
	if (isOnline == false) return false;

	bool ret = true;

	std::string strSubject = "";
	std::string strBody = "";
	std::string  now_count(std::to_string(cpath_ct));		//測定完了数
	std::string  total_count(std::to_string(path_ct));		//全測定数

	static TCHAR strModulePath[MAX_PATH];
	static TCHAR strSetPath[MAX_PATH];
	static TCHAR strExePath[MAX_PATH];
	TCHAR* last;

	GetModuleFileName(NULL, strModulePath, MAX_PATH);
	last = _tcsrchr(strModulePath, '\\');
	*last = '\0';

	wsprintf(strSetPath, TEXT("%s\\%s"), strModulePath, TEXT(SETTING_FILE_NAME));

	TCHAR name[MAX_PATH];
	std::string robot_name;
	::GetPrivateProfileString("ROBOT", "ROBOT_NAME", "照度測定ロボット", name, MAX_PATH, strSetPath);
    robot_name = std::string(name);

	switch (kind)
	{
	case RUN_FINISH_MAIL:
		strSubject = "【" + robot_name + "】走行完了通知";
		strBody = robot_name + "が走行を完了しました。";
		break;
	case FALL_SENSOR_MAIL:
		strSubject = "【" + robot_name + "】落下防止センサ反応通知";
		strBody = robot_name + "の落下防止センサが反応したため走行を中止しました。<n>確認をお願いします。";
		break;
	case RUN_DIFFICULT_MAIL:
		strSubject = "【" + robot_name + "】走行継続困難通知";
		strBody = robot_name + "の走行継続が困難な状態になっています。<n>走行を中止し、再走行をお願いします。";
		break;
	case RUNNING_MAIL:
		strSubject = "【" + robot_name + "】走行継続中通知";
		strBody = robot_name + "走行中の定期通知です。<n>" + now_count + "/" + total_count + " 完了しました。";
		break;
	default:
		break;
	}

	controlMotor(0, 0);	//メール送信に時間が掛かるのでモータを一旦停止

	// 本文中の"<n>"は、smtpsend.exe で"\n"に変換される
	std::string message = "\"" + std::string(strModulePath) + "\\smtpsend.exe " + "\"" + strSubject + "\" " + "\"" + strBody + "\"";
	int iret = system(message.c_str());
	if (iret != 0) ret = false;

	return ret;
}

//
// 照度データ保存
//
int illuminance_save(double ill_data, char fileName[255], char tmpFileName[255])
{
	try
	{
		int x = (int)((robot_pos->map_x - map_offset) / (TR / map_rate));		//オリジナル図面データの座標に変換
		int y = (int)((robot_pos->map_y - map_offset) / (TR / map_rate));		//オリジナル図面データの座標に変換
		FILE *fp = fopen(fileName, "a");
		fprintf(fp, "%d,%d,%d,%d\n", measurementId[cpath_ct -1], x, y, (int)ill_data);
		fclose(fp);

		fp = fopen(tmpFileName, "a");
		fprintf(fp, "%d,%d,%d,%d\n", measurementId[cpath_ct - 1], x, y, (int)ill_data);
		fclose(fp);

		return 1;
	}
	catch (...)
	{
		printf("Illuminance Value Save Error\n");
		return -1;
	}

	return 0;
}

//
// 軌跡データ保存
//
void trajectory_save()
{
	try
	{
		int x = (int)((robot_pos->map_x - map_offset) / (TR / map_rate));
		int y = (int)((robot_pos->map_y - map_offset) / (TR / map_rate));
		FILE *fp = fopen("C:\\kinden\\log\\trajectory.csv", "w");		//最新の1つだけを保存
		fprintf(fp, "%d,%d\n", x, y);
		fclose(fp);

		if (engineering_mode) 
		{
			FILE *fp = fopen("C:\\kinden\\log\\trajectory_all.csv", "a");		//全ての軌跡を保存
			//FILE *fp = fopen("C:\\kinden\\result\\csv\\trajectory_0903_1.csv", "a");		//TMU20190821
			fprintf(fp, "%d,%d\n", x, y);
			fclose(fp);
		}

	}
	catch (...)
	{
		printf("Trajectory Value Save Error\n");
	}
}

void Play_Sound(bool playmode)
{
	static bool mode = false;

	if (!mode && playmode)
	{
		// 再生開始
		if (!PlaySound(strSoundPath, NULL, SND_ASYNC | SND_LOOP)) {//SND_LOOP,SND_SYNC
			std::cout << "再生できません" << std::endl;
		}
		mode = playmode;
	}
	if(mode && !playmode)
	{
		// 再生停止
		PlaySound(NULL, NULL, 0);
		mode = playmode;
	}
}

//外部ネットワーク接続確認
bool checkInternet()
{
	bool ret = true;

	HINTERNET hInternet;
	HINTERNET hInetFile;

	// InternetOpenでWinInetを初期化して, 戻り値のインターネットハンドルをhInternetに格納する.
	hInternet = InternetOpen(_T(" Sample InternetOpen"), INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0);
	// 指定のURLを開く.ここではgoogleにアクセス.
	hInetFile = InternetOpenUrl(hInternet, _T("https://www.google.com/?hl=ja"), NULL, 0, INTERNET_FLAG_RELOAD, 0);

	if (hInetFile == NULL)
	{
		printf("Internet not Connect.\n");
		ret = false;
	}
	else
	{
		printf("Internet Connect.\n");
		ret = false;//add
	}

	// URLハンドルを閉じる.
	InternetCloseHandle(hInetFile);
	// インターネットハンドルを閉じる.
	InternetCloseHandle(hInternet);

	return ret;
}

bool copyFile(const char* srcPath)
{
	bool ret = true;
	char destPath[255];

	char dirBuf[255] = "\\kinden\\result\\";

	strcpy(destPath, BoxPath);
	strcat(destPath, dirBuf);
	strcat(destPath, nameBuf);

	if (CopyFileA(srcPath, destPath, FALSE) == false)
	{
		ret = false;
	}

	return ret;
}

int main()
{
	DispVersionInfo();// アプリバージョン情報表示

	// メール送信テスト用
	//bool ret;
	//ret = sendMail(RUN_FINISH_MAIL);
	//if (ret != true) printf("Mail send error\r\n");
	//ret = sendMail(FALL_SENSOR_MAIL);
	//if (ret != true) printf("Mail send error\r\n");
	//ret = sendMail(RUN_DIFFICULT_MAIL);
	//if (ret != true) printf("Mail send error\r\n");
	//ret = sendMail(RUNNING_MAIL);
	//if (ret != true) printf("Mail send error\r\n");

#if defined(MOTOR_TEST)
#if !defined(MOTOR_NON)
	// 設定値読出し
	ini_Read();

	printf("\nMotor test start.\n");
	iRobotInitialise(); // Motor initialization
	Beep(554, 300);//ミ
	Sleep(50);
	Beep(554, 300);//ミ
	Sleep(50);
	// 停止
	controlMotor(0, 0);
	Sleep(3000);
	// 前進
	printf("Forward\n");
	controlMotor(lowSpeed, lowSpeed);
	Sleep(4000);
	// 後退
	printf("Backward\n");
	controlMotor(-lowSpeed, -lowSpeed);
	Sleep(4000);
	// 右旋回
	printf("Turn right\n");
	controlMotor(-lowSpeed, lowSpeed);
	Sleep(4000);
	// 左旋回
	printf("Turn left\n");
	controlMotor(lowSpeed, -lowSpeed);
	Sleep(4000);
	// 停止
	printf("Motor stop\n");
	controlMotor(0, 0);
	printf("Motor test end.\n\n");
	Beep(440, 400);//ド
	Sleep(50);
	Beep(440, 400);//ド
	Sleep(50);
	Sleep(3000);
#endif
#endif

	while (true)
	{
		int init_sensor;
		bool sensor_chk = true;
		onTick = false;
		onTick2 = true;
		srand((unsigned)time(NULL));

		robot_pos = (struct robot_position*)malloc(sizeof(struct robot_position));
		memset(robot_pos, 0, sizeof(robot_pos));
		//initialize_RobotPosition(robot_pos, 0);
		//initial_Position = (struct es_gene*)malloc(sizeof(struct es_gene));
		//initial_Position->best_gene = 0;
		//initial_Position->wei_ave = 0;
		//initial_Position->lo_count = 0;

		RobotStatus = STANDBY;
		StatusSet(RobotStatus);

		// 設定値読出し
		ini_Read();
		//Parameter.ini読み込み
		readParameter();

		//readLRF();//henkoIP
		//readLRF2();//henkoIP

		//Play_Sound(1, strSoundPath);
		//Sleep(5000);
		//Play_Sound(0, NULL);

		//Save_Settings(); //debug

		// 照度計測定開始
		if (restartFlg == false)//再スタートの場合は周辺機器の接続処理はスキップ
		{
#if !defined(ILLM_NON)
			if (T10A_Initial() == 0)
			{
				std::thread th_i(illuminanceThread);
				th_i.detach();
			}
#endif

			// バンパ・落下センサ収集開始
#if !defined(SENSOR_NON)
			if (sensorComm_Init() == 0)
			{
				std::thread th_s(SensorDataReceive);
				th_s.detach();
			}
#else
			//std::thread th_s(SensorDataReceive);
			//th_s.detach();
#endif

			// PipeComm開始
			//std::thread th_a(PipeCommReceive);
			//th_a.detach();
			//memset(PipeReceive, 0, sizeof(PipeReceive));
		}
		// LRF準備
		FILE* dfp = fopen("C:\\kinden\\log\\LRF.dat", "w");
		if (!dfp)
		{
			RobotStatus = LRF_ERR;
			StatusSet(RobotStatus);
		}
		else
		{
			fclose(dfp);
		}

		if (restartFlg == false)//再スタートの場合は周辺機器の接続処理はスキップ
		{
//#if !defined(TABLET_NON)
			//ServerInitialise();

			//std::thread th_s(socketMain);
			//th_s.detach();
//#endif

#if !defined(MOTOR_TEST)
#if !defined(MOTOR_NON)
			iRobotInitialise(); //Roomba initialization
#endif
#endif
		}
#if defined(_ROOMBA)	// ルンバ実験用コマンド
		roombaStart();		//Activate roomba
		Sleep(50);
		roombaSafe();		//Set roomba to safe mode
		Sleep(50);
#endif

		if (restartFlg == false)//再スタートの場合は周辺機器の接続処理はスキップ
		{
#if !defined(LRF_NON)
			//RS-LiDAR用
			if (SENSOR_MODE == 0) {
				search_data();	// UTM-30LX 接続
			}
			else {
				bool ret = RS_Initial();	// RS-Lidar-16 接続
				if (ret == false) {
					RobotStatus = LRF_COMM_ERR;
					StatusSet(RobotStatus);
				}
				else {
					if (rs16_thread_mode == 1) {
						startThread();
					}
				}
			}
#endif
		}

		sprintf(resultViewflie, "C:\\kinden\\result\\resultView.csv");		//iPad表示用のtmp結果ファイル
		remove(resultViewflie);		//iPad表示用のtmp結果ファイルの削除

	//起動音
		Beep(440, 400);//ド
		Sleep(50);
		Beep(494, 300);//レ
		Sleep(50);
		Beep(554, 300);//ミ
		Sleep(50);

#if !defined(TABLET_NON)
		if (RobotStatus >= USB_CONNECT_ERR)
		{
			//周辺機器の接続エラーはロボットの再起動になるのでここで待ち
			while (true) Sleep(1000);
		}
#endif

#if !defined(SENSOR_NON)
		while (sensor_chk)		//初期化前にセンサチェックを行い、が反応していたら通知し反応がなくなるまで初期化は進まない。
		{
			Sleep(1000);
			init_sensor = getBumpData();
			switch (init_sensor)
			{
			case 1:		// 右バンパ接触
				RobotStatus = BUMP_SENSOR;
				sensor_chk = true;
				break;
			case 2:		// 左バンパ接触
				RobotStatus = BUMP_SENSOR;
				sensor_chk = true;
				break;
			case 3:		// 中央(左右同時)バンパ接触
				RobotStatus = BUMP_SENSOR;
				sensor_chk = true;
				break;
			case 4:		// 右落下センサ反応
				RobotStatus = FALL_SENSOR;
				sensor_chk = true;
				break;
			case 8:		// 左落下センサ反応
				RobotStatus = FALL_SENSOR;
				sensor_chk = true;
				break;
			default:	//バンパ、落下センサの反応がなければスタンバイに戻す
				RobotStatus = STANDBY;
				sensor_chk = false;
				break;
			}

			StatusSet(RobotStatus);
		}
#endif
		//while (true)
		//{
			// ロボット設置位置初期化
		memset(stratPosition, 0x00, sizeof(stratPosition));

		//if (RobotStatus = END)
		//{
		//	RobotStatus = STANDBY;
		//	StatusSet(RobotStatus);
		//}

#if !defined(TABLET_NON)
		// Mapファイル取得待ち
		MapFileNameReceive();
#else
		//sprintf(mapFileName, "gym.png");
		sprintf(mapFileName, MAPFILENAME);
		//sprintf(mapFileName, "elevhall-pp.png");
		stratPosition[0] = InitX;
		stratPosition[1] = InitY;
		stratPosition[2] = InitAngle;
#endif
		// Mapファイル情報取得により「開始」ボタン押下を
		//RobotStatus = READY;
		//StatusSet(RobotStatus);


		//Parameter.ini読み込み
		readParameter();

		if (running_notice)
		{
			runningStartTime = 0;
			runningEndTime = 0;

			runningStartTime = clock();
		}
		//mapFileName->mapFileNameEdgeの作成 henko2
		if (edge_cad)
		{
			INT64 point2;
			char* tp2;
			char nameBuf2[255];
			char str12 = '.';
			char fileExtension2[12] = "_edge.png";
			tp2 = strrchr(mapFileName, str12);// ファイル名の最後から"."を検索する
			point2 = tp2 - mapFileName;
			memcpy(nameBuf2, mapFileName, point2);
			memcpy(mapFileNameEdge, nameBuf2, point2);
			memcpy(mapFileNameEdge + point2, fileExtension2, sizeof(fileExtension2));
		}
		//debug
		//RobotStatus = READY;
		//StatusSet(RobotStatus);
		//ここまで

		// Mapファイル読出し
		char pathMap[255];
		char mapBoxDirBuf[255] = "\\kinden\\map\\";

		isOnline = checkInternet();
		if (isOnline == false)
		{
			//外部ネットワーク接続エラーだったらロカールフォルダから
			strcpy(BoxPath, "C:");
			isInternetConnect = false;
			sprintf(pathMap, "C:\\kinden\\map\\%s", mapFileName);
			printf("Map Path:%s\n", pathMap);
			limage = cvLoadImage(pathMap, CV_LOAD_IMAGE_COLOR);
		}
		else
		{
			//外部ネットワーク接続できていればBOXフォルダから
			isInternetConnect = true;
			sprintf(pathMap, "%s", BoxPath);
			strcat(pathMap, mapBoxDirBuf);
			strcat(pathMap, mapFileName);
			printf("Map Path:%s\n", pathMap);
			limage = cvLoadImage(pathMap, CV_LOAD_IMAGE_COLOR);
		}

		if (limage == 0)
		{
			printf("Image file(%s) open error!\n", pathMap);
			RobotStatus = MAP_FILE_ERR;
			StatusSet(RobotStatus);
			restartSet();
			continue;
		}

		// GNG&Path Planning 準備

#if !defined(TABLET_NON)
		// TR読出し
		std::string rcv_buff;
		double tr_buf = tr;		//cmd.txtから読み出したTR値

		//while (true)
		//{
			//rcv_buff = std::string(PipeReceive);
			//tr_buf = std::stod(rcv_buff);
		if (tr_buf != 0)
		{
			TR = TR_o = tr_buf;
			printf("TR = %lf mm/px\n", TR);

			if ((TR < TR_THRESHOLD_MIN) || (TR > TR_THRESHOLD_MAX))		//計算されたTR値が範囲外の場合、走行しないでエラー通知
			{
				printf("TR value is out of range.  TR=%f\n", TR);
				RobotStatus = TR_VALUE_ERR;
				StatusSet(RobotStatus);
				restartSet();
				continue;
			}
		}
		else
		{
			printf("TR value is 0  \n");
			RobotStatus = TR_VALUE_ERR;
			StatusSet(RobotStatus);
			restartSet();
			continue;
		}
		//}
#endif
		// 計測ポイントファイル読出し
		char pathname[255];
		sprintf(pathname, PATHFILENAME);
		//sprintf(pathname, "C:\\kinden\\map\\path-hino7F2.txt");
		FILE* mfp3 = fopen(pathname, "r");
		if (mfp3 == NULL)
		{
			printf("Path file(%s) open error!\n", pathname);
			RobotStatus = POINT_FILE_ERR;
			StatusSet(RobotStatus);
			restartSet();
			continue;
		}

		if (area_length_tmp != 0) area_length = area_length_tmp;
		if (judge_range_tmp != 0) judge_range = judge_range_tmp;
		printf("Search Range = %d mm\n", (int)area_length);
		printf("Position Gap = %d mm\n", (int)judge_range);
		printf("Fall Range = %d cm\n", (int)fallThresHold);

		//LRFエリア読み込み
		readLRF();
		readLRF2();

		//CAD画像から格子地図への変換----------------------------------------------------------------------------
		int rx, ry;
		int px, py;
		int id;
		path_ct = 0;
		while (fscanf(mfp3, "%d\t%d\t%d\n", &rx, &ry, &id) != EOF)
		{
			rx = (int)((double)rx * TR);
			ry = (int)((double)ry * TR);
			px = (int)(rx / map_rate);
			py = (int)(ry / map_rate);
			fpath[path_ct][0] = px + map_offset;
			fpath[path_ct][1] = py + map_offset;

			measurementId[path_ct] = id;

			path_ct++;
		}
		fclose(mfp3);

		//memset(map, 0, sizeof(map));
		//memset(map2, 0, sizeof(map2));
		//memset(cad_map, 0, sizeof(cad_map));
		//memset(cad_map2, 0, sizeof(cad_map2));

		width = (int)(limage->width * TR / map_rate);
		height = (int)(limage->height * TR / map_rate);
		if ((width * height) > SLAM_MAP_SIZE_MAX)
		{
			printf("Image file(%s) open error!(size over)\n", pathMap);
			RobotStatus = MAP_FILE_ERR;
			StatusSet(RobotStatus);
			restartSet();
			continue;
		}

		map_size_width = width;
		map_size_height = height;
		baseMap.width = width;
		baseMap.height = height;
		baseMap.map = malloc2d_int(width, height);
		baseMap.map2 = malloc2d_int(width, height);
		baseMap.cad_map = malloc2d_int(width, height);
		baseMap.cad_map2 = malloc2d_int(width, height);
		if (edge_cad)
		{
			baseMap.edge_map = malloc2d_int(width, height);
			baseMap.edge_map2 = malloc2d_int(width, height);
		}
		map_center_width = (int)(map_size_width / 2);
		map_center_height = (int)(map_size_height / 2);

		printf("TR: %f [mm/pixel], MAPRATE: %f [mm/grid]\n", TR, map_rate);
		printf("CADsize: %d x %d [pixel]\n", limage->width, limage->height);
		printf("MAPsize: %d x %d [grid]\n", map_size_width, map_size_height);

		for (int i = 0; i < width; i++)
		{
			for (int j = 0; j < height; j++)
			{
				baseMap.map[i][j] = 0;
				baseMap.map2[i][j] = 0;
				baseMap.cad_map[i][j] = 0;
				baseMap.cad_map2[i][j] = 0;
				if (edge_cad)
				{
					baseMap.edge_map[i][j] = 0;
					baseMap.edge_map2[i][j] = 0;
				}
			}
		}

		initialize_RobotPosition(robot_pos, 0);
		initial_Position = (struct es_gene*)malloc(sizeof(struct es_gene));
		initial_Position->best_gene = 0;
		initial_Position->wei_ave = 0;
		initial_Position->lo_count = 0;


		for (int j = 0; j < limage->width; j++)
		{
			for (int i = 0; i < limage->height; i++)
			{
				int k = (unsigned char)limage->imageData[limage->widthStep * i + j * 3];
				int l = (unsigned char)limage->imageData[limage->widthStep * i + j * 3 + 1];
				int m = (unsigned char)limage->imageData[limage->widthStep * i + j * 3 + 2];


				rx = (int)((double)j * TR);
				ry = (int)((double)i * TR);
				px = (int)(rx / map_rate);
				py = (int)(ry / map_rate);


				if (k > 200 && l < 200 && m > 200)
				{
					//何もしない？
				}

				if (k < 200)
				{
					baseMap.map[px + map_offset][py + map_offset] = 100;
					baseMap.map2[px + map_offset][py + map_offset] = 100;
					//henko2
					baseMap.cad_map[px + map_offset][py + map_offset] = 100;
					baseMap.cad_map2[px + map_offset][py + map_offset] = 100;
				}
			}
		}

		// 結果保存ファイル名作成
		INT64 point;
		char* tp;
		//char nameBuf[255];
		char illuminanceflie[255];
		char str1 = '.';
		char dirName[22] = "C:\\kinden\\result\\";
		char fileExtension[12] = "_result.csv";
		
		memset(illuminanceflie, 0x00, sizeof(illuminanceflie));
		memset(nameBuf, 0x00, sizeof(nameBuf));

		tp = strrchr(mapFileName, str1);// ファイル名の最後から"."を検索する
		point = tp - mapFileName;

		time_t now = time(NULL);
		struct tm* pnow;
		pnow = localtime(&now);
		char timeBuf[14];
		strftime(timeBuf, 255, "%Y%m%d%H%M%S", pnow);

		char idBuf[64];
		sprintf(idBuf, "%d", measurementId[0]);

		memcpy(nameBuf, mapFileName, point);
		memcpy(nameBuf + point, "_", 1);								//ファイル名 + _
		point += 1;
		memcpy(nameBuf + point, timeBuf, strlen(timeBuf));				//ファイル名 + _ + YYYYMMDDhhmmss
		point += 14;
		memcpy(nameBuf + point, "_", 1);								//ファイル名 + _ + YYYYMMDDhhmmss + _  
		point += 1;
		memcpy(nameBuf + point, idBuf, strlen(idBuf));					//ファイル名 + _ + YYYYMMDDhhmmss + _ + 先頭ID番号 
		point += strlen(idBuf);
		memcpy(nameBuf + point, fileExtension, strlen(fileExtension));	//ファイル名 + _ + YYYYMMDDhhmmss + _ + 先頭ID番号 + _result.csv

		memcpy(illuminanceflie, dirName, strlen(dirName));						//ディレクトリパス
		memcpy(illuminanceflie + strlen(dirName), nameBuf, sizeof(nameBuf));	//ディレクトリパス + ファイル名

		//memcpy(illuminanceflie, dirName, sizeof(dirName) - 1);	//ディレクトリURL
		//memcpy(illuminanceflie + 21, nameBuf, point);			//マップファイル名(拡張子除く)
		//memcpy(illuminanceflie + 21 + point, "_", 1);
		//memcpy(illuminanceflie + 21 + point + 1, timeBuf, sizeof(timeBuf));	//ファイル作成時刻
		//memcpy(illuminanceflie + 21 + point + 1 + sizeof(timeBuf), "_", 1);
		//memcpy(illuminanceflie + 21 + point + 1 + sizeof(timeBuf) + 1, idBuf, strlen(idBuf));	//先頭ID番号
		//memcpy(illuminanceflie + 21 + point + 1 + sizeof(timeBuf) + 1 + strlen(idBuf), fileExtension, sizeof(fileExtension));

		printf(illuminanceflie);
		printf("\n");

		//sprintf(resultViewflie, "C:\\kinden\\result\\resultView.csv");		//iPad表示用のtmp結果ファイル
		//remove(resultViewflie);		//iPad表示用のtmp結果ファイルの削除


		//デバッグ用　仮結果ファイル作成とBOXコピー処理
		//if (cpath_ct == 0) cpath_ct = 1;
		//illuminance_save(999, illuminanceflie, resultViewflie);
		//copyFile(illuminanceflie);
		//ここまで


		// ロボットの大まかな初期値の入力----------------------------------------------------------------------------
		//while (true)	// ロボット設置位置の更新待ち
		//{
		//	if (stratPosition[0] != 0 && stratPosition[1] != 0)
		//	{
		//		break;
		//	}
		//}

		if (stratPosition[0] <= 0 || stratPosition[1] <= 0)
		{
			printf("Robot Initial Position Error! X=%d, Y=%d\n", stratPosition[0], stratPosition[1]);
			RobotStatus = STRAT_POINT_ERR;
			StatusSet(RobotStatus);
			restartSet();
			continue;
		}

		initial_Position->init_pos[0] = ((double)stratPosition[0] * TR) / map_rate + map_offset; // = 850
		initial_Position->init_pos[1] = ((double)stratPosition[1] * TR) / map_rate + map_offset; // = 950;

		if (scaleMode == 0)
		{
			printf("Robot Initial Position : X = %d / Y = %d\n", stratPosition[0], stratPosition[1]);
		}
		else
		{
			printf("Robot Initial Position : X = %d / Y = %d\n", (int)initial_Position->init_pos[0], (int)initial_Position->init_pos[0]);
		}

		if (start_deg == 1)
		{
			initial_Position->init_pos[2] = (double)stratPosition[2];
			printf("Robot Angle = %lf\n", initial_Position->init_pos[2]);
		}
		else
		{
			//initial_Position->init_pos[2] = AREA_ANGLE; //TMU20190821 図面上での向きの指定：上(0),右(90),下(180),左(270)//henkoIPz
		}

		// Startコマンド待ち
		RobotStatus = READY;
		StatusSet(RobotStatus);
#if !defined(TABLET_NON)
		while (true)
		{
			rcvBuff = std::string(PipeReceive);
			command = rcvBuff.find("Start");
			if (command != std::string::npos)
			{
				Beep(740, 200);
				break;
			}
		}
#endif
		//edgeCAD図生成と初期位置推定ステータス
		RobotStatus = SEARCH;
		StatusSet(RobotStatus);


		//初期位置を使ってエッジ抽出 henko2
		if (edge_cad)
		{
			sprintf(pathMap, "C:\\kinden\\log\\%s", mapFileNameEdge);
			printf("\nget Edge...\n\n");
			int ret = getEdge(BoxPath, mapFileName, mapFileNameEdge, stratPosition);
			if (ret == -1)
			{
				// 停止ボタンによる中止
				restartSet();
				continue;
			}
			limage2 = cvLoadImage(pathMap, CV_LOAD_IMAGE_COLOR);

			if (limage2 == 0)  //変更 henko2
			{
				printf("Image file(%s) open error!\n", pathMap);
				RobotStatus = MAP_FILE_ERR;
				StatusSet(RobotStatus);
				restartSet();
				continue;
			}
			//henko2
			//memset(edge_map, 0, sizeof(edge_map));
			//memset(edge_map2, 0, sizeof(edge_map2));

			for (int j = 0; j < limage2->width; j++)
			{
				for (int i = 0; i < limage2->height; i++)
				{
					int k = (unsigned char)limage2->imageData[limage2->widthStep * i + j * 3];
					int l = (unsigned char)limage2->imageData[limage2->widthStep * i + j * 3 + 1];
					int m = (unsigned char)limage2->imageData[limage2->widthStep * i + j * 3 + 2];


					rx = (int)((double)j * TR);
					ry = (int)((double)i * TR);
					px = (int)(rx / map_rate);
					py = (int)(ry / map_rate);


					if (k > 200 && l < 200 && m > 200)
					{
						//何もしない？
					}

					if (k < 200)
					{
						baseMap.edge_map[px + map_offset][py + map_offset] = 100;
						baseMap.edge_map2[px + map_offset][py + map_offset] = 100;
					}
				}
			}
		}
		// 走行設定ファイルの保存 ------------------------------------------------------------------------------
#if 1
		if (save_settings == 1)
		{
			if (Save_Settings() == false)
			{
				printf("Setting File Save Fault!\n");
				RobotStatus = SAVE_SETTING_ERR;
				StatusSet(RobotStatus);

				restartSet();
				continue;
			}
		}
#endif

		//Multi-resolution Mapの作成----------------------------------------------------------------------------
		//root = new multi_map;
		root = NULL;
		for (int i = 0; i < 5; i++)
		{
			//root = reduce_resolution(root, cad_map);  //henko2
			if (edge_cad)
			{
				root = reduce_resolution(root, baseMap.edge_map);  //henkoIP3
			}
			else
			{
				root = reduce_resolution(root, baseMap.cad_map);
			}
		}

		cmap = root;
		double h;
		while (cmap->reso != 1)
		{
			for (int i = 0; i < MAPSIZE_WIDTH; i++)
			{
				for (int j = 0; j < MAPSIZE_HIGHT; j++)
				{
					h = -cmap->anorm[i][j] - cmap->norm[i][j];
				}
			}
			cmap = cmap->parent;
		}

		viewImage2 = cvCreateImage(cvSize(baseMap.width, baseMap.height), IPL_DEPTH_8U, 3);
		mapImage = cvCreateImage(cvSize(baseMap.width, baseMap.height), IPL_DEPTH_8U, 3);
		for (int i = 0; i < 10; i++)
		{
			multimapImage[i] = cvCreateImage(cvSize(baseMap.width, baseMap.height), IPL_DEPTH_8U, 3);
		}
		draw_globalMap(baseMap.cad_map, baseMap.cad_map2, viewImage2, mapImage, multimapImage);  //henko2

		if (scaleMode == 0)
		{
			cmap = root;//henkoSCALE
		}

		// 自律移動開始 ---------------------------------------------------------------------------------
		// SLAMに必要な変数を初期化
		initPosiJugde = false;
		lflag = 0;
		flag = 5;
		cpath_ct = 0;

		slam_On = false;
		mapping_On = false;
		moveMode = 5;
		char rbuf[255];
		//char tbuf[255];
		//DWORD numberOfRead;
		int rct = 0;
		int sflag = 0;
		slam_eror_count = 0;
		//std::string rcvBuff;
		//std::string::size_type command;

		//// Startコマンド待ち
		//RobotStatus = READY;
		//StatusSet(RobotStatus);
		//while (true)
		//{
		//	rcvBuff = std::string(PipeReceive);
		//	command = rcvBuff.find("Start");
		//	if (command != std::string::npos)
		//	{
		//		Beep(740, 200);
		//		break;
		//	}
		//}

#if defined(ILLM_NON)	// 照度計未装着時にダミーデータ挿入
		ill_data = (int)(230 + 6.0 * rndn());
#endif

		InitPosiSerchStartTime = time(NULL);		//自己位置推定開始時間

		//追加 henko2
		time_t startTime, endTime;
		FILE *fp = fopen("C:\\kinden\\log\\time.csv", "w");
		fclose(fp);
		FILE *fp1 = fopen("C:\\kinden\\log\\SLAMPos.csv", "w");
		fclose(fp1);
		FILE* mfpTR = fopen("C:\\kinden\\log\\TR.txt", "w");
		fclose(mfpTR);

		if (engineering_mode)
		{
			FILE* fp = fopen("C:\\kinden\\log\\trajectory_all.csv", "w");
			fclose(fp);
		}


		while (true)
		{
			if (running_notice)
			{
				runningEndTime = clock();

				if ((runningEndTime - runningStartTime) > (running_notice_time * 60 * 1000))	//分をミリ秒換算
				{
					sendMail(RUNNING_MAIL);
					runningStartTime = runningEndTime;
				}
			}



			startTime = clock();  //追加 henko2

			// Stopコマンドによる自律走行の停止
			rcvBuff = std::string(PipeReceive);
			command = rcvBuff.find("Stop");
			if (command != std::string::npos)
			{
				break;
			}

			// PauseとResumeコマンドによる一時停止と再開
			rcvBuff = std::string(PipeReceive);
			command = rcvBuff.find("Pause");
			if (command != std::string::npos)
			{
				controlMotor(0, 0);
				RobotStatus = STOP;
				StatusSet(RobotStatus);
				while (true)
				{
					rcvBuff = std::string(PipeReceive);
					command = rcvBuff.find("Resume");
					if (command != std::string::npos)
					{
						break;
					}
				}
			}

			if (moveMode == 11)
			{
				RobotStatus = RUN;

				StatusSet(RobotStatus);
			}

			//自己位置推定----------------------------------------------------------------------------
			sflag = SLAM();
			//WriteFile(hCom2, &writeByte, 8, NULL, &o);
			rct = 0;
			memset(rbuf, 0, sizeof(rbuf));

			//照度データの受取----------------------------------------------------------------------------
			// 照度計データは別スレッドにて常時取得している

#if defined(ILLM_NON)	// 照度計未装着時にダミーデータ挿入
			illmData = ill_data;
#endif

			//計測結果の書き込み
			//printf("%d\t%d\t%lf\n", robot_pos->map_x, robot_pos->map_y, illmData); TMU20190819 非表示化
			if (sflag == 1)
			{
				RobotStatus = MEASUREMENT;
				StatusSet(RobotStatus);
				illuminance_save(illmData, illuminanceflie, resultViewflie);
				Beep(830, 300);//測定音
				sflag = 0;
			}

			// エンジニアリングモードによる軌跡データの出力
			if (moveMode == 11)
			{
				trajectory_save();
			}
			//if (engineering_mode)
			//{
			//	trajectory_save();
			//}

			if (sflag == 9)
			{
				RobotStatus = MEASUREMENT;
				StatusSet(RobotStatus);
				illuminance_save(illmData, illuminanceflie, resultViewflie);
				sflag = 0;
				cpath_ct = 0;
#if !defined(_INFINITE_LOOP)	// 試験用無限ループ
				break;
#endif
			}
			else if (sflag == 99)
			{
				break;
			}
			//追加 henko2
			endTime = clock();
			FILE *fp = fopen("C:\\kinden\\log\\time.csv", "a");
			fprintf(fp, "%lf\n", (double)(endTime - startTime) / CLOCKS_PER_SEC);  //1ループ処理時間[s]
			fclose(fp);
			FILE *fp1 = fopen("C:\\kinden\\log\\SLAMPos.csv", "a");
			fprintf(fp1, "%lf,%lf,%lf\n", robot_pos->real_x, robot_pos->real_y, robot_pos->dangle);
			fclose(fp1);
		}

		cvSaveImage("C:\\kinden\\log\\result1.png", viewImage2);  //追加 henko2
		cvSaveImage("C:\\kinden\\log\\result2.png", mapImage);

		controlMotor(0, 0);
#if defined(_ROOMBA)	// ルンバ実験用コマンド
		roombaStop(); //stop roomba
		Sleep(50);
#endif
		//free(data);

		//WSACleanup();

		if (sflag == 0)
		{
			RobotStatus = END;
			StatusSet(RobotStatus);


			//結果csvファイルのBOXコピー
			if (isOnline == false)
			{
				//外部ネットワーク接続エラーなら通知してBOXにコピーしない
			}
			else
			{
				if (copyFile(illuminanceflie) == false)
				{
					RobotStatus = INERNET_ERR;
					StatusSet(RobotStatus);
				}
			}

			if (run_finish_notice)
			{
				sendMail(RUN_FINISH_MAIL);	//走行完了メール送信
			}
		}

		//終了音
		Sleep(200);
		Beep(880, 300);//ド
		Sleep(200);
		Beep(880, 300);//ド

		printf("final TR value:%lf \n", TR);

		restartSet();

	}
	// Windows での終了設定
	urg_exit(&urg, "URG connection closed");
	//system("PAUSE");
	return 0;
}

void memFree()
{
	//mallocで確保したメモリの解放
	if(robot_pos != NULL) free(robot_pos);
	if(initial_Position != NULL) free(initial_Position);

	if (baseMap.map != NULL) free2d_int(baseMap.map);
	if (baseMap.map2 != NULL) free2d_int(baseMap.map2);
	if (baseMap.cad_map != NULL) free2d_int(baseMap.cad_map);
	if (baseMap.cad_map2 != NULL) free2d_int(baseMap.cad_map2);
	if (edge_cad)
	{
		if (baseMap.edge_map != NULL) free2d_int(baseMap.edge_map);
		if (baseMap.edge_map2 != NULL) free2d_int(baseMap.edge_map2);
	}
	if (data != NULL) {	free(data); data = NULL;}
	// 動的に取得したメモリを最下層から順に解放する
	struct multi_map* r;
	r = root;
	while (1)
	{
		if (r == NULL) break;
		if (r->mmap != NULL) free2d_int(r->mmap);
		if (r->amap != NULL) free2d_int(r->amap);
		if (r->norm != NULL) free2d_double(r->norm);
		if (r->anorm != NULL) free2d_double(r->anorm);
		if (r->chmap != NULL) free2d_double(r->chmap);
		if (r->child != NULL) free(r->child);
		if (r->parent == NULL)
		{
			free(r);
			break;
		}
		else
		{
			r = r->parent;
		}
	}

	if(viewImage2 != NULL) cvReleaseImage(&viewImage2);
	if(mapImage != NULL) cvReleaseImage(&mapImage);
	for (int i = 0; i < 10; i++)
	{
		if(multimapImage[i] != NULL) cvReleaseImage(&multimapImage[i]);
	}
	if(limage != NULL) cvReleaseImage(&limage);
	if(limage2 != NULL) cvReleaseImage(&limage2);
}

void restartSet()
{
	//リスタートの指示があるまで待機
	while (true)
	{
		rcvBuff = std::string(PipeReceive);
		command = rcvBuff.find("Restart");
		if (command != std::string::npos)
		{
			restartFlg = true;
			cvDestroyWindow("Map");

			memFree();
			
			RobotStatus = STANDBY;
			StatusSet(RobotStatus);

			sprintf(resultViewflie, "C:\\kinden\\result\\resultView.csv");		//iPad表示用のtmp結果ファイル
			remove(resultViewflie);		//iPad表示用のtmp結果ファイルの削除

			colflag = 0;

			break;
		}
		Sleep(500);
	}
}

static void urg_exit(urg_t *urg, const char *message)
{
	printf("%s: %s\n", message, urg_error(urg));
	urg_close(urg);
#if defined(URG_MSC)
	getchar();
#endif
}

static void print_echo_data(long data[], int index)
{
	// [mm]
	for (int i = 0; i < URG_MAX_ECHO; ++i) 
	{
		printf("%ld, ", data[(URG_MAX_ECHO * index) + i]);
	}
}


static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
#if 1
	int front_index;

	(void)data_n;

	// Shows only the front step
	front_index = urg_step2index(urg, 0);
	printf("%ld [mm], (%ld [msec])\n", data[front_index], time_stamp);

#else
	(void)time_stamp;

	int i;
	long min_distance;
	long max_distance;

	// Prints the X-Y coordinates for all the measurement points
	urg_distance_min_max(urg, &min_distance, &max_distance);
	for (i = 0; i < data_n; ++i) {
		long l = data[i];
		double radian;
		long x;
		long y;

		if ((l <= min_distance) || (l >= max_distance)) {
			continue;
		}
		radian = urg_index2rad(urg, i);
		x = (long)(l * cos(radian));
		y = (long)(l * sin(radian));
		printf("(%ld, %ld), ", x, y);
	}
	printf("\n");
#endif
}

bool search_data()
{
	//printf("Sensing..\n");
	static int ct;
	enum 
	{
		CAPTURE_TIMES = 1,
	};

#if defined(URG_WINDOWS_OS)
	char device[6];
	sprintf_s(device, "COM%d", lrfPort);
#elif defined(URG_LINUX_OS)
	const char *device = "/dev/ttyACM0";
#else
	const char *device = "/dev/tty.usbmodemfa131";
#endif

	if (ct == 0) 
	{
		int ret = urg_open(&urg, connection_type, device, baudrate_or_port);
		if (ret < 0)
		{
			RobotStatus = LRF_COMM_ERR;
			StatusSet(RobotStatus);
			printf("urg_open: %s, %ld: %s\n",
			device, baudrate_or_port, urg_error(&urg));
			urg_exit(&urg, "urg_connect()");

			return false;
		}
		printf("LRF Port(COM %d) connected. \n", lrfPort);
	}
	ct++;

	//Get Single measurement
	data_max = urg_max_data_size(&urg);
	data = (long *)malloc(data_max * sizeof(data[0]));
	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	for (int i = 0; i < CAPTURE_TIMES; ++i) 
	{
		int n = urg_get_distance(&urg, data, &time_stamp);
		if (n <= 0) 
		{
			RobotStatus = LRF_ERR;
			StatusSet(RobotStatus);
			printf("urg_get_distance: %s\n", urg_error(&urg));
			if (data != NULL) { free(data); data = NULL; }
			urg_close(&urg);
			return false;
		}
	}
	return true;
}

int SLAM(void)
{
	int nmode[10];
	char posbuf[255];
	int sflag = 0;
	IplImage* cpImage = NULL;
	IplImage* sImage = NULL;
	static bool planning_flag = false;

	cpImage = cvCreateImage(cvSize(baseMap.width, baseMap.height), IPL_DEPTH_8U, 3);

	int w, w2, h;
	if ((baseMap.width > MAP_DISP_SIZE) || (baseMap.height > MAP_DISP_SIZE))
	{
		if (baseMap.width > baseMap.height)
		{
			sImage = cvCreateImage(cvSize(MAP_DISP_SIZE, (int)((MAP_DISP_SIZE / (double)baseMap.width) * (double)baseMap.height)), IPL_DEPTH_8U, 3);
		}
		else if (baseMap.width < baseMap.height)
		{
			// SLAM表示用画像の幅は4ピクセル単位に整形する
			w = (int)(((double)MAP_DISP_SIZE / (double)baseMap.height) * (double)baseMap.width);
			w2 = w / 4;
			w2 *= 4;
			h = (int)((double)MAP_DISP_SIZE * (double)w2 / (double)w);
			sImage = cvCreateImage(cvSize(w2, h), IPL_DEPTH_8U, 3);
			//sImage = cvCreateImage(cvSize((int)((MAP_DISP_SIZE / (double)baseMap.height) * (double)baseMap.width), MAP_DISP_SIZE), IPL_DEPTH_8U, 3);
		}
		else
		{
			sImage = cvCreateImage(cvSize(MAP_DISP_SIZE, MAP_DISP_SIZE), IPL_DEPTH_8U, 3);
		}
	}
	else
	{
		// SLAM表示用画像の幅は4ピクセル単位に整形する
		w = baseMap.width / 4;
		w *= 4;
		h = baseMap.height * w / baseMap.width;
		sImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, 3);
		//sImage = cvCreateImage(cvSize(baseMap.width, baseMap.height), IPL_DEPTH_8U, 3);
	}

	cvCopy(viewImage2, cpImage);
	memset(nmode, 0, sizeof(nmode));
	nmode[0] = 1;

	//---------------- SLAM----------------------
#if !defined(LRF_NON)
	//RS-LiDAR用
	if (SENSOR_MODE == 0) {
		if (!search_data()) return 0;
	}
	else {
		//RS-LiDAR用
		if (rs16_thread_mode == 0) {
			RSRecvThread(NULL);
			RSProcess(NULL);
		}
	}

#else
	if (SENSOR_MODE == 0) {
		data_max = 1080;
		data = (long*)malloc(data_max * sizeof(long));
		for (int i = 0; i < data_max; i++)
		{
			data[i] = data_max;
		}
		//return 0;
	}
#endif
	try {
		//RS-LiDAR用
		if (SENSOR_MODE == 0) {
			FILE* lfp = fopen("C:\\kinden\\log\\LRF.dat", "a");
			for (int i = 0; i < SCAN_MARKS; i++)
			{
				fprintf(lfp, "%d\t", data[i]);
				if (data[i] > 50000 || data[i] < 10)
				{
					data[i] = 50000;
				}
			}
			fprintf_s(lfp, "\n");
			fclose(lfp);
		}
	}
	catch (...)
	{
		printf("LRF.dat Save Error\n");
	}

	//static int flag = 5;
	if (slam_On == true)	//局所的自己位置推定----------------------------------------------------------------------------
	{
		if (slamMode == 0) {  //if文追加 henko2 henkoModel
			//RS-LiDAR用
			if (SENSOR_MODE == 0) {
				slam(robot_pos, data, baseMap.map, baseMap.map2, poutp);
			}
			else {
				slam2(robot_pos, dataD, dataA, baseMap.map, baseMap.map2, poutp);
			}
		}
		else if (slamMode == 1) {
			if (SENSOR_MODE == 0) {
				slam(robot_pos, data, baseMap.cad_map, baseMap.cad_map2, poutp);
			}
			else {
				slam2(robot_pos, dataD, dataA, baseMap.cad_map, baseMap.cad_map2, poutp);
			}
		}
		else {
			if (edge_cad)
			{
				if (SENSOR_MODE == 0) {
					slam(robot_pos, data, baseMap.edge_map, baseMap.edge_map2, poutp);
				}
				else {
					slam2(robot_pos, dataD, dataA, baseMap.edge_map, baseMap.edge_map2, poutp);
				}
			}
			else
			{
				if (SENSOR_MODE == 0) {
					slam(robot_pos, data, baseMap.map, baseMap.map2, poutp);
				}
				else {
					slam2(robot_pos, dataD, dataA, baseMap.map, baseMap.map2, poutp);
				}
			}
		}
		if (mapping_On == true)
		{
			if (moveMode != 4 || moveMode == 5)
			{
				map_building2(robot_pos, baseMap.map, baseMap.map2, cpImage, mapImage);
			}
		}
	}
	else //大域的自己位置推定とスケール調整----------------------------------------------------------------------------
	{
		if (cmap != NULL)
		{
			//printf("resolution\n"); 0819
		}
		//printf("lflag:%d.", lflag);
		if (lflag == 0) // 大域的自己位置推定
		{
			//RS-LiDAR用
			if (SENSOR_MODE == 0) {
				cmap = initial_Localization(initial_Position, robot_pos, data, baseMap.edge_map, cmap, cpImage, &lflag);
				draw_LRFdata(0, robot_pos, data, cpImage, 255, 0, 0);
			}
			else {
				cmap = initial_Localization2(initial_Position, robot_pos, dataD, dataA, baseMap.edge_map, cmap, cpImage, &lflag);
				draw_LRFdata2(0, robot_pos, dataD, dataA, cpImage, 255, 0, 0);
			}

			//if (scaleMode == 0)
			//{
				//henkoSCALE--------------
			if (lflag == 1 && scaleMode == 0)
			{
				flag = 2;
			}
			//henkoSCALE--------------
		//}

			if (flag == 2 && lflag == 1)//henkoIP2
			{
				lflag = 2;
				//追加 henko2
				FILE *fp = fopen("C:\\kinden\\log\\time.csv", "a");
				fprintf(fp, "999999\n");
				fclose(fp);
				FILE *fp1 = fopen("C:\\kinden\\log\\define.csv", "w");
				fprintf(fp1, "MAPRATE,%lf\n", MAPRATE);
				fprintf(fp1, "TR,%lf\n", TR);
				fprintf(fp1, "MAPOFFSET,%d\n", MAPOFFSET);
				fprintf(fp1, "MAPSIZE_WIDTH,%d\n", MAPSIZE_WIDTH);
				fprintf(fp1, "MAPSIZE_HIGHT,%d\n", MAPSIZE_HIGHT);
				fprintf(fp1, "SLAM_MODE,%d\n", SLAM_MODE);
				fclose(fp1);
			}
			time_t InitPosiSerchEndTime = time(NULL);		//自己位置推定終了時間;
			if ((InitPosiSerchEndTime - InitPosiSerchStartTime) > init_time)
			{
				RobotStatus = INITIAL_POSITION_ERR2;
				StatusSet(RobotStatus);

				printf("Init Position Search Error.\n");
				//return 99;
				//while (true)
				//{
				//	Sleep(10000);
				//}

				sflag = 99;

				if (&cpImage != NULL) cvReleaseImage(&cpImage);
				if (&sImage != NULL) cvReleaseImage(&sImage);

				return sflag;

			}
		}
		else if (lflag == 1)
		{
			if (flag != 1)
			{
				while (cmap->reso != (flag - 1))//henkoIP2
				{
					cmap = cmap->child;
				}
				flag--;
			}
			else
			{
				lflag = 2;
			}

			addjust_scale_multi(robot_pos, data, cmap);
			CvPoint p1, p2;
			p1.x = 0;
			p1.y = 0;
			p2.x = MAPSIZE_WIDTH;
			p2.y = MAPSIZE_HIGHT;
			cvRectangle(viewImage2, p1, p2, CV_RGB(125, 125, 125), CV_FILLED, 8, 0);
			cvRectangle(mapImage, p1, p2, CV_RGB(255, 255, 255), CV_FILLED, 8, 0);
			for (int i = 0; i < 6; i++)
			{
				cvRectangle(multimapImage[i], p1, p2, CV_RGB(125, 125, 125), CV_FILLED, 8, 0);
			}

			initial_Position->init_pos[0] = (initial_Position->init_pos[0] - map_offset) / (TR / map_rate);
			initial_Position->init_pos[1] = (initial_Position->init_pos[1] - map_offset) / (TR / map_rate);

			TR /= robot_pos->gen_s[robot_pos->best_gene][0];
			FILE *mfpTR = fopen("C:\\kinden\\log\\TR.txt", "a");//henkoIP2
			fprintf(mfpTR, "%f\n", TR);//henkoIP2
			fclose(mfpTR);
			initial_Position->init_pos[0] = (initial_Position->init_pos[0] * TR) / map_rate + map_offset;
			initial_Position->init_pos[1] = (initial_Position->init_pos[1] * TR) / map_rate + map_offset;


			//int robotPositionOrgX = (int)((initial_Position->init_pos[0] - map_offset) / (TR / map_rate));		//オリジナル座標でのロボット位置X
			//int robotPositionOrgY = (int)((initial_Position->init_pos[1] - map_offset) / (TR / map_rate));		//オリジナル座標でのロボット位置Y

			//int defX = abs(robotPositionOrgX - stratPosition[0]);	//オリジナル座標と自己位置推定の座標の差分の絶対値
			//int defY = abs(robotPositionOrgY - stratPosition[1]);	//オリジナル座標と自己位置推定の座標の差分の絶対値

			//printf("[%d,%d],[%d,%d]", robotPositionOrgX, robotPositionOrgY, stratPosition[0], stratPosition[1]);

			//if (((defX * TR) > judge_range) || ((defY * TR) > judge_range))
			//{
			//	RobotStatus = INITIAL_POSITION_ERR1;
			//	StatusSet(RobotStatus);

			//	printf("Init Potision is out of Range.");

			//	while (true)
			//	{
			//		Sleep(10000);
			//	}
			//}

			//memset(map, 0, sizeof(map));
			//memset(map2, 0, sizeof(map2));
			//memset(cad_map, 0, sizeof(cad_map));  //追加 henko2
			//memset(cad_map2, 0, sizeof(cad_map2));
			//memset(edge_map, 0, sizeof(edge_map));  //追加 henko2
			//memset(edge_map2, 0, sizeof(edge_map2));

			//mallocで確保した配列の0クリア
			for (int i = 0; i < width; i++)
			{
				for (int j = 0; j < height; j++)
				{
					baseMap.map[i][j] = 0;
					baseMap.map2[i][j] = 0;
					baseMap.cad_map[i][j] = 0;
					baseMap.cad_map2[i][j] = 0;
					if (edge_cad)
					{
						baseMap.edge_map[i][j] = 0;
						baseMap.edge_map2[i][j] = 0;
					}
				}
			}

			for (int j = 0; j < limage->width; j++)
			{
				for (int i = 0; i < limage->height; i++)
				{
					int k = (unsigned char)limage->imageData[limage->widthStep*i + j * 3];
					int rx, ry;
					int px, py;

					rx = (int)((double)j * TR);
					ry = (int)((double)i * TR);
					px = (int)(rx / map_rate);
					py = (int)(ry / map_rate);

					if (k < 200)
					{
						baseMap.map[px + map_offset][py + map_offset] = 100;
						baseMap.map2[px + map_offset][py + map_offset] = 100;
						baseMap.cad_map[px + map_offset][py + map_offset] = 100;  //追加 henko2
						baseMap.cad_map2[px + map_offset][py + map_offset] = 100;
					}
				}
			}

			//henko2
			if (edge_cad)
			{

				for (int j = 0; j < limage2->width; j++)
				{
					for (int i = 0; i < limage2->height; i++)
					{
						int k = (unsigned char)limage2->imageData[limage2->widthStep * i + j * 3];
						int rx, ry;
						int px, py;

						rx = (int)((double)j * TR);
						ry = (int)((double)i * TR);
						px = (int)(rx / map_rate);
						py = (int)(ry / map_rate);

						if (k < 200)
						{
							baseMap.edge_map[px + map_offset][py + map_offset] = 100;
							baseMap.edge_map2[px + map_offset][py + map_offset] = 100;
						}
					}
				}
			}

			draw_globalMap(baseMap.cad_map, baseMap.cad_map2, viewImage2, mapImage, multimapImage);  //henko2

			// 動的に取得したメモリを最下層から順に解放する
			struct multi_map* r;
			r = root;
			while (1)
			{
				if (r == NULL) break;
				if (r->mmap != NULL) free2d_int(r->mmap);
				if (r->amap != NULL) free2d_int(r->amap);
				if (r->norm != NULL) free2d_double(r->norm);
				if (r->anorm != NULL) free2d_double(r->anorm);
				if (r->chmap != NULL) free2d_double(r->chmap);
				if (r->child != NULL)
				{
					free(r->child);
				}
				if (r->parent == NULL)
				{
					free(r);
					break;
				}
				else
				{
					r = r->parent;
				}
			}

			root = NULL;
			for (int i = 0; i < 5; i++)//henkoIP2
			{
				//root = reduce_resolution(root, cad_map);  //henko2
				if (edge_cad)
				{
					root = reduce_resolution(root, baseMap.edge_map);  //henkoIP3
				}
				else
				{
					root = reduce_resolution(root, baseMap.cad_map);
				}
			}
			cmap = root;
			while (cmap->reso != 3)
			{
				cmap = cmap->parent;
			}
			lflag = 0;

		}
		else //局所的自己位置推定（スケール調整込み）-----------------------------------------------------------
		{
			if (nmode[0] != 6)
			{
				if (initPosiJugde == false) { //一回だけ実行：位置登録、パス登録
					initPosiJugde = true;

					int robotPositionOrgX = (int)((robot_pos->map_x - map_offset) / (TR / map_rate));		//オリジナル座標でのロボット位置X
					int robotPositionOrgY = (int)((robot_pos->map_y - map_offset) / (TR / map_rate));		//オリジナル座標でのロボット位置Y

					int defX = abs(robotPositionOrgX - stratPosition[0]);	//オリジナル座標と自己位置推定の座標の差分の絶対値
					int defY = abs(robotPositionOrgY - stratPosition[1]);	//オリジナル座標と自己位置推定の座標の差分の絶対値

					printf("org_posi[%d,%d],robot_posi[%d,%d]\n", robotPositionOrgX, robotPositionOrgY, stratPosition[0], stratPosition[1]);

					int dist = (int)(sqrt((defX*defX) + (defY*defY)) * TR);
					printf("diff_dist=%d,jugde_dist=%d\n", dist, judge_range);

					if (dist > judge_range)
					{
						RobotStatus = INITIAL_POSITION_ERR1;
						StatusSet(RobotStatus);

						printf("Init Potision is out of Range.\n");
						controlMotor(0, 0);

						//while (true)
						//{
						//	Sleep(10000);
						//}

						sflag = 99;
						return sflag;
					}
				}
				nmode[0] = 6;
				static int tflag = 1;
				if (tflag == 1)
				{
					try
					{
						FILE *ffp = fopen("C:\\kinden\\log\\fpath.txt", "w");
						for (int i = 0; i < path_ct; i++)
						{
							fpath[i][0] = (int)((fpath[i][0] - map_offset) / (TR_o / map_rate));
							fpath[i][1] = (int)((fpath[i][1] - map_offset) / (TR_o / map_rate));
							fpath[i][0] = (int)((fpath[i][0]) * TR / map_rate + map_offset);
							fpath[i][1] = (int)((fpath[i][1]) * TR / map_rate + map_offset);
							fprintf(ffp, "%d\t%d\n", fpath[i][0], fpath[i][1]);
						}
						fclose(ffp);
						tflag = 0;
					}
					catch (...)
					{
						printf("fpath.txt Save Error\n");
					}
					// GNG学習＆パスプランニングクラス準備
					printf("try gng.\n");
					PathPlannerSetting(viewImage2);
					planning_flag = true;
					printf("finish gng.\n");
				}
			}
			// 走行中実行処理
			if (slamMode == 0) {  //if文追加 henko2 henkoModel
				//RS-LiDAR用
				if (SENSOR_MODE == 0) {
					slam(robot_pos, data, baseMap.map, baseMap.map2, poutp);
				}
				else {
					slam2(robot_pos, dataD, dataA, baseMap.map, baseMap.map2, poutp);
				}
			}
			else if (slamMode == 1) {
				if (SENSOR_MODE == 0) {
					slam(robot_pos, data, baseMap.cad_map, baseMap.cad_map2, poutp);
				}
				else {
					slam2(robot_pos, dataD, dataA, baseMap.cad_map, baseMap.cad_map2, poutp);
				}
			}
			else {
				if (edge_cad)
				{
					if (SENSOR_MODE == 0) {
						slam(robot_pos, data, baseMap.edge_map, baseMap.edge_map2, poutp);
					}
					else {
						slam2(robot_pos, dataD, dataA, baseMap.edge_map, baseMap.edge_map2, poutp);
					}
				}
				else
				{
					if (SENSOR_MODE == 0) {
						slam(robot_pos, data, baseMap.map, baseMap.map2, poutp);
					}
					else {
						slam2(robot_pos, dataD, dataA, baseMap.map, baseMap.map2, poutp);
					}
				}
			}
			map_building2(robot_pos, baseMap.map, baseMap.map2, viewImage2, mapImage);

			//RS-LiDAR用
			if (SENSOR_MODE == 0) {
				draw_LRFdata(1, robot_pos, data, cpImage, 255, 0, 0);
			}
			else {
				draw_LRFdata2(1, robot_pos, dataD, dataA, cpImage, 255, 0, 0);
			}
		}
	}

	//結果の描画----------------------------------------------------------------------------
	for (int i = 0; i < path_ct - 1; i++)
	{
		cvLine(cpImage, cvPoint(fpath[i][0], fpath[i][1]), cvPoint(fpath[i + 1][0], fpath[i + 1][1]), CV_RGB(255, 0, 0), 1, 8, 0);
		cvCircle(cpImage, cvPoint(fpath[i][0], fpath[i][1]), 5, CV_RGB(255, 0, 0), 1, 8, 0);
	}
	draw_robot(robot_pos, 10, cpImage, 255, 0, 0);
	if (planning_flag) {
		if (pp->draw_flag) {
			pp->drawPathPlanningResult(cpImage);
		}
	}
	cvResize(cpImage, sImage, CV_INTER_AREA);
	sendImage(sImage);

	cvShowImage("Map", sImage);
	cvWaitKey(1);
	cvReleaseImage(&cpImage);
	cvReleaseImage(&sImage);

	memset(posbuf, 0, sizeof(posbuf));
	rtoc_robotPosition(robot_pos, posbuf, 0, 0, 0);
	//nmode = communication_data(posbuf, map, map2, NULL, tpath);	//大学が作成した操作アプリへの通信処理
	//printf("nmode: %d \n", nmode[0]); TMU20190819 非表示化

	switch (nmode[0])
	{
	case 0:
		moveMode = 99;
		break;
	case 6:
		moveMode = 11;
		break;
	case 8:
		printf("%d\n", nmode[1]);
		break;
	case 11:
		moveMode = 11;
		break;
	default:
		break;
	}

	//ロボットの行動制御----------------------------------------------------------------------------
	if (moveMode == 11)
	{
		static double preoutput[2];
		//if (colflag < 1 || colflag > 3)
		if (colflag < 1)
		{
			colflag = getBumpData();
			//RS-LiDAR用
			if (SENSOR_MODE == 0) {
				//moveMode = decision_making(robot_pos, poutp, data, fpath, &sflag, path_ct, moveMode);
				moveMode = movement_controller(robot_pos, poutp, data, fpath, &sflag, path_ct, moveMode, pp);
				//printf("output: %f,%f\n", poutp[0], poutp[1]);
			}
			else {
				moveMode = decision_making2(robot_pos, poutp, dataD, dataA, fpath, &sflag, path_ct, moveMode);
			}

			if (fabs(poutp[0] - preoutput[0]) > 50 || fabs(poutp[1] - preoutput[1]) > 50)
			{
				double alpha = 1.0;
				poutp[0] = alpha * poutp[0] + (1.0 - alpha) * preoutput[0];
				poutp[1] = alpha * poutp[1] + (1.0 - alpha) * preoutput[1];
			}

#if defined(_SPEED_LIMIT)	// 速度下限リミッター
			if (fabs(poutp[0]) < lowLimitSpeed || fabs(poutp[1]) < lowLimitSpeed)
			{
				// 符号判定
				int sign0 = (poutp[0] > 0) - (poutp[0] < 0);
				int sign1 = (poutp[1] > 0) - (poutp[1] < 0); //TMU20190819　修正

				if (fabs(poutp[0]) == fabs(poutp[1]))
				{
					poutp[0] = lowLimitSpeed * sign0;
					poutp[1] = lowLimitSpeed * sign1;
				}
				else
				{
					int add = 0;
					float coef = 0;
					if (fabs(poutp[0]) < fabs(poutp[1]))
					{
						add = (int)(lowLimitSpeed - abs(poutp[0]));
					}
					else
					{
						add = (int)(lowLimitSpeed - abs(poutp[1]));
					}
					poutp[0] = (abs(poutp[0]) + add) * sign0;
					poutp[1] = (abs(poutp[1]) + add) * sign1;
				}
			}
#endif

			controlMotor((short)poutp[0], (short)poutp[1]);
			preoutput[0] = poutp[0];
			preoutput[1] = poutp[1];
		}
		else
		{
			// ロボット回避行動(バンパセンサ)
			static time_t start, end;
			static int caflag = 0;
			if (caflag == 0)
			{
				start = clock();
				caflag = 1;
			}

			end = clock();
			switch (colflag)
			{
			case 1:	// 低速左旋回(右バンパ接触)
				controlMotor(-lowSpeed, -(lowSpeed + 80));
				//Sleep(2000);
				break;
			case 2:	// 低速右旋回(左バンパ接触)
				controlMotor(-(lowSpeed + 80), -lowSpeed);
				//Sleep(2000);
				break;
			case 3:	// 低速左旋回(中央バンパ接触)
				controlMotor(-lowSpeed, -(lowSpeed + 80));
				//Sleep(2000);
				break;
			case 4:	// 停止(右落下センサ反応)
				// その場で停止する。
				controlMotor(0, 0);
				//nmode[0] = 0;
				//moveMode = 99;
				sflag = 99;
				RobotStatus = FALL_SENSOR_ACT;
				StatusSet(RobotStatus);
				if (fall_sensor_notice)
				{
					sendMail(FALL_SENSOR_MAIL);
				}

				//controlMotor(-lowSpeed, -(lowSpeed + 20));
				//Sleep(1000);
				break;
			case 8:	// 停止(左落下センサ反応)
				// その場で停止する
				controlMotor(0, 0);
				//nmode[0] = 0;
				//moveMode = 99;
				sflag = 99;
				RobotStatus = FALL_SENSOR_ACT;
				StatusSet(RobotStatus);
				if (fall_sensor_notice)
				{
					sendMail(FALL_SENSOR_MAIL);
				}
				//controlMotor(-(lowSpeed + 20), -lowSpeed);
				//Sleep(1000);
				break;
			default:
				break;
			}

			if ((double)(end - start) / CLOCKS_PER_SEC > 3.5)
			{
				controlMotor((constantSpeed), (constantSpeed));
				if ((double)(end - start) / CLOCKS_PER_SEC > 7.0)//1.5 
				{
					colflag = 0;
					caflag = 0;
				}
			}
		}
		slam_eror_count = 0;
	}
	else if (moveMode == 99)	// 異常発生
	{
		slam_eror_count++;
		controlMotor(0, 0);	// ロボット停止

		if (slam_eror_count > SLAM_ERROR_COUNT_MAX)
		{
			printf("SLAM ERROR!\n");
			RobotStatus = SLAM_ERR;
			StatusSet(RobotStatus);

			sflag = 99;
			return sflag;
		}
	}
	else if (nmode[0] == 8) // 通常走行
	{
		switch (nmode[1])
		{
		case 0:	// 停止
			controlMotor(0, 0);
			break;
		case 1:	//	前進
			controlMotor(constantSpeed, constantSpeed);
			break;
		case 2:	// 後退
			controlMotor(-constantSpeed, -constantSpeed);
			break;
		case 3:	// 右旋回
			controlMotor(-constantSpeed, constantSpeed);
			break;
		case 4:	// 左旋回
			controlMotor(constantSpeed, -constantSpeed);
			break;
		default:
			break;
		}
		slam_eror_count = 0;
	}
	else
	{
		//char sendbyte3[256];
		controlMotor(0, 0);	// ロボット停止
	}

	if (moveMode == 9)	// SRAM停止
	{
		controlMotor(0, 0);
		sflag = 9;
	}

	if (FREE_MODE == 1) {  //追加 henko2
		if (data != NULL) { free(data); data = NULL; }
	}

	return sflag;
}

void rtoc_robotPosition(struct robot_position *robot, char *buf, int nmode, int robotID, int fType)
{
	int k = 0;
	const char textdata[20] = "0123456789";

	memset(buf, 0, sizeof(buf));

	buf[k] = textdata[nmode];
	k++;

	buf[k] = textdata[robotID];
	k++;

	if (robotID == 0) {
		buf[k] = textdata[fType];
		k++;
	}

	buf[k] = textdata[robot->map_x / 1000];
	k++;
	buf[k] = textdata[((int)(robot->map_x / 100)) % 10];
	k++;
	buf[k] = textdata[((int)(robot->map_x / 10)) % 10];
	k++;
	buf[k] = textdata[robot->map_x % 10];
	k++;

	buf[k] = textdata[robot->map_y / 1000];
	k++;
	buf[k] = textdata[((int)(robot->map_y / 100)) % 10];
	k++;
	buf[k] = textdata[((int)(robot->map_y / 10)) % 10];
	k++;
	buf[k] = textdata[robot->map_y % 10];
	k++;

	if (robot->rangle >= 0) {
		buf[k] = textdata[0];
		k++;
		buf[k] = textdata[(int)robot->rangle / 100];
		k++;
		buf[k] = textdata[((int)(robot->rangle / 10)) % 10];
		k++;
		buf[k] = textdata[(int)robot->rangle % 10];
		k++;
	}
	else {
		buf[k] = textdata[1];
		k++;
		buf[k] = textdata[(int)(-robot->rangle / 100)];
		k++;
		buf[k] = textdata[((int)(-robot->rangle / 10)) % 10];
		k++;
		buf[k] = textdata[(int)(-robot->rangle) % 10];
		k++;
	}

	buf[k] = textdata[ill_data / 1000];
	k++;
	buf[k] = textdata[((int)(ill_data / 100)) % 10];
	k++;
	buf[k] = textdata[((int)(ill_data / 10)) % 10];
	k++;
	buf[k] = textdata[ill_data % 10];
	k++;
}

bool Save_Settings()
{
	//char mapFileName[9] = "test.png"; // debug
	//debug
	//return false;
	//ここまで
	try
	{
		// 保存ファイル名作成
		char dateBuf[20];
		time_t timer = time(NULL);
		struct tm *date = localtime(&timer);
		sprintf(dateBuf, "_%d%02d%02d%2d%02d%02d.ksf",
			date->tm_year + 1900, date->tm_mon + 1, date->tm_mday,
			date->tm_hour, date->tm_min, date->tm_sec);

		INT64 point;
		char* tp;
		char nameBuf[255];
		char settingsFlieName[255];
		char str1 = '.';
		char dirName[15] = "C:\\kinden\\run\\";
		tp = strrchr(mapFileName, str1);// ファイル名の最後から"."を検索する
		point = tp - mapFileName;
		memcpy(nameBuf, mapFileName, point);

		memcpy(settingsFlieName, dirName, sizeof(dirName) - 1);
		memcpy(settingsFlieName + 14, nameBuf, point);
		memcpy(settingsFlieName + 14 + point, dateBuf, sizeof(dateBuf));
		printf(settingsFlieName);
		printf("\r\n");

		// データ書き込み
		FILE *fp = fopen(settingsFlieName, "w");
		fprintf(fp, "%s\n",mapFileName);
		fprintf(fp, "%lf\n", TR);
		fprintf(fp, "%d\n", (int)area_length);
		fprintf(fp, "%d\n", judge_range);
		fprintf(fp, "%d\n", measurementId[0]);
		fprintf(fp, "%d\t%d\t%d\n", stratPosition[0], stratPosition[1], stratPosition[2]);

		char pathname[255];
		sprintf(pathname, "C:\\kinden\\map\\path.txt");
		FILE *mfp3 = fopen(pathname, "r");
		int rx, ry, id;
		while (fscanf(mfp3, "%d\t%d\t%d\n", &rx, &ry, &id) != EOF)
		{
			fprintf(fp, "%d\t%d\n", rx, ry);
		}
		fclose(mfp3);


		fclose(fp);

	}
	catch (...)
	{
		printf("setting file save error\n");
		return false;
	}
	return true;
}

void DispVersionInfo()
{
#pragma comment(lib, "version.lib")

	TCHAR             fileName[MAX_PATH + 1];
	DWORD             size;
	BYTE* pVersion;
	VS_FIXEDFILEINFO* pFileInfo;
	UINT              queryLen;
	std::string       fileVersion;

	::GetModuleFileName(NULL, fileName, sizeof(fileName));

	size = ::GetFileVersionInfoSize(fileName, NULL);
	pVersion = new BYTE[size];
	if (::GetFileVersionInfo(fileName, NULL, size, pVersion)) {
		::VerQueryValue(pVersion, _T("\\"), (void**)&pFileInfo, &queryLen);

		printf("Version %d.%d.%d.%d\n",
			HIWORD(pFileInfo->dwFileVersionMS),
			LOWORD(pFileInfo->dwFileVersionMS),
			HIWORD(pFileInfo->dwFileVersionLS),
			LOWORD(pFileInfo->dwFileVersionLS));
	}
	delete[] pVersion;
}

void PathPlannerSetting(IplImage* Image) {
	// Image: viewImage2 << グリッドマップ
	// 1. IplImageをcv::Matに変換
	cv::Mat cadMat = cv::cvarrToMat(Image);
	//cadMat = cv::cvarrToMat(Image);
	// 2. 領域拡張法>>regiongrowing
	getInputForGNG(cadMat, robot_pos->map_x, robot_pos->map_y);

	// 3. 入力準備
	int hit_data_size = getHitDataSize();
	int** hit_data = getHitInput();
	int err_data_size = getErrorDataSize();
	int** err_data = getErrorInput();
	printf("Start Path Planner: hit_size: %d, err_size: %d\n", hit_data_size, err_data_size);
	/*for (int i = 0; i < hit_data_size; i++) {
		cadMat.at<cv::Vec3b>(hit_data[i][1], hit_data[i][0])[0] = 155;
		cadMat.at<cv::Vec3b>(hit_data[i][1], hit_data[i][0])[1] = 0;
		cadMat.at<cv::Vec3b>(hit_data[i][1], hit_data[i][0])[2] = 0;
	}*/
	cv::Mat img = cadMat.clone();
	cv::imshow("gng_img", cadMat);
	cv::waitKey(1);

	// 4. GNG
	gng = new GNG();
	int k = 0;
	int ramdam_index; //ramdam number
	float x, y, occ;
	while (k < 400000) {
		if ((double)rand() / RAND_MAX < 0.3) { //0.35
			while ((ramdam_index = (int)((double)hit_data_size * rand() / RAND_MAX)) == hit_data_size);
			x = (float)hit_data[ramdam_index][0];
			y = (float)hit_data[ramdam_index][1];
			occ = 0.9f;
		}
		else {
			while ((ramdam_index = (int)((double)err_data_size * rand() / RAND_MAX)) == err_data_size);
			x = (float)err_data[ramdam_index][0];
			y = (float)err_data[ramdam_index][1];
			occ = 0.1f;
		}
		//printf("x: %d, y: %d\n",x,y);
		gng->LearningWithOccupancy(x, y, occ);
		if (k % 10000 == 0) {
			cadMat = img.clone();
			gng->drawGNG(cadMat);
			cv::imshow("gng_img", cadMat);
			cv::waitKey(1);
		}
		k++;
	}
	gng->ExtractionOfMovingArea();
	// 描画
	cadMat = img.clone();
	gng->drawGNG(cadMat);
	cv::imshow("gng_img", cadMat);
	cv::waitKey(1);
	// 反映
	Image = cvCreateImageHeader(cvSize(cadMat.cols, cadMat.rows), IPL_DEPTH_8U, cadMat.channels());
	Image->imageData = reinterpret_cast<char*>(cadMat.data);

	// 5. Astar準備
	pp = new FastAstar(gng);
}