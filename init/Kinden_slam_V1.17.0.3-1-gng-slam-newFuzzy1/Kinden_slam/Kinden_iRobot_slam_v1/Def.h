#pragma warning(disable : 4996)

#if !defined(DEF_HEADER)
#define DEF_HEADER
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#ifdef EXTERN_REF
#define EXTERN
#else
#define EXTERN extern
#endif // EXTERN_REF

#define TABLET_NON
#define ILLM_NON
#define _AREA_LENGTH
#define SENSOR_NON
//#define MOTOR_NON
#define _DEG_ON
#define _MBCS


//#define ILLM_NON // 照度計が未接続の場合は有効にする

//---- Roomba Command ----
enum roombaCommand
{
	ZERO = 0,				// 初期値
	START = 128,			// Startコマンド
	BAUD = 129,			// Baudコマンド
	SAFE = 131,			// Safeコマンド
	FULL = 132,			// Fullコマンド
	POWER = 133,			// Powerコマンド
	SPOT = 134,			// Spotコマンド
	CLEAN = 135,			// Cleanコマンド
	MAX = 136,			// Maxコマンド
	DRIVE = 137,			// Driveコマンド
	MOTORS = 138,			// Motorsコマンド
	LEDS = 139,			// LEDsコマンド
	SONG = 140,			// Songコマンド
	PLAY = 141,			// Playコマンド
	SENSORS = 142,			// Sensorsコマンド
	SEEK_DOCK = 143,			// Seek Dockコマンド
	PWM_MOTORS = 144,			// PWM Motorsコマンド
	DRIVE_DIRECT = 145,			// DriveDirectコマンド
	DRIVE_PWM = 146,			// Drive PWMコマンド
	STREAM = 148,			// Streamコマンド
	QUERY_LIST = 149,			// Query Listコマンド
	PAUSE_RESUME_STREAM = 150,			// Pause/Resume Streamコマンド
	SCHEDULING_LEDS = 162,			// Scheduling LEDsコマンド
	DIGIT_LEDS_RAW = 163,			// Digit LEDs Rawコマンド
	DIGIT_LEDS_ASCII = 164,			// Digit LEDs ASCIIコマンド
	BUTTONS = 165,			// Buttonsコマンド
	SCHEDULE = 167,			// Scheduleコマンド
	SET_DAY_TIME = 168,			// Set Day/Timeコマンド
};

enum robotStatus
{
	STANDBY = 0,						// 待機中(準備中)
	READY = 1,							// 自律走行準備完了
	SEARCH = 2,							// edgeCAD生成と初期位置推定中
	RUN = 10,							// 自律走行中
	STOP = 11,							// 自律走行停止中
	END = 12,							// 自律走行終了
	MEASUREMENT = 20,					// 照度測定中(データ保存中)
	USB_CONNECT_ERR = 100,				// USBメモリエラー
	MAP_FILE_ERR = 101,					// Mapファイル読み込みエラー
	POINT_FILE_ERR = 102,				// 測定ポイントファイル読み込みエラー
	SENSOR_ARDUINO_COMM_ERR = 103,		// センサ用Arduino COMMエラー
	SENSOR_ARDUINO_CONNECT_ERR = 104,	// センサ用Arduino 接続エラー
	SLAM_ERR = 105,						// 自律走行エラー
	LRF_COMM_ERR = 106,					// LRF COMMエラー
	LRF_ERR = 107,						// LRF エラー
	ILLM_COMM_ERR = 108,				// 照度計 COMMエラー
	ILLM_ERR = 109,						// 照度計 エラー
	STRAT_POINT_ERR = 110,				// 開始位置エラー
	MOTOR_ARDUINO_COMM_ERR = 111,		// モータ用Arduino COMMエラー
	MOTOR_ARDUINO_CONNECT_ERR = 112,	// モータ用Arduino 接続エラー
	INITIAL_POSITION_ERR1 = 113,		// 初期位置推定失敗(指定位置周辺外)
	INITIAL_POSITION_ERR2 = 114,		// 初期位置推定失敗(収束失敗)
	SAVE_SETTING_ERR = 115,				// 走行設定ファイルの保存失敗
	TR_VALUE_ERR = 116,					// TR計算値の範囲外
	FALL_SENSOR = 117,					// 起動時に落下センサが反応している
	BUMP_SENSOR = 118,					// 起動時にバンパセンサが反応している
	FALL_SENSOR_ACT = 119,				// 走行中に落下防止センサが反応
	INERNET_ERR = 120,					// 外部ネットワーク接続エラー
};

#define _SUM_CHECK

#define ENGINEERING_MODE	0				// エンジニアリングモード
//#define MAX_SPEED			278				// 最大速度設定値(mm/s) 278
#define CONSTANT_SPEED		208				// 定速(mm/s) 208
#define LOW_SPEED			139				// 低速度(mm/s)
#define LOW_LIMIT_SPEED		80				// 下限スピード(これ以下になると動かなくなる)
#define FALL_THRESHOLD		15				// 落下検出閾値(cm)
#define ILLUM_PORT			5				// 上部照度計ポート番号
#define ILLUM_PORT2			6				// 後部照度計ポート番号
#define ARDN_PORT			4				// Arduinoセンサ用ポート番号
#define ROOMBA_PORT			3				// ルンバ(Arduino走行用)ポート番号
#define LRF_PORT			7				// LRFポート番号

#define RS16_ROTATION_SPEED	300				// RS-Lidar-16 回転速度(rpm)

#define SLAM_MODE           0				// SLAMモード henko2
// --0:通常のSLAM
// --1:CAD図面を利用した自己位置推定
// --2:エッジ抽出後のCAD図面を利用した自己位置推定

#define FREE_MODE			1				//0:メモリ開放なし, 1:メモリ開放あり henko2

#define RG_MODE				1				//0:エッジ抽出マウス入力, 1:エッジ抽出自動実行 henko2
#define SCALE_MODE			0				//0:スケール調整なし, 1:スケール調整あり //henkoSCALE

#define MODEL_MODE			1				//0:従来の手法, 1:ロボットモデルを適用した手法 henkoModel

//#define THREAD_MODE			0				//スレッドモード(0:スレッド処理なし、1:スレッド処理あり)

#define SENSOR_MODE			0				//センサモード(0:UTM-30LX, 1:RS-Lidar-16)

//henkoModel
const bool miniMapMode = true;
const int miniMapSize = 400;
const int gridNum = 10;
const double miniMapRate = 30.0;
//static double TR = 14.5 * 2.54;	//mm/pixel コメント uni
//static double TR = 54.0;	//mm/pixel
static double TR = 59.5; //55 //1gou60

//#define MAPSIZE				2000//1500			// 読み込み可能最大マップサイズ
#define MAPSIZE_WIDTH		width
#define MAPSIZE_HIGHT		height
#define CENTER				MAPSIZE/2				// MAPSIZEの半分の値
#define MAPOFFSET			0//400//200//400				// 地図オフセット
#define ERR					-999999999		// エラー初期値
#define MAPRATE				60.0			// Map解像度
#define GALM				3
#define GANM				30
#define GANS				300
#define TS					1000
#define T2					600
#define T3					1200

#define MAP_DISP_SIZE		700				// MAP画像表示サイズ(pixel)
#define SLAM_MAP_SIZE_MAX	(3000*3000)		// SLAM制御に使用する図面の最大サイズ(pixel)

//#define FALL_DIST			7				//落下防止センサー閾値(cm)

#define PI					3.141592		//標準円周率
#if defined M_PI
#else
#define M_PI				3.14159265358979323846
#endif

#define MAXRANGE			500				//(=)80cm
#define MEMNUM				11       //OA入力:前方グループ数
#define RULENUM				11      //OAルール数
#define CTRLNUM				2       //出力数
#define MEMNUM1				2       //TT入力:状態
#define RULENUM1			10       //TTルール数
#define MEMNUM2WF           1       //WF入力 (いらない)
#define RULENUM2WF          5      //WFルール数
#define TRANGE				800

#define INIT_TIME			50				//初期位置推定時間
#define JUDGE_RANGE			2000			//初期位置指定と推定後の誤差距離(mm)

//テスト用
#define RULE				16//6			//TMU20190826	// Fuzzy Rules
#define MEMF				5				// Membership Function
#define CNTL				2				// Control Parameters
#define MAX_LRF_DISTANCE	1000.0			// Maximum distance using collision avoidance
#define MIN_LRF_DISTANCE	300.0       	// Minimum distance using collision avoidance TMU20190823
//#define MAX_OUTPUT 550.0					// Maximum output value of omni-directional mobile robot
#define MAX_OUTPUT			350 			// Maximum output value of omni-directional mobile robot
//#define ILLM_NON			1				// 照度計搭載:1

#define TCP_PORT			2000			//ポート番号を合わせる
#define UDP_PORT			5000			// 未使用   

#define EGAN				1000
#define AREA_LENGTH         2000			//TMU20190820 初期位置推定の探索範囲 単位：mm
#define AREA_ANGLE          90.0			//TMU20190820 初期位置推定の探索範囲 単位：degree
#define START_DEG			1				// スタート位置角度指定　1:TRUE 0:FALSE
#define DEF_AL				1				// 初期位置推定の探索範囲 1:TURE 0;FALSE
#define SCAN_MARKS			1080			// LRF走査角度270度の走査点数

#define SCAN_MARKS_RS16		(125*DATA_NUM*BLOCKS_PER_PACKET)	// RS-Lidar-16 走査角度270度の走査点数(125:300rpmのデータ数/packet)

#define PGAL				30
#define PGAN				30
#define ALPHA_DANGER		100
#define ALPHA_UNC			0
#define ALPHA_DIS			10

#define INI_FILE_NAME		"Setting.ini"
#define SAVE_SETTINGS		1				// 設定値保存 1:保存 0:破棄
//#define DEF_TR				14.5 * 2.54
#define DEF_TR				14.5 * 2.54 * 1.03

#define SOUND_PATH          "C:\\Windows\\Media\\Ring10.wav"

#define MAX_MEASURE_POINT   2000			// 測定点最大数

#define TR_THRESHOLD_MAX		100.0	//tr計算値の閾値上限(これ以上の値は自律走行ができないのでNG)
#define TR_THRESHOLD_MIN		1.0		//tr計算値の閾値下限(これ以下の値は自律走行ができないのでNG)

#define SLAM_ERROR_COUNT_MAX 5

// エンジニアリングモード
//static bool engineering_mode = ENGINEERING_MODE;

// T-10A(照度計)通信
#define ILLM_TIMEOUT		5000
#define ILLM_RCV_LEN		32				// 照度受信データ数
extern char Illm_Send[14];
extern int illmPort;
extern int illmPort2;
const double exponent[10] = { pow(10, -4), pow(10, -3), pow(10, -2), pow(10, -1), pow(10, 0), pow(10, 1), pow(10, 2), pow(10, 3), pow(10, 4), pow(10, 5) };
extern double illmData;

// センサー系Ardino通信
#define SENC_TIMEOUT		500
#define SENC_RCV_LEN		18				// センサ受信データ数
#define SENC_RCV_LEN_BUF	1000				// センサ受信データ数
#define FALL_LEVEL			15				// 落下閾値
#define CODE_T				0x54
#define CODE_C				0x43
#define CODE_R				0x52
#define READ_CYCLE			50
extern int ardnPort;
extern char bumper[3];						// バンパーセンサー値
extern int falling[2];						// 落下センサー値

// 走行系Arduino通信
extern int roombaPort;
extern int maxSpeed;
extern int constantSpeed;
extern int lowSpeed;
extern int lowLimitSpeed;
extern int tcpPort;

// LRF通信
extern int lrfPort;

// System設定
static int start_deg = START_DEG;
static int save_settings = SAVE_SETTINGS;
//static int map_size = MAPSIZE;
//static int map_center = (int)(map_size / 2);
static double map_rate = MAPRATE;
static int map_offset = MAPOFFSET;
extern int map_size_width;
extern int map_size_height;
extern int map_center_width;
extern int map_center_height;

extern int rs16_thread_mode;	// RS-LiDAR-16 スレッドモード(0:スレッド処理なし,1:スレッド処理あり)
extern int rs16_rpm;			// RS-LiDAR-16 回転速度[rpm]
extern int rs16_NumOfPackage;	// RS-LiDAR-16 パケットデータ数


extern int init_time ;
extern int judge_range;
extern double area_length;
extern int judge_range_tmp;
extern double area_length_tmp;
extern double tr_threshold;

// ---- iPad情報 ----
extern int stratPosition[3];				// ロボット設置ポジション
extern double tr;							// TR値

extern char mapFileName[255];				// 地図ファイル名

extern char PipeReceive[255];
extern char RobotStatus;						// ロボット状態

extern int cpath_ct;

extern char scaleMode;
extern char slamMode;
extern char modelMode;

extern double ga_search_range;

//車輪情報
extern double wheelDia;
extern double wheelWidth;
extern double lidarWheelDist;
extern double microTime;
extern double outputCoefL;
extern double outputCoefR;

#define WHEEL_DIA 125.0
#define WHEEL_WIDTH 350.0
#define LIDAR_WHEEL_DIST 0.0
#define MICRO_TIME 0.020
#define OUTPUT_COEF_L 0.2
#define OUTPUT_COEF_R 0.2

extern int width;
extern int height;

//fitness値判定
#define FITNESS_TH 0.3
#define FITNESS_TIME 1
#define FITNESS_CNT 5

extern double fitness_th;
extern int fitness_time;
extern int fitness_cnt;

//PC_NAME
#define PC_NAME "kinden"

//Parameter.ini情報
#define FILE_NAME "Parameter.ini"

//Setting.ini情報
#define SETTING_FILE_NAME "Setting.ini"

#define CONSTRUCTION_SITE ""

#define EDGE_CAD 1
#define FALL_SENSOR_ENABLE 1
#define FALL_SENSOR_TH 10
#define RUN_FINISH_NOTICE 1
#define FALL_SENSOR_NOTICE 0
#define RUN_DIFFICULT_NOTICE 1
#define RUNNNING_NOTICE 0
#define RUNNNING_NOTICE_TIME 30

#define MAIL1 ""
#define MAIL2 ""
#define MAIL3 ""
#define MAIL4 ""
#define MAIL5 ""

#define SMTP_SERVER ""
#define SMTP_PASSSWORD ""
#define SMTP_PORT 0
#define SEND_MAIL_ADDRESS ""

extern int edge_cad;
extern int fall_sensor;
extern int fallThresHold;
extern int run_finish_notice;
extern int fall_sensor_notice;
extern int run_difficult_notice;
extern int running_notice;
extern int running_notice_time;

typedef enum
{
	RUN_FINISH_MAIL = 0,
	FALL_SENSOR_MAIL,
	RUN_DIFFICULT_MAIL,
	RUNNING_MAIL
}MAIL_STATUS;

bool sendMail(MAIL_STATUS);

struct measurement_position
{
	double X;
	double Y;
};

struct robot_position
{
	int map_x;
	int map_y;
	double real_x;
	double real_y;
	double rangle;
	double dangle;

    double model[3];  //2022.04.28追加
    
	double longitude;
	double latitude;

	double gen_m[GANM + 1][GALM + 1];
	double fit_m[GANM + 1];

	double gen_s[GANS + 1][GALM + 1];
	double fit_s[GANS + 1];

	int best_gene;
	int slam_count;
	double fs;

	double **node;
	int **edge;
	int node_n;
	int node_x;
	int node_y;
	int node_x2;
	int node_y2;
	int buf_prevWinner;
	int buf_winner;
	int buf_edge;
};

struct multi_map
{
	int **mmap;
	int **amap;
	double **norm;
	double **anorm;
	double **chmap;
	int reso;
	double rate;
	struct multi_map *parent;
	struct multi_map *child;
};

struct org_map
{
	int **map;
	int **map2;
	int **cad_map;
	int **cad_map2;
	int **edge_map;
	int **edge_map2;

	int width;
	int height;
};

#define GA_SEARCH_RANGE		0		// GA初期個体配置改善＋探索範囲倍率(0:従来処理、0<　探索範囲倍率)

////////////////////////////////////////////////////////////
//RSLidar
#define RS16_THREAD_MODE	0		// RS-LiDAR-16 スレッドモード(0:スレッド処理なし,1:スレッド処理あり)
//@@#define NumOfPackage 125  //167, 84, 42(360deg)  125, 63, 32(270deg)
#define RS16_NUM_OF_PACKAGE_MAX 125	// RS-LiDAR-16 最大データ数 
//@@#define RPM 300  //300, 600, 1200
#define RS16_ROTATION_SPEED 600		// RS-LiDAR-16 デフォルト回転速度[rpm]
#define LASER_RANGE_MIN (0.4*1000)  //SLAMで使用する最小距離[mm](RS-Lidar=16)
#define LASER_RANGE_MAX (150*1000)  //SLAMで使用する最大距離[mm](RS-Lidar=16)

//static const int rs16_NumOfPackage = 125;
static const int DATA_NUM = 2;
static const int BLOCKS_PER_PACKET = 12;

static const int TEMPERATURE_MIN = 31;
static const int TEMPERATURE_RANGE = 40;

static const float DISTANCE_MAX = 200.0f;            /**< meters */
static const float DISTANCE_MIN = 0.2f;              /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f;      /**< meters */
static const float DISTANCE_RESOLUTION_NEW = 0.005f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0f);

static const float RS16_BLOCK_TDURATION = 100.0f;  // [µs]
static const float RS16_DSR_TOFFSET = 3.0f;        // [µs]
static const float RS16_FIRING_TOFFSET = 50.0f;    // [µs]

static const float Calibration_Angle[16] = { -15.0161, -13.0276, -11.0066, -9.0274, -7.0263, -5.0078, -3.0124, -1.0097, 14.9693, 12.98, 10.9928, 8.9785, 6.9769, 4.9935, 2.9981, 0.9954 };

static const float aIntensityCal[7][32] =
{
	15.93, 14.87, 15.57, 15.2, 15.74, 15.41, 14.85, 14.34, 13.65, 14.13, 11.75, 14.07, 14.95, 12.78, 13.16, 14.71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	2.326, 1.968, 2.24, 2.045, 2.274, 2.193, 1.961, 1.861, 1.83, 1.803, 1.64, 1.79, 2.057, 1.751, 1.794, 1.931, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1.251, 1, 1.089, 1, 1.113, 1.31, 1.159, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	17.21, 13.97, 18.22, 15.71, 15.71, 18.77, 12.68, 7.538, 6.427, 7.733, 6.566, 7.815, 6.33, 4.101, 6.153, 6.828, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0.07167, 0.05687, 0.06466, 0.04543, 0.05589, 0.04863, 0.04687, 0.04295, 0.06817, 0.05155, 0.05767, 0.05295, 0.05875, 0.05891, 0.04616, 0.04495, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	-1.04, -0.1706, -0.5561, -0.00134, -0.9172, -0.1956, -0.3575, -0.3689, -0.9922, -0.5034, -0.8223, -0.4162, -1.086, -1.022, -0.7208, -0.2351, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	15.79, 10.55, 15.7, 7.863, 13.21, 8.758, 11.74, 8.483, 11.23, 7.975, 10.7, 7.957, 12.22, 9.969, 10, 6.786, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

const int dis_resolution_mode = 0;// 0 -> 0.5cm resolution 1-> 1cm resolution

static const int gRSPort = 6699;
//static const char gRSIP[50] = "192.168.1.200";//the ip of rs lidar
static const int bindflag = 1;

extern long dataD[RS16_NUM_OF_PACKAGE_MAX * BLOCKS_PER_PACKET * DATA_NUM];  //距離データ
extern float dataA[RS16_NUM_OF_PACKAGE_MAX * BLOCKS_PER_PACKET * DATA_NUM];  //角度データ

////////////////////////////////////////////////////////////

#endif
