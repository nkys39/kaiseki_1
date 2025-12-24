#define _CRT_SECURE_NO_WARNINGS
#include "myRSLidar.h"
#include "Def.h"

//UDP通信
struct sockaddr_in gRSAddress;
int gSocketRS;

//スレッド
pthread_mutex_t gRecvAndProcMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t gProcAndShowMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t gStop = PTHREAD_MUTEX_INITIALIZER;
pthread_t lRS, lRSProcess;

//データパケット(3次元データ)
UDP_Data udpdata;
char gOriginalData[1200 * RS16_NUM_OF_PACKAGE_MAX] = { 0 };
char gOriginalHead[42 * RS16_NUM_OF_PACKAGE_MAX] = { 0 };
char gProcessedData[1200 * RS16_NUM_OF_PACKAGE_MAX] = { 0 };
char gProcessedHead[42 * RS16_NUM_OF_PACKAGE_MAX] = { 0 };

//反射データ強度関連(使っていない)
int tempPacketNum = 0;
int gPointBias = RS16_NUM_OF_PACKAGE_MAX / 2;
float temper = 31.0;
int g_ChannelNum[16][41];
int numOfLasers = 16;
float CurvesRate[32];


bool RS_Initial()
{
	UINT64 time64;
	SYSTEMTIME st;
	FILE* initTimeFile;

#if OUTPUT_FILE_ENABLE
	//タイミングチャート上書き
	initTimeFile = fopen("output/Timing/initTime.csv", "a");
	GetLocalTime(&st);
	SystemTimeToFileTime(&st, (FILETIME*)&time64);
	fprintf(initTimeFile, "ConnectStart,%d,%d,%d,%d\n", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	fclose(initTimeFile);
#endif
	gPointBias = rs16_NumOfPackage / 2;

	WSADATA WSAData;	//WSAStartup initialize，only once
	if (WSAStartup(MAKEWORD(1, 1), &WSAData) != 0)  //winsock初期化(バージョン1.1)
	{
		//初期化失敗
		return false;
		//Sleep(10000);
		//exit(1);
	}
	
	struct sockaddr_in ca;
	gSocketRS = socket(AF_INET, SOCK_DGRAM, 0);  //UDPで設定

	ca.sin_family = AF_INET;
	ca.sin_addr.S_un.S_addr = htonl(INADDR_ANY);  //全IPアドレス受け入れ指定
	ca.sin_port = htons(gRSPort);
	if (1 == bindflag)  //バインド
	{
		gRSAddress = ca;
		if (::bind(gSocketRS, (sockaddr*)&gRSAddress, sizeof(SOCKADDR)) == SOCKET_ERROR)
		{
			//Bind error
			DWORD dwErr = GetLastError();
			printf("bind failed!\n");
			//Error
			printf("UDP Socket For RS16 ... Initial Error!\n");
			return false;
			//Sleep(10000);
			//exit(1);
		}
	}
	//Success
	printf("UDP Socket For RS16 ... Ready!\n");

#if OUTPUT_FILE_ENABLE
	//タイミングチャート上書き
	initTimeFile = fopen("output/Timing/initTime.csv", "a");
	GetLocalTime(&st);
	SystemTimeToFileTime(&st, (FILETIME*)&time64);
	fprintf(initTimeFile, "ConnectEnd,%d,%d,%d,%d\n", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	fclose(initTimeFile);
#endif

	return true;
}

void startThread()
{
	UINT64 time64;
	SYSTEMTIME st;
	FILE* initTimeFile;
#if OUTPUT_FILE_ENABLE
	//タイミングチャート上書き
	initTimeFile = fopen("output/Timing/initTime.csv", "a");
	GetLocalTime(&st);
	SystemTimeToFileTime(&st, (FILETIME*)&time64);
	fprintf(initTimeFile, "AllThreadStart,%d,%d,%d,%d\n", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
	fclose(initTimeFile);
#endif

	pthread_setconcurrency(2);  //並列処理レベルの設定(スレッド2つ？)
	pthread_create(&lRS, NULL, RSRecvThread, NULL);  //新規スレッド1(RS-16データ受信)作成
	pthread_create(&lRSProcess, NULL, RSProcess, NULL);  //新規スレッド2(RS-16受信データ処理)作成
	return;
}

void endThread()
{

	return;
}


void* RSRecvThread(void *arg)
{
	int lLength, lCount, lCounti, lCountj;
	int SockSize = sizeof(SOCKADDR_IN);
	//int ANGLE_HEAD = -360001, last_azimuth = -36001, now_azimuth;
	char lOriginalData[1200 * RS16_NUM_OF_PACKAGE_MAX];
	char lOriginalHead[42 * RS16_NUM_OF_PACKAGE_MAX];
	int lPointBias;
	//lCount = rs16_NumOfPackage+1 ;
	lCount = 0;
	lPointBias = 0;

	static int initflag = 0;
	static int flag = 1;
	static int dataID = 0;
	FILE* RSReceiveTimeFile;
	FILE* receiveDataFile;
	UINT64 time64;
	SYSTEMTIME st;

	do
	{
		while (1) {
			if (initflag == 0) {
#if OUTPUT_FILE_ENABLE
				//タイミングチャート新規作成
				RSReceiveTimeFile = fopen("output/Timing/RSReceiveTime.csv", "w");
				fprintf(RSReceiveTimeFile, "Name,Hour,Minute,Second,Millisecond\n");
				fclose(RSReceiveTimeFile);
				//計測データ新規作成
				receiveDataFile = fopen("output/RSLidar/receiveData.csv", "w");
				fprintf(receiveDataFile, "packetID,Hour,Minute,Second,Millisecond,Azimuth\n");
				fclose(receiveDataFile);
#endif
				initflag = 1;
			}
			if (flag == 1) {
#if OUTPUT_FILE_ENABLE
				//タイミングチャート上書き
				RSReceiveTimeFile = fopen("output/Timing/RSReceiveTime.csv", "a");
				GetLocalTime(&st);
				SystemTimeToFileTime(&st, (FILETIME*)&time64);
				fprintf(RSReceiveTimeFile, "RSReceive%dStart,%d,%d,%d,%d\n", dataID, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
				fclose(RSReceiveTimeFile);
#endif
				flag = 0;
			}

			lLength = recvfrom(gSocketRS, udpdata.Recvchars, 1248, 0, (struct sockaddr*)&gRSAddress, &SockSize);

			if (lLength == 1248)  //全データが正常に取得できた場合
			{
				//rs16_NumOfPackage分データがたまったら全コピー
				if (lCount == rs16_NumOfPackage)
				{
					pthread_mutex_lock(&gRecvAndProcMutex);  //排他制御ON(他スレッドからのアクセス衝突防止)
					for (lCountj = 0; lCountj < 1200 * rs16_NumOfPackage; lCountj++)
						gOriginalData[lCountj] = lOriginalData[lCountj];
					for (lCountj = 0; lCountj < 42 * rs16_NumOfPackage; lCountj++)
						gOriginalHead[lCountj] = lOriginalHead[lCountj];
					gPointBias = lPointBias;
					pthread_mutex_unlock(&gRecvAndProcMutex);  //排他制御OFF
					dataID++;
					flag = 1;
				}

				//rs16_NumOfPackage分データ貯める
				if (lCount < rs16_NumOfPackage)
				{
					for (lCountj = 0; lCountj < 1200; lCountj++)
					{
						lOriginalData[lCount * 1200 + lCountj] = udpdata.package.FiringData[lCountj];
					}
					for (lCountj = 0; lCountj < 42; lCountj++)
					{
						lOriginalHead[lCount * 42 + lCountj] = udpdata.package.Head[lCountj];
					}
					lCount++;

#if OUTPUT_FILE_ENABLE
					//計測データ上書き
					int azi = (float)(256 * (lOriginalData[lCount * 1200 + 2] & 0xff) + (lOriginalData[lCount * 1200 + 3] & 0xff));
					GetLocalTime(&st);
					SystemTimeToFileTime(&st, (FILETIME*)&time64);
					receiveDataFile = fopen("output/RSLidar/receiveData.csv", "a");
					fprintf(receiveDataFile, "%d,%d,%d,%d,%d,%f\n", lCount, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds, (float)azi / 100.0);
					fclose(receiveDataFile);
#endif
				}
				else
					lCount = 0;
				if (flag == 1) {
					lCount = 0;
					break;
				}
			}
			else  //データに欠損がある場合
			{
				//printf("Length: %d is not 1248!\n", lLength);
			}
		}
	} while (rs16_thread_mode);

	return nullptr;
}


float computeTemperature(unsigned char bit1, unsigned char bit2)
{
	float Temp;
	float bitneg = bit2 & 128;   // 10000000
	float highbit = bit2 & 127;  // 01111111
	float lowbit = bit1 >> 3;
	if (bitneg == 128)
	{
		Temp = -1 * (highbit * 32 + lowbit) * 0.0625f;
	}
	else
	{
		Temp = (highbit * 32 + lowbit) * 0.0625f;
	}

	return Temp;
}


int estimateTemperature(float Temper)
{
	int temp = (int)floor(Temper + 0.5);
	if (temp < TEMPERATURE_MIN)
	{
		temp = TEMPERATURE_MIN;
	}
	else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE)
	{
		temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
	}

	return temp;
}


float CalibrateIntensity(float intensity, int calIdx, int distance)
{
	int algDist;
	int sDist;
	int uplimitDist;
	float realPwr;
	float refPwr;
	float tempInten;
	float distance_f;
	float endOfSection1, endOfSection2;
	int intensity_mode_ = 1, intensityFactor = 51;

	int temp = estimateTemperature(temper);

	realPwr = max((float)(intensity / (1 + (temp - TEMPERATURE_MIN) / 24.0f)), 1.0f);
	// realPwr = intensity;

	if (intensity_mode_ == 1)
	{
		// transform the one byte intensity value to two byte
		if ((int)realPwr < 126)
			realPwr = realPwr * 4.0f;
		else if ((int)realPwr >= 126 && (int)realPwr < 226)
			realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
		else
			realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;
	}
	else if (intensity_mode_ == 2)
	{
		// the caculation for the firmware after T6R23V8(16) and T9R23V6(32)
		if ((int)realPwr < 64)
			realPwr = realPwr;
		else if ((int)realPwr >= 64 && (int)realPwr < 176)
			realPwr = (realPwr - 64.0f) * 4.0f + 64.0f;
		else
			realPwr = (realPwr - 176.0f) * 16.0f + 512.0f;
	}
	else
	{
		printf("The intensity mode is not right\n");
	}

	int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
	uplimitDist = g_ChannelNum[calIdx][indexTemper] + DISTANCE_MAX_UNITS;
	// limit sDist
	sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];
	sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
	// minus the static offset (this data is For the intensity cal useage only)
	algDist = sDist - g_ChannelNum[calIdx][indexTemper];

	// calculate intensity ref curves
	float refPwr_temp = 0.0f;
	int order = 3;
	endOfSection1 = 5.0f;
	endOfSection2 = 40.0;

	if (dis_resolution_mode == 0)
	{
		distance_f = (float)algDist * DISTANCE_RESOLUTION_NEW;
	}
	else
	{
		distance_f = (float)algDist * DISTANCE_RESOLUTION;
	}

	if (intensity_mode_ == 1)
	{
		if (distance_f <= endOfSection1)
		{
			refPwr_temp = aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] - aIntensityCal[2][calIdx] * distance_f) +
				aIntensityCal[3][calIdx];
			//   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
		else
		{
			for (int i = 0; i < order; i++)
			{
				refPwr_temp += aIntensityCal[i + 4][calIdx] * (pow(distance_f, order - 1 - i));
			}
			// printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
	}
	else if (intensity_mode_ == 2)
	{
		if (distance_f <= endOfSection1)
		{
			refPwr_temp = aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] - aIntensityCal[2][calIdx] * distance_f) +
				aIntensityCal[3][calIdx];
			//   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
		else if (distance_f > endOfSection1 && distance_f <= endOfSection2)
		{
			for (int i = 0; i < order; i++)
			{
				refPwr_temp += aIntensityCal[i + 4][calIdx] * (pow(distance_f, order - 1 - i));
			}
			// printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
		}
		else
		{
			float refPwr_temp0 = 0.0f;
			float refPwr_temp1 = 0.0f;
			for (int i = 0; i < order; i++)
			{
				refPwr_temp0 += aIntensityCal[i + 4][calIdx] * (pow(40.0f, order - 1 - i));
				refPwr_temp1 += aIntensityCal[i + 4][calIdx] * (pow(39.0f, order - 1 - i));
			}
			refPwr_temp = 0.3f * (refPwr_temp0 - refPwr_temp1) * distance_f + refPwr_temp0;
		}
	}
	else
	{
		printf("The intensity mode is not right\n");
	}

	refPwr = max(min(refPwr_temp, 500.0f), 4.0f);

	tempInten = (intensityFactor * refPwr) / realPwr;
	if (numOfLasers == 32)
	{
		tempInten = tempInten * CurvesRate[calIdx];
	}
	tempInten = (int)tempInten > 255 ? 255.0f : tempInten;
	//std::cout << tempInten << endl;
	return tempInten; // 0~1
}


void* RSProcess(void *arg)
{
	float azimuth, azimuth_diff, azimuth_corrected_f, intensity;
	int lDistance, azimuth_corrected;
	float temper = 31.0;

	static int initflag = 0;
	static int dataID = 0;
	FILE* RSProcessTimeFile;
	FILE* dataFile;
	UINT64 time64;
	SYSTEMTIME st;

	do
	{
		if (initflag == 0) {
#if OUTPUT_FILE_ENABLE
			//タイミングチャート新規作成
			RSProcessTimeFile = fopen("output/Timing/RSProcessTime.csv", "w");
			fprintf(RSProcessTimeFile, "Name,Hour,Minute,Second,Millisecond\n");
			fclose(RSProcessTimeFile);
			//計測データ新規作成
			dataFile = fopen("output/RSLidar/data.csv", "w");
			fprintf(dataFile, "dataID,packetID,blockID,channelData...\n");
			fclose(dataFile);
#endif
			initflag = 1;
		}
#if OUTPUT_FILE_ENABLE
		//タイミングチャート上書き
		RSProcessTimeFile = fopen("output/Timing/RSProcessTime.csv", "a");
		GetLocalTime(&st);
		SystemTimeToFileTime(&st, (FILETIME*)&time64);
		fprintf(RSProcessTimeFile, "RSProcess%dStart,%d,%d,%d,%d\n", dataID, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
		fclose(RSProcessTimeFile);
#endif

#if OUTPUT_FILE_ENABLE
		//計測データ上書き
		dataFile = fopen("output/RSLidar/data.csv", "a");
#endif
		pthread_mutex_lock(&gStop);  //排他制御ON
		pthread_mutex_lock(&gRecvAndProcMutex);
		for (int i = 0; i < 1200 * rs16_NumOfPackage; i++)
		{
			gProcessedData[i] = gOriginalData[i];
		}
		for (int i = 0; i < 42 * rs16_NumOfPackage; i++)
		{
			gProcessedHead[i] = gOriginalHead[i];
		}
		//lPointBias = gPointBias;
		pthread_mutex_unlock(&gRecvAndProcMutex);
		pthread_mutex_unlock(&gStop);

		for (int lCounti = 0; lCounti < rs16_NumOfPackage; lCounti++)
		{
			for (int lCountj = 0; lCountj < 12; lCountj++)
			{
#if OUTPUT_FILE_ENABLE
				//計測データ上書き
				fprintf(dataFile, "%d,%d,%d,", dataID, lCounti, lCountj);
#endif
				//パケット数20000に達するまで
				if (tempPacketNum < 20000 && tempPacketNum > 0)  // update temperature information per 20000 packets
				{
					tempPacketNum++;
				}
				//パケット数20000に達したら
				else
				{
					//温度更新？
					temper = computeTemperature(gProcessedHead[lCounti * 42 + 38], gProcessedHead[lCounti * 42 + 39]);//pkt.data[38], pkt.data[39]);
					tempPacketNum = 1;
				}

				//パケットiデータjのレーザ水平角度計算(角度分解能0.01degなので、azimuth/100でdeg単位となる)
				azimuth = (float)(256 * (gProcessedData[lCounti * 1200 + lCountj * 100 + 2] & 0xff) + (gProcessedData[lCounti * 1200 + lCountj * 100 + 3] & 0xff));
				/*	TwoCharsInt A;
					A.datain[1] = gProcessedData[lCounti * 1200 + lCountj * 100 + 2];
					A.datain[0] = gProcessedData[lCounti * 1200 + lCountj * 100 + 3];
					azimuth = A.dataout;*/

					//データ0～10
				if (lCountj < 11)
				{
					//次のデータとの角度差を計算
					int azi1, azi2;
					azi1 = 256 * ((gProcessedData[lCounti * 1200 + (lCountj + 1) * 100 + 2] & 0xff) + gProcessedData[lCounti * 1200 + (lCountj + 1) * 100 + 3] & 0xff);
					azi2 = 256 * ((gProcessedData[lCounti * 1200 + lCountj * 100 + 2] & 0xff) + gProcessedData[lCounti * 1200 + lCountj * 100 + 3] & 0xff);
					azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

					// Ingnore the block if the azimuth change abnormal
					//異常な角度差を示すブロックは無視(外れ値除去)
					if (azimuth_diff <= 0.0 || azimuth_diff > 75.0)
					{
						//continue;
					}
				}
				//データ11(次のデータがない)
				else
				{
					//前のデータとの角度差を計算
					int azi1, azi2;
					azi1 = 256 * (gProcessedData[lCounti * 1200 + lCountj * 100 + 2] & 0xff) + (gProcessedData[lCounti * 1200 + lCountj * 100 + 3] & 0xff);
					azi2 = 256 * (gProcessedData[lCounti * 1200 + (lCountj - 1) * 100 + 2] & 0xff) + (gProcessedData[lCounti * 1200 + (lCountj - 1) * 100 + 3] & 0xff);
					azimuth_diff = (float)((36000 + azi1 - azi2) % 36000);

					// Ingnore the block if the azimuth change abnormal
					//異常な角度差を示すブロックは無視(外れ値除去)
					if (azimuth_diff <= 0.0 || azimuth_diff > 75.0)
					{
						//continue;
					}
				}
				
				//for (int lCountk = 0; lCountk < DATA_NUM; lCountk++)  //全レーザ
				for (int lCountk = 0; lCountk < DATA_NUM; lCountk++)  //水平のみ
				{
					//int lCountk_v[DATA_NUM] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
					int lCountk_v[DATA_NUM] = {15, 31};  //15, 31:水平レーザ(+1deg)のみ

					int dsr = lCountk_v[lCountk] % 16;  //0～15を2回繰り返す
					int firing = (int)lCountk_v[lCountk] / 16;  //0 or 1

					//水平方向角度計算(補正？？？)
					//azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr * RS16_DSR_TOFFSET) + (firing * RS16_FIRING_TOFFSET)) / RS16_BLOCK_TDURATION);
					azimuth_corrected_f = azimuth + ((float)dsr * RS16_DSR_TOFFSET + (float)firing * RS16_FIRING_TOFFSET) * ((float)rs16_rpm * 360.0 / 60.0 / 1000.0 / 1000.0) * 100.0;

					azimuth_corrected = ((int)round(azimuth_corrected_f)) % 36000;  // convert to integral value...

					TwoCharsInt tmp;
					//計測距離(2バイト分データ)
					tmp.datain[1] = gProcessedData[lCounti * 1200 + lCountj * 100 + lCountk_v[lCountk] * 3 + 4];//raw->blocks[block].data[k];
					tmp.datain[0] = gProcessedData[lCounti * 1200 + lCountj * 100 + lCountk_v[lCountk] * 3 + 5];// raw->blocks[block].data[k + 1];
					lDistance = tmp.dataout;
					//計測密度(1バイト分データ)->あんま使わない
					tmp.datain[1] = 0x00;
					tmp.datain[0] = gProcessedData[lCounti * 1200 + lCountj * 100 + lCountk_v[lCountk] * 3 + 6];
					intensity = tmp.dataout;//gProcessedData[lCounti * 1200 + lCountj * 100 + lCountk * 3 + 6];

					//Intensity Calibration Code Here
					bool Curvesis_new = true;
					intensity = CalibrateIntensity(intensity, dsr, lDistance);


					float distance2 = lDistance;
					//float distance2 = pixelToDistance(lDistance, dsr);// still dont understand this function
					if (dis_resolution_mode == 0)  //距離分解能5mm
					{
						distance2 = distance2 * DISTANCE_RESOLUTION_NEW;  //[m]
						//printf("distance : %.2f\n", distance2);
					}
					else  //距離分解能10mm
					{
						distance2 = distance2 * DISTANCE_RESOLUTION;  //[m]
					}
					//水平方向角度[rad]、垂直方向角度[rad]
					float arg_horiz = (float)azimuth_corrected / 18000.0f * 3.1415926;
					float arg_vert = Calibration_Angle[dsr] / 180 * 3.1415926;
					//最大距離200[m]、最小距離20[cm]
					float max_distance = 200.0, min_distance = 0.2;

#if OUTPUT_FILE_ENABLE
					//計測データ上書き
					fprintf(dataFile, "%f,%f,", arg_horiz * 180.0f / 3.1415926, distance2);
#endif
					//最終データ作成
					dataD[lCounti * 12 * DATA_NUM + lCountj * DATA_NUM + lCountk] = (long)(distance2 * 1000.0);
					dataA[lCounti * 12 * DATA_NUM + lCountj * DATA_NUM + lCountk] = -arg_horiz;
				}
#if OUTPUT_FILE_ENABLE
				fprintf(dataFile, "\n");
#endif
			}
		}
#if OUTPUT_FILE_ENABLE
		fclose(dataFile);
#endif
		dataID++;
	}while (rs16_thread_mode);

	return nullptr;
}
