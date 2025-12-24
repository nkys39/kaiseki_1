//Roomba irobot control c++ code 
//Modified by Wei Hong
//Date: 12/07/2017
#include <string.h>
#include <io.h>
#include <stdio.h>
#include <time.h>
#include "Serial.h"
#include "Pipe_Comm.h"
// example program demonstrating how functions can be used
HANDLE hTimer = NULL;
using namespace std;

//Variable declarations
HANDLE hCom;
DCB dcb;
OVERLAPPED o = { 0 };
BYTE rxString[3] = { NULL };
DWORD bytesRx = 0;

// ポート番号，ソケット
SOCKET srcSocket;  // 自分
struct sockaddr_in srcAddr;

// 各種パラメータ
int numrcv;
char buffer[1024];

extern void Play_Sound(bool playmode);

unsigned long _stdcall Timer(void*)
{
	int nCount = 0;
	while (nCount < 200)
	{
		WaitForSingleObject(hTimer, 50);
		//printf("Hello");
		//drive(10, 10);
		nCount++;
	}
	roombaStop();
	//cout << "50 secs\n";
	return 0;
}

//インタフェースとの通信（TCP）用 ここから
//void itc_mapInformation(int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], char *mbuf)
//{
//	int n1 = 0, n2 = 0;
//	const char textdata[20] = "0123456789";
//
//	memset(mbuf, 0, sizeof(mbuf));
//	for (int i = 0; i < MAPSIZE; i++) 
//	{
//		for (int j = 0; j < MAPSIZE; j++) 
//		{
//			if (i >= 0 && i < MAPSIZE && j >= 0 && j < MAPSIZE) 
//			{
//				if (omap[i][j] > 10 && amap[i][j] > 0) 
//				{
//					mbuf[n1] = textdata[4];
//					n2++;
//				}
//				else if (omap[i][j] > 10 && amap[i][j] <= 0) 
//				{
//					mbuf[n1] = textdata[3];
//					n2++;
//				}
//				else if (omap[i][j] < -10 && amap[i][j] > 0)
//				{
//					mbuf[n1] = textdata[2];
//				}
//				else if (omap[i][j] < -10 && amap[i][j] == 0)
//				{
//					mbuf[n1] = textdata[0];
//				}
//				else
//				{
//					mbuf[n1] = textdata[1];
//				}
//			}
//			else
//			{
//				mbuf[n1] = textdata[5];
//			}
//			n1++;
//		}
//	}
//}
SOCKET sock0;
socklen_t len;
bool accept_flg = false;
//char map_buf[MAPSIZE * MAPSIZE];
struct sockaddr_in client;

//int *communication_data(char *buf, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], char *nodebuf, int tpath[][2])
//{
//	//SOCKET sock0;
//	//socklen_t len;
//	char recv_buf[500], posi_buf[255], node_buf[255];
//	char v[10];
//	//struct sockaddr_in client;
//	struct timeval timeout;
//	fd_set fds, readfds;
//	int result;
//	int n1, k = 0;
//	int cmode;
//	int fType;
//	int *rnum;
//
//	rnum = (int *)malloc(sizeof(int) * 2);
//	memset(rnum, 0, sizeof(rnum));
//
//	memset(recv_buf, 0, sizeof(recv_buf));
//	len = sizeof(client);
//	FD_ZERO(&readfds);
//	FD_SET(srcSocket, &readfds);
//	timeout.tv_sec = 0;
//	timeout.tv_usec = 1000;
//	memcpy(&fds, &readfds, sizeof(fd_set));
//	result = select(FD_SETSIZE, &fds, (fd_set *)0, (fd_set *)0, &timeout);
//
//	if (result == 0) 
//	{
//		//何もしない
//	}
//	else if (result == -1)
//	{
//		//何もしない
//	}
//	else 
//	{
//		if (accept_flg == false)
//		{
//			sock0 = accept(srcSocket, (struct sockaddr*)&client, &len);
//		}
//		if (sock0 < 0) 
//		{
//			printf("error: accept()\n");
//			exit(EXIT_FAILURE);
//		}
//		else 
//		{
//			if (accept_flg == false)
//			{
//				accept_flg = true;
//			}
//
//			n1 = recv(sock0, recv_buf, sizeof(recv_buf), 0);
//			memset(v, 0, sizeof(v));
//			v[0] = recv_buf[k];
//			//printf("k: %d \n", k);
//			//printf("recv_buf: %d \n", v[0]);
//			cmode = atoi(v);
//			
//			printf("cmode: %d \n", cmode);
//			k++;
//
//			if (cmode == 0 || cmode == 2) 
//			{
//				memset(posi_buf, 0, sizeof(posi_buf));
//				strcpy_s(posi_buf, buf);
//				send(sock0, posi_buf, sizeof(posi_buf), 0);
//				rnum[0] = 11;
//			}
//			else if (cmode == 9)
//			{
//				rnum[0] = 99;
//			}
//			else if (cmode == 7) 
//			{
//				memset(node_buf, 0, sizeof(node_buf));
//				strcpy_s(node_buf, nodebuf);
//				send(sock0, node_buf, sizeof(node_buf), 0);
//			}
//			else if (cmode == 8) 
//			{
//				rnum[0] = 8;
//				v[0] = recv_buf[k];
//				rnum[1] = atoi(v);
//				return rnum;
//			}
//			else if (cmode == 1 || cmode == 3) 
//			{
//				memset(map_buf, 0, sizeof(map_buf));
//				itc_mapInformation(omap, amap, map_buf);
//				send(sock0, map_buf, sizeof(map_buf), 0);
//			}
//			else if (cmode == 4) 
//			{
//				v[0] = recv_buf[k];
//				fType = atoi(v);
//				memset(posi_buf, 0, sizeof(posi_buf));
//				strcpy_s(posi_buf, buf);
//				send(sock0, posi_buf, sizeof(posi_buf), 0);
//				closesocket(sock0);
//				rnum[0] = 4;
//				rnum[1] = fType;
//				return rnum;
//			}
//			else if (cmode == 5) 
//			{
//				memset(posi_buf, 0, sizeof(posi_buf));
//				strcpy_s(posi_buf, buf);
//				send(sock0, posi_buf, sizeof(posi_buf), 0);
//			}
//			else if (cmode == 6) 
//			{
//				memset(posi_buf, 0, sizeof(posi_buf));
//				strcpy_s(posi_buf, buf);
//				send(sock0, posi_buf, sizeof(posi_buf), 0);
//				int tct;
//				int ptx = 0, pty = 0;
//				v[0] = recv_buf[k];
//				int nmode = atoi(v);
//				k++;
//
//				if (nmode == 2) 
//				{
//					v[0] = recv_buf[k];
//					tct = 10 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					tct += atoi(v);
//					k++;
//					for (int i = 0; i<tct; i++) 
//					{
//						v[0] = recv_buf[k];
//						ptx = 1000 * atoi(v);
//						k++;
//						v[0] = recv_buf[k];
//						ptx += 100 * atoi(v);
//						k++;
//						v[0] = recv_buf[k];
//						ptx += 10 * atoi(v);
//						k++;
//						v[0] = recv_buf[k];
//						ptx += atoi(v);
//						k++;
//
//						v[0] = recv_buf[k];
//						pty = 1000 * atoi(v);
//						k++;
//						v[0] = recv_buf[k];
//						pty += 100 * atoi(v);
//						k++;
//						v[0] = recv_buf[k];
//						pty += 10 * atoi(v);
//						k++;
//						v[0] = recv_buf[k];
//						pty += atoi(v);
//						k++;
//						tpath[i][0] = ptx;
//						tpath[i][1] = pty;
//					}
//				}
//				
//				if (nmode == 3) 
//				{
//					int i = 0;
//					tct = 1;
//					v[0] = recv_buf[k];
//					ptx = 1000 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					ptx += 100 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					ptx += 10 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					ptx += atoi(v);
//					k++;
//
//					v[0] = recv_buf[k];
//					pty = 1000 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					pty += 100 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					pty += 10 * atoi(v);
//					k++;
//					v[0] = recv_buf[k];
//					pty += atoi(v);
//					k++;
//					tpath[i][0] = ptx;
//					tpath[i][1] = pty;
//				}
//
//				rnum[0] = 6;
//				rnum[1] = tct;
//				return rnum;
//
//			}
//
//			//closesocket(sock0);
//		}
//	}
//	return rnum;
//}

void ServerInitialise() 
{

	// Windows の場合
	WSADATA data;
	WSAStartup(MAKEWORD(2, 0), &data);
	// sockaddr_in 構造体のセット
	memset(&srcAddr, 0, sizeof(srcAddr));
	srcAddr.sin_port = htons(tcpPort);
	srcAddr.sin_family = AF_INET;
	srcAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	// ソケットの生成（ストリーム型）
	srcSocket = socket(AF_INET, SOCK_STREAM, 0);
	// ソケットのバインド
	bind(srcSocket, (struct sockaddr *) &srcAddr, sizeof(srcAddr));
	// 接続の許可
	listen(srcSocket, 10);
	printf("Server initialized.\n");

}


//ルンバとの通信（シリアル）用 ここから
void iRobotInitialise(void) 
{
	// Creates port settings (for COM1, change number if different port is used)
	char roombaPortStr[255];
	sprintf_s(roombaPortStr, "\\\\.\\COM%d", roombaPort);
	hCom = CreateFile(TEXT(roombaPortStr),
		GENERIC_READ | GENERIC_WRITE,
		0,	
		NULL,
		OPEN_EXISTING,
		NULL,
		NULL
	);

	// Error to catch wrong port
	if (hCom == INVALID_HANDLE_VALUE)
	{
		// Handle the error. 
		RobotStatus = MOTOR_ARDUINO_COMM_ERR;
		StatusSet(RobotStatus);
		printf("Probable wrong port number(COM %d)\n. CreateFile failed with error %d.\n", roombaPort, GetLastError());
		return;
	}
	else 
	{
#if defined(_ROOMBA)
		printf("iRobot Port connected. \n");
#else
		printf("Speed control Arduino Port(COM %d) connected. \n", roombaPort);
#endif
	}

	// Get dcb from com port
	GetCommState(hCom, &dcb);
	if (GetCommState(hCom, &dcb) == 0)
	{
		// Handle the error. 
		RobotStatus = MOTOR_ARDUINO_CONNECT_ERR;
		StatusSet(RobotStatus);
		printf("GetCommState failed with error %d.\n", GetLastError());
		return;
	}
	else 
	{
		//何もしない
	}

	// Set com port params
	dcb.BaudRate = CBR_38400;		// Arduinoの通信が不安定なため115200→38400に変更
	dcb.StopBits = ONESTOPBIT;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;


	if (SetCommState(hCom, &dcb) == 0)
	{
		// Handle the error. 
		RobotStatus = MOTOR_ARDUINO_COMM_ERR;
		StatusSet(RobotStatus);
		printf("SetCommState failed with error %d.\n", GetLastError());
		return;
	}
	else 
	{
		//何もしない
	}

	Sleep(50);
}

//
// controlMotor:モータ速度コントロール
// rightspeed  :右モータ速度
// leftspeed   :左モータ速度
// 戻り値　　  :なし
//
void controlMotor(short rightspeed, short leftspeed)
{
	BYTE driveBytes[6] = { DRIVE_DIRECT,0,0,0,0, NULL };
	//printf("----------------------%d,%d\n", rightspeed, leftspeed);
	unsigned char *tmp;

	driveBytes[0] = DRIVE_DIRECT;

	if (rightspeed > maxSpeed)
	{
		rightspeed = maxSpeed;
	}
	else if (rightspeed < -maxSpeed)
	{
		rightspeed = -maxSpeed;
	}

	if (leftspeed > maxSpeed)
	{
		leftspeed = maxSpeed;
	}
	else if (leftspeed < -maxSpeed)
	{
		leftspeed = -maxSpeed;
	}

	tmp = (unsigned char*)(&rightspeed);
	driveBytes[2] = *tmp;
	driveBytes[1] = *(tmp + 1);

	tmp = (unsigned char*)(&leftspeed);
	driveBytes[4] = *tmp;
	driveBytes[3] = *(tmp + 1);

#if !defined(MOTOR_NON)
	WriteFile(hCom, &driveBytes, 5, NULL, &o);
#endif

	if ((rightspeed == 0) && (leftspeed == 0))
	{
		Play_Sound(false);
	}
	else
	{
		Play_Sound(true);
	}

	try
	{
		char dirName[29] = "C:\\kinden\\log\\motorSpeed.log";

		FILE *fp;
		fopen_s(&fp, dirName, "a");

		SYSTEMTIME sys;
		GetLocalTime(&sys);
		fprintf(fp, "%04d.%02d.%02d %02d:%02d:%02d.%03d ", sys.wYear, sys.wMonth, sys.wDay, sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
		fprintf(fp, "R=%4dmm/s, L=%4dmm/s\n", rightspeed, leftspeed);

		fclose(fp);
	}
	catch (...)
	{
	}
}

int getBumpData() 
{
#if defined(_ROOMBA)
	BYTE sendBytes[6] = { ZERO,0,0,0,0, NULL };
	int res;
	sendBytes[0] = SENSORS;
	sendBytes[1] = 0x07;
	WriteFile(hCom, &sendBytes, 6, NULL, &o);
	Sleep(15);
	ReadFile(hCom, &rxString, 1, &bytesRx, &o);
	res = rxString[0];

	return res;
#else
	int res=0;

	if (bumper[0] == 1)
	{
		res = 1;
	}
	else if (bumper[1] == 1)
	{
		res = 2;
	}
	else if (bumper[2] == 1)
	{
		res = 3;
	}
	else
	{
		res = 0;
	}

	if (fall_sensor)
	{
		if (falling[0] >= fallThresHold && res == 0)
		{
			res = 4;
		}
		else if (falling[1] >= fallThresHold && res == 0)
		{
			res = 8;
		}
	}
	return res;

#endif
}

void roombaStop(void) 
{
	BYTE stopBytes[6] = { DRIVE, 0, 0, 0, 0, NULL };
	WriteFile(hCom, stopBytes, 5, NULL, &o);
}

void roombaSafe(void) 
{
	BYTE safeBytes[6] = { SAFE, 0, 0, 0, 0, NULL };
	WriteFile(hCom, safeBytes, 5, NULL, &o);
}

void roombaStart(void)
{
	BYTE startBytes[6] = { START, 0, 0, 0, 0, NULL };
	WriteFile(hCom, startBytes, 5, NULL, &o);
}




std::string filename;
int sendCounter = 0;
bool accept_flag = false;
cv::Mat src_img;
IplImage *src_img_org;
//char cpImage_data[MAPSIZE * MAPSIZE * 3];

char *sImage_data;
int sImage_size;
bool socket_send_busy = false;
bool slam_copy_busy = false;



bool tcpConnect()
{
	//SOCKET sock0;
	socklen_t len;
	//char recv_buf[500], posi_buf[255], node_buf[255];
	//char v[10];
	struct sockaddr_in client;
	struct timeval timeout;
	fd_set fds, readfds;
	int result;
	//int n1, k = 0;
	//int cmode;
	//int* rnum;

	//rnum = (int*)malloc(sizeof(int) * 2);
	//memset(rnum, 0, sizeof(rnum));

	//memset(recv_buf, 0, sizeof(recv_buf));
	len = sizeof(client);
	FD_ZERO(&readfds);
	FD_SET(srcSocket, &readfds);
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;
	memcpy(&fds, &readfds, sizeof(fd_set));
	result = select(FD_SETSIZE, &fds, (fd_set*)0, (fd_set*)0, &timeout);

	if (result == 0)
	{
		//何もしない
	}
	else if (result == -1)
	{
		//何もしない
	}
	else
	{
		if (accept_flag == false)
		{
			sock0 = accept(srcSocket, (struct sockaddr*)&client, &len);
		}
		if (sock0 < 0)
		{
			printf("error: accept()\n");
			return false;
		}
		else
		{
			accept_flag = true;
			return true;
		}
	}
	return false;
}


int receiveMessage()
{
	int n1, k = 0;
	char recv_buf[500];
	char v[10];
	int cmode;

	//memset(recv_buf, 0, sizeof(recv_buf));

	n1 = recv(sock0, recv_buf, sizeof(recv_buf), 0);

	//printf("receiveRet:%d\n", n1);
	memset(v, 0, sizeof(v));
	v[0] = recv_buf[k];
	cmode = v[0];

	if (n1 < 0)		//クライアントが切断
	{
		closesocket(sock0);
		cmode = n1;
	}
	return cmode;
}

int sendMessage(int cmode)
{
	switch (cmode)
	{
	case 100:
		while (slam_copy_busy)	//slamが地図情報更新中は待つ
		{
			Sleep(10);
		}

		socket_send_busy = true;	//ソケット送信開始

		int32_t image_info[5];
		int32_t cols;
		int32_t cols_size;

		image_info[0] = (int32_t)src_img.rows;
		image_info[1] = (int32_t)src_img.cols;
		image_info[2] = (int32_t)src_img.type();
		image_info[3] = (int32_t)src_img.total();
		image_info[4] = (int32_t)src_img.elemSize();
		//printf("Send rows:%d, cols:%d, type:%d, total:%d, elemSize:%d, sImagesize:%d\r\n", image_info[0], image_info[1], image_info[2], image_info[3], image_info[4], sImage_size);

		try
		{
			send(sock0, (char*)image_info, sizeof(image_info), 0);	//画像情報を送信
		}
		catch (...)
		{
			printf("slam map info send error.");
		}

		try
		{
			send(sock0, sImage_data, (sImage_size * sizeof(char)), 0);		//画像を送信
		}
		catch (...)
		{
			printf("slam map data send error.");
		}

		//sendCounter++;
		//printf("Send Num : %d\n", sendCounter);

		socket_send_busy = false;	//ソケット送信終了フラグ


		break;
	default:
		break;
	}
	return 1;
}

void socketMain()
{
	bool ret = false;
	int cmode;

	while (true)
	{
		while (ret == false)
		{
			Sleep(10);

			ret = tcpConnect();

		}
		while (true)
		{
			cmode = receiveMessage();

			if (cmode > 0)
			{
				sendMessage(cmode);
			}
			else if (cmode < 0)
			{
				ret = false;
				accept_flag = false;
				break;
			}
			Sleep(1);
		}
	}
}

void sendImage(IplImage* image)
{
	if (socket_send_busy == false)	//ソケット通信が送信中でなければデータ更新
	{
		slam_copy_busy = true;	//slam地図情報更新開始

		src_img = cv::cvarrToMat(image);

		free(sImage_data);
		sImage_data = (char *)malloc((image->imageSize) * sizeof(char));

		memcpy(sImage_data, image->imageData, image->imageSize);

		sImage_size = image->imageSize;
		//デバッグ
		//char j = 0x10;
		//memset(sImage_data, 0, sizeof(sImage_data));
		//for (int i = 0; i < sizeof(sImage_data); i++)
		//{
		//	sImage_data[i] = j;

		//	j++;
		//	if (j == 0x0)
		//	{
		//		j = 0x10;
		//	}
		//}
		//FILE* fp = fopen("sendData.log", "w");
		//for (int i = 0; i < sizeof(sImage_data); i++)
		//{
		//	fprintf(fp, "%c", sImage_data[i]);
		//}
		//fclose(fp);
		//ここまで

		slam_copy_busy = false;	//slam地図情報更新終了
	}
}