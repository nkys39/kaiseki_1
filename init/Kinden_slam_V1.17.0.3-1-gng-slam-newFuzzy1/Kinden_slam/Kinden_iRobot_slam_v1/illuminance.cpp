#include <stdio.h>
#include <string.h>
#include <winsock2.h>
#include <time.h>
#include "illuminance.h"
#include "Pipe_Comm.h"
HANDLE hIllmCom;
DCB illmDcb;
COMMTIMEOUTS timeout;
char BCC[2];

int T10A_Initial()
{
	char illmPortStr[255];
	sprintf_s(illmPortStr, "\\\\.\\COM%d", illmPort);

	hIllmCom = CreateFile(TEXT(illmPortStr),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		NULL,
		NULL
	);

	// UpperPortがエラーだったらBackPortを使用する
	if (hIllmCom == INVALID_HANDLE_VALUE)
	{
		sprintf_s(illmPortStr, "\\\\.\\COM%d", illmPort2);

		hIllmCom = CreateFile(TEXT(illmPortStr),
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			NULL,
			NULL
		);
		// Error to catch wrong port
		if (hIllmCom == INVALID_HANDLE_VALUE)
		{
			// Handle the error. 
			RobotStatus = ILLM_COMM_ERR;
			StatusSet(RobotStatus);

			printf("Probable wrong T-10A port number\n. CreateFile failed with error %d.\n", GetLastError());
			return -1;
		}
		else
		{
			printf("T-10A BackPort connected. \n");
		}
	}
	else
	{
		printf("T-10A UpperPort connected. \n");
	}

	// Get dcb from com port
	GetCommState(hIllmCom, &illmDcb);
	if (GetCommState(hIllmCom, &illmDcb) == 0)
	{
		// Handle the error. 
		RobotStatus = ILLM_COMM_ERR;
		StatusSet(RobotStatus);
		printf("T-10A GetCommState failed with error %d.\n", GetLastError());
		return -1;
	}
	else
	{
		printf("Connect to T-10A\n");
	}

	// Set com port params
	illmDcb.BaudRate = CBR_9600;
	illmDcb.StopBits = ONESTOPBIT;
	illmDcb.ByteSize = 7;
	illmDcb.Parity = EVENPARITY;
	SetCommState(hIllmCom, &illmDcb);

	GetCommTimeouts(hIllmCom, &timeout);
	timeout.ReadIntervalTimeout = MAXDWORD;
	timeout.ReadTotalTimeoutMultiplier = MAXDWORD;
	timeout.ReadTotalTimeoutConstant = 1000;
	timeout.WriteTotalTimeoutMultiplier = 0;
	timeout.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(hIllmCom, &timeout);        // タイムアウトパラメータを設定

	T_10A_PC_MODE();
	illmData = getIlluminanceData(3000);
	Sleep(500);
	return 0;
}

int T_10A_PC_MODE()
{
	PurgeComm(hIllmCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	char sendData[14] = { 0x02, 0x30, 0x30, 0x35, 0x34, 0x31, 0x20, 0x20, 0x20, 0x03, 0x31, 0x33, 0x0D, 0x0A };
	char RecievedData[14] = { 0x02, 0x30, 0x30, 0x35, 0x34, 0x20, 0x20, 0x20, 0x20, 0x03, 0x30, 0x32, 0x0D, 0x0A };
	DWORD lengthOfSent = 14;
	DWORD numberOfPut;
	char recievedData[14];
	BCC_Calculate(RecievedData, 10);

	WriteFile(hIllmCom, sendData, lengthOfSent, &numberOfPut, NULL);
	Sleep(500);

	// T-10Aからの応答を受信
	DWORD errors;
	COMSTAT comStat;
	clock_t start = clock();
	clock_t end;
	int lengthOfRecieved = 0;
	ClearCommError(hIllmCom, &errors, &comStat);
	lengthOfRecieved = comStat.cbInQue;
	while (lengthOfRecieved != 14)
	{
		lengthOfRecieved = comStat.cbInQue;
		end = clock();
		if ((double)(end - start) > ILLM_TIMEOUT)
		{
			RobotStatus = ILLM_COMM_ERR;
			StatusSet(RobotStatus);
			printf("T-10A Time out of connect\n");
			return -1;
		}
	}

	ReadFile(hIllmCom, recievedData, lengthOfRecieved, &numberOfPut, NULL);

	if (memcmp(recievedData, RecievedData, 14) != 0)
	{
		RobotStatus = ILLM_COMM_ERR;
		StatusSet(RobotStatus);
		printf("T-10A PC mode setting failure\n");
		return -1;
	}

	// PC接続モード設定を行ったら必ず500msec待つ
	Sleep(500);
	return 0;

}

double getIlluminanceData(int wait)
{
	// 照度データ要求コマンド
	char sendData[14] = { 0x02, 0x30, 0x30, 0x31, 0x30, 0x30, 0x32, 0x30, 0x30, 0x03, 0x30, 0x30, 0x0D, 0x0A };
	
	PurgeComm(hIllmCom,	PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	// T-10Aからの応答を受信
	DWORD errors;
	DWORD lengthOfSent = 14;
	COMSTAT comStat;
	clock_t start = clock();
	clock_t end;
	int lengthOfRecieved = 0;
	DWORD numberOfPut;
	char recievedData[ILLM_RCV_LEN];

	WriteFile(hIllmCom, sendData, lengthOfSent, &numberOfPut, NULL);
	Sleep(wait);

	ClearCommError(hIllmCom, &errors, &comStat);
	lengthOfRecieved = comStat.cbInQue;
	while (lengthOfRecieved != ILLM_RCV_LEN)
	{
		lengthOfRecieved = comStat.cbInQue;
		end = clock();
		if ((double)(end - start) > ILLM_TIMEOUT)
		{
			RobotStatus = ILLM_COMM_ERR;
			StatusSet(RobotStatus);
			printf("T-10A Time out of GetData\n");
			return -1;
		}
	}
	ReadFile(hIllmCom, recievedData, lengthOfRecieved, &numberOfPut, NULL);

	// BCCチェック
	BCC_Calculate(recievedData, ILLM_RCV_LEN - 4);
	if (BCC[0] != recievedData[28] || BCC[1] != recievedData[29])
	{
		printf("T-10A getIlluminanceData BCC Error\n");
		return -1;
	}

	int illm[4];
	memset(illm, 0, sizeof(illm));

	illm[0] = recievedData[10] - 0x30; illm[1] = recievedData[11] - 0x30;
	illm[2] = recievedData[12] - 0x30; illm[3] = recievedData[13] - 0x30;
	if (illm[0] < 0x0) illm[0] = 0x0; if (illm[1] < 0x0) illm[1] = 0x0;
	if (illm[2] < 0x0) illm[2] = 0x0; if (illm[3] < 0x0) illm[3] = 0x0;

	illm[0] *= 1000; illm[1] *= 100; illm[2] *= 10; illm[3] *= 1;
	illm[0] = illm[0] + illm[1] + illm[2] + illm[3];

	double illmBuf = illm[0] * exponent[recievedData[14] - 48];
	if (recievedData[9] == 0x2D)
	{
		illmBuf = -illmBuf;
	}
	//printf("ill=%f\n", illmBuf);

	return illmBuf;
}

// BCC算出
// Command:STXからETXまでのデータを格納すること
// len:Commandのバイト数
//
void BCC_Calculate(char *Command, int len)
{
	int intBCC = 0;

	memset(BCC, 0, sizeof(BCC));

	// STXとETXを除いたデータで算出する
	for (int i = 1; i < len; i++)
	{
		intBCC = intBCC ^ Command[i];
	}

	char strBCC[3];
	sprintf_s(strBCC, "%02X", intBCC);

	BCC[0] = strBCC[0];
	BCC[1] = strBCC[1];

	return;
}

void illuminanceThread()
{
	while (true)
	{
		//T_10A_PC_MODE();
		illmData = getIlluminanceData(100);
		Sleep(0);
	}

}

