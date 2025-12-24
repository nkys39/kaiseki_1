#include <stdio.h>
#include <string.h>
#include <winsock2.h>
#include <time.h>
#include "sensing.h"
#include "Pipe_Comm.h"
#include "Serial.h"
HANDLE hArdnCom;
DCB ardnDcb;
COMMTIMEOUTS timeOut;
//char BCC[1];

int sensorComm_Init()
{
	char ardnPortStr[255];
	sprintf_s(ardnPortStr, "\\\\.\\COM%d", ardnPort);

	hArdnCom = CreateFile(TEXT(ardnPortStr),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		NULL,
		NULL
	);

	// Error to catch wrong port
	if (hArdnCom == INVALID_HANDLE_VALUE)
	{
		// Handle the error. 
		RobotStatus = SENSOR_ARDUINO_COMM_ERR;
		StatusSet(RobotStatus);
		printf("Probable wrong Sensor Ardino port number(COM %d)\n. CreateFile failed with error %d.\n", ardnPort, GetLastError());
		return -1;
	}
	else
	{
		printf("Sensor Ardino Port(COM %d) connected. \n", ardnPort);
	}

	// Get dcb from com port
	GetCommState(hArdnCom, &ardnDcb);
	if (GetCommState(hArdnCom, &ardnDcb) == 0)
	{
		// Handle the error. 
		RobotStatus = SENSOR_ARDUINO_CONNECT_ERR;
		StatusSet(RobotStatus);
		printf("Sensor Ardino GetCommState failed with error %d.\n", GetLastError());
		return -1;
	}
	else
	{
		printf("Connect to Sensor Ardino\n");
	}

	// Set com port params
	ardnDcb.BaudRate = CBR_38400;		// Arduinoの通信が不安定なため38400に変更すること
	ardnDcb.StopBits = ONESTOPBIT;
	ardnDcb.ByteSize = 8;
	ardnDcb.Parity = NOPARITY;
	SetCommState(hArdnCom, &ardnDcb);

	GetCommTimeouts(hArdnCom, &timeOut);
	timeOut.ReadIntervalTimeout = MAXDWORD;
	timeOut.ReadTotalTimeoutMultiplier = MAXDWORD;
	timeOut.ReadTotalTimeoutConstant = 1000;
	timeOut.WriteTotalTimeoutMultiplier = 0;
	timeOut.WriteTotalTimeoutConstant = 0;
	SetCommTimeouts(hArdnCom, &timeOut);        // タイムアウトパラメータを設定

	return 0;

}

void SensorDataReceive()
{
	DWORD errors;
	COMSTAT comStat;
	clock_t start = 0;
	clock_t end = 0;
	int lengthOfRecieved = 1000;

	DWORD numberOfPut = 0;
	unsigned char recievedDataBuf[SENC_RCV_LEN_BUF];
	unsigned char recievedData[SENC_RCV_LEN];
	unsigned char ret=0;

	bool readBufFlg = false;
	unsigned char numOfRead = 0;

	PurgeComm(hArdnCom, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);

	//FILE* fp = fopen("C:\\kinden\\log\\SensorArduino.log", "w");
	//fclose(fp);
	//FILE* fpp = fopen("C:\\kinden\\log\\SensorArduinoRead.log", "w");
	//fclose(fpp);

	while (true)
	{
		//if (end - start < READ_CYCLE)
		//{
		//	Sleep(READ_CYCLE - (end - start));
		//}
		//else
		//{
		//	Sleep(READ_CYCLE);
		//}
		//start = clock();

		ClearCommError(hArdnCom, &errors, &comStat);

		//while (SENC_RCV_LEN > comStat.cbInQue)
		//{
		//	if (clock() - start > READ_CYCLE)
		//	{
		//		printf("Sensor Arduino Time Out\n");
		//		break;
		//	}
		//	ClearCommError(hArdnCom, &errors, &comStat);
		//}

		//fp = fopen("C:\\kinden\\log\\SensorArduino.log", "a");
		//fpp = fopen("C:\\kinden\\log\\SensorArduinoRead.log", "a");

		memset(recievedDataBuf, 0, sizeof(recievedDataBuf));
		memset(recievedData, 0, sizeof(recievedData));

		numberOfPut = 0;
		while (numberOfPut == 0)
		{
			ret = ReadFile(hArdnCom, recievedDataBuf, lengthOfRecieved, &numberOfPut, NULL);
			//logSave(fpp, recievedDataBuf);
			Sleep(10);
		}
		if ((ret == 0) || ((ret != 0) && numberOfPut == 0))
		{
			printf("Sensor Arduino Data Receive Error.\n");
			//fprintf(fp, "Sensor Ardino Data Receive Error.\n");
			//fclose(fp);
			//fclose(fpp);
			continue;
		}

		int i;

		for (i = 0; i < numberOfPut; i++)
		{
			if (recievedDataBuf[i] == CODE_T)
			{
				if ((i + SENC_RCV_LEN) > numberOfPut)
				{
					readBufFlg = true;
					numOfRead = numberOfPut - i;

					memcpy(&recievedData[0], &recievedDataBuf[i], sizeof(unsigned char) * ((size_t)numOfRead));
				}
				else
				{
					memcpy(&recievedData[0], &recievedDataBuf[i], sizeof(unsigned char) * ((size_t)SENC_RCV_LEN));
				}
				break;
			}
		}

		if (i == numberOfPut)
		{
			continue;
		}

		if (readBufFlg == true)
		{
			readBufFlg = false;
			numberOfPut = 0;
			while (numberOfPut == 0)
			{
				ret = ReadFile(hArdnCom, recievedDataBuf, lengthOfRecieved, &numberOfPut, NULL);
				//logSave(fpp, recievedDataBuf);
				Sleep(10);
			}
			//fprintf(fpp, "\n");
			//logSave(fpp, recievedDataBuf);

			if ((ret == 0) || ((ret != 0) && numberOfPut == 0))
			{
				printf("Sensor Arduino Data Receive Error.\n");
				//fprintf(fp, "Sensor Arduino Data Receive Error.\n");
				//fclose(fp);
				//fclose(fpp);
				continue;
			}

			memcpy(&recievedData[numOfRead], &recievedDataBuf[0], sizeof(unsigned char) * (size_t)(SENC_RCV_LEN - numOfRead));
		}
		//logSave(fp, recievedData);

#if defined(_SUM_CHECK)
		// SUMチェック
		int integral = 0;
		unsigned char sum = 0;

		// 受信内容チェック
		if (recievedData[0] != 0x54 || recievedData[1] != 0x43 || recievedData[2] != 0x52)
		{
			//RobotStatus = SENSOR_ARDUINO_CONNECT_ERR;
			//StatusSet(RobotStatus);
			printf("Sensor Arduino Data Error\n");
			//fprintf(fp, " Sensor Arduino Data Error.");
		}
		else
		{
			for (int i = 0; i < SENC_RCV_LEN - 3; i++)
			{
				integral += recievedData[i];
			}
			sum = (integral) & 0xFF;


			if (recievedData[15] != sum)
			{
				//RobotStatus = SENSOR_ARDUINO_CONNECT_ERR;
				//StatusSet(RobotStatus);
				printf("Sensor Arduino SUM Error\n");
				//fprintf(fp," Sensor Arduino SUM Error.");
			}
			else
			{
				// データ更新
				bumper[0] = recievedData[3] - 0x30;
				bumper[1] = recievedData[4] - 0x30;
				bumper[2] = recievedData[5] - 0x30;

				falling[0] = (recievedData[7] - 0x30) * 10 + (recievedData[8] - 0x30);
				falling[1] = (recievedData[9] - 0x30) * 10 + (recievedData[10] - 0x30);

				//printf("Sensor Data Bumper1:%d, Bumper2:%d, Bumper3:%d\n", bumper[0], bumper[1], bumper[2]);
				//printf("Sensor Data Fall1:%d, Fall2:%d\n", falling[0], falling[1]);
				//printf("Sensor Shutdown: %d\n", recievedData[12]);

				// シャットダウン信号でWindows終了
				if (recievedData[12] == 0x00)
				{
					printf("ROBOT PC shutdown...\n");
					//fprintf(fp, " ROBOT PC shutdown...");
					//fprintf(fp, "\n");

					Shutdown();
				}
			}
		}

		//fprintf(fpp, "\n");
		//fclose(fpp);
		//fprintf(fp, "\n");
		//fclose(fp);

#else
		// 受信内容チェック
		if (recievedData[0] != 0x54 || recievedData[1] != 0x43 || recievedData[2] != 0x52)
		{
			//RobotStatus = SENSOR_ARDUINO_CONNECT_ERR;
			//StatusSet(RobotStatus);
			//printf("Sensor Ardino Data Error\n");
		}
		else
		{
			// データ更新
			bumper[0] = recievedData[3] - 0x30;
			bumper[1] = recievedData[4] - 0x30;
			bumper[2] = recievedData[5] - 0x30;

			falling[0] = (recievedData[7] - 0x30) * 10 + (recievedData[8] - 0x30);
			falling[1] = (recievedData[9] - 0x30) * 10 + (recievedData[10] - 0x30);

			//if ((falling[0] >= FALL_DIST) || (falling[1] >= FALL_DIST))
			//{
			//	controlMotor(0, 0);
			//}

			//printf("Sensor Data Bumper1:%d, Bumper2:%d, Bumper3:%d\n", bumper[0], bumper[1], bumper[2]);
			//printf("Sensor Data Fall1:%d, Fall2:%d\n", falling[0], falling[1]);
			//printf("Sensor Shutdown: %d\n", recievedData[12]);

			// シャットダウン信号でWindows終了
			if (recievedData[12] == 0x00) Shutdown();
		}

#endif
		//end = clock();
		Sleep(10);
	}
	return;
}

BOOL EnablePrivileges(LPTSTR lpPrivilegeName, BOOL bEnable)
{
	HANDLE hToken;
	LUID luid;
	TOKEN_PRIVILEGES tokenPrivileges;
	BOOL bRet;


	// プロセストークンを取得
	bRet = OpenProcessToken(GetCurrentProcess(),
		TOKEN_ADJUST_PRIVILEGES | TOKEN_QUERY,
		&hToken);
	if (!bRet) {
		return FALSE;
	}

	// 特権に対応するLUID(ローカル一意識別子)を取得
	bRet = LookupPrivilegeValue(NULL, lpPrivilegeName, &luid);
	if (bRet) {

		// TOKEN_PRIVILEGES型のオブジェクトに、LUID(ローカル一意識別子)と特権の属性(有効にするか無効にするか)を指定
		tokenPrivileges.PrivilegeCount = 1;
		tokenPrivileges.Privileges[0].Luid = luid;
		tokenPrivileges.Privileges[0].Attributes = bEnable ? SE_PRIVILEGE_ENABLED : 0;

		// 特権有効
		AdjustTokenPrivileges(hToken,
			FALSE,
			&tokenPrivileges, 0, 0, 0);

		bRet = GetLastError() == ERROR_SUCCESS;

	}

	CloseHandle(hToken);

	return bRet;
}

int Shutdown()
{
	//SE_SHUTDOWN_NAME(シャットダウン特権) を有効にする
	if (!EnablePrivileges(SE_SHUTDOWN_NAME, TRUE)) {
		puts("EnablePrivileges failed");
		return 1;
	}

	if (!ExitWindowsEx(EWX_SHUTDOWN | EWX_FORCE, 0)) {
		puts("ExitWindowsEx failed");
		return 1;
	}

	puts("ExitWindowsEx succeeded !");

	return 0;
}


void logSave(FILE* fp, unsigned char buf[])
{
	SYSTEMTIME sysTime;
	GetLocalTime(&sysTime);

	fprintf(fp, "%04d.%02d.%02d %02d:%02d:%02d.%03d : ", sysTime.wYear, sysTime.wMonth, sysTime.wDay, sysTime.wHour, sysTime.wMinute, sysTime.wSecond, sysTime.wMilliseconds);
	fprintf(fp, "%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x",
		buf[0],
		buf[1],
		buf[2],
		buf[3],
		buf[4],
		buf[5],
		buf[6],
		buf[7],
		buf[8],
		buf[9],
		buf[10],
		buf[11],
		buf[12],
		buf[13],
		buf[14],
		buf[15],
		buf[16],
		buf[17]);

}
