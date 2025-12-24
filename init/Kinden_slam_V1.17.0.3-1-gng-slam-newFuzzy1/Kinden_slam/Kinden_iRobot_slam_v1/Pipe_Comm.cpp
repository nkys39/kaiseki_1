#include <string.h>
#include <stdio.h>
#include <windows.h>
#include <iostream>
#include <io.h>
#include <stdlib.h>
#include <algorithm>

#include "Pipe_Comm.h"
#include "Def.h"

HANDLE pipeHandle;

int PipeCreate()
{
	//pipeHandle = CreateFile("\\\\.\\pipe\\ServiceComm",
	//	//GENERIC_READ | GENERIC_WRITE,
	//	GENERIC_ALL,
	//	0,
	//	NULL,
	//	OPEN_EXISTING,
	//	0,
	//	NULL);

	//if (pipeHandle == INVALID_HANDLE_VALUE)
	//{
	//	return -1;
	//}

	pipeHandle = CreateNamedPipe("\\\\.\\pipe\\ServiceComm",
		PIPE_ACCESS_DUPLEX,
		PIPE_TYPE_MESSAGE,
		1,
		sizeof(PipeReceive),
		sizeof(PipeReceive),
		1000,
		NULL);

	if (pipeHandle == INVALID_HANDLE_VALUE)
	{
		std::cout << "パイプ作成に失敗" << std::endl;
		return -1;
	}

	std::cout << "クライアント起動待ち.." << std::endl;
	ConnectNamedPipe(pipeHandle, NULL);
	std::cout << "クライアント起動" << std::endl;

	return 0;
}

void PipeCommReceive()
{
	std::string rcvBuff;
	std::string::size_type command;

	FILE *fp;
	char pathname[255];
	sprintf_s(pathname, "C:\\kinden\\cmd\\cmd.txt");
	char cmdBuff[255];

	fopen_s(&fp, pathname, "w");
	fclose(fp);

	while (true)
	{
		Sleep(500);

		try
		{
			fopen_s(&fp, pathname, "r");
			fscanf_s(fp, "%s\n", PipeReceive, (unsigned int)sizeof(PipeReceive));


			if (strcmp(cmdBuff, PipeReceive) == 0)
			{
				memset(PipeReceive, 0x0, sizeof(PipeReceive));
				//continue;
			}
			else
			{
				memcpy(&cmdBuff, PipeReceive, sizeof(PipeReceive));
				std::cout << PipeReceive << std::endl;
			}
		}
		catch (...)
		{
			fclose(fp);
		}

		rcvBuff = std::string(PipeReceive);

		fclose(fp);

		// ロボットInitPosition受信
		command = rcvBuff.find("InitPosition");
		if (command != std::string::npos)
		{
			int ret;
			ret = sscanf_s(PipeReceive, "InitPosition,%d,%d,%d,%lf,%lf,%d", &stratPosition[0], &stratPosition[1], &stratPosition[2], &tr, &area_length_tmp, &judge_range_tmp);	//x,y,angle,TR,探索範囲,位置ずれ
			memset(PipeReceive, 0, sizeof(PipeReceive));
			//memset(szBuff, 0, sizeof(szBuff));
		}
	}

	return;

}

void PipeCommSend(char *szBuff)
{
	char szBuff2[2];
	DWORD dwNumberofBytesWritten;

	szBuff2[0] = *szBuff;
	szBuff2[1] = '\n';
	char timeout = 0;

	while (true)
	{
		if (WriteFile(pipeHandle, szBuff2, sizeof(szBuff), &dwNumberofBytesWritten, NULL) != false)
		{
			break;
		}
		else
		{
			timeout++;
			Sleep(100);

			if (timeout > 10)
			{
				break;		//10回失敗したら送信処理を抜ける。
			}
		}
	}

	return;
}

void MapFileNameReceive()
{
	std::string namebuff;

	while (true)
	{
		namebuff = std::string(PipeReceive);
		std::string::size_type trip_png = namebuff.find(".png");
		if (trip_png != std::string::npos)
		{
			break;
		}

	}
	memcpy(mapFileName, PipeReceive, sizeof(PipeReceive));

//	std::cout << PipeReceive << std::endl;
}



void StatusSet(char status)
{
	char pathname[255];
	FILE *mfp3;

	sprintf_s(pathname, "C:\\kinden\\cmd\\status.txt");
	fopen_s(&mfp3, pathname, "w");

	fprintf_s(mfp3, "%d\n", status);
	fclose(mfp3);
}