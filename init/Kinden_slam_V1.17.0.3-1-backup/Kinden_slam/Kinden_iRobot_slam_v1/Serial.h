#if !defined(SERIAL_HEADER)
#define SERIAL_HEADER

#include <winsock2.h>
#include <ws2tcpip.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>

#include "Def.h"

void ServerInitialise(void);
void iRobotInitialise(void);
void roombaStop(void);
void roombaStart(void);
void roombaSafe(void);
void controlMotor(short rightspeed, short leftspeed);
//int *communication_data(char *buf, int omap[MAPSIZE][MAPSIZE], int amap[MAPSIZE][MAPSIZE], char *nodebuf, int tpath[][2], cv::Mat cpImage_mat);
int getBumpData();

void socketMain();
void sendImage(IplImage* image);
#endif