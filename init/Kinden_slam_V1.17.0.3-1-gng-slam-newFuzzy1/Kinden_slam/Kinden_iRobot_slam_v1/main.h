#pragma once
#if !defined MAIN_HEADER
#define MAIN_HEADER
//
//  recvdata.h
//  sensor4
//
//  Created by Yuichiro on 10/03/31.
//  Copyright 2010 首都大学東京. All rights reserved.
//


//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include "Serial.h"
#include "Pipe_Comm.h"
#include "sensing.h"
#include <Windows.h>
#include <iostream>
#include <Shlwapi.h>


BOOL trajectory_On;
BOOL pointing_On;
bool mapping_On;
bool slam_On;
BOOL GPS_On;
BOOL remote_On;

bool onTick;
bool onTick2;
bool onCamera;

CvCapture *capture;

bool isInternetConnect;

#endif