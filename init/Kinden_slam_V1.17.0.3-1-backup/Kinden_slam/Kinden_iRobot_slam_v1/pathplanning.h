#pragma once
/*
*  pathplanning.h
*  header file for pathplanning.cpp
*
*  Created by Naoyuki Kubota on 12/10/12 and modified by Wei Hong on 12/07/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#if !defined(PATH_PLANNING_HEADER)
#define PATH_PLANNING_HEADER

#include "random.h"
#include "multi_resolution.h"
#include "slam.h"

int path_main(int x_s, int y_s, int x_g, int y_g, struct multi_map *multi_Map, int fpath[][2]);

#endif