/*
*  malloc.h
*  Cluster
*
*  Created by Naoyuki Kubota on 03/07/14 and modified by Wei Hong on 12/07/17.
*  Copyright 2014 首都大学東京. All rights reserved.
*
*/
#pragma once
#if !defined(MALLOC_HEADER)
#define MALLOC_HEADER

void free2d_double(double ** a);
double **malloc2d_double(int x, int y);

void free2d_int(int ** a);
int **malloc2d_int(int x, int y);

void free3d_double(double *** a);
double ***malloc3d_double(int x, int y, int z);

#endif