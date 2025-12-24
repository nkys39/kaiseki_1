
/*
*  multi_resolution.h
*  slam
*
*  Created by Naoyuki Kubota on 11/04/12 and modified by Wei Hong on 12/7/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/
#pragma once
#if !defined(MULI_RESOLUTION_HEADER)
#define MULI_RESOLUTION_HEADER

#include "slam.h"

struct multi_map *reduce_resolution(struct multi_map *root_map, int **omap);

#endif
