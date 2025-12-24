/*
*  multi_resolution.c
*  slam
*
*  Created by Naoyuki Kubota on 12/04/11.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/

#include "multi_resolution.h"
#include "random.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "malloc.h"
struct multi_map *reduce_resolution(struct multi_map *root_map, int **omap)
{
	int k = 0, l = 0;
	int i, j;
	int occ = 0, emp = 0;
	double r_max = 1.0;
	struct multi_map *r;

	r = (struct multi_map *)malloc(sizeof(struct multi_map));
	r->mmap = malloc2d_int(MAPSIZE_WIDTH, MAPSIZE_HIGHT);
	r->amap = malloc2d_int(MAPSIZE_WIDTH, MAPSIZE_HIGHT);
	r->norm = malloc2d_double(MAPSIZE_WIDTH, MAPSIZE_HIGHT);
	r->anorm = malloc2d_double(MAPSIZE_WIDTH, MAPSIZE_HIGHT);
	r->chmap = malloc2d_double(MAPSIZE_WIDTH, MAPSIZE_HIGHT);

	if (root_map == NULL) {
		r_max = 2.0;
		for (i = 0; i<MAPSIZE_WIDTH; i += 2) {
			l = 0;
			for (j = 0; j<MAPSIZE_HIGHT; j += 2) {
				occ = 0, emp = 0;
				if (cal_probability(omap[i][j]) >= 0.8)
					occ++;
				else if (cal_probability(omap[i][j]) <= -0.8)
					emp++;
				if (cal_probability(omap[i + 1][j]) >= 0.8)
					occ++;
				else if (cal_probability(omap[i + 1][j]) <= -0.8)
					emp++;
				if (cal_probability(omap[i][j + 1]) >= 0.8)
					occ++;
				else if (cal_probability(omap[i][j + 1]) <= -0.8)
					emp++;
				if (cal_probability(omap[i + 1][j + 1]) >= 0.8)
					occ++;
				else if (cal_probability(omap[i + 1][j + 1]) <= -0.8)
					emp++;
				r->mmap[k][l] = occ - emp;
				r->amap[k][l] = occ + emp;
				r->norm[k][l] = (double)r->mmap[k][l] / r_max;
				r->anorm[k][l] = (double)r->amap[k][l] / r_max;
				l++;
			}
			k++;
		}
		r->reso = 1;
		r->rate = r_max;
		r->parent = NULL;
		r->child = NULL;
		//root_map = r;
	}
	else {
		for (i = 0; i<2 * (root_map->reso + 1); i++)
			r_max *= 2.0;
		for (i = 0; i<MAPSIZE_WIDTH; i += 2) {
			l = 0;
			for (j = 0; j<MAPSIZE_HIGHT; j += 2) {
				r->mmap[k][l] = 0;
				r->amap[k][l] = 0;
				r->mmap[k][l] += root_map->mmap[i][j] + root_map->mmap[i + 1][j] + root_map->mmap[i][j + 1] + root_map->mmap[i + 1][j + 1];
				r->amap[k][l] += root_map->amap[i][j] + root_map->amap[i + 1][j] + root_map->amap[i][j + 1] + root_map->amap[i + 1][j + 1];
				r->norm[k][l] = (double)r->mmap[k][l] / r_max;
				r->anorm[k][l] = (double)r->amap[k][l] / r_max;
				l++;
			}
			k++;
		}
		r->rate = 1.0;
		r->reso = root_map->reso + 1;
		for (int i = 0; i<r->reso; i++)
			r->rate *= 2.0;
		r->parent = root_map;
		r->parent->child = r;
		r->child = NULL;
		//root_map = r;
	}

	return r;
}