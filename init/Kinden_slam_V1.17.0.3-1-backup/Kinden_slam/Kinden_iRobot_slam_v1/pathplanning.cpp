/*
*  pathplanning.cpp
*  path planning using built map
*
*  Created by Naoyuki Kubota on 12/10/12 and modified by Wei Hong on 12/07/17.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/

#include"pathplanning.h"

int pgen[PGAN + 1][PGAL + 1];
double pfit[PGAN + 2];
int n_best, n_worst;
double fit_best, fit_worst;

int path_planning(int x_s, int y_s, int x_g, int y_g, int path[][2], struct multi_map *multi_Map)
{
	int dir[2];
	int pote_ct = 0;
	int curr_x = x_s, curr_y = y_s;
	double dis = 0;
	double e;
	double min_E = 99999.0, pre_E;

	pote_ct = 0;
	curr_x = x_s, curr_y = y_s;
	pre_E = 9999.0;

	while (true) 
	{
		min_E = 99999.0;
		for (int i = -1; i <= 1; i++) 
		{
			for (int j = -1; j <= 1; j++) 
			{
				if (i != 0 || j != 0) 
				{
					dis = (double)((x_g - (curr_x + i)) * (x_g - (curr_x + i)) + (y_g - (curr_y + j)) * (y_g - (curr_y + j)));
					e = multi_Map->norm[curr_x + i][curr_y + j] / (dis + 0.001);
					if (min_E > e) 
					{
						min_E = e;
						dir[0] = i;
						dir[1] = j;
					}
				}
			}
		}

		if (min_E == 0) 
		{
			pote_ct = 0;
			break;
		}

		curr_x += dir[0];
		curr_y += dir[1];
		path[pote_ct][0] = curr_x;
		path[pote_ct][1] = curr_y;
		pote_ct++;
		if (curr_x == x_g && curr_y == y_g) 
		{
			break;
		}
		else if (pote_ct > 3 && path[pote_ct - 3][0] == path[pote_ct - 1][0] && path[pote_ct - 3][1] == path[pote_ct - 1][1]) 
		{
			pote_ct -= 2;
			break;
		}
		pre_E = min_E;
	}
	return pote_ct;
}

void fitcal_path(int i, int x_s, int y_s, int x_g, int y_g, struct multi_map *multi_Map)
{
	int r;
	int sub_x, sub_y, sub_x2, sub_y2;
	int path[200][2], result;
	int ct1 = 0, ct2 = 0, g_ct = 0;
	double f_dang = 0.0, f_unc = 0.0;
	r = pgen[i][0];
	sub_x = x_s, sub_y = y_s;
	sub_x2 = pgen[i][1], sub_y2 = pgen[i][2];

	g_ct = 3;
	pfit[i] = 0;
	if (r == 0) 
	{
		sub_x2 = x_g, sub_y2 = y_g;
		ct1 = path_planning(x_s, y_s, x_g, y_g, path, multi_Map);
		if (sub_x2 != path[ct1 - 1][0] || sub_y2 != path[ct1 - 1][1]) 
		{
			result = -1;
			pfit[i] = (path[ct1 - 1][0] - x_g)*(path[ct1 - 1][0] - x_g) + (path[ct1 - 1][1] - y_g)*(path[ct1 - 1][1] - y_g);

		}
		else 
		{
			ct2 += ct1;
			pfit[i] = (path[ct1 - 1][0] - x_g)*(path[ct1 - 1][0] - x_g) + (path[ct1 - 1][1] - y_g)*(path[ct1 - 1][1] - y_g);
			result = 1;
		}
	}
	else 
	{
		while (true) 
		{
			for (int j = 0; j < r; j++) 
			{
				ct1 = path_planning(sub_x, sub_y, sub_x2, sub_y2, path, multi_Map);
				for (int l = 0; l < ct1; l++) 
				{
					if (multi_Map->norm[path[l][0]][path[l][1]] < 0)
					{
						f_dang += 1 + multi_Map->norm[path[l][0]][path[l][1]];
					}
					f_unc += 1.0 - fabs(multi_Map->norm[path[l][0]][path[l][1]]) + multi_Map->anorm[path[l][0]][path[l][1]];
				}

				if (sub_x2 != path[ct1 - 1][0] || sub_y2 != path[ct1 - 1][1]) 
				{
					ct2 += ct1;
					result = -1;
					pfit[i] = sqrt((path[ct1 - 1][0] - x_g)*(path[ct1 - 1][0] - x_g) + (path[ct1 - 1][1] - y_g)*(path[ct1 - 1][1] - y_g));
					break;
				}
				else 
				{
					ct2 += ct1;
					sub_x = sub_x2, sub_y = sub_y2;
					sub_x2 = pgen[i][g_ct];
					g_ct++;
					sub_y2 = pgen[i][g_ct];
					g_ct++;
					result = 0;
				}
			}

			sub_x2 = x_g, sub_y2 = y_g;
			ct1 = path_planning(sub_x, sub_y, sub_x2, sub_y2, path, multi_Map);
			for (int l = 0; l < ct1; l++) 
			{
				if (multi_Map->norm[path[l][0]][path[l][1]] < 0)
				{
					f_dang += 1 + multi_Map->norm[path[l][0]][path[l][1]];
				}
				f_unc += 1.0 - fabs(multi_Map->norm[path[l][0]][path[l][1]]) + multi_Map->anorm[path[l][0]][path[l][1]];
			}

			if (x_g != path[ct1 - 1][0] || y_g != path[ct1 - 1][1]) 
			{
				ct2 += ct1;
				result = -1;
				if (result != -1)
				{
					pfit[i] += ALPHA_DIS * sqrt((path[ct1 - 1][0] - x_g) * (path[ct1 - 1][0] - x_g) + (path[ct1 - 1][1] - y_g) * (path[ct1 - 1][1] - y_g)) + ct2 + ALPHA_DANGER * f_dang + ALPHA_UNC * f_unc;
				}
				else
				{
					pfit[i] += ALPHA_DIS * sqrt((path[ct1 - 1][0] - x_g) * (path[ct1 - 1][0] - x_g) + (path[ct1 - 1][1] - y_g) * (path[ct1 - 1][1] - y_g)) + ct2 + ALPHA_DANGER * f_dang + ALPHA_UNC * f_unc;
				}
				break;
			}
			else 
			{
				ct2 += ct1;
				sub_x = sub_x2, sub_y = sub_y2;
				pfit[i] += ALPHA_DIS * sqrt((path[ct1 - 1][0] - x_g) * (path[ct1 - 1][0] - x_g) + (path[ct1 - 1][1] - y_g) * (path[ct1 - 1][1] - y_g)) + ct2 + ALPHA_DANGER * f_dang + ALPHA_UNC * f_unc;
				result = 1;
				break;
			}
		}
	}
}


void cal_best()
{
	fit_best = 100000.0;
	fit_worst = -100.0;
	for (int i = 0; i < PGAN; i++) 
	{
		if (fit_best > pfit[i]) 
		{
			fit_best = pfit[i];
			n_best = i;
		}

		if (fit_worst < pfit[i]) 
		{
			n_worst = i;
			fit_worst = pfit[i];
		}
	}
}

void path_init(int x_s, int y_s, int x_g, int y_g, struct multi_map *multi_Map)
{
	int sub_x, sub_y, sub_x2, sub_y2;
	int r;


	if (x_s > x_g) 
	{
		sub_x = x_g;
		sub_x2 = x_s;
	}
	else 
	{
		sub_x2 = x_g;
		sub_x = x_s;
	}

	if (y_s > y_g) 
	{
		sub_y = y_g;
		sub_y2 = y_s;
	}
	else 
	{
		sub_y2 = y_g;
		sub_y = y_s;
	}
	
	int gal = 0;
	for (int i = 0; i < PGAN; i++) 
	{
		gal = 0;
		r = (int)round(9.0*rnd()) + 1;
		pgen[i][0] = r;
		gal++;
		for (int j = 0; j <= r; j++) 
		{
			if (rnd() < 0.5)
			{
				pgen[i][gal] = (int)((sub_x2 - sub_x) * rnd() + sub_x + (int)((20.0 - 2 * j) * rndn()));
			}
			else
			{
				pgen[i][gal] = (int)((sub_x2 - sub_x) * rnd() + sub_x - (int)((20.0 - 2 * j) * rndn()));
			}
			
			gal++;
			if (rnd() < 0.5)
			{
				pgen[i][gal] = (int)((sub_y2 - sub_y) * rnd() + sub_y + (int)((20.0 - 2 * j) * rndn()));
			}
			else
			{
				pgen[i][gal] = (int)((sub_y2 - sub_y) * rnd() + sub_y - (int)((20.0 - 2 * j) * rndn()));
			}
			gal++;
			
			while (multi_Map->norm[pgen[i][gal - 2]][pgen[i][gal - 1]] != -1.0) 
			{
				if (rnd() < 0.5)
				{
					pgen[i][gal - 2] = (int)((sub_x2 - sub_x) * rnd() + sub_x + (int)((20.0 - j) * rndn()));
				}
				else
				{
					pgen[i][gal - 2] = (int)((sub_x2 - sub_x) * rnd() + sub_x - (int)((20.0 - j) * rndn()));
				}
				
				if (rnd() < 0.5)
				{
					pgen[i][gal - 1] = (int)((sub_y2 - sub_y) * rnd() + sub_y + (int)((20.0 - j) * rndn()));
				}
				else
				{
					pgen[i][gal - 1] = (int)((sub_y2 - sub_y) * rnd() + sub_y - (int)((20.0 - j) * rndn()));
				}
			}
			if (pgen[i][gal - 2] > x_g) 
			{
				sub_x = x_g;
				sub_x2 = pgen[i][gal - 2];
			}
			else 
			{
				sub_x2 = x_g;
				sub_x = pgen[i][gal - 2];
			}

			if (pgen[i][gal - 1] > y_g) 
			{
				sub_y = y_g;
				sub_y2 = pgen[i][gal - 1];
			}
			else 
			{
				sub_y2 = y_g;
				sub_y = pgen[i][gal - 1];
			}
		}
		fitcal_path(i, x_s, y_s, x_g, y_g, multi_Map);

	}
	cal_best();
}

int path_main(int x_s, int y_s, int x_g, int y_g, struct multi_map *multi_Map, int fpath[][2])
{

	int st_x = (int)(x_s / multi_Map->rate);
	int st_y = (int)(y_s / multi_Map->rate);
	int go_x = (int)(x_g / multi_Map->rate);
	int go_y = (int)(y_g / multi_Map->rate);
	int t = 0, g;
	int pote_num;
	int gal = 0;
	int sub_x, sub_y, sub_x2, sub_y2;
	int ct1 = 0, ct2 = 0;
	int g_ct;
	int path[200][2];
	int f_ct;

	if (multi_Map->anorm[go_x][go_y] == 0)
	{
		return 0;
	}

	if (st_x > go_x)
	{
		sub_x = go_x;
		sub_x2 = st_x;
	}
	else 
	{
		sub_x2 = go_x;
		sub_x = st_x;
	}

	if (st_y > go_y) 
	{
		sub_y = go_y;
		sub_y2 = st_y;
	}
	else 
	{
		sub_y2 = go_x;
		sub_y = st_y;
	}
	path_init(st_x, st_y, go_x, go_y, multi_Map);
	while (t < T3)
	{
		double r;
		if (n_worst != n_best) 
		{
			g = n_worst;
			r = rnd();
			gal = 0;

			gal++;
			pote_num = pgen[n_best][0];
			int b_x, b_y;
			pgen[g][0] = (int)round(9.0 * rnd()) + 1;

			for (int j = 0; j <= pgen[g][0]; j++) 
			{
				r = rnd();

				if (pote_num != 0) 
				{

					b_x = (int)((2 * pote_num + 1) * (rnd())) + 1;
					while (b_x % 2 != 1) 
					{
						b_x = (int)((2 * pote_num + 1) * (rnd())) + 1;
					}

					b_y = b_x + 1;
					if (r < 0.2) 
					{
						pgen[g][gal] = pgen[n_best][b_x];
						gal++;
						pgen[g][gal] = pgen[n_best][b_y];
						gal++;
					}
					else if (r < 0.8) 
					{
						pgen[g][gal] = pgen[n_best][b_x] + (int)((20.0) * rndn());
						gal++;
						pgen[g][gal] = pgen[n_best][b_y] + (int)((20.0) * rndn());
						gal++;
						while (multi_Map->norm[pgen[g][gal - 2]][pgen[g][gal - 1]] != -1.0) 
						{
							pgen[g][gal - 2] = pgen[n_best][b_x] + (int)((20.0)*rndn());
							pgen[g][gal - 1] = pgen[n_best][b_y] + (int)((20.0)*rndn());
						}
					}
					else 
					{
						if (rnd() < 0.5)
						{
							pgen[g][gal] = (int)((sub_x2 - sub_x) * rnd() + sub_x + (int)((20.0) * rndn()));
						}
						else
						{
							pgen[g][gal] = (int)((sub_x2 - sub_x) * rnd() + sub_x - (int)((20.0) * rndn()));
						}

						gal++;
						if (rnd() < 0.5)
						{
							pgen[g][gal] = (int)((sub_y2 - sub_y) * rnd() + sub_y + (int)((20.0) * rndn()));
						}
						else
						{
							pgen[g][gal] = (int)((sub_y2 - sub_y) * rnd() + sub_y - (int)((20.0) * rndn()));
						}
						
						gal++;
						while (multi_Map->norm[pgen[g][gal - 2]][pgen[g][gal - 1]] != -1.0) 
						{
							if (rnd() < 0.5)
							{
								pgen[g][gal - 2] = (int)((sub_x2 - sub_x) * rnd() + sub_x + (int)((20.0) * rndn()));
							}
							else
							{
								pgen[g][gal - 2] = (int)((sub_x2 - sub_x) * rnd() + sub_x - (int)((20.0) * rndn()));
							}
							
							if (rnd() < 0.5)
							{
								pgen[g][gal - 1] = (int)((sub_y2 - sub_y) * rnd() + sub_y + (int)((20.0) * rndn()));
							}
							else
							{
								pgen[g][gal - 1] = (int)((sub_y2 - sub_y) * rnd() + sub_y - (int)((20.0) * rndn()));
							}
						}
					}
				}
				else 
				{
					if (rnd() < 0.5)
					{
						pgen[g][gal] = (int)((sub_x2 - sub_x) * rnd() + sub_x + (int)((20.0) * rndn()));
					}
					else
					{
						pgen[g][gal] = (int)((sub_x2 - sub_x) * rnd() + sub_x - (int)((20.0) * rndn()));
					}
					
					gal++;
					if (rnd() < 0.5)
					{
						pgen[g][gal] = (int)((sub_y2 - sub_y) * rnd() + sub_y + (int)((20.0) * rndn()));
					}
					else
					{
						pgen[g][gal] = (int)((sub_y2 - sub_y) * rnd() + sub_y - (int)((20.0) * rndn()));
					}

					gal++;
					while (multi_Map->norm[pgen[g][gal - 2]][pgen[g][gal - 1]] != -1.0) 
					{
						if (rnd() < 0.5)
						{
							pgen[g][gal - 2] = (int)((sub_x2 - sub_x) * rnd() + sub_x + (int)((12.0) * rndn()));
						}
						else
						{
							pgen[g][gal - 2] = (int)((sub_x2 - sub_x) * rnd() + sub_x - (int)((12.0) * rndn()));
						}
						
						if (rnd() < 0.5)
						{
							pgen[g][gal - 1] = (int)((sub_y2 - sub_y) * rnd() + sub_y + (int)((12.0) * rndn()));
						}
						else
						{
							pgen[g][gal - 1] = (int)((sub_y2 - sub_y) * rnd() + sub_y - (int)((12.0) * rndn()));
						}
					}
				}

				if (pgen[g][gal - 2] > go_x) 
				{
					sub_x = go_x;
					sub_x2 = pgen[g][gal - 2];
				}
				else 
				{
					sub_x2 = go_x;
					sub_x = pgen[g][gal - 2];
				}

				if (pgen[g][gal - 1] > go_y) 
				{
					sub_y = go_y;
					sub_y2 = pgen[g][gal - 1];
				}
				else 
				{
					sub_y2 = go_y;
					sub_y = pgen[g][gal - 1];
				}
			}

			fitcal_path(g, st_x, st_y, go_x, go_y, multi_Map);
			cal_best();
		}
		t++;
	}

	sub_x = st_x, sub_y = st_y;
	sub_x2 = pgen[n_best][1], sub_y2 = pgen[n_best][2];
	g_ct = 3;

	for (int j = 0; j < pgen[n_best][0]; j++) 
	{
		ct1 = path_planning(sub_x, sub_y, sub_x2, sub_y2, path, multi_Map);
		for (int i = 0; i < ct1; i++) 
		{
			fpath[ct2 + i][0] = (int)(multi_Map->rate*path[i][0] + multi_Map->rate / 2);
			fpath[ct2 + i][1] = (int)(multi_Map->rate*path[i][1] + multi_Map->rate / 2);
		}
		ct2 += ct1;

		if (sub_x2 != path[ct1 - 1][0] || sub_y2 != path[ct1 - 1][1]) 
		{
			break;
		}
		else 
		{
			sub_x = sub_x2, sub_y = sub_y2;
			sub_x2 = pgen[n_best][g_ct];
			g_ct++;
			sub_y2 = pgen[n_best][g_ct];
			g_ct++;
		}
	}
	sub_x2 = go_x, sub_y2 = go_y;
	ct1 = path_planning(sub_x, sub_y, sub_x2, sub_y2, path, multi_Map);

	for (int i = 0; i < ct1; i++) 
	{
		fpath[ct2 + i][0] = (int)(multi_Map->rate*path[i][0] + multi_Map->rate / 2);
		fpath[ct2 + i][1] = (int)(multi_Map->rate*path[i][1] + multi_Map->rate / 2);
	}
	ct2 += ct1;
	f_ct = ct2;

	return f_ct;
}