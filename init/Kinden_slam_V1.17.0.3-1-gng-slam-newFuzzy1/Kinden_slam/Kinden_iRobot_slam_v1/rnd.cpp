/*
*  rnd.c
*  Claster
*
*  Created by Naoyuki Kubota on 12/05/18.
*  Copyright 2012 首都大学東京. All rights reserved.
*
*/

#include "rnd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

double rnd2() /* uniform random number  */
{
	return((double)(rand() % 30001) / 30000.0);
}


double rndn2()            /*   normal random number */
{
	return (rnd2() + rnd2() + rnd2() + rnd2() + rnd2() + rnd2() +
		    rnd2() + rnd2() + rnd2() + rnd2() + rnd2() + rnd2() - 6.0);
}