//	Gode homolosine map projection functions
//
/*
Copyright © 2021 Doug Jones

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include "ghproj.h"
#include <math.h>
#include <stdio.h>

#define PI 	3.141592653589793238
#define HALF_PI PI*0.5
#define TWO_PI 	PI*2.0
#define EPSLN	1.0e-10
#define R2D     57.2957795131
#define D2R     0.0174532925199

inline double sign(double x)
{
	return x<0. ? -1. : 1.;
}

inline double adjust_lon(double x)
{
	return fabs(x)<PI ? x : x-sign(x)*TWO_PI;
}

GHProjection::GHProjection(double r)
{
	R= r;
	lon_center[0] = -1.74532925199;		/* -100.0 degrees */
	lon_center[1] = -1.74532925199;		/* -100.0 degrees */
	lon_center[2] =  0.523598775598;	/*   30.0 degrees */
	lon_center[3] =  0.523598775598;	/*   30.0 degrees */
	lon_center[4] = -2.79252680319;		/* -160.0 degrees */
	lon_center[5] = -1.0471975512;		/*  -60.0 degrees */
	lon_center[6] = -2.79252680319;		/* -160.0 degrees */
	lon_center[7] = -1.0471975512;		/*  -60.0 degrees */
	lon_center[8] =  0.349065850399;	/*   20.0 degrees */
	lon_center[9] =  2.44346095279;		/*  140.0 degrees */
	lon_center[10] = 0.349065850399;	/*   20.0 degrees */
	lon_center[11] = 2.44346095279;		/*  140.0 degrees */
	feast[0] = R * -1.74532925199;
	feast[1] = R * -1.74532925199;
	feast[2] = R * 0.523598775598;
	feast[3] = R * 0.523598775598;
	feast[4] = R * -2.79252680319;
	feast[5] = R * -1.0471975512;
	feast[6] = R * -2.79252680319;
	feast[7] = R * -1.0471975512;
	feast[8] = R * 0.349065850399;
	feast[9] = R * 2.44346095279;
	feast[10] = R * 0.349065850399;
	feast[11] = R * 2.44346095279;
}

//	converts lat,long to x,y
void GHProjection::ll2xy(double lat, double lon, double* x, double *y)
{
	lat*= D2R;
	lon*= D2R;
	int region;
	if (lat >= 0.710987989993) { /* if on or above 40 44' 11.8" */
		if (lon <= -0.698131700798)
			region = 0;   /* If to the left of -40 */
		else
			region = 2;
	} else if (lat >= 0.0) { /* Between 0.0 and 40 44' 11.8" */
		if (lon <= -0.698131700798)
			region = 1;   /* If to the left of -40 */
		else
			region = 3;
	} else if (lat >= -0.710987989993) { /* Between 0.0 & -40 44' 11.8" */
		if (lon <= -1.74532925199)
			region = 4;  	/* If between -180 and -100 */
		else if (lon <= -0.349065850399)
			region = 5;	/* If between -100 and -20 */
		else if (lon <= 1.3962634016)
			region = 8;	/* If between -20 and 80 */
		else
			region = 9;	/* If between 80 and 180 */
	} else { /* Below -40 44' */
		if (lon <= -1.74532925199)
			region = 6;       /* If between -180 and -100 */
		else if (lon <= -0.349065850399)
			region = 7;     /* If between -100 and -20 */
		else if (lon <= 1.3962634016)
			region = 10;   /* If between -20 and 80 */
		else
			region = 11;	/* If between 80 and 180 */
	}
	if (region==1 || region==3 || region==4 || region==5 || region==8 ||
	  region==9) {
		double delta_lon= adjust_lon(lon - lon_center[region]);
		*x= feast[region] + R * delta_lon * cos(lat);
		*y= R * lat;
	} else {
		double delta_lon= adjust_lon(lon - lon_center[region]);
		double theta= lat;
		double constant= PI * sin(lat);
		for (int i=0; i<30; i++) {
			double delta_theta= -(theta + sin(theta) - constant) /
			  (1.0 + cos(theta));
			theta+= delta_theta;
			if (fabs(delta_theta) < EPSLN)
				break;
		}
		theta/= 2.0;
		*x= feast[region] + 0.900316316158 * R * delta_lon * cos(theta);
		*y= R*(1.4142135623731*sin(theta)-0.0528035274542*sign(lat));
	}
}
	
//	converts x,y to lat,long
int GHProjection::xy2ll(double x, double y, double* lat, double *lon)
{
	int region;
	if (y >= R*0.710987989993) { /* if on or above 40 44' 11.8" */
		if (x <= R*-0.698131700798)
			region= 0; /* If to the left of -40 */
		   else
			region= 2;
	} else if (y >= 0.0) { /* Between 0.0 and 40 44' 11.8" */
		if (x <= R*-0.698131700798)
			region = 1; /* If to the left of -40 */
		else
			region = 3;
	} else if (y >= R*-0.710987989993) { /* Between 0.0 & -40 44' 11.8" */
		if (x <= R*-1.74532925199)
			region = 4;     /* If between -180 and -100 */
		else if (x <= R*-0.349065850399)
			region = 5;	/* If between -100 and -20 */
		else if (x <= R*1.3962634016)
			region = 8;	/* If between -20 and 80 */
		else
			region = 9;	/* If between 80 and 180 */
	} else { /* Below -40 44' 11.8" */
		if (x <= R*-1.74532925199)
			region = 6;     /* If between -180 and -100 */
		else if (x <= R*-0.349065850399)
			region = 7;	/* If between -100 and -20 */
		else if (x <= R * 1.3962634016)
			region = 10;	/* If between -20 and 80 */
		else
			region = 11;	/* If between 80 and 180 */
	}
	x= x - feast[region];
	if (region==1 || region==3 || region==4 || region==5 || region==8 ||
	  region==9) {
		*lat = y / R;
		if (fabs(*lat) > HALF_PI) 
			return 0;
		double temp= fabs(*lat) - HALF_PI;
		if (fabs(temp) > EPSLN) {
			temp = lon_center[region] + x / (R * cos(*lat));
			*lon = adjust_lon(temp);
		} else {
			*lon = lon_center[region];
		}
	} else {
		double arg= (y + 0.0528035274542 * R * sign(y)) /
		  (1.4142135623731 * R);
		if (fabs(arg) > 1.0)
			return 0;
		double theta= asin(arg);
		*lon= lon_center[region]+(x/(0.900316316158 * R * cos(theta)));
		if (*lon < (-PI))
			return 0;
		arg = (2.0 * theta + sin(2.0 * theta)) / PI;
		if (fabs(arg) > 1.0)
			return 0;
		*lat = asin(arg);
	}
	if (region == 0 && (*lon < -PI || *lon > -0.698131700798))
		return 0;
	if (region == 1 && (*lon < -PI || *lon > -0.698131700798))
		return 0;
	if (region == 2 && (*lon < -0.698131700798 || *lon > PI))
		return 0;
	if (region == 3 && (*lon < -0.698131700798 || *lon > PI))
		return 0;
	if (region == 4 && (*lon < -PI || *lon > -1.74532925199))
		return 0;
	if (region == 5 && (*lon<-1.74532925199 ||*lon>-0.349065850399))
		return 0;
	if (region == 6 && (*lon < -PI || *lon > -1.74532925199))
		return 0;
	if (region == 7 && (*lon<-1.74532925199 || *lon>-0.349065850399))
		return 0;
	if (region == 8 && (*lon<-0.349065850399 || *lon>1.3962634016))
		return 0;
	if (region == 9 && (*lon < 1.3962634016 || *lon > PI))
		return 0;
	if (region ==10 && (*lon<-0.349065850399 ||*lon>1.3962634016))
		return 0;
	if (region ==11 && (*lon < 1.3962634016 || *lon > PI))
		return 0;
	*lat= R2D * *lat;
	*lon= R2D * *lon;
	return 1;
}
