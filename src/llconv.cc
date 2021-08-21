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
#include <stdio.h>
#include <math.h>

const double TILEDEG= 54.2945;
const double TILEX0= (-16384+180*TILEDEG);
const double TILEZ0= 12149;

double centerLongitude(double lat, double lng)
{
	if (lat>=0 && lng<-40)
		return -100;
	if (lat>=0)
		return 30;
	if (lng<-100)
		return -150;
	if (lng<-20)
		return -60;
	if (lng<80)
		return 20;
	return 140;
}

void lltot(double lat, double lng, int* tx, int* tz, float *x, float *z)
{
	double clng= centerLongitude(lat,lng);
	double tileX= (clng + (lng-clng)*cos(lat*M_PI/180))*TILEDEG + TILEX0;
	double tileZ= lat*TILEDEG + TILEZ0;
	*tx= (int)floor(tileX);
	*tz= (int)floor(tileZ);
	*x= 2048*(tileX-*tx) - 1024;
	*z= 2048*(tileZ-*tz) - 1024;
}

void ttoll(int tx, int tz, double x, double z, double* plat, double* plng)
{
	double lat= (tz+z/2048+.5-TILEZ0)/TILEDEG;
	double lng0= (tx+x/2048+.5-TILEX0)/TILEDEG;
	double clng= centerLongitude(lat,lng0);
	double c= cos(lat*M_PI/180);
	*plat= lat;
	*plng= clng + (c>0 ? (lng0-clng)/c : 0);
}
