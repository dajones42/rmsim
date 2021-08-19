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
