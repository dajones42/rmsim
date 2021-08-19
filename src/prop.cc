//	ship propeller force function
//
//Copyright 2009 Doug Jones
#include "rmsim.h"

float Prop::getThrust(float boatSpeed, float power, float maxRPM)
{
	if (power == 0)
		return 0;
	if (power < 0)
		return -getThrust(-boatSpeed,-power,-maxRPM);
	if (boatSpeed < 0)
		boatSpeed= 0;
	if (rpmFactor>0 && boatSpeed>rpmFactor*maxRPM)
		return 0;
	if (speed < boatSpeed)
		speed= boatSpeed;
	int i= 0;
	for (i=0; i<20; i++) {
		float df= speed*(3*speed-2*boatSpeed);
		if (df < .1)
			df= .1;
		float f= speed*speed*(speed-boatSpeed) - .5*power/massFactor;
		float ds= f<0 ? .5*f/df : f/df;
		speed-= ds;
		if (speed < boatSpeed)
			speed= boatSpeed;
		if (rpmFactor>0 && speed > rpmFactor*maxRPM) {
			speed= rpmFactor*maxRPM;
			break;
		}
//		fprintf(stderr,"%d %f %f %f %f\n",i,speed,ds,f,df);
		if (-.0001<ds && ds<.0001)
			break;
	}
	float t= 2*massFactor*speed*(speed-boatSpeed);
//	fprintf(stderr,"t=%f %f %f %f %f %d %f\n",
//	  t,speed,boatSpeed,power,rpmFactor*maxRPM,i,massFactor);
	return t;
}
