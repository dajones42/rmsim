//	ship propeller force function
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
