//	Ship engine state variable
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
#ifndef ENGINE_H
#define ENGINE_H

#include "prop.h"

#define WDENSITY	1027

struct Engine {
	float throttle;
	float thrustFactor;
	float power;
	float maxPower;
	float maxRPM;
	float propDiameter;
	float propPitch;
	float workFactor;
	float torqueFactor;
	float cEngineWork;
	float cEngineTorque;
	float cProp;
	float maxPropSpeed;
	float propSpeed;
	float revs;
	Prop prop;
	Engine() {
		maxPower= 0;
		maxRPM= 118;
		workFactor= 1.;
		torqueFactor= 1.;
		setProp(9/3.281,12.25/3.281);
		setThrottle(0);
	}
	void setProp(float d, float p) {
		propDiameter= d;
		propPitch= p;
		propSpeed= 0;
		cProp= 2*3.14159*.25*d*d*993*64/62.5;
//		fprintf(stderr,"cProp=%f\n",cProp);
		prop.setSize(d,p);
	}
	void setThrottle(float p);
	void incThrottle() { setThrottle(throttle+.333); }
	void decThrottle() { setThrottle(throttle-.333); }
	float getThrust(float speed);
	void setPower(float kw) {
		maxPower= kw*1e3;
	};
};

float sinInt(float);

#endif
