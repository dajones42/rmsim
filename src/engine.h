//	Ship engine state variable
//
//Copyright 2009 Doug Jones
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
