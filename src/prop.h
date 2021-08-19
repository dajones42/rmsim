//	ship propeller model
//
//Copyright 2009 Doug Jones
#ifndef PROP_H
#define PROP_H

struct Prop {
	float massFactor;
	float speed;
	float rpmFactor;
	Prop() {
		setSize(9/3.281,12.25/3.281);
	}
	void setSize(float d, float p) {
		speed= 0;
		massFactor= 3.14159*.25*d*d*1000*64/62.5;
		rpmFactor= p/60;
	}
	float getThrust(float boatSpeed, float power, float maxRPM);
};

#endif
