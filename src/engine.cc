//	ship engine functions
//
//Copyright 2009 Doug Jones

#include "rmsim.h"

float Engine::getThrust(float speed)
{
	return prop.getThrust(speed,throttle*maxPower,throttle*maxRPM);
}

void Engine::setThrottle(float p) {
	throttle= p;
	if (throttle > 1)
		throttle= 1;
	if (throttle < -1)
		throttle= -1;
}
