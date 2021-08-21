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
#ifndef AIRTANK_H
#define AIRTANK_H

struct AirTank {
	float volume;	// in cubic meters
	float pressure;	// in Pascals
	AirTank(float v) { volume= v; pressure= STDATM; };
	float getVolume() { return volume; };
	float getPa() { return pressure; };
	void setPa(float pa) { pressure= pa; };
	float getPsig();
	float getDensity();
	void setPsig(float psig);
	virtual void addAir(float kg);
	float vent(float area, float timeStep);
	void moveAir(AirTank* other, float kg);
	float moveAir(AirTank* other, float area, float timeStep);
	static const float STDATM;
	static const float PSI2PA;
	static const float DENSITY;
	static const float VISCOSITY;
	static const float HEATRATIO;
	static float massFlowRate(float p1, float p2, float area);
};

struct AirPipe : public AirTank {
	float length;		// m
	float diameter;		// m
	float airSpeed;		// m/s
	float friction;
	float airFlow;		// kg
	AirPipe* next;
	AirPipe* prev;
	bool nextOpen;
	bool prevOpen;
	AirPipe(float l, float d) : AirTank(l*d*d/4*M_PI) {
		length= l;
		diameter= d;
		airSpeed= 0;
		airFlow= 0;
		friction= .01235;
		next= 0;
		prev= 0;
		nextOpen= false;
		prevOpen= false;
	};
	virtual void addAir(float kg);
	void updateAirSpeed(float timeStep);
	void setNext(AirPipe* n) { next= n; };
	void setPrev(AirPipe* p) { prev= p; };
	void setNextOpen(bool open) { nextOpen= open; };
	void setPrevOpen(bool open) { prevOpen= open; };
};

#endif
