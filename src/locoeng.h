//	interface and classes for locomotive engine models
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
#ifndef LOCOENG_H
#define LOCOENG_H

#include <stdio.h>
#include <list>
#include <vector>

#include "commandreader.h"
#include "spline.h"

class RailCarInst;

class LocoEngine {
 public:
	virtual float getThrottleInc() = 0;
	virtual float getReverserInc() = 0;
	virtual float getForce(float tControl, float dControl,
	  RailCarInst* car, float dt) = 0;
	virtual LocoEngine* copy() = 0;
	virtual float getMaxForce() = 0;
	virtual float getSmokeRate() = 0;
	virtual float getSmokeSpeed() = 0;
};

class DieselEngine : public LocoEngine, public CommandBlockHandler {
	float maxForce;
	float maxCForce;
	float maxPower;
	float forceTimeConstant;
	double averageForce;
	float smoke;
	int nNotches;
 public:
	DieselEngine() {
		maxForce= 0;
		maxCForce= 0;
		maxPower= 0;
		smoke= 0;
		nNotches= 8;
		averageForce= 0;
		forceTimeConstant= 600;
	};
	void setMaxPower(float p) { maxPower= p; }
	void setMaxForce(float p) { maxForce= p; }
	void setMaxCForce(float p) { maxCForce= p; }
	virtual float getThrottleInc();
	virtual float getReverserInc();
	virtual float getForce(float tControl, float dControl,
	  RailCarInst* car, float dt);
	virtual LocoEngine* copy();
	virtual float getMaxForce() { return maxForce; };
	virtual float getSmokeRate() { return smoke; };
	virtual float getSmokeSpeed() { return smoke; };
	virtual bool handleCommand(CommandReader& reader);
	virtual void handleBeginBlock(CommandReader& reader);
	virtual void handleEndBlock(CommandReader& reader);
};

class ElectricEngine : public LocoEngine, public CommandBlockHandler {
	struct Step {
		int notch;
		float voltageRatio;
		float resistanceMult;
		Step(int n, float v, float r) {
			notch= n;
			voltageRatio= v;
			resistanceMult= r;
		};
	};
	float maxForce;
	float maxCForce;
	float maxPower;
	float topSpeed;
	float efficiency;
	float minAccelForce;
	float force;
	float powerFactor;
	int fwdPantograph;
	int revPantograph;
	int nNotches;
	int state;
	vector<Step> steps;
 public:
	ElectricEngine() {
		maxForce= 0;
		maxCForce= 0;
		minAccelForce= 0;
		force= 0;
		maxPower= 0;
		topSpeed= 0;
		efficiency= .85;
		powerFactor= 1;
		fwdPantograph= 1;
		revPantograph= 1;
		nNotches= 8;
		state= 0;
	};
	void setMaxPower(float p) { maxPower= p; }
	void setMaxForce(float p) { maxForce= p; }
	void setMaxCForce(float p) { maxCForce= p; }
	virtual float getThrottleInc();
	virtual float getReverserInc();
	virtual float getForce(float tControl, float dControl,
	  RailCarInst* car, float dt);
	virtual LocoEngine* copy();
	virtual float getMaxForce() { return maxForce; };
	virtual float getSmokeRate() { return 0; };
	virtual float getSmokeSpeed() { return 0; };
	virtual bool handleCommand(CommandReader& reader);
	virtual void handleBeginBlock(CommandReader& reader);
	virtual void handleEndBlock(CommandReader& reader);
};

class SteamEngine : public LocoEngine, public CommandBlockHandler {
	float prevDControl;
	enum { LOWSPEED, HISPEED} prevState;
	float cylDiameter;
	float cylStroke;
	float clearanceVolume;
	float mainRodLength;
	struct CylState {
		float pressure;
		float volume;
		CylState() { pressure= volume= 0; };
	};
	std::vector<CylState> cylState;
	int numCylinders;
	float wheelDiameter;
	float boilerPressure;
	float maxBoilerPressure;
	float boilerVolume;
	float heat;
	float maxHeat;
	float satSteamHeatMaxBP;
	float usageMult;
	float usage;
	float evap;
	float firingRate;
	float firingRateInc;
	float fireMass;
	float shovelMass;
	float mass;
	float waterFraction;
	float blowerFraction;
	float damperFraction;
	float injectorRate;
	float injectorFraction[2];
	bool injectorOn[2];
	float grateArea;
	float blowerRate;
	float blowerUsage;
	float auxSteamUsage;
	bool autoFire;
	bool safetyOn;
	float safetyUsage;
	float safetyDrop;
	float exhaustTime;
	Spline<float> heat2psig;
	Spline<float> cylSteamDensity;
	Spline<float> satSteamHeat;
	Spline<float> cylPressureDrop;
	Spline<float> backPressure;
	Spline<float> release;
	Spline<float> forceFactor1;
	Spline<float> forceFactor2;
	Spline<float> burnRate;
	Spline<float> evapRate;
	Spline<float> burnFactor;
	Spline<float> evapFactor;
	void init();
 public:
	SteamEngine();
	virtual float getThrottleInc();
	virtual float getReverserInc();
	float getBoilerPressure() { return boilerPressure; };
	float getWaterFraction() { return waterFraction; };
	virtual float getForce(float tControl, float dControl,
	  RailCarInst* car, float dt);
	virtual LocoEngine* copy();
	virtual float getMaxForce();
	virtual float getSmokeRate();
	virtual float getSmokeSpeed();
	virtual bool handleCommand(CommandReader& reader);
	virtual void handleBeginBlock(CommandReader& reader);
	virtual void handleEndBlock(CommandReader& reader);
	bool getAutoFire() { return autoFire; };
	void setAutoFire(bool on) { autoFire= on; };
	float getInjectorFraction(int i) { return injectorFraction[i]; };
	bool getInjectorOn(int i) { return injectorOn[i]; };
	void setInjectorOn(int i, bool on) { injectorOn[i]= on; };
	void toggleInjector(int i) { injectorOn[i]= !injectorOn[i]; };
	void setInjectorFraction(int i, float f) {
		if (f < 0)
			injectorFraction[i]= 0;
		else if (f > 1)
			injectorFraction[i]= 1;
		else
			injectorFraction[i]= f;
	};
	void incInjector(int i) {
		setInjectorFraction(i,getInjectorFraction(i)+.1);
	}
	void decInjector(int i) {
		setInjectorFraction(i,getInjectorFraction(i)-.1);
	}
	void incFiringRate() {
		if (shovelMass > 0)
			fireMass+= shovelMass;
		else
			firingRate+= firingRateInc;
	}
	void decFiringRate() {
		firingRate-= firingRateInc;
		if (firingRate < 0)
			firingRate= 0;
	}
	void incBlowerFraction() {
		blowerFraction+= .1;
		if (blowerFraction > 1)
			blowerFraction= 1;
	}
	void decBlowerFraction() {
		blowerFraction-= .1;
		if (blowerFraction < 0)
			blowerFraction= 0;
	}
	void incDamperFraction() {
		damperFraction+= .1;
		if (damperFraction > 1)
			damperFraction= 1;
	}
	void decDamperFraction() {
		damperFraction-= .1;
		if (damperFraction < 0)
			damperFraction= 0;
	}
	float getUsage() {
		return usage+blowerUsage+auxSteamUsage+(safetyOn?safetyUsage:0);
	};
	float getUsagePercent() {
		return grateArea>0 ? 100.*getUsage()/grateArea*3600./775. : 0;
	};
	float getEvap() { return evap; };
	float getFiringRate() { return firingRate; };
	float getBlowerFraction() { return blowerFraction; };
	float getDamperFraction() { return damperFraction; };
	float getFireMass() { return fireMass; };
	float crankAngle(float x);
	float pistonPosition(float angle);
	void setCylDiameter(float x) {
		cylDiameter= x;
	}
	void setCylStroke(float x) {
		cylStroke= x;
	}
	void setClearanceVolume(float x) {
		clearanceVolume= x;
	}
	void setMainRodLength(float x) {
		mainRodLength= x;
	}
	void setNumCylinders(int n) {
		numCylinders= n;
	}
	void setWheelDiameter(float x) {
		wheelDiameter= x;
	}
	void setMaxBoilerPressure(float x) {
		maxBoilerPressure= x;
	}
	void setBoilerVolume(float x) {
		boilerVolume= x;
	}
	void setInjectorRate(float x) {
		injectorRate= x/3600;
	}
	void setGrateArea(float x) {
		grateArea= x;
	}
	void setBlowerRate(float x) {
		blowerRate= x/3600;
	}
	void setAuxSteamUsage(float x) {
		auxSteamUsage= x/3600;
	}
	void setSafetyUsage(float x) {
		safetyUsage= x/3600;
	}
	void setSafetyDrop(float x) {
		safetyDrop= x;
	};
	void setMaxBoilerOutput(float x);
	void setExhaustLimit(float x);
	void setIdealFireMass(float x);
	void printForceVsSpeed();
};

extern SteamEngine* mySteamEngine;

#endif
