//	locomotive motive force calculating functions
//	this file includes both diesel and steam, should probably be split
//
//	the steam code uses a lot of table lookups that look like function calls
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

float DieselEngine::getThrottleInc()
{
	return 1/(float)nNotches;
}

float DieselEngine::getReverserInc()
{
	return 1;
}

float DieselEngine::getForce(float tControl, float dControl,
  RailCarInst* car, float dt)
{
	if (dControl == 0)
		return 0;
	float s= car->speed;
	if (s < 0)
		s= -s;
	smoke= tControl;
	float maxF= maxForce;
	if (maxCForce > 0) {
		float x= averageForce/maxCForce;
		maxF= (1-x)*maxForce + x*maxCForce;
	}
	float force= tControl*(maxF*s>tControl*maxPower ?
	  tControl*maxPower/s : maxF);
	float w= dt/forceTimeConstant;
	averageForce= w*force + (1-w)*averageForce;
	return dControl<0 ? -force : force;
}

LocoEngine* DieselEngine::copy()
{
	DieselEngine* e= new DieselEngine();
	*e= *this;
	return e;
}

bool DieselEngine::handleCommand(CommandReader& reader)
{
	if (reader.getString(0) == "maxforce") {
		maxForce= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "maxcforce") {
		maxCForce= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "maxpower") {
		maxPower= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "notches") {
		nNotches= reader.getInt(1,1,50);
		return true;
	}
	return false;
}

void DieselEngine::handleBeginBlock(CommandReader& reader)
{
}

void DieselEngine::handleEndBlock(CommandReader& reader)
{
}

float ElectricEngine::getThrottleInc()
{
	return 1/(float)nNotches;
}

float ElectricEngine::getReverserInc()
{
	return 1;
}

float ElectricEngine::getForce(float tControl, float dControl,
  RailCarInst* car, float dt)
{
	if (dControl == 0) {
		car->animState= 0;
		return 0;
	}
	if (dControl > 0)
		car->animState= 1<<fwdPantograph;
	else if (dControl < 0)
		car->animState= 1<<revPantograph;
	float s= car->speed;
	if (s < 0)
		s= -s;
	if (topSpeed>0 && maxCForce==0) {
		maxCForce= maxPower/(.6*topSpeed);
		fprintf(stderr,"maxcforce %f\n",maxCForce);
	}
	float resistance= 1;
	if (steps.size() > 0) {
		int prev= state;
		int i= (int)(tControl*nNotches+.5);
		if (i<steps[state].notch && state>0)
			state--;
		if (minAccelForce==0 && i>steps[state].notch &&
		  state<steps.size()-1)
			state++;
		if (minAccelForce>0 && force<minAccelForce &&
		  state<steps.size()-1 && (i>steps[state].notch ||
		  (i==steps[state].notch && i==steps[state+1].notch)))
			state++;
		tControl= steps[state].voltageRatio;
		resistance= steps[state].resistanceMult;
//		if (state != prev)
//			fprintf(stderr,"state %d %d %f %f %f %f\n",
//			  i,state,tControl,resistance,force,car->distance);
	}
	if (maxCForce > 0) {
		float sm= maxPower/maxCForce;
		float x= resistance*(1-efficiency) + efficiency*s/sm;
		float pfsq= powerFactor*powerFactor;
		float z= (1-pfsq)/pfsq;
		force= tControl*tControl*maxCForce*(1+z)/(x*x+z);
		if (force > maxForce)
			force= maxForce;
	} else {
		force= tControl*(maxForce*s>tControl*maxPower ?
		  tControl*maxPower/s : maxForce);
	}
	return dControl<0 ? -force : force;
}

LocoEngine* ElectricEngine::copy()
{
	ElectricEngine* e= new ElectricEngine();
	*e= *this;
	return e;
}

bool ElectricEngine::handleCommand(CommandReader& reader)
{
	if (reader.getString(0) == "maxforce") {
		maxForce= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "maxcforce") {
		maxCForce= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "minaccelforce") {
		minAccelForce= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "maxpower") {
		maxPower= 1e3*reader.getDouble(1,0,1e10);
		return true;
	}
	if (reader.getString(0) == "topspeed") {
		topSpeed= reader.getDouble(1,0,100);
		return true;
	}
	if (reader.getString(0) == "efficiency") {
		efficiency= reader.getDouble(1,.5,.999);
		return true;
	}
	if (reader.getString(0) == "powerfactor") {
		powerFactor= reader.getDouble(1,.5,1.);
		return true;
	}
	if (reader.getString(0) == "pantographs") {
		fwdPantograph= reader.getInt(1,0,10);
		revPantograph= reader.getInt(2,0,10);
		return true;
	}
	if (reader.getString(0) == "notches") {
		nNotches= reader.getInt(1,1,50);
		return true;
	}
	if (reader.getString(0) == "step") {
		if (steps.size() == 0)
			steps.push_back(Step(0,0,1));
		steps.push_back(Step(reader.getInt(1,1,nNotches),
		  reader.getDouble(2,0,1),reader.getDouble(3,1,10)));
		return true;
	}
	return false;
}

void ElectricEngine::handleBeginBlock(CommandReader& reader)
{
}

void ElectricEngine::handleEndBlock(CommandReader& reader)
{
}

SteamEngine* mySteamEngine= NULL;

struct SteamTable {
	float psig;
	float waterHeat;
	float waterDensity;
	float steamHeat;	// btu from water at 32F
	float steamDensity;	// pounds per cubic foot
};
static SteamTable steamTable[] = {
	{   0, 180.15, 59.83, 1150.28, 0.0373 },
	{  10, 207.83, 59.11, 1160.31, 0.0606 },
	{  20, 227.51, 58.57, 1167.01, 0.0834 },
	{  30, 243.08, 58.12, 1172.03, 0.1057 },
	{  40, 256.09, 57.73, 1176.00, 0.1277 },
	{  50, 267.34, 57.39, 1179.27, 0.1496 },
	{  60, 277.31, 57.07, 1182.04, 0.1713 },
	{  70, 286.30, 56.79, 1184.42, 0.1928 },
	{  80, 294.49, 56.52, 1186.50, 0.2143 },
	{  90, 302.05, 56.27, 1188.33, 0.2356 },
	{ 100, 309.08, 56.03, 1189.95, 0.2569 },
	{ 110, 315.66, 55.81, 1191.41, 0.2782 },
	{ 120, 321.85, 55.59, 1192.72, 0.2994 },
	{ 130, 327.70, 55.39, 1193.91, 0.3205 },
	{ 140, 333.26, 55.19, 1194.99, 0.3416 },
	{ 150, 338.56, 55.00, 1195.97, 0.3627 },
	{ 160, 343.62, 54.82, 1196.86, 0.3838 },
	{ 170, 348.47, 54.65, 1197.68, 0.4048 },
	{ 180, 353.13, 54.48, 1198.43, 0.4259 },
	{ 190, 357.62, 54.31, 1199.12, 0.4469 },
	{ 200, 361.95, 54.15, 1199.75, 0.4680 },
	{ 210, 366.14, 53.99, 1200.33, 0.4890 },
	{ 220, 370.19, 53.84, 1200.86, 0.5101 },
	{ 230, 374.12, 53.69, 1201.35, 0.5312 },
	{ 240, 377.94, 53.54, 1201.79, 0.5522 },
	{ 250, 381.65, 53.40, 1202.20, 0.5733 },
	{ 260, 385.26, 53.26, 1202.58, 0.5944 },
	{ 270, 388.78, 53.12, 1202.92, 0.6155 },
	{ 280, 392.21, 52.99, 1203.23, 0.6367 },
	{ 290, 395.56, 52.86, 1203.51, 0.6578 },
	{ 300, 398.84, 52.73, 1203.77, 0.6790 },
	{ -1, 0, 0, 0, 0 }
};

//	interpolates steam and water properties at pressure p (psig)
void getSteamTable(SteamTable& out, float p)
{
	out.psig= p;
	p*= .1;
	int i= (int)p;
	float b= p-i;
	float a= 1-b;
	SteamTable* t= &steamTable[i];
	SteamTable* t1= &steamTable[i+1];
	out.waterHeat= a*t->waterHeat + b*t1->waterHeat;
	out.waterDensity= a*t->waterDensity + b*t1->waterDensity;
	out.steamHeat= a*t->steamHeat + b*t1->steamHeat;
	out.steamDensity= a*t->steamDensity + b*t1->steamDensity;
//	fprintf(stderr,"st %f %f %f %f %f\n",out.psig,
//	  out.waterHeat,out.waterDensity,out.steamHeat,out.steamDensity);
}

SteamEngine::SteamEngine()
{
	prevDControl= 0;
	prevState= LOWSPEED;
	boilerPressure= 180;
	numCylinders= 2;
	cylState.resize(2);
	clearanceVolume= .07;
	usage= 0;
	waterFraction= .9;
	injectorFraction[0]= 0;
	injectorFraction[1]= .5;
	injectorOn[0]= false;
	injectorOn[1]= false;
	mainRodLength= 0;
	autoFire= true;
	firingRate= 0;
	firingRateInc= 0;
	fireMass= 0;
	shovelMass= 0;
	blowerFraction= 0;
	blowerRate= 0;
	damperFraction= 0;
	auxSteamUsage= 0;
	safetyUsage= 0;
	safetyDrop= 5;
	safetyOn= false;
	exhaustTime= 0;
}

float SteamEngine::getThrottleInc()
{
	return .1;
}

float SteamEngine::getReverserInc()
{
	return .1;
}

//	returns the tractive force for a steam engine
//	also adjusts the state of the boiler which includes:
//		total heat in boiler
//		mass of water and steam in boiler
//		mass of coal in fire if coal fired
//	for high speeds uses average force given cutoff and pressure
//	for low speed simulates pressure in piston given position
//	Its not clear that the low speed code is worth the trouble,
//	 it mostly prevents starting when values are not open.
//	The state change code should probably be moved to a separate method.
float SteamEngine::getForce(float throttle, float reverser,
  RailCarInst* car, float dt)
{
	if (heat2psig.size() == 0)
		init();
	float cutoff= reverser<0 ? -reverser : reverser;
	if (cutoff > release.getMaxX())
		cutoff= release.getMaxX();
	else if (cutoff < release.getMinX())
		cutoff= 0;
	float s= car->speed;
	if (s < 0)
		s= -s;
	if (s>(.5/.25) && (reverser==1. || reverser==-1.)) {
		// AI cutoff adjustment logic
		// TODO: replace .5/.25 with something else
		float max= (.5/.25)/s;
		cutoff= throttle*max;
		if (cutoff < release.getMinX()) {
			throttle= cutoff/release.getMinX();
			cutoff= release.getMinX();
		} else {
			throttle= 1;
		}
//		fprintf(stderr,"ai %f %f %f %f\n",
//		  cutoff,throttle,max,boilerPressure);
	}
	SteamTable st;
	getSteamTable(st,boilerPressure);
	float cylPressure= throttle*boilerPressure - cylPressureDrop(usage);
	float backPres= backPressure(usage);
	usage*= .6; // usage is moving average
	if (cylPressure < backPres)
		cylPressure= backPres;
	else
		usage+= .4*usageMult*s*(cutoff+clearanceVolume)*
		  (cylSteamDensity(cylPressure)-cylSteamDensity(backPres));
	float blowerUsage= 0;
	if (!autoFire) {
		blowerUsage= .4*blowerFraction*boilerPressure*blowerRate;
		mass-= dt*(usage+blowerUsage+auxSteamUsage)*st.steamDensity;
		for (int i=0; i<2; i++)
			if (injectorOn[i])
				mass+= dt*injectorFraction[i]*injectorRate;
		if (boilerPressure > maxBoilerPressure)
			safetyOn= true;
		else if (boilerPressure < maxBoilerPressure-safetyDrop)
			safetyOn= false;
		if (safetyOn) {
			mass-= dt*safetyUsage*st.steamDensity;
			heat-= dt*safetyUsage*st.steamHeat;
		}
	} else {
		if (boilerPressure < maxBoilerPressure-2)
			blowerUsage= .4*boilerPressure*blowerRate;
		else if (usage < autoBlowerUsage)
			blowerUsage= autoBlowerUsage-usage;
	}
	waterFraction= (mass/boilerVolume-st.steamDensity)/
	  (st.waterDensity-st.steamDensity);
	float br= burnRate(usage+blowerUsage);
	if (autoFire) {
		evap= evapRate(br);
	} else if (burnFactor.size() > 0) { //coal burner
		br*= burnFactor(fireMass);
		br*= 1-.5*damperFraction;
		evap= evapRate(br)*evapFactor(fireMass);
		fireMass+= dt*(firingRate-br);
	} else { // oil burner
		evap= evapRate(firingRate<br?firingRate:br)*
		  evapFactor(firingRate/br);
		fireMass= firingRate/br; // to show burn ratio in hud
//		fprintf(stderr,"%f %f %f %f %f\n",evap,br,firingRate,
//		  evapRate(firingRate),burnFactor(firingRate/br));
	}
	heat+= dt*(evap*satSteamHeatMaxBP-
	  (usage+blowerUsage+auxSteamUsage)*st.steamHeat);
	float hs= boilerVolume*(1-waterFraction)*st.steamDensity*st.steamHeat;
	if (autoFire && heat-hs>waterFraction*maxHeat)
		heat= waterFraction*maxHeat+hs;
	boilerPressure= heat2psig((heat-hs)/waterFraction);
//	fprintf(stderr,"%f %f %f\n",dt,cylPressure,backPres);
//	fprintf(stderr,"%f %f %f %f %f %f %f\n",
//	  usage,br,evap,heat,mass,waterFraction,
//	  heat/boilerVolume-waterFraction*
//	  (st.waterDensity*st.waterHeat-st.steamDensity*st.steamHeat));
	if (reverser == 0) {
		for (int i=0; i<numCylinders; i++)
			cylState[i].pressure= 0;
		return 0;
	}
	exhaustTime-= dt;
	float dist= 1000*s*dt;
	if (dist > wheelDiameter) {
		prevState= HISPEED;
		float force= backPres*forceFactor1(cutoff) +
		  cylPressure*forceFactor2(cutoff);
		return reverser<0 ? -force: force;
	}
	if (prevState == HISPEED) {
		for (int i=0; i<numCylinders; i++) {
			cylState[i].pressure= cylPressure;
			cylState[i].volume= cutoff+clearanceVolume;
		}
	}
	prevState= LOWSPEED;
	float force= 0;
	float state= 2*car->getMainWheelState();
	for (int i=0; i<numCylinders; i++) {
		float p= 0;
		float angle= state<1 ? state : state-1;
		float h= .5*cylStroke*sin(3.14159*angle);
		float c= sqrt(mainRodLength*mainRodLength-h*h);
		float c1= sqrt(.25*cylStroke*cylStroke-h*h);
		if (angle < .5)
			c+= c1;
		else
			c-= c1;
		c+= .5*cylStroke-mainRodLength;
		c/= cylStroke;
		if (car->speed < 0)
			c= 1-c;
		float v= c;
//		fprintf(stderr,"%f %f\n",v,c);
		float v1= v+clearanceVolume;
		if (v < cutoff) {
			p= cylPressure;
			if (cylState[i].pressure>p &&
			  cylState[i].volume<=v1)
				p= cylState[i].pressure*cylState[i].volume/v1;
			cylState[i].pressure= p;
		} else  if (v < release(cutoff)) {
			if (cylState[i].volume > v1)
				p= cylPressure*
				  (cutoff+clearanceVolume)/v1;
			else if (cylState[i].volume > cutoff+clearanceVolume)
				p= cylState[i].pressure*cylState[i].volume/v1;
			else
				p= cylState[i].pressure*cutoff/v1;
			cylState[i].pressure= p;
		} else {
			if (cylState[i].pressure > 2*backPres)
				exhaustTime= .5;
			cylState[i].pressure= p= backPres;
		}
//		fprintf(stderr,"%f %f\n",p,backPres);
		p-= backPres;
		if (p < 0)
			p= 0;
		cylState[i].volume= v1;
		float s=sin(state*3.14159);
		if (s < 0)
			s= -s;
		force+= p*s*4.4482*
		  .25*3.14159*cylDiameter*cylDiameter*cylStroke/wheelDiameter;
//		fprintf(stderr," %.1f %d %.1f %.3f %.1f %.2f %f %.1f\n",
//		  throttle,i,cutoff,state,p,v,s,force);
		state+= 1/(float)numCylinders;
		if (state >= 2)
			state-= 2;
	}
	return reverser<0 ? -force : force;
}

float SteamEngine::getMaxForce()
{
	return numCylinders*4.4482*maxBoilerPressure*
	  .25*3.14159*cylDiameter*cylDiameter*cylStroke/wheelDiameter;
}

float SteamEngine::getSmokeRate()
{
	return usage/burnRate.getMaxX();
}

float SteamEngine::getSmokeSpeed()
{
	if (prevState == HISPEED)
		return .5 + .5*usage/burnRate.getMaxX();
	else
		return exhaustTime>0 ? .5 : 0;
}

LocoEngine* SteamEngine::copy()
{
	SteamEngine* e= new SteamEngine();
	*e= *this;
	return e;
}

bool SteamEngine::handleCommand(CommandReader& reader)
{
	if (reader.getString(0) == "cyldiameter") {
		cylDiameter= reader.getDouble(1,1,100);
		return true;
	}
	if (reader.getString(0) == "cylstroke") {
		cylStroke= reader.getDouble(1,1,100);
		return true;
	}
	if (reader.getString(0) == "mainrodlength") {
		mainRodLength= reader.getDouble(1,1,1000);
		return true;
	}
	if (reader.getString(0) == "numcylinders") {
		numCylinders= reader.getInt(1,1,4);
		cylState.resize(numCylinders);
		return true;
	}
	if (reader.getString(0) == "maxpressure") {
		boilerPressure= maxBoilerPressure= reader.getDouble(1,100,300);
		return true;
	}
	if (reader.getString(0) == "wheeldiameter") {
		wheelDiameter= reader.getDouble(1,1,100);
		return true;
	}
	if (reader.getString(0) == "clearancevolume") {
		clearanceVolume= reader.getDouble(1,0,.2);
		return true;
	}
	if (reader.getString(0) == "boilervolume") {
		boilerVolume= reader.getDouble(1,1,1e10);
		return true;
	}
	if (reader.getString(0) == "waterfraction") {
		waterFraction= reader.getDouble(1,0,1);
		return true;
	}
	if (reader.getString(0) == "cylpressuredrop") {
		SplineParser<float> parser(&cylPressureDrop);
		reader.parseBlock((CommandBlockHandler*)&parser);
		cylPressureDrop.scaleX(1/3600.);
		return true;
	}
	if (reader.getString(0) == "backpressure") {
		SplineParser<float> parser(&backPressure);
		reader.parseBlock((CommandBlockHandler*)&parser);
		backPressure.scaleX(1/3600.);
		return true;
	}
	if (reader.getString(0) == "release") {
		SplineParser<float> parser(&release);
		reader.parseBlock((CommandBlockHandler*)&parser);
		return true;
	}
	if (reader.getString(0) == "gratearea") {
		grateArea= reader.getDouble(1,1,1000);
		return true;
	}
	if (reader.getString(0) == "shovelmass") {
		shovelMass= reader.getDouble(1,0,1000);
		return true;
	}
	if (reader.getString(0) == "firingrateinc") {
		firingRateInc= reader.getDouble(1,0,10000)/3600;
		return true;
	}
	if (reader.getString(0) == "blowerrate") {
		blowerRate= reader.getDouble(1,0,10000)/3600;
		return true;
	}
	if (reader.getString(0) == "auxsteamusage") {
		auxSteamUsage= reader.getDouble(1,0,10000)/3600;
		return true;
	}
	if (reader.getString(0) == "safetyusage") {
		safetyUsage= reader.getDouble(1,0,100000)/3600;
		return true;
	}
	if (reader.getString(0) == "safetydrop") {
		safetyDrop= reader.getDouble(1,0,100);
		return true;
	}
	if (reader.getString(0) == "evaprate") {
		SplineParser<float> parser(&evapRate);
		reader.parseBlock((CommandBlockHandler*)&parser);
		evapRate.scaleX(1/3600.);
		evapRate.scaleY(1/3600.);
		return true;
	}
	if (reader.getString(0) == "burnrate") {
		SplineParser<float> parser(&burnRate);
		reader.parseBlock((CommandBlockHandler*)&parser);
		burnRate.scaleX(1/3600.);
		burnRate.scaleY(1/3600.);
		return true;
	}
	if (reader.getString(0) == "burnfactor") {
		SplineParser<float> parser(&burnFactor);
		reader.parseBlock((CommandBlockHandler*)&parser);
		return true;
	}
	if (reader.getString(0) == "evapfactor") {
		SplineParser<float> parser(&evapFactor);
		reader.parseBlock((CommandBlockHandler*)&parser);
		return true;
	}
	if (reader.getString(0) == "print") {
		printForceVsSpeed();
		return true;
	}
	return false;
}

void SteamEngine::handleBeginBlock(CommandReader& reader)
{
}

void SteamEngine::handleEndBlock(CommandReader& reader)
{
}

//	set up internal values that need configuation data
void SteamEngine::init()
{
//	fprintf(stderr,"steam init\n");
	SteamTable t;
	getSteamTable(t,maxBoilerPressure);
	mass= boilerVolume*(waterFraction*t.waterDensity+
	  (1-waterFraction)*t.steamDensity);
	maxHeat= boilerVolume*t.waterDensity*t.waterHeat;
	satSteamHeatMaxBP= t.steamHeat;
	heat= maxHeat;
	usage= 0;
	// compute table used to convert heat into pressure
	for (int i=0; steamTable[i].psig>=0; i++) {
		SteamTable& t= steamTable[i];
		satSteamHeat.add(t.psig,t.steamHeat);
		float h= boilerVolume*
		  (waterFraction*t.waterDensity*t.waterHeat+
		  (1-waterFraction)*t.steamDensity*t.steamHeat);
		heat2psig.add(boilerVolume*t.waterDensity*t.waterHeat,t.psig);
//		fprintf(stderr,"%.0f %.1f %f\n",
//		  t.psig,boilerVolume*t.steamDensity*t.steamHeat,
//		  boilerVolume*t.waterDensity*t.waterHeat);
	}
	heat2psig.compute();
	//boilerPressure= heat2psig.getY(heat);
//	fprintf(stderr,"%f %f %f\n",
//	  maxHeat,heat,boilerPressure);
	if (cylSteamDensity.size() == 0) {
		for (int i=0; steamTable[i].psig>=0; i++) {
			SteamTable& t= steamTable[i];
			cylSteamDensity.add(t.psig,t.steamDensity);
		}
		cylSteamDensity.compute();
	}
	if (mainRodLength == 0)
		mainRodLength= 4*cylStroke;
	if (wheelDiameter == 0)
		wheelDiameter= 2*cylStroke;
	usageMult= 2*numCylinders*
	  .25*3.14159*cylDiameter*cylDiameter*cylStroke/1728 /
	  (3.14159*wheelDiameter*.0254);
//	fprintf(stderr,"usageMult %f %d %f %f %f\n",usageMult,2*numCylinders,
//	  .25*3.14159*cylDiameter*cylDiameter*cylStroke/1728,
//	  1./(3.14159*wheelDiameter*.0254),
//	  (3.14159*wheelDiameter*.0254));
	if (release.size() == 0) {
		release.add(.2,.7);
		release.add(.37,.8);
		release.add(.5,.85);
		release.add(.75,.93);
		release.add(.85,.96);
		release.compute();
	}
	if (backPressure.size() == 0)
		backPressure.add(0,0);
	for (int i=0; i<=10; i++) {
		float cutoff= release.getMinX() +
		  .1*i*(release.getMaxX()-release.getMinX());
		float rel= release(cutoff);
		float cutoffAngle= crankAngle(cutoff);
		float releaseAngle= crankAngle(rel);
		float s0= (cos(releaseAngle)-1)/3.14159;
		float s1= (1-cos(cutoffAngle))/3.14159;
		int n= 30;
		float dx= (rel-cutoff)/n;
		for (int j=0; j<=n; j++) {
			float x= cutoff + j*dx;
			float y= sin(crankAngle(x))*
			   (cutoff+clearanceVolume)/(x+clearanceVolume);
			if (j==0 || j==n)
				s1+= dx*y/3;
			else if (j%2==0)
				s1+= dx*4*y/3;
			else
				s1+= dx*2*y/3;
		}
		forceFactor1.add(cutoff,s0);
		forceFactor2.add(cutoff,s1);
//		fprintf(stderr,"%.3f %.3f %3.0f %3.0f",
//		  cutoff,rel,180*cutoffAngle/3.14159,180*releaseAngle/3.14159);
//		fprintf(stderr," %f %f %f\n",s0,s1,
//		  .85*cutoff*(1-(1-cutoff)*log(cutoff)));
//		fprintf(stderr,"%.3f %.3f %3.0f %3.0f %.3f %.3f\n",
//		  cutoff,rel,180*cutoffAngle/3.14159,180*releaseAngle/3.14159,
//		  pistonPosition(cutoffAngle),pistonPosition(releaseAngle));
	}
	forceFactor1.scaleY(numCylinders*4.4482*
	  .25*3.14159*cylDiameter*cylDiameter*cylStroke/wheelDiameter);
	forceFactor2.scaleY(numCylinders*4.4482*
	  .25*3.14159*cylDiameter*cylDiameter*cylStroke/wheelDiameter);
	if (grateArea > 0) {
		for (int i=0; i<=10; i++) {
			float u= burnRate.getMinX() + .1*i*burnRate.getMaxX();
			float br= burnRate(u);
			float er= evapRate(br);
			printf("%d %.0f %.0f %.0f\n",i,u*3600,br*3600,er*3600);
		}
		evapRate.scaleX(grateArea);
		evapRate.scaleY(grateArea);
		burnRate.scaleX(grateArea);
		burnRate.scaleY(grateArea);
		printf("gratearea %f %f %f\n",
		  grateArea,3600*burnRate.getMinX(),3600*burnRate.getMaxX());
		for (int i=0; i<=10; i++) {
			float u= burnRate.getMinX() + .1*i*burnRate.getMaxX();
			float br= burnRate(u);
			float er= evapRate(br);
			printf("%d %.0f %.0f %.0f\n",i,u*3600,br*3600,er*3600);
		}
	}
	injectorRate= 0;
	for (int i=0; i<=10; i++) {
		float x= evapRate.getMinX() + .1*i*evapRate.getMaxX();
		float y= evapRate(x);
		if (injectorRate < y)
			injectorRate= y;
	}
	if (safetyUsage == 0)
		safetyUsage= injectorRate;
//	fprintf(stderr,"injectorRate %f\n",injectorRate);
	if (firingRateInc == 0)
		firingRateInc= (evapRate.getMaxX()-evapRate.getMinX())/40;
	if (blowerRate == 0)
//		blowerRate= .5*evapRate.getMaxX()/maxBoilerPressure;
		blowerRate= .5*burnRate.getMaxX()/maxBoilerPressure;
//	fprintf(stderr,"blowerRate %f\n",blowerRate);
	if (fireMass==0 && burnFactor.size()>0)
		fireMass= (burnFactor.getMaxX()-burnFactor.getMinX())/2;
	float hi= burnRate.getMaxX();
	float lo= 0;
	for (int i=0; i<10; i++) {
		float x= .5*(hi+lo);
		if (evapRate(burnRate(x)) > auxSteamUsage+x)
			hi= x;
		else
			lo= x;
	}
	autoBlowerUsage= hi;
	fprintf(stderr,"auto blower usage %f %f %f %f\n",autoBlowerUsage,
	  auxSteamUsage,evapRate(burnRate(autoBlowerUsage)),
	  evapRate(burnRate(autoBlowerUsage))-auxSteamUsage-autoBlowerUsage);
}

//	returns the crank angle given piston position
float SteamEngine::crankAngle(float x)
{
	float a= mainRodLength;
	float b= .5*cylStroke;
	float c= b+a-x*cylStroke;
	return acos(.5*(b*b+c*c-a*a)/(b*c));
}

float SteamEngine::pistonPosition(float angle)
{
	float h= .5*cylStroke*sin(angle);
	float c= sqrt(mainRodLength*mainRodLength-h*h)-mainRodLength+
	 .5*cylStroke*(1-cos(angle));
	return c/cylStroke;
}

//	sets various curves based on MSTS wag exhaust limit
void SteamEngine::setExhaustLimit(float x)
{
	cylPressureDrop.add(0,0);
	cylPressureDrop.add(.2*x,0);
	cylPressureDrop.add(.5*x,0);//2);
	cylPressureDrop.add(x,0);//10);
	cylPressureDrop.add(2*x,0);//20);
	cylPressureDrop.compute();
	cylPressureDrop.scaleX(1/3600.);
	backPressure.add(0,0);
	backPressure.add(x,0);//6);
	backPressure.add(1.2*x,0);//30);
	backPressure.scaleX(1/3600.);
}

//	sets various curves based on MSTS wag max boiler output
void SteamEngine::setMaxBoilerOutput(float x)
{
	burnRate.add(0,2);
	burnRate.add(775,180);
	burnRate.scaleX(1/3600.);
	burnRate.scaleY(1/3600.);
	evapRate.add(0,0);
	evapRate.add(20,170);
	evapRate.add(40,315);
	evapRate.add(60,440);
	evapRate.add(80,550);
	evapRate.add(100,630);
	evapRate.add(120,700);
	evapRate.add(140,740);
	evapRate.add(160,770);
	evapRate.add(180,775);
	evapRate.add(200,760);
	evapRate.compute();
	evapRate.scaleX(1/3600.);
	evapRate.scaleY(1/3600.);
	grateArea= x/775;
	burnRate.clear();
	for (int i=0; i<evapRate.size()-1; i++)
		burnRate.add(evapRate.getY(i),
		  evapRate.getX(i)+2./3600.);
//	burnRate.compute();
}

//	sets various curves based on MSTS wag ideal fire mass
void SteamEngine::setIdealFireMass(float x)
{
#if 0
	burnFactor.add(0,1);//0);
	burnFactor.add(x,1);
	burnFactor.add(2*x,1);
	evapFactor.add(0,1);//.9);
	evapFactor.add(x,1);
	evapFactor.add(2*x,1);//.6);
	evapFactor.add(3*x,1);//.6);
#else
	burnFactor.add(0,0);
	burnFactor.add(x,1.01);
	burnFactor.add(2*x,1);
	evapFactor.add(0,.9);
	evapFactor.add(x,1);
	evapFactor.add(2*x,.6);
	evapFactor.add(3*x,.6);
#endif
}

//	prints a table of max force vs speed using the Kiesel force method.
void SteamEngine::printForceVsSpeed()
{
	SteamTable st;
	getSteamTable(st,maxBoilerPressure);
	float maxEvapRate= 775*grateArea;
	float cylVolume= M_PI*cylDiameter*cylDiameter*cylStroke/(12*12*12);
//	printf("%f %f %f\n",maxEvapRate,cylVolume,maxBoilerPressure);
	printf("mph cutoff mep      te    hp\n");
	for (float mph=0; mph<100; mph+=5) {
		float strokesPerHour= mph*5280*12/(M_PI*wheelDiameter);
		float steamPerStroke= maxEvapRate/strokesPerHour;
		float cutoff= steamPerStroke/(cylVolume*st.steamDensity);
		if (cutoff > .9)
			cutoff= .9;
		float mep= 1.95*maxBoilerPressure/(1+1/cutoff);
		float force=
		  mep*cylDiameter*cylDiameter*cylStroke/wheelDiameter;
		float power= force*mph/375;
		printf("%3.0f %6.3f %3.0f %7.0f %5.0f\n",
		  mph,cutoff,mep,force,power);
	}
}
