//	test program for air brake
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
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "brakevalve.h"

using namespace std;

struct TestData {
	BrakeValve* valve;
	vector<AirTank*> tanks;
	int valveState;
	void addTank(string name, float volume, float psig) {
		int i= valve->getTankIndex(name);
		if (i >= 0) {
			AirTank* t= new AirTank(volume*pow(.0254,3));
			t->setPsig(psig);
			tanks.push_back(t);
		}
	}
	TestData(string valveName, float bpPsig, float bcPsig) {
		valve= BrakeValve::get(valveName);
		addTank("BP",435,bpPsig);
		addTank("AR",2500,bpPsig);
		addTank("BC",800,bcPsig);
		addTank("ER",3500,bpPsig);
		addTank("QSV",.0006/pow(.0254,3),0);
//		addTank("QAC",145,bpPsig);
		addTank("QAC",14,bpPsig);//disable qac for service test
		valveState= 0;
	}
	void update(float timeStep) {
		valveState= valve->updateState(valveState,tanks);
		valve->updatePressures(valveState,timeStep,tanks,0);
	}
	float getPsig(string name) {
		int i= valve->getTankIndex(name);
		return i>=0 ? tanks[i]->getPsig() : 0;
	}
};

//	Tests the brake cylinder fill time for an applcation.
//	From Wabtec Instruction Leaflet Standard S-479-94, 5039.19 Sup. 1,
//	Sep 1994, Code_of_Test_for_ABD.pdf, section 7.7.2.1.
int testApplicationTime(string valveName)
{
	TestData data(valveName,100,0);
	float area= .1405*pow(.0254,2); // valve A position 7
	float dt= .001;
	float t= 0;
	for (; t<=6 && data.getPsig("BC")<60; t+=dt) {
		if (data.getPsig("BP") > 70)
			data.tanks[0]->vent(area,dt);
		data.update(dt);
//		printf("%f %.1f %.1f\n",
//		  t,data.getPsig("BP"),data.getPsig("BC"));
	}
	if (data.getPsig("BC")>=60 && t>=4 && t<=6) {
		printf("%s application time passed, time=%.1f\n",
		  valveName.c_str(),t);
		return 0;
	} else {
		printf("%s application time failed, time=%.1f, pressure=%.1f\n",
		  valveName.c_str(),t,data.getPsig("BC"));
		return 1;
	}
}

//	Tests the brake cylinder empty time for a release.
//	From Wabtec Instruction Leaflet Standard S-479-94, 5039.19 Sup. 1,
//	Sep 1994, Code_of_Test_for_ABD.pdf, section 7.7.2.2.
int testReleaseTime(string valveName)
{
	TestData data(valveName,60,60);
	float t40= 0;
	float area= .03125*pow(.0254,2); // valve A position 2
	float dt= .001;
	float t= 0;
	for (; t<=60 && data.getPsig("BC")>20; t+=dt) {
		if (data.getPsig("BP") < 100) {
			float m= dt*AirTank::massFlowRate(
			  120*AirTank::PSI2PA+AirTank::STDATM,
			  data.tanks[0]->pressure,area);
			data.tanks[0]->addAir(m);
		}
		data.update(dt);
		if (t40==0 && data.getPsig("BC")<40)
			t40= t;
	}
	if (data.getPsig("BC")<=20 && t40>0 && t-t40>=8 && t-t40<=12) {
		printf("%s release time passed, time=%.1f\n",
		  valveName.c_str(),t-t40);
		return 0;
	} else {
		printf("%s release time failed, time=%.1f, pressure=%.1f\n",
		  valveName.c_str(),t-t40,data.getPsig("BC"));
		return 1;
	}
}

main(int argc, char** argv)
{
	int nFailures= 0;
	nFailures+= testApplicationTime("K");
	nFailures+= testReleaseTime("K");
	nFailures+= testApplicationTime("AB");
	nFailures+= testReleaseTime("AB");
	exit(nFailures>0?1:0);
}
