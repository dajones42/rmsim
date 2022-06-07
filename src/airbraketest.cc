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

#include "airbrake.h"

const float arTestVolume= 1620;//1800
const float bcTestVolume= 603;//800

extern float chokeAreaI(float diam, float cContract);

//	3-t test rack auxiliary reservior related leakage tests
//	used to reverse engineer AR volume
void test3tARVol()
{
	AirTank ar(arTestVolume*pow(.0254,3));
	AirTank mainr(80000*pow(.0254,3));
	float area4= .25*pow(.0254*.031,2)*M_PI;// valve R position 4
	float area5= .25*pow(.0254*.113,2)*M_PI;// valve R position 5
	float area6= .25*pow(.0254*.1495,2)*M_PI;// valve R position 6
	ar.setPsig(60);
	float dt= .001;
	for (float t=0; t<62; t+=dt) {
		ar.vent(area4,dt);
		if (ar.getPsig()<=47) {
			if (t<55)
				fprintf(stderr,
				  "test ar volume too small %f\n",t);
			fprintf(stderr,"test ar time %f\n",t);
			break;
		}
	}
	if (ar.getPsig()>47)
		fprintf(stderr,"test ar volume too big %f\n",ar.getPsig());
	for (int i=0; i<2; i++) {
		float area= i==0 ? area5 : area6;
		float limit= i==0 ? 6 : 4;
		ar.setPsig(50);
		mainr.setPsig(80);
		for (float t=0; t<limit; t+=dt) {
			mainr.moveAir(&ar,area,dt);
			mainr.setPsig(80);
			if (ar.getPsig()>=70) {
				if (i==0 ? t<5 : t<2)
					fprintf(stderr,
					  "test ar charge too fast %f\n",t);
				fprintf(stderr,"test ar charge time %f\n",t);
				break;
			}
		}
		if (ar.getPsig()<70)
			fprintf(stderr,"test ar not fully charged %f\n",
			  ar.getPsig());
	}
}

void test3tARCharge(std::string name, float target, float minT, float maxT)
{
	AirBrake* brake= AirBrake::create(false,0,name);
	brake->setVolume("AR",arTestVolume*pow(.0254,3));
	brake->setPressure("BP",80);
	brake->setPressure("ER",target);
	brake->setPressure("AR",0);
	brake->setPressure("BC",0);
	float dt= .001;
	for (float t=0; t<maxT; t+=dt) {
		brake->updatePressures(dt);
		brake->setPressure("BP",80);
		if (brake->getAuxResPressure() >= target) {
			if (t < minT)
				fprintf(stderr,"%s AR charge too fast %f\n",
				  name.c_str(),t);
			fprintf(stderr,"%s AR charge time %f\n",
			  name.c_str(),t);
			break;
		}
	}
	if (brake->getAuxResPressure() < target)
		fprintf(stderr,"%s AR not charged %f\n",
		  name.c_str(),brake->getAuxResPressure());
}

void test3tBCRelease(std::string name, float minT, float maxT)
{
	AirBrake* brake= AirBrake::create(false,0,name);
	brake->setVolume("BC",bcTestVolume*pow(.0254,3));
	brake->setPressure("BP",80);
	brake->setPressure("AR",65);
	brake->setPressure("BC",40);
	float dt= .001;
	for (float t=0; t<maxT; t+=dt) {
		brake->updatePressures(dt);
		brake->setPressure("BP",80);
		if (brake->getCylPressure() < 10) {
			if (t < minT)
				fprintf(stderr,"%s BC relrease too fast %f\n",
				  name.c_str(),t);
			fprintf(stderr,"%s BC release time %f\n",
			  name.c_str(),t);
			break;
		}
	}
	if (brake->getCylPressure() > 10)
		fprintf(stderr,"%s BC not released %f\n",
		  name.c_str(),brake->getCylPressure());
}

void testVent(float volume, float p0, float p1, float area, float maxT)
{
	AirTank tank(volume*pow(.0254,3));
	tank.setPsig(p0);
	float dt= .001;
	float t= 0;
	for (; t<maxT; t+=dt) {
		tank.vent(area,dt);
		if (tank.getPsig()<=p1)
			break;
	}
	fprintf(stderr,"testvent %.0f %.0f %.0f %.6e %.2f %.1f\n",
	  volume,p0,p1,area,t,tank.getPsig());
}

void testXfer(float v1, float v2, float p1, float p2, float p2a,
  float area, float maxT)
{
	AirTank tank1(v1*pow(.0254,3));
	AirTank tank2(v2*pow(.0254,3));
	tank1.setPsig(p1);
	tank2.setPsig(p2);
	float dt= .001;
	float t= 0;
	for (; t<maxT; t+=dt) {
		tank1.moveAir(&tank2,area,dt);
		if (tank2.getPsig()>p2a ||
		  tank1.getPsig()-tank2.getPsig() < .01)
			break;
	}
	fprintf(stderr,"testxfer %.0f %.0f %.0f %.0f %.6e %.2f %.2f %.2f\n",
	  v1,v2,p1,p2,area,t,tank1.getPsig(),tank2.getPsig());
}

void testUCChokes()
{
	testVent(1847,40,10,chokeAreaI(.1875,1),10);
	testVent(2413,40,10,chokeAreaI(.2031,1),10);
	testVent(3054,40,10,chokeAreaI(.2188,1),10);
	testVent(3695,40,10,chokeAreaI(.2656,1),10);
	testVent(4825,40,10,chokeAreaI(.3125,1),10);
	testVent(603,40,10,chokeAreaI(.1875,1),10);
	testVent(603,40,10,chokeAreaI(.2031,1),10);
	testVent(603,40,10,chokeAreaI(.2188,1),10);
	testVent(603,40,10,chokeAreaI(.2656,1),10);
	testVent(603,40,10,chokeAreaI(.3125,1),10);
	testXfer(5213,1847,100,0,50,chokeAreaI(.12,1),20);
	testXfer(6601,2413,100,0,50,chokeAreaI(.136,1),20);
	testXfer(7849,3054,100,0,50,chokeAreaI(.156,1),20);
	testXfer(9886,3695,100,0,50,chokeAreaI(.18,1),20);
	testXfer(11665,4825,100,0,50,chokeAreaI(.22,1),20);
	testXfer(2500,800,100,0,50,chokeAreaI(.12,1),20);
	testXfer(2500,1000,100,0,50,chokeAreaI(.12,1),20);
}

void testD22Chokes()
{
	testVent(664,40,10,chokeAreaI(.1094,1),10);
	testXfer(1659,664,100,0,50,chokeAreaI(.086,1),20);
	testXfer(3619,1659,100,80,82,chokeAreaI(.073,1),20);
	testVent(664,50,40,chokeAreaI(.1094,1),10);
	testXfer(2500,800,100,0,50,chokeAreaI(.086,1),20);
}

main(int argc, char** argv)
{
	int nCars= 10;
	float start= 80;
	float target= 70;
	if (argc > 1)
		nCars= atoi(argv[1]);
	if (argc > 2)
		start= atof(argv[2]);
	if (argc > 3)
		target= atof(argv[3]);
	test3tARVol();
	test3tARCharge("K",30,20,30);
	test3tARCharge("K",70,55,85);
	test3tBCRelease("K",2,4);
	test3tARCharge("L",70,11,40);
	test3tBCRelease("L",2,3);
	testUCChokes();
	testD22Chokes();
	AirBrake** airBrake= (AirBrake**) malloc((nCars+1)*sizeof(AirBrake*));
	airBrake[0]= AirBrake::create(true,start<target?target:start,"H6");
	for (int i=1; i<=nCars; i++)
		airBrake[i]= AirBrake::create(false,0,"K");
	for (int i=0; i<=nCars; i++) {
		airBrake[i]->setPipePressure(start);
		airBrake[i]->setAuxResPressure(start);
		if (start > target)
			airBrake[i]->setCylPressure(0);
		else if (5*target/7 < 2.5*(target-start))
			airBrake[i]->setCylPressure(5*target/7);
		else
			airBrake[i]->setCylPressure(2.5*(target-start));
		if (i > 0) {
			airBrake[i]->setPrev(airBrake[i-1]);
			airBrake[i]->setPrevOpen(true);
		}
		if (i < nCars) {
			airBrake[i]->setNext(airBrake[i+1]);
			airBrake[i]->setNextOpen(true);
		}
	}
	EngAirBrake* engBrake= (EngAirBrake*) airBrake[0];
	engBrake->setEngCutOut(false);
	if (start < target) {
		engBrake->setAutoControl(-1);
		//engDef.feedThreshold= target+20;
	}
	engBrake->setEqResPressure(target);
	int every= 1000;
	float dt= 1/(float)every;
	for (int j=0; j<100000; j++) {
//		if (j == 10*every)
//			engDef.feedThreshold= target;
		float t= j*dt;
		int m= 0;
		for (int i=0; i<=nCars; i++)
			airBrake[i]->updateAirSpeeds(dt);
		for (int i=0; i<=nCars; i++)
			airBrake[i]->updatePressures(dt);
		if (start < target) {
			for (int i=0; i<=nCars; i++)
				if (airBrake[i]->getCylPressure()>5)
					m++;
		} else {
			for (int i=0; i<=nCars; i++)
				if (airBrake[i]->getPipePressure()-target<-.5 ||
				  airBrake[i]->getPipePressure()-target>.5)
					m++;
		}
		if (j%every==0 || m==0)
			printf("%5.1f %3.0f %5.1f  %4.1f %4.1f  %4.1f %4.1f"
			  "  %4.1f %4.1f  %4.1f %4.1f\n",
			  t,engBrake->getEqResPressure(),
			  engBrake->getMainResPressure(),
			  airBrake[0]->getPipePressure(),
			  //airBrake[0]->getAuxResPressure(),
			  airBrake[0]->getCylPressure(),
			  airBrake[nCars/3]->getPipePressure(),
			  //airBrake[nCars/3]->getAuxResPressure(),
			  airBrake[nCars/3]->getCylPressure(),
			  airBrake[2*nCars/3]->getPipePressure(),
			  //airBrake[2*nCars/3]->getAuxResPressure(),
			  airBrake[2*nCars/3]->getCylPressure(),
			  airBrake[nCars]->getPipePressure(),
			  //airBrake[nCars]->getAuxResPressure(),
			  airBrake[nCars]->getCylPressure()
			 );
		if (m==0)
			break;
	}
	exit(0);
}
