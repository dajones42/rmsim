//	class for train state data and control
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
#ifndef TRAIN_H
#define TRAIN_H

#include <list>

#include "airbrake.h"

struct Signal;
typedef std::list<std::pair<Signal*,float> > SigDistList;

struct Train {
	string name;
	int id;
	Track::Location location;
	Track::Location endLocation;
	RailCarInst* firstCar;
	RailCarInst* lastCar;
	Train* otherTrain;
	float speed;
	float accel;
	float positionError;
	float dControl;
	float tControl;
	float bControl;
	float engBControl;
	float mass;
	float length;
	float drag0;
	float drag1;
	float drag2;
	float maxBForce;
	float maxCForce;
	EngAirBrake* engAirBrake;
	int moving;
	int modelCouplerSlack;
	int remoteControl;
	float targetSpeed;
	float maxTargetSpeed;
	float decelMult;
	float accelMult;
	float maxAccel;
	float maxDecel;
	float nextStopDist;
	float nextStopTime;
	SigDistList signalList;
	Train(int id= -1);
	~Train();
	void stop();
	void move(float dt);
	void calcAccel1(float dt);
	void calcAccel2(float dt);
	void calcRemoteControlAccel(float dt);
	void adjustControls(float dt);
	void convertToAirBrakes();
	void convertFromAirBrakes();
	void setMaxEqResPressure(float maxEqRes);
	void reverseCars();
	void reverse();
	void calcPerf();
	void setOccupied();
	void clearOccupied();
	float otherDist(Track::Location* other);
	void selectRandomCars(int min, int max, int nEng, int nCab);
	void selectCars(int nEng, int nCab, std::list<int>& carList);
	void positionCars();
	void coupleOther();
	void connectAirHoses();
	void setEngAirBrake(EngAirBrake* eab);
	void uncouple(RailCarInst* car, bool keepRear=false, int newID=-1);
	void uncouple(int nCars, bool keepRear, int newID=-1);
	void setUncouple(RailCarInst* car);
	void uncouple(osg::Vec3d& location);
	RailCarInst* findCar(osg::Vec3d location, float maxDist);
	void findCouplers(osg::Vec3d& location, osg::Vec3d& pf,
	  osg::Vec3d& pc, osg::Vec3d& pr);
	void initCouplerForces(int speed);
	void calcCouplerForces(float dt);
	void solveCouplerForces();
	void setModelsOn();
	void setModelsOff();
	void setHeadLight(bool on);
	float getThrottleInc();
	void scaleMaxTargetSpeed(float s) {
		maxTargetSpeed*= s;
		if (nextStopDist > 0)
			targetSpeed= maxTargetSpeed;
		else if (nextStopDist < 0)
			targetSpeed= 2*maxTargetSpeed;
	}
	void incThrottle() {
		if (nextStopDist == 0) {
			tControl+= getThrottleInc();
			if (tControl > 1)
				tControl= 1;
		} else {
			scaleMaxTargetSpeed(1.4142);
		}
	}
	void decThrottle() {
		if (nextStopDist == 0) {
			tControl-= getThrottleInc();
			if (tControl < 0)
				tControl= 0;
		} else {
			scaleMaxTargetSpeed(1/1.4142);
		}
	}
	float getReverserInc();
	void incReverser() {
		dControl+= getReverserInc();
		if (dControl > 1)
			dControl= 1;
	}
	void decReverser() {
		dControl-= getReverserInc();
		if (dControl < -1)
			dControl= -1;
	}
	void incBrakes() {
		bControl+= 1;
		if (bControl > 1)
			bControl= 1;
	}
	void decBrakes() {
		if (nextStopDist!=0) {
			nextStopDist= 0;
			bControl= -1;
		} else {
			bControl-= 1;
			if (bControl < -1)
				bControl= -1;
		}
	}
	void decEngBrakes() {
		engBControl-= .1;
		if (engBControl < 0)
			engBControl= 0;
	}
	void incEngBrakes() {
		engBControl+= .1;
		if (engBControl > 1)
			engBControl= 1;
	}
	void bailOff();
	float getLength();
	float getPassOffset();
	void alignSwitches(Track::Vertex* farv);
	void setPositionError(Track::SSEdge* ssEdge, float ssOffset, int rev);
	static Train* findTrain(int id);
	float coupleDistance(bool behind);
	void stopAtSwitch(Track::SwVertex* sw);
};

typedef map<string,Train*> TrainMap;
extern TrainMap trainMap;
typedef list<Train*> TrainList;
extern TrainList trainList;
extern TrainList oldTrainList;
extern Train *following;
extern Train *riding;
extern Train *myTrain;

void updateTrains(double dt);
Train* findTrain(double x, double y, double z);
Train* findTrain(double x, double y);
Train* findTrain(RailCarInst* car);
void selectWaybills(int min, int max, string& destination);

#endif
