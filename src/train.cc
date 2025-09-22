//	train coupling and movement code
//	also contains code for controlling speed of AI train
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
#include "changelog.h"

TrainMap trainMap;
TrainList trainList;
TrainList oldTrainList;
Train* myTrain= NULL;
Train* following= NULL;
Train* riding= NULL;
extern RailCarInst* myRailCar;

typedef map<int,Train*> TrainIDMap;
static TrainIDMap trainIDMap;

Train*  Train::findTrain(int id)
{
	TrainIDMap::iterator i= trainIDMap.find(id);
	return i==trainIDMap.end() ? NULL : i->second;
}

Train::Train(int id)
{
	if (id < 0) {
		static int nextID= 0;
		while (findTrain(nextID) != NULL)
			nextID++;
		id= nextID++;
	}
	this->id= id;
	trainIDMap[id]= this;
	firstCar= NULL;
	lastCar= NULL;
	otherTrain= NULL;
	engAirBrake= NULL;
	speed= 0;
	accel= 0;
	positionError= 0;
	dControl= 0;
	tControl= 0;
	bControl= 0;
	engBControl= 0;
	mass= 0;
	length= 0;
	drag0= 0;
	drag1= 0;
	drag2= 0;
	maxBForce= 0;
	moving= 5;
	modelCouplerSlack= 1;
	remoteControl= 0;
	targetSpeed= 0;
	maxTargetSpeed= 2.235;
	decelMult= .3;
	accelMult= .1;
	nextStopDist= 0;
	nextStopTime= 0;
};

Train::~Train()
{
	trainIDMap.erase(id);
	while (firstCar != NULL) {
		RailCarInst* t= firstCar;
		firstCar= t->next;
		//remove models
		delete t;
	}
}

//	calculates train acceleration for user controled trains
//	assumes air brakes and non-rigid couplers
//	speed is calculates separately for each car
//	speed and accel for train is averaged over the cars
//	air brake state is calculated in multiple steps if the time step
//	is large to prevent overshoot
void Train::calcAccel2(float dt)
{
	if (engAirBrake != NULL)
		engAirBrake->setAutoControl(bControl);
	int n= (int)(dt/.005)+1;
	float dt1= dt/n;
	for (int i=0; i<n; i++) {
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
			if (car->airBrake != NULL)
				car->airBrake->updateAirSpeeds(dt1);
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
			if (car->airBrake != NULL)
				car->airBrake->updatePressures(dt1);
#if 0
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
			if (car->airBrake != NULL)
				car->airBrake->calcDeltas(dt1);
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
			if (car->airBrake != NULL)
				car->airBrake->applyDeltas();
#endif
	}
	calcCouplerForces(dt);
	speed= 0;
	accel= 0;
	n= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		float s= car->speed;
		if (s == 0) {
			if (car->force > car->drag)
				car->force-= car->drag;
			else if (car->force < -car->drag)
				car->force+= car->drag;
			else
				car->force= 0;
		}
		float a= car->force*car->massInv;
		car->speed+= a*dt;
		if ((s>0 && car->speed<0) || (s<0 && car->speed>0)) {
			car->speed= 0;
		}
//		fprintf(stderr,"%s %f %f %f %f %f %f\n",car->def->name.c_str(),
//		  a,car->speed,car->force,car->slack,car->cU,car->mass);
//		fprintf(stderr,"%s %f %f %f %f %f %f\n",car->def->name.c_str(),
//		  a,car->speed,car->force,car->drag,car->cU,car->mass);
		speed+= car->speed;
		accel+= a;
		n++;
	}
	speed/= n;
	accel/= n;
}

//	calculates train acceleration for AI trains
//	assumes simple brakes and rigid couplers
//	speed is calculates for the entire train
void Train::calcAccel1(float dt)
{
	float s= speed>0 ? speed : -speed;
	float f= 0;
	float ef= 0;
	float cf= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		f+= car->mass*car->grade;
		cf+= car->mass*car->curvature;
		if (car->engine)
			ef+= car->engine->getForce(tControl,dControl,car,dt);
	}
	float a1= (9.8*f+ef)/mass;
	float a2= (bControl*maxBForce*(s>=7.5?1:(.2*(7.5-s)+1)) +
	  drag0 + s*(drag1 + drag2*s) + .0004*9.8*cf)/mass;
	if (speed > 0)
		accel= a1 - a2;
	else if (speed < 0)
		accel= a1 + a2;
	else if (a1 > a2)
		accel= a1 - a2;
	else if (a1 < -a2)
		accel= a1 + a2;
	else
		accel= 0;
//	if (s != 0)
//		fprintf(stderr," %f %f %f\n",accel,a1,a2);
	s= speed;
	speed+= accel*dt;
	if ((s>0 && speed<0) || (s<0 && speed>0)) {
		speed= 0;
		accel= 0;
	}
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		car->speed= speed;
}

//	calculates train acceleration for remote control trains
//	assumes simple brakes and rigid couplers
//	speed is calculates for the entire train
void Train::calcRemoteControlAccel(float dt)
{
	float s= speed;
	speed+= accel*dt;
	if ((s>0 && speed<0) || (s<0 && speed>0)) {
		speed= 0;
		accel= 0;
	}
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		car->speed= speed;
}

//	converts simple brake control to equivalent air brake pressures
//	used when user takes control of AI train
void Train::convertToAirBrakes()
{
	if (bControl > 0) {
		float max= engAirBrake->getMaxEqResPressure();
		float cyl= bControl*5*max/7;
		float eq= max-2*cyl/7;
		if (engAirBrake != NULL)
			engAirBrake->setEqResPressure(eq);
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
			car->handBControl= 0;
			if (car->airBrake != NULL) {
				car->airBrake->setAuxResPressure(eq);
				car->airBrake->setEmergResPressure(max);
				car->airBrake->setPipePressure(eq);
				car->airBrake->setCylPressure(cyl);
				car->airBrake->setRetainer(0);
			}
		}
	} else {
		float max= engAirBrake->getMaxEqResPressure();
		if (engAirBrake != NULL)
			engAirBrake->setEqResPressure(max);
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
			car->handBControl= 0;
			if (car->airBrake != NULL) {
				car->airBrake->setAuxResPressure(max);
				car->airBrake->setEmergResPressure(max);
				car->airBrake->setPipePressure(max);
				car->airBrake->setCylPressure(0);
				car->airBrake->setRetainer(0);
			}
		}
	}
	bControl= 0;
	modelCouplerSlack= 1;
	connectAirHoses();
	fprintf(stderr,"convertToAir %f %f %f %f %d\n",
	  engAirBrake->getEqResPressure(),
	  engAirBrake->getCylPressure(),
	  engAirBrake->getAuxResPressure(),
	  engAirBrake->getPipePressure(),
	  engAirBrake->getTripleValveState());
}

void Train::setMaxEqResPressure(float max)
{
	if (engAirBrake != NULL)
		engAirBrake->setMaxEqResPressure(max);
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		if (car->airBrake != NULL) {
			car->airBrake->setAuxResPressure(max-2);
			car->airBrake->setEmergResPressure(max);
			car->airBrake->setPipePressure(max);
			car->airBrake->setCylPressure(0);
			car->airBrake->setRetainer(0);
		}
	}
}

//	converts from air brake to simple brake control
//	used when user releases control of AI train
void Train::convertFromAirBrakes()
{
	bControl= 0;
	engBControl= 0;
	modelCouplerSlack= 0;
}

//	adjusts throttle and brake controls for AI train based on
//	distance and time to next stop
//	calculates a target speed range and then adjusts controls
//	to get speed into range
//	assumes notched throttle and infinitely variable brakes
void Train::adjustControls(float dt)
{
	const float tStep= .01;
	float s= speed;
	float a= accel;
	float target= targetSpeed;
	float d= nextStopDist;
	if (d > 0) {
		dControl= 1;
	} else {
		dControl= -1;
		s= -s;
		a= -a;
		if (target > 5)
			target= -target/2;
		else
			target= -target;
		d= -d;
	}
	for (SigDistList::iterator i=signalList.begin(); i!=signalList.end();
	  ++i) {
		float d1= d-i->second;
		i->first->trainDistance= d1;//+length;
		if (i->first->getState()==Signal::CLEAR ||
		  i->first->isDistant())
			continue;
		if (d1 < -1)
			continue;
		if (d > d1)
			d= d1;
		moving= 5;
	}
	if (d < 0) {
		tControl= 0;
		bControl= 1;
		stop();
		return;
	}
	float decel= decelMult*maxDecel;
	float speedSq= speed*speed;
	float maxSpeedSq= target*target;
	if (maxSpeedSq > 2*d*decel)
		maxSpeedSq= 2*d*decel;
	float minSpeedSq= .96*maxSpeedSq;
	if (nextStopTime > 0) {
		float s2= (d+maxSpeedSq/(2*decel))/nextStopTime;
		s2= s2*s2;
		if (minSpeedSq > s2) {
			minSpeedSq= s2;
			if (maxSpeedSq > 1.4*s2)
				maxSpeedSq= 1.4*s2;
		}
	}
	if (minSpeedSq > (d-.2)*decel)
		minSpeedSq= (d-.2)*decel;
	if (minSpeedSq < 0)
		minSpeedSq= 0;
//	fprintf(stderr,"%s %.3f %.2f %.1f %f %f %f %f %f\n",
//	  name.c_str(),tControl,bControl,dControl,
//	  s*s,maxSpeedSq,minSpeedSq,d,target);
	if (speedSq > .98*maxSpeedSq) {
		float ta= 0;
		if (speedSq > maxSpeedSq) {
			ta= -.5*speedSq/(d>.01?d:.01);
			if (speedSq > 2*maxSpeedSq && ta > -.2*maxDecel)
				ta= -.2*maxDecel;
			else if (ta > -.1*maxDecel)
				ta= -.1*maxDecel;
		}
		if (tControl > 0) {
			while (a>ta && tControl>.5*tStep) {
				tControl-= tStep;
				a-= tStep*maxAccel;
			}
			if (tControl < 0)
				tControl= 0;
		}
		if (tControl < .5*tStep) {
			tControl= 0;
			float b= bControl + (a-ta)/maxDecel;
			if (b < 0)
				b= 0;
			if (b > 1)
				b= 1;
			if (bControl < b)
				bControl= b;
		}
#if 0
		fprintf(stderr,"%s %.3f %.2f %.0f %f %f %f %f %f %f\n",
		  name.c_str(),tControl,bControl,dControl,
		  s,sqrt(maxSpeedSq),a,ta,maxDecel,nextStopDist);
#endif
	} else if (speedSq < minSpeedSq) {
		float mina= 0;
		if (speedSq < .98*minSpeedSq)
			mina= accelMult*maxAccel;
		if (s == 0)
			bControl= 0;
		if (bControl > 0) {
			float b= bControl + (a-mina)/maxDecel;
			if (b < 0)
				b= 0;
			if (bControl > b)
				bControl= b;
		} else if (a<mina && tControl<1-tStep) {
			tControl+= tStep;
		}
#if 0
		fprintf(stderr,"%s %.3f %.1f %.0f %f %f %f %f %f\n",
		  name.c_str(),tControl,bControl,dControl,
		  s,sqrt(minSpeedSq),a,mina,nextStopDist);
#endif
	} else if (tControl==0 && speedSq>2*decelMult*maxDecel*d) {
		float ta= -.5*speedSq/(d>.01?d:.01);
		bControl+= (a-ta)/maxDecel;
		if (bControl < 0)
			bControl= 0;
		if (bControl > 1)
			bControl= 1;
	}
#if 0
	if (d < 100)
		fprintf(stderr,"%s %.3f %.2f %.0f %f %f %f %f %f %f\n",
		  name.c_str(),tControl,bControl,dControl,
		  s,sqrt(minSpeedSq),sqrt(maxSpeedSq),a,d,nextStopDist);
#endif
}

//	forces train to stop
//	used at end of track etc
void Train::stop()
{
	speed= 0;
	accel= 0;
	tControl= 0;
	bControl= 1;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		car->speed= 0;
	fprintf(stderr,"stopped %f\n",nextStopDist);
	nextStopDist= 0;
	nextStopTime= 0;
}

//	moves train given time step dt
//	checks for coupling with another train after move
//	also checks to split an uncouped train
void Train::move(float dt)
{
	if (nextStopDist != 0) {
		adjustControls(dt);
		if (modelCouplerSlack) {
			for (RailCarInst* car=firstCar; car!=NULL &&
			  car!=lastCar; car=car->next)
				if (car->maxSlack > .5)
					uncouple(car);
		}
	}
//	fprintf(stderr,"%.3f %.3f %.3f %f %f %f\n",
//	  tControl,bControl,dControl,speed,accel,nextStopDist);
	if (remoteControl)
		calcRemoteControlAccel(dt);
	else if (modelCouplerSlack && nextStopDist==0)
		calcAccel2(dt);
	else
		calcAccel1(dt);
#if 0
	if (speed==0 && accel==0 && location.edge->track->matrix==NULL &&
	  endLocation.edge->track->matrix==NULL && nextStopDist==0)
		moving--;
	else
		moving= 5;
#endif
	float err= .1*positionError;
	positionError-= err;
	float dx1= firstCar->speed*dt + err;
	float dx2= lastCar->speed*dt + err;
//	fprintf(stderr,"move %s %lf %lf %lf %lf %lf\n",name.c_str(),
//	  dt,accel,speed,dx1,dx2);
	int couple= 0;
	if (dx1>0 && location.edge->occupied>1) {
		float d= otherDist(&location);
		if (d <= dx1) {
			dt*= d/dx1;
			dx1= d;
			couple= 1;
		}
	} else if (dx2<0 && endLocation.edge->occupied>1) {
		float d= -otherDist(&endLocation);
		if (d >= dx2) {
			dt*= d/dx2;
			dx2= d;
			couple= 1;
		}
	}
	if (location.move(dx1,1,1)) {
		location.move(-dx1,0,-1);
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
			if (car->speed < 0)
				car->move(dt*car->speed+err);
		if (dx2 < 0)
			endLocation.move(dx2,0,1);
//		fprintf(stderr,"cantmove %s %lf %lf %lf %lf %lf\n",
//		  name.c_str(),dt,accel,speed,dx1,dx2);
		//stop();
		return;
	}
	if (endLocation.move(dx2,1,-1)) {
		endLocation.move(-dx2,0,1);
		if (dx1 > 0)
			location.move(dx1,0,-1);
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
			if (car->speed > 0)
				car->move(dt*car->speed+err);
//		fprintf(stderr,"cantmove %s %lf %lf %lf %lf %lf\n",
//		  name.c_str(),dt,accel,speed,dx1,dx2);
		//stop();
		return;
	}
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		car->move(dt*car->speed+err);
	if (nextStopDist != 0) {
		if (speed==0 && accel==0 &&
		  -.5<nextStopDist && nextStopDist<.5) {
			stop();
		} else if (nextStopDist > 0) {
			nextStopDist-= dx1;
			nextStopTime-= dt;
			if (nextStopDist <= 0) {
				location.move(nextStopDist,0,1);
				endLocation.move(nextStopDist,0,-1);
				for (RailCarInst* car=firstCar; car!=NULL;
				  car=car->next)
					car->move(nextStopDist);
				stop();
			}
		} else {
			nextStopDist-= dx1;
			nextStopTime-= dt;
			if (nextStopDist >= 0) {
				location.move(-nextStopDist,0,1);
				endLocation.move(-nextStopDist,0,-1);
				for (RailCarInst* car=firstCar; car!=NULL;
				  car=car->next)
					car->move(-nextStopDist);
				stop();
			}
		}
		if (couple)// && modelCouplerSlack)
			coupleOther();
		if (couple)
			stop();
	} else if (couple) {
		coupleOther();
	} else if (modelCouplerSlack) {
		for (RailCarInst* car=firstCar; car!=NULL && car!=lastCar;
		  car=car->next) {
			if (car->maxSlack>.5 && car->slack>.5) {
				uncouple(car);
			} else if (car->maxSlack<.5) {
				float draftGearGap=
				  .5*(car->maxSlack-car->def->couplerGap);
				float dg1= draftGearGap*car->couplerState[1];
				float dg2=
				  draftGearGap*car->next->couplerState[0];
				float s= car->slack;
				if (s < 0)
					s= 0;
				else if (s > car->maxSlack)
					s= car->maxSlack;
				if (s < dg1+dg2) {
					dg1= dg2= .5*s;
				} else if (s > dg1+dg2+car->def->couplerGap) {
					dg1= dg2= .5*(s-car->def->couplerGap);
				}
				car->couplerState[1]= dg1/draftGearGap;
				car->next->couplerState[0]= dg2/draftGearGap;
			}
		}
	}
}

//	moves all trains and cleans up if any coupling
void updateTrains(double dt)
{
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t->firstCar!=NULL && (t==myTrain || t->moving>0))
			t->move(dt);
	}
	if (oldTrainList.size() > 0) {
		for (TrainList::iterator i=oldTrainList.begin();
		  i!=oldTrainList.end(); ++i) {
			Train* t= *i;
			trainList.remove(t);
			delete t;
		}
		oldTrainList.clear();
	}
}

//	finds train with head end nearest specified point
Train* findTrain(double x, double y, double z)
{
	double bestd= 1e40;
	Train* bestt= NULL;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		WLocation loc;
		t->location.getWLocation(&loc);
		double dx= loc.coord[0]-x;
		double dy= loc.coord[1]-y;
		double dz= loc.coord[2]-z;
		double d= dx*dx + dy*dy + dz*dz;
		if (bestd > d) {
			bestd= d;
			bestt= t;
		}
	}
	return bestt;
}

//	finds train with head end nearest specified point
Train* findTrain(double x, double y)
{
	double bestd= 1e40;
	Train* bestt= NULL;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		WLocation loc;
		t->location.getWLocation(&loc);
		double dx= loc.coord[0]-x;
		double dy= loc.coord[1]-y;
		double d= dx*dx + dy*dy;
		if (bestd > d) {
			bestd= d;
			bestt= t;
		}
	}
	return bestt;
}

//	finds train containing specified car
Train* findTrain(RailCarInst* car)
{
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		for (RailCarInst* c=t->firstCar; c!=NULL; c=c->next)
			if (c == car)
				return t;
	}
	return NULL;
}

//	calculates train performance numbers from cars
//	used for AI accel
//	this is old code that could probably be eliminated
void Train::calcPerf()
{
	length= 0;
	mass= 0;
	drag0= 0;
	drag1= 0;
	drag2= 0;
	maxBForce= 0;
	maxAccel= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		RailCarDef* def= car->def;
		length+= def->length;
		mass+= car->mass + def->mass2;
		drag0+= .001*car->mass*def->drag0a +
		  def->axles*def->drag0b;
		drag1+= def->drag1;
		drag2+= def->area *
		  (car==firstCar ? def->drag2a : def->drag2);
		maxBForce+= def->maxBForce;
		if (car->engine)
			maxAccel+= car->engine->getMaxForce();
	}
	maxDecel= maxBForce/mass;
	maxAccel/= mass;
	if (maxAccel > 0) {
		float am= .5*decelMult*maxDecel/maxAccel;
		if (accelMult < am)
			accelMult= am;
	}
//	fprintf(stderr,"perf %s %f %f %f %f\n",
//	  name.c_str(),maxDecel,maxAccel,decelMult,accelMult);
//	fprintf(stderr," %f %f %f %f %f\n",length,mass,drag0,drag1,drag2);
}

//	uncouples train at specified car
//	shuts air hoses and disables pulling via coupler
//	train will be split later when the distance between cars increases
//	pushing via coupler is still possible
void Train::setUncouple(RailCarInst* car)
{
	int n= 1;
	RailCarInst* c= firstCar;
	for (; c!=NULL; c=c->next) {
		if (c == car)
			break;
		n++;
	}
	if (c==NULL || c->next==NULL)
		return;
	car->maxSlack= 1;
	fprintf(stderr,"setUncouple %d\n",n);
	if (car->airBrake != NULL)
		car->airBrake->setNextOpen(false);
	if (car->next->airBrake != NULL)
		car->next->airBrake->setPrevOpen(false);
}

//	uncouples an AI train
void Train::uncouple(int nCars, bool keepRear, int newID)
{
	if (nCars < 0) {
		int n= 0;
		for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
			n++;
		nCars+= n;
	}
	fprintf(stderr,"uncouple %d %d\n",nCars,keepRear);
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
		if (nCars-- == 1)
			uncouple(c,keepRear,newID);
}

//	complete uncoupling by spliting train
//	uncouples after the specified car
void Train::uncouple(RailCarInst* car, bool keepRear, int newID)
{
	fprintf(stderr,"uncouple %s %p %p %p\n",
	  name.c_str(),firstCar,lastCar,car);
	RailCarInst* c= firstCar;
	for (; c!=NULL; c=c->next)
		if (c == car)
			break;
	if (c==NULL || c->next==NULL)
		return;
	if (car->airBrake != NULL)
		car->airBrake->setNext(NULL);
	if (car->next->airBrake != NULL)
		car->next->airBrake->setPrev(NULL);
	listener.removeTrain(this);
	Train* newt= new Train(newID);
	trainList.push_back(newt);
	newt->name= name;
	if (keepRear) {
		newt->firstCar= firstCar;
		newt->lastCar= car;
		firstCar= car->next;
		car->next->prev= NULL;
		car->next= NULL;
		newt->location= location;
		newt->endLocation= location;
		location= endLocation;
	} else {
		newt->firstCar= car->next;
		car->next->prev= NULL;
		car->next= NULL;
		newt->lastCar= lastCar;
		lastCar= car;
		newt->endLocation= endLocation;
		newt->location= endLocation;
		endLocation= location;
	}
	ChangeLog::instance()->addUncouple(this,newt,keepRear);
	car->slack= 0;
	EngAirBrake* eab= engAirBrake;
	if (engAirBrake != NULL)
		engAirBrake->setEngCutOut(true);
	engAirBrake= NULL;
	float d= 0;
	for (c=firstCar; c!=NULL; c=c->next) {
		d+= c->def->length;
		if (c->next != NULL)
			d+= c->slack;
		if (c->airBrake!=NULL && c->airBrake==eab)
			engAirBrake= eab;
		else if (c->airBrake!=NULL &&
		  dynamic_cast<EngAirBrake*>(c->airBrake)!=NULL)
			engAirBrake= dynamic_cast<EngAirBrake*>(c->airBrake);
	}
	if (engAirBrake != NULL)
		engAirBrake->setEngCutOut(false);
//	fprintf(stderr,"len1 %f\n",d);
	if (keepRear)
		location.move(d,0,0);
	else
		endLocation.move(-d,0,0);
	d= 0;
	for (c=newt->firstCar; c!=NULL; c=c->next) {
		d+= c->def->length;
		if (c->next != NULL)
			d+= c->slack;
		if (c->airBrake!=NULL && c->airBrake==eab)
			newt->engAirBrake= eab;
		else if (c->airBrake!=NULL &&
		  dynamic_cast<EngAirBrake*>(c->airBrake)!=NULL)
			newt->engAirBrake=
			  dynamic_cast<EngAirBrake*>(c->airBrake);
	}
	if (newt->engAirBrake != NULL)
		newt->engAirBrake->setEngCutOut(false);
//	fprintf(stderr,"len2 %f\n",d);
	if (keepRear) {
		newt->endLocation.move(-d,0,0);
		newt->endLocation.edge->occupied++;
	} else {
		newt->location.move(d,0,0);
		newt->location.edge->occupied++;
	}
	calcPerf();
	newt->calcPerf();
	if (myTrain != NULL) {
		newt->dControl= myTrain->dControl;
		newt->tControl= myTrain->tControl;
		newt->bControl= 0;
		newt->engBControl= 0;
	} else {
		newt->dControl= 0;
		newt->tControl= 0;
		newt->bControl= 1;
		newt->engBControl= 0;
	}
	if (myTrain == this) {
		int nEng= 0;
		for (c=firstCar; c!=NULL; c=c->next)
			if (c->engine || c==myRailCar)
				nEng++;
		for (c=newt->firstCar; c!=NULL; c=c->next)
			if (c == myRailCar)
				nEng= 0;
		if (nEng == 0) {
			newt->bControl= myTrain->bControl;
			newt->engBControl= myTrain->engBControl;
			newt->nextStopDist= myTrain->nextStopDist;
			newt->nextStopTime= myTrain->nextStopTime;
			newt->targetSpeed= myTrain->targetSpeed;
			myTrain->bControl= 0;
			myTrain->engBControl= 0;
			myTrain->nextStopDist= 0;
			myTrain->nextStopTime= 0;
			myTrain->targetSpeed= 0;
			myTrain= newt;
		}
	}
	listener.addTrain(this);
	listener.addTrain(newt);
#if 0
	fprintf(stderr,"uncouple %p %p\n",this,newt);
	fprintf(stderr," %p-%p %f %d\n",
	  location.edge->v1,location.edge->v2,
	  location.offset,location.rev);
	fprintf(stderr," %p-%p %f %d\n",
	  endLocation.edge->v1,endLocation.edge->v2,
	  endLocation.offset,endLocation.rev);
	fprintf(stderr," %p-%p %f %d\n",
	  newt->location.edge->v1,newt->location.edge->v2,
	  newt->location.offset,newt->location.rev);
	fprintf(stderr," %p-%p %f %d\n",
	  newt->endLocation.edge->v1,newt->endLocation.edge->v2,
	  newt->endLocation.offset,newt->endLocation.rev);
#endif
}

//	uncouples train nearest a mouse click location
void Train::uncouple(osg::Vec3d& clickLocation)
{
	float bestd= 1000;
	RailCarInst* bestc= NULL;
	Track::Location loc= location;
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		loc.move(-c->def->length,0,0);
		WLocation wloc;
		loc.getWLocation(&wloc);
		float dx= clickLocation[0]-wloc.coord[0];
		float dy= clickLocation[1]-wloc.coord[1];
		float dz= clickLocation[2]-wloc.coord[2];
		float d= dx*dx + dy*dy + dz*dz;
		fprintf(stderr,"%s %f\n",c->def->name.c_str(),d);
		if (d < bestd) {
			bestd= d;
			bestc= c;
		}
	}
	fprintf(stderr,"uncouple distsq %f\n",bestd);
	if (bestc != NULL)
		setUncouple(bestc);
}

//	finds the car that is closest to the specified location
RailCarInst* Train::findCar(osg::Vec3d location, float maxDist)
{
	float bestd= maxDist*maxDist;
	RailCarInst* bestc= NULL;
	Track::Location loc= this->location;
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next) {
		loc.move(-c->def->length/2,0,0);
		WLocation wloc;
		loc.getWLocation(&wloc);
		float dx= location[0]-wloc.coord[0];
		float dy= location[1]-wloc.coord[1];
		float dz= location[2]-wloc.coord[2];
		float d= dx*dx + dy*dy + dz*dz;
		fprintf(stderr,"%s %f\n",c->def->name.c_str(),d);
		if (d < bestd) {
			bestd= d;
			bestc= c;
		}
		loc.move(-c->def->length/2,0,0);
	}
	fprintf(stderr,"findcar distsq %f %p\n",bestd,bestc);
	return bestc;
}

//	couple to another train
//	train identified by move code
//	air brake hoses are not connected
void Train::coupleOther()
{
	if (otherTrain == NULL)
		return;
	if (otherTrain == myTrain) {
		myTrain->otherTrain= this;
		myTrain->coupleOther();
		return;
	}
	if (ttoSim.takeControlOfAI(otherTrain))
		otherTrain->convertToAirBrakes();
	ChangeLog::instance()->addCouple(this,otherTrain);
	listener.removeTrain(this);
	listener.removeTrain(otherTrain);
	if (endLocation.distance(&otherTrain->location) < .001) {
		fprintf(stderr,"couple rf\n");
		endLocation= otherTrain->endLocation;
		otherTrain->location.edge->occupied--;
		lastCar->next= otherTrain->firstCar;
		otherTrain->firstCar->prev= lastCar;
		lastCar= otherTrain->lastCar;
	} else if (endLocation.distance(&otherTrain->endLocation) < .001) {
		fprintf(stderr,"couple rr\n");
		endLocation= otherTrain->location;
		endLocation.rev= !endLocation.rev;
		otherTrain->endLocation.edge->occupied--;
		otherTrain->reverseCars();
		lastCar->next= otherTrain->firstCar;
		otherTrain->firstCar->prev= lastCar;
		lastCar= otherTrain->lastCar;
	} else if (location.distance(&otherTrain->endLocation) < .001) {
		fprintf(stderr,"couple fr\n");
		location= otherTrain->location;
		otherTrain->endLocation.edge->occupied--;
		otherTrain->lastCar->next= firstCar;
		firstCar->prev= otherTrain->lastCar;
		firstCar= otherTrain->firstCar;
	} else if (location.distance(&otherTrain->location) < .001) {
		fprintf(stderr,"couple ff\n");
		location= otherTrain->endLocation;
		location.rev= !location.rev;
		otherTrain->location.edge->occupied--;
		otherTrain->reverseCars();
		otherTrain->lastCar->next= firstCar;
		firstCar->prev= otherTrain->lastCar;
		firstCar= otherTrain->firstCar;
	}
	if (engAirBrake == NULL)
		engAirBrake= otherTrain->engAirBrake;
	else if (otherTrain->engAirBrake != NULL)
		otherTrain->engAirBrake->setEngCutOut(true);
	float momentum= speed*mass;
	calcPerf();
	speed= momentum/mass;
	listener.addTrain(this);
#if 0
	if (myTrain!=NULL && myTrain==otherTrain) {
		dControl= otherTrain->dControl;
		tControl= otherTrain->tControl;
		bControl= otherTrain->bControl;
		engBControl= otherTrain->engBControl;
		name= otherTrain->name;
		myTrain= this;
	}
#endif
	if (following!=NULL && following==otherTrain)
		following= this;
	if (riding!=NULL && riding==otherTrain)
		riding= this;
	otherTrain->firstCar= NULL;
	otherTrain->lastCar= NULL;
	oldTrainList.push_back(otherTrain);
	otherTrain= NULL;
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
		c->maxSlack= c->def->maxSlack;
#if 0
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
		fprintf(stderr,"%s %f %f\n",
		  c->def->name.c_str(),c->auxResPres,c->cylPres);
#endif
}

//	finds distance to another train on same track edge
float Train::otherDist(Track::Location* loc)
{
	float bestd= 1e30;
//	fprintf(stderr,"otherDist %p %p\n",this,otherTrain);
	if (otherTrain != NULL) {
		float d= loc->distance(&otherTrain->location);
		if (bestd > d)
			bestd= d;
		d= loc->distance(&otherTrain->endLocation);
		if (bestd > d)
			bestd= d;
	}
	if (bestd < 1e30)
		return bestd;
	for (TrainList::iterator i=trainList.begin();
	  i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t == this)
			continue;
		float d= loc->distance(&t->location);
		if (bestd > d) {
			bestd= d;
			otherTrain= t;
		}
		d= loc->distance(&t->endLocation);
		if (bestd > d) {
			bestd= d;
			otherTrain= t;
		}
	}
	if (bestd < 1e30)
		fprintf(stderr,"otherdist %f\n",bestd);
	return bestd;
}

//	reverses a train so that the other end is the forward end
void Train::reverse()
{
	reverseCars();
	Track::Location loc= location;
	location= endLocation;
	endLocation= loc;
	location.rev= !location.rev;
	endLocation.rev= !endLocation.rev;
}

//	reverses the order of train's list of cars
//	also reverses air brake hoses and swaps coupler slack information
void Train::reverseCars()
{
//	if (lastCar == firstCar)
//		return;
	RailCarInst* car= firstCar;
	lastCar= firstCar;
	firstCar= NULL;
	bool connect= false;
	while (car != NULL) {
		RailCarInst* t= car->next;
		bool tconn= car->airBrake!=NULL &&
		  car->airBrake->getNext()!=NULL;
		if (car->airBrake != NULL) {
			car->airBrake->setNext(NULL);
			car->airBrake->setPrev(NULL);
		}
		for (int i=0; i<car->wheels.size(); i++)
			car->wheels[i].location.rev=
			  !car->wheels[i].location.rev;
		car->rev= !car->rev;
		car->next= firstCar;
		if (firstCar != NULL) {
			car->slack= firstCar->slack;
			firstCar->prev= car;
			if (connect && car->airBrake!=NULL) {
				car->airBrake->setNext(firstCar->airBrake);
				firstCar->airBrake->setPrev(car->airBrake);
			}
		}
		firstCar= car;
		car= t;
		connect= tconn;
	}
	firstCar->prev= NULL;
	lastCar->slack= 0;
}

//	connects air brake hoses between all cars in train and opens
//	valves so air can flow
void Train::connectAirHoses()
{
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		if (c->airBrake!=NULL && c->next->airBrake!=NULL) {
			c->airBrake->setNext(c->next->airBrake);
			c->airBrake->setNextOpen(true);
			c->next->airBrake->setPrev(c->airBrake);
			c->next->airBrake->setPrevOpen(true);
		} else if (c->airBrake != NULL) {
			c->airBrake->setNext(NULL);
			c->airBrake->setNextOpen(false);
		} else if (c->next->airBrake != NULL) {
			c->next->airBrake->setPrev(NULL);
			c->next->airBrake->setPrevOpen(false);
		}
	}
	if (firstCar->airBrake != NULL) {
		firstCar->airBrake->setPrev(NULL);
		firstCar->airBrake->setPrevOpen(false);
	}
	if (lastCar->airBrake != NULL) {
		lastCar->airBrake->setNext(NULL);
		lastCar->airBrake->setNextOpen(false);
	}
	if (engAirBrake == NULL) {
		for (RailCarInst* c=firstCar; c!=NULL; c=c->next) {
			if (c->airBrake!=NULL &&
			  dynamic_cast<EngAirBrake*>(c->airBrake)!=NULL) {
				engAirBrake=
				  dynamic_cast<EngAirBrake*>(c->airBrake);
				engAirBrake->setEngCutOut(false);
				bControl= 0;
				break;
			}
		}
	}
}

//	sets the engine air brake to use for controlling brakes in train
void Train::setEngAirBrake(EngAirBrake* eab)
{
	fprintf(stderr,"setEngAirBrake %p %p\n",engAirBrake,eab);
	if (engAirBrake != NULL)
		engAirBrake->setEngCutOut(true);
	engAirBrake= eab;
	if (engAirBrake != NULL)
		engAirBrake->setEngCutOut(false);
}

//	selects random cars from the trains car list
//	min and max specify the range of the number of cars to select
//	nEng is the number of cars on the front of train that must be kept
//	nCab is the number of cars on the back of train that must be kept
void Train::selectRandomCars(int min, int max, int nEng, int nCab)
{
	int n= 0;
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
		n++;
	if (max > min)
		min+= (int)((max-min+1)*drand48());
	if (min+nEng+nCab >= n)
		return;
	n-= nEng+nCab;
	std::list<int> carList;
	for (int i=0; i<min; i++) {
		if (n <= 0)
			break;
		int j= ((int)(100*n*drand48()))/100;
		carList.push_back(j);
		n--;
	}
	ChangeLog::instance()->addRandomSelection(this,nEng,nCab,carList);
	selectCars(nEng,nCab,carList);
}

//	second half of selectRandomCars
//	this half is called by clients to duplicate server's selection
void Train::selectCars(int nEng, int nCab, std::list<int>& carList)
{
	typedef vector<RailCarInst*> CarVector;
	CarVector cars;
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
		cars.push_back(c);
	int n= cars.size();
	CarVector cabs;
	for (int i=0; i<nCab; i++) {
		int j= n-nCab+i;
		cabs.push_back(cars[j]);
	}
	n-= nCab;
	RailCarInst* list= firstCar;
	firstCar= lastCar= NULL;
	for (int i=0; i<nEng; i++) {
		RailCarInst* c= cars[i];
		cars[i]= cars[--n];
		c->next= NULL;
		c->prev= lastCar;
		if (lastCar == NULL)
			firstCar= c;
		else
			lastCar->next= c;
		lastCar= c;
	}
	for (std::list<int>::iterator i=carList.begin(); i!=carList.end();
	  i++) {
		int j= *i;
		RailCarInst* c= cars[j];
		cars[j]= cars[--n];
		c->next= NULL;
		c->prev= lastCar;
		if (lastCar == NULL)
			firstCar= c;
		else
			lastCar->next= c;
		lastCar= c;
	}
	for (int i=0; i<nCab; i++) {
		RailCarInst* c= cabs[i];
		c->next= NULL;
		c->prev= lastCar;
		if (lastCar == NULL)
			firstCar= c;
		else
			lastCar->next= c;
		lastCar= c;
	}
	for (int i=0; i<n; i++) {
		RailCarInst* c= cars[i];
		delete c;
	}
}

//	calculates coupler and other car forces
//	couplers modeled as one dimensional collisions
//	first calculates impulse forces needed to make relative speeds
//	zero when needed
//	then calculates other forces acting on each car
//	finally calculates forces needed to make accelerations equal
//	for connected cars
//	if some cars are not moving, the last step is performed twice to
//	make sure the forces are large enough to start cars moving
void Train::calcCouplerForces(float dt)
{
	if (firstCar == NULL)
		return;
	if (firstCar == lastCar) {
		firstCar->calcForce(tControl,dControl,engBControl,dt);
		if (firstCar->speed != 0)
			return;
		if (firstCar->force > firstCar->drag)
			firstCar->force-= firstCar->drag;
		else if (firstCar->force < -firstCar->drag)
			firstCar->force+= firstCar->drag;
		else
			firstCar->force= 0;
		return;
	}
	maxCForce= 0;
	initCouplerForces(1);
	solveCouplerForces();
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		if (maxCForce < c->cU)
			maxCForce= c->cU;
		else if (maxCForce < -c->cU)
			maxCForce= -c->cU;
		c->speed+= c->cU*c->massInv;
		c->next->speed-= c->cU*c->next->massInv;
	}
	int n= 0;
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next) {
		if (c->speed == 0)
			n++;
		c->calcForce(tControl,dControl,engBControl,dt);
	}
	if (n > 0) {
		initCouplerForces(0);
		solveCouplerForces();
		for (RailCarInst* c=firstCar; c!=NULL; c=c->next) {
			if (c->speed != 0)
				continue;
			float f= c->force + c->cU;
			if (c != lastCar)
				f-= c->next->cU;
			if (f > 0)
				c->force-= c->drag;
			else if (f < 0)
				c->force+= c->drag;
		}
	}
	initCouplerForces(0);
	solveCouplerForces();
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		if (maxCForce < c->cU)
			maxCForce= c->cU;
		else if (maxCForce < -c->cU)
			maxCForce= -c->cU;
		c->force+= c->cU;
		c->next->force-= c->cU;
	}
}

//	creates a tridiagonal system of equations for calculating
//	coupler forces
//	if speed is not zero
//		the righthand side is set for calculating impulse forces
//	otherwise
//		the righthand side is set for accelation balancing forces
void Train::initCouplerForces(int speed)
{
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		if (0<c->slack && c->slack<c->maxSlack) {
			c->cB= 10;
			c->cA= c->cC= c->cR= 0;
			continue;
		}
		c->cB= c->massInv;
		c->cA= -c->cB;
		if (c->next != NULL) {
			c->cC= -c->next->massInv;
			c->cB-= c->cC;
		} else {
			c->cC= 0;
		}
//		fprintf(stderr,"%p %f %f %f %p\n",c,c->cA,c->cB,c->cC,c->next);
	}
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		if (c->cB > 1)
			continue;
		if (speed) {
			c->cR= -c->speed;
			c->cR+= c->next->speed;
		} else {
			c->cR= -c->force*c->massInv;
			c->cR+= c->next->force*c->next->massInv;
		}
	}
}

//	solves coupler force equations (similar to tridiag.cc)
//	removes equations and recursively calls self if forces don't match
//	faces in contact
void Train::solveCouplerForces()
{
	if (firstCar == lastCar)
		return;
	float b= firstCar->cB;
	firstCar->cU= firstCar->cR/b;
	for (RailCarInst* c=firstCar->next; c!=lastCar; c=c->next) {
		c->cG= c->prev->cC/b;
		b= c->cB - c->cA*c->cG;
		c->cU= (c->cR-c->cA*c->prev->cU) / b;
	}
	for (RailCarInst* c=lastCar->prev->prev; c!=NULL; c=c->prev)
		c->cU-= c->next->cG*c->next->cU;
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next) {
		if (c->slack>=c->maxSlack || c->cU>=-1e-5)
			continue;
//		fprintf(stderr,"zeroed %s %f %f\n",
//		  c->def->name.c_str(),c->slack,c->cU);
#if 0
		for (RailCarInst* c1=firstCar; c1!=lastCar; c1=c1->next)
			fprintf(stderr,"%p %s %f %f %f %f %f\n",
			  c1,c1->def->name.c_str(),c1->cA,c1->cB,c1->cC,
			  c1->cR,c1->cU);
#endif
		if (c->cB >= 1)
			break;
		c->cB= 1;
		c->cA= c->cC= c->cR= 0;
		solveCouplerForces();
		break;
	}
	for (RailCarInst* c=lastCar->prev; c!=NULL; c=c->prev) {
		if (c->slack<=0 || c->cU<=1e-5)
			continue;
//		fprintf(stderr,"zeroed %s %f %f\n",
//		  c->def->name.c_str(),c->slack,c->cU);
#if 0
		for (RailCarInst* c1=firstCar; c1!=lastCar; c1=c1->next)
			fprintf(stderr,"%p %s %f %f %f %f %f\n",
			  c1,c1->def->name.c_str(),c1->cA,c1->cB,c1->cC,
			  c1->cR,c1->cU);
#endif
		if (c->cB >= 1)
			break;
		c->cB= 1;
		c->cA= c->cC= c->cR= 0;
		solveCouplerForces();
		break;
	}
#if 0
	for (RailCarInst* c=firstCar; c!=lastCar; c=c->next)
		fprintf(stderr,"%p %s %f %f %f %f %f\n",
		  c,c->def->name.c_str(),c->cA,c->cB,c->cC,c->cR,c->cU);
#endif
}

//	calculates length of train
//	TODO: why doesn't this include coupler slack
float Train::getLength()
{
	float l= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		l+= car->def->length;
	return l;
}

//	return estimated distance from front of train to center of coaches.
//	TODO: why doesn't this include coupler slack
float Train::getPassOffset()
{
	float l1= 0;
	float l2= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		l1+= car->def->length;
		if (car->def->engine)
			l2+= car->def->length;
	}
	if (l1 == l2)
		return l1/2;
	else
		return (l1+l2)/2;
}

//	turns on rendering of train's 3D models
void  Train::setModelsOn()
{
	bool hasEngine= false;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		car->setModelsOn();
		if (car->def->engine)
			hasEngine= true;
	}
	if (hasEngine) {
		firstCar->setHeadLight(2,firstCar->rev,true);
		lastCar->setHeadLight(3,lastCar->rev,true);
	}
}

//	turns off rendering of train's 3D models
void  Train::setModelsOff()
{
	bool hasEngine= false;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		car->setModelsOff();
		if (car->def->engine)
			hasEngine= true;
	}
	if (hasEngine) {
		firstCar->setHeadLight(2,firstCar->rev,false);
		lastCar->setHeadLight(3,lastCar->rev,false);
	}
}

void  Train::setHeadLight(bool on)
{
	firstCar->setHeadLight(2,firstCar->rev,on);
	lastCar->setHeadLight(3,lastCar->rev,on);
}

//	marks the track occupied by train as occupied
void  Train::setOccupied()
{
	Track::Location loc= endLocation;
	loc.edge->occupied++;
	loc.move(getLength(),0,1);
}

//	marks the track occupied by train as unoccupied
void  Train::clearOccupied()
{
	Track::Location loc= endLocation;
	loc.move(getLength(),0,-1);
	loc.edge->occupied--;
}

float Train::getThrottleInc()
{
	float min= 1;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		if (car->engine && min>car->engine->getThrottleInc())
			min= car->engine->getThrottleInc();
	return min;
}

float Train::getReverserInc()
{
	float min= 1;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		if (car->engine && min>car->engine->getReverserInc())
			min= car->engine->getReverserInc();
	return min;
}

void Train::bailOff()
{
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next)
		if (c->airBrake!=NULL &&
		  dynamic_cast<EngAirBrake*>(c->airBrake)!=NULL)
			  dynamic_cast<EngAirBrake*>(c->airBrake)->bailOff();
}

//	aligns switch from the trains current location to farv
//	Track->findSPT must have been called from the trains current location
void Train::alignSwitches(Track::Vertex* farv)
{
	Track::Vertex* v= farv;
	for (SigDistList::iterator i=signalList.begin(); i!=signalList.end();
	  ++i)
		i->first->trainDistance= 0;
	signalList.clear();
	Track::Edge* pe= NULL;
	for (;;) {
		if (v->type==Track::VT_SWITCH && pe!=NULL)
			((Track::SwVertex*)v)->throwSwitch(pe,false);
		Track::Edge* e= v->inEdge;
		if (e==NULL || e==pe)
			break;
		for (list<Signal*>::iterator i=e->signals.begin();
		  i!=e->signals.end(); ++i) {
			Signal* s= *i;
			for (int j=0; j<s->getNumTracks(); j++) {
				Track::Location& loc= s->getTrack(j);
				if (loc.edge != e)
					continue;
//				fprintf(stderr,"signal %p %p %f %f %d\n",
//				  s,e,e->v1->dist,e->v2->dist,loc.rev);
				if (loc.rev ? e->v1->dist<e->v2->dist :
				  e->v2->dist<e->v1->dist)
					continue;
//				fprintf(stderr,"signal %p %f\n",
//				  s,nextStopDist-loc.getDist());
				signalList.push_front(
				  make_pair(s,nextStopDist-loc.getDist()));
			}
		}
		if (v->type==Track::VT_SWITCH)
			((Track::SwVertex*)v)->throwSwitch(e,false);
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
	}
}

void Train::setPositionError(Track::SSEdge* ssEdge, float ssOffset, int rev)
{
	if (location.edge->ssEdge == ssEdge) {
		float d= ssOffset - (location.offset+location.edge->ssOffset);
		if (rev)
			d= -d;
		positionError= .5*positionError + .5*d;
//		fprintf(stderr," %s error %f\n",name.c_str(),positionError);
		return;
	}
	Track::Location loc;
	loc.set(ssEdge,ssOffset,rev);
	if (loc.edge == NULL)
		return;
//	fprintf(stderr,"%s %f %f %f\n",
//	  name.c_str(),loc.offset,loc.edge->ssOffset,loc.edge->length);
	float d= location.dDistance(&loc);
//	fprintf(stderr," diff ssedge %f\n",d);
	if (-100<d && d<100) {
		positionError= .5*positionError + .5*d;
//		fprintf(stderr," %s error %f\n",name.c_str(),positionError);
	} else {
		clearOccupied();
		location= loc;
		float x= 0;
		for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
			car->setLocation(x-car->def->length/2,&location);
			x-= car->def->length;
		}
		endLocation= location;
		endLocation.move(x,1,0);
		setOccupied();
//		fprintf(stderr," replace %s %f\n",name.c_str(),d);
	}
}

void Train::positionCars()
{
	float len= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next)
		len+= car->def->length;
	endLocation= location;
	endLocation.move(-len,1,0);
	float x= 0;
	for (RailCarInst* car=firstCar; car!=NULL; car=car->next) {
		car->setLocation(x-car->def->length/2,&location);
		x-= car->def->length;
	}
}

void Train::findCouplers(osg::Vec3d& pb, osg::Vec3d& pf,
  osg::Vec3d& pc, osg::Vec3d& pr)
{
	Track::Location loc= location;
	WLocation wloc;
	loc.getWLocation(&wloc);
	vector<WLocation> wlocs;
	wlocs.push_back(wloc);
	for (RailCarInst* c=firstCar; c!=NULL; c=c->next) {
		loc.move(-c->def->length,0,0);
		loc.getWLocation(&wloc);
		wlocs.push_back(wloc);
	}
	float bestd= 1e20;
	float besti= 0;
	for (int i=0; i<wlocs.size(); i++) {
		float dx= pb[0]-wlocs[i].coord[0];
		float dy= pb[1]-wlocs[i].coord[1];
		float dz= pb[2]-wlocs[i].coord[2];
		float d= dx*dx + dy*dy + dz*dz;
		if (d < bestd) {
			bestd= d;
			besti= i;
		}
	}
	pf= wlocs[besti>0?besti-1:0].coord;
	pc= wlocs[besti].coord;
	pr= wlocs[besti<wlocs.size()-1?besti+1:wlocs.size()-1].coord;
}

float Train::coupleDistance(bool behind)
{
	if (behind) {
		float mind= -endLocation.maxDistance(true,20)+.5;
		for (TrainList::iterator i=trainList.begin();
		  i!=trainList.end(); ++i) {
			Train* t= *i;
			if (t == this)
				continue;
			float d= endLocation.dDistance(&t->location);
			if (d<0 && mind < d-.5)
				mind= d-.5;
			d= endLocation.dDistance(&t->endLocation);
			if (d<0 && mind < d-.5)
				mind= d-.5;
		}
		return mind;
	} else {
		float mind= location.maxDistance(false,20)-.5;
		for (TrainList::iterator i=trainList.begin();
		  i!=trainList.end(); ++i) {
			Train* t= *i;
			if (t == this)
				continue;
			float d= location.dDistance(&t->location);
			if (d>0 && d<1e10 && mind > d+.5)
				mind= d+.5;
			d= location.dDistance(&t->endLocation);
			if (d>0 && d<1e10 && mind > d+.5)
				mind= d+.5;
		}
		return mind;
	}
}

void selectWaybills(int min, int max, string& destination)
{
	vector<RailCarInst*> cars;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		for (RailCarInst* c=t->firstCar; c!=NULL; c=c->next)
			if (c->waybill!=NULL &&
			  c->waybill->destination==destination)
				cars.push_back(c);
	}
	if (max > min)
		min+= (int)((max-min)*drand48());
	if (min >= cars.size())
		return;
	int n= cars.size();
	for (int i=0; i<min; i++) {
		int j= ((int)(100*n*drand48()))/100;
		n--;
		for (int k=0; k<cars.size(); k++) {
			if (cars[k] == NULL)
				continue;
			if (j == 0) {
				cars[k]= NULL;
				break;
			}
			j--;
		}
	}
	for (int k=0; k<cars.size(); k++)
		if (cars[k] != NULL)
			cars[k]->waybill= NULL;
}

void Train::stopAtSwitch(Track::SwVertex* sw)
{
	bool facing;
	float d;
	if (speed<0 && sw->occupied)
		d= location.vDistance(sw,true,&facing);
	else if (speed < 0)
		d= endLocation.vDistance(sw,true,&facing);
	else if (sw->occupied)
		d= endLocation.vDistance(sw,false,&facing);
	else
		d= location.vDistance(sw,false,&facing);
	if (d < 0)
		return;
	if (sw->occupied && facing)
		d+= 10;
	else if (facing)
		d-= .5;
	else if (sw->occupied)
		d+= .5;
	else if (speed < 0)
		d= location.vDistance(sw,true,&facing)+.5;
	else
		d= endLocation.vDistance(sw,false,&facing)+.5;
	float mind= .5*speed*speed/maxDecel;
	if (d < mind)
		d= mind;
	if (speed<0 && nextStopDist<-d)
		nextStopDist= -d;
	if (speed>0 && nextStopDist>d)
		nextStopDist= d;
	fprintf(stderr,"stopAtSwitch %f %f %d %d\n",
	  nextStopDist,speed,sw->occupied,facing);
}
