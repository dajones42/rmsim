//	timetable simulation and AI logic code
//	an event simulation with one class declared for each kind of event
//
//	There is usually one event for each AI train that is not moving.
//	The event will either start the train or schedule another event.
//	When a train stops an event is schedule to restart it.
//
//	This code can handle two different types of AI trains: AI trains that
//	have preset paths like MSTS and trains that need a path calculated
//	as needed.  The code that calculates paths doesn't always work well.
//	These AI trains cannot handle meets involving more than 2 trains
//	but there is nothing to prevent this from happening.
//	Problems also occur when a train doesn't arrive at the next station
//	when it thought it would.  The preset path code should handle
//	these issues and it can also handle a partial timetable.
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
#include "eventsim.h"
#include "changelog.h"
#include "switcher.h"

using namespace std;

struct AIEvent : public tt::Event<double> {
	AITrain* train;
 public:
	AIEvent(double time, AITrain* train) :
	  tt::Event<double>(time) {
		this->train= train;
		train->event= this;
	};
	void handle(tt::EventSim<double>* sim);
	virtual void handleAI(tt::EventSim<double>* sim)=0;
};

void AIEvent::handle(tt::EventSim<double>* sim)
{
	if (train->event == this) {
		train->message= "";
		handleAI(sim);
	}
}

class CreateTrain : public AIEvent {
	int row;
 public:
	CreateTrain(double time, AITrain* train, int row) :
	  AIEvent(time,train) {
		this->row= row;
	};
	void handleAI(tt::EventSim<double>* sim);
};

class PathStart : public AIEvent {
	int row;
	bool checkLastRow;
 public:
	PathStart(double time, AITrain* train, int row, bool checkLast) :
	  AIEvent(time,train) {
		this->row= row;
		checkLastRow= checkLast;
	};
	void handleAI(tt::EventSim<double>* sim);
};

class Departure : public AIEvent {
	int row;
 public:
	Departure(double time, AITrain* train, int row) :
	  AIEvent(time,train) {
		this->row= row;
	};
	void handleAI(tt::EventSim<double>* sim);
};

class Stopped : public AIEvent {
 public:
	Stopped(double time, AITrain* train) :
	  AIEvent(time,train) {
	};
	void handleAI(tt::EventSim<double>* sim);
};

class TakeSiding : public AIEvent {
	int row;
 public:
	TakeSiding(double time, AITrain* train, int row) :
	  AIEvent(time,train) {
		this->row= row;
	};
	void handleAI(tt::EventSim<double>* sim);
};

class LeaveSiding : public AIEvent {
	int row;
 public:
	LeaveSiding(double time, AITrain* train, int row) :
	  AIEvent(time,train) {
		this->row= row;
	};
	void handleAI(tt::EventSim<double>* sim);
};

class LocalSwitching : public AIEvent {
	int row;
 public:
	LocalSwitching(double time, AITrain* train, int row) :
	  AIEvent(time,train) {
		this->row= row;
	};
	void handleAI(tt::EventSim<double>* sim);
};

//	calculates estimated time when train will arrival at station nextRow
double calcETA(double time, AITrain* train, int row, int nextRow)
{
	double eta= time;
	if (nextRow >= 0) {
		eta+= train->getSchedAr(nextRow)-train->getSchedLv(row);
		double eta1= time + timeTable->distance(row,nextRow)*5280/3.281/
		  train->consist->targetSpeed;
		fprintf(stderr,"eta %f %f %d %d %f\n",eta,eta1,row,nextRow,
		  timeTable->distance(row,nextRow));
		if (eta < eta1)
			eta= eta1;
	}
	return eta;
}

void promptForBlock(AITrain* train)
{
	extern bool commandMode;
	extern std::string command;
	if (!commandMode) {
		command= string("block for|")+train->getName();
		commandMode= true;
	}
}

//	creates a new train at the start of its run
//	doesn't create the train if it would wait before leaving
//	this is done to prevent new train from blocking an incoming train
void CreateTrain::handleAI(tt::EventSim<double>* sim)
{
	fprintf(stderr,"create %s at %f\n",train->getName().c_str(),time);
	int t= time+train->getWait(row);
	if (t < train->getSchedLv(row))
		t= train->getSchedLv(row);
	if (train->path != NULL) {
		train->moveAuth= ((TTOSim*)sim)->dispatcher.
		  requestAuth(train->consist);
		fprintf(stderr,"create auth %f %d %f\n",
		  train->moveAuth.distance,train->moveAuth.waitTime,
		  train->moveAuth.updateDistance);
		if (train->moveAuth.distance == 0)
			sim->schedule(new CreateTrain(
			  time+train->moveAuth.waitTime,train,row));
	} else {
		int nextRow= train->getNextRow(1);
		double eta= calcETA(t,train,row,nextRow);
		string reason;
		if (train->mustWait(t,train->getRow(),nextRow,eta,&reason)) {
			fprintf(stderr,"would wait for %s\n",reason.c_str());
			sim->schedule(new CreateTrain(time+60,train,row));
			return;
		}
	}
	if (!train->startVisible) {
		trainList.push_back(train->consist);
		listener.addTrain(train->consist);
		train->consist->setModelsOn();
		train->consist->setOccupied();
	}
	train->consist->stop();
	train->setArrival(row,time);
	train->takeSiding= 0;
	if (train->getRow(row)->getPromptForBlock())
		promptForBlock(train);
	if (train->path != NULL)
		sim->schedule(new PathStart(t,train,row,false));
	else
		sim->schedule(new Departure(t,train,row));
	ChangeLog::instance()->addCreate(train->consist);
}

void checkNextStop(AITrain* train, int nextRow)
{
	if (nextRow < 0)
		return;
	Station* s= (Station*) timeTable->getRow(nextRow);
	if (s->locations.size() == 0)
		return;
	float d= 0;
	int i= 0;
	int n= s->locations.size();
	if (s->getNumTracks()>1 && train->getReadDown() && s->nDownLocations>0)
		n= s->nDownLocations;
	else if (s->getNumTracks()>1 && !train->getReadDown() &&
	  s->nDownLocations<n)
		i= s->nDownLocations;
	fprintf(stderr," %d %d %d\n",
	  i,n,s->getNumTracks());
	int m= 0;
	for (; i<n; i++) {
		double dist= s->locations[i].getDist();
		fprintf(stderr," %d %f\n",i,dist);
		if (dist<0 || dist>1e10)
			continue;
		d+= dist;
		m++;
	}
	if (m == 0) {
		for (i=0; i<s->locations.size(); i++) {
			double dist= s->locations[i].getDist();
			fprintf(stderr," %d %f\n",i,dist);
			if (dist<0 || dist>1e10)
				continue;
			d+= dist;
			m++;
		}
	}
	if (m > 0)
		d/= m;
	d+= train->consist->getPassOffset();
	if (d>0 && d<train->consist->nextStopDist) {
		fprintf(stderr,"nextstopdist %f\n",d);
		train->consist->nextStopDist= d;
		train->moveAuth.waitTime= train->getWait(nextRow);
		train->moveAuth.updateDistance= 0;
	}
	if (train->moveAuth.farVertex)
		train->findSignals(train->moveAuth.farVertex);
}

//	handles restart of a train that has a define path
//	event is reschuled if train must wait
void PathStart::handleAI(tt::EventSim<double>* sim)
{
	int nextRow= train->getNextRow(0);
	fprintf(stderr,"PathStart %s at %f %f %d\n",
	  train->getName().c_str(),time,
	  train->moveAuth.distance,nextRow);
	if (checkLastRow && nextRow<0) {
		train->recordOnSheet(row,time,true);
		((TTOSim*)sim)->dispatcher.release(train->consist);
		if (train->getNextTrain() == NULL) {
			ChangeLog::instance()->addDestroy(train->consist);
			if (!train->endVisible) {
				trainList.remove(train->consist);
				listener.removeTrain(train->consist);
				train->consist->setModelsOff();
				train->consist->clearOccupied();
			} else {
				train->consist->modelCouplerSlack= 1;
			}
			train->consist= NULL;
		}
		return;
	}
	if (train->moveAuth.nextNode != NULL) {
		fprintf(stderr,"nextNode type %d\n",
		  train->moveAuth.nextNode->type);
	}
	train->moveAuth= ((TTOSim*)sim)->dispatcher.
	  requestAuth(train->consist);
	fprintf(stderr,"start auth %f %d %f\n",
	  train->moveAuth.distance,train->moveAuth.waitTime,
	  train->moveAuth.updateDistance);
	if (train->moveAuth.distance == 0) {
		fprintf(stderr,"no auth\n");
		if (train->moveAuth.waitTime == 0) {
			if (train->getNextTrain() == NULL) {
				((TTOSim*)sim)->dispatcher.
				  release(train->consist);
				ChangeLog::instance()->addDestroy(
				  train->consist);
				if (!train->endVisible) {
					trainList.remove(train->consist);
					listener.removeTrain(train->consist);
					train->consist->setModelsOff();
					train->consist->clearOccupied();
				}
				train->consist= NULL;
			} else {
				AITrain* t1= (AITrain*) train->getNextTrain();
				t1->consist= train->consist;
				train->consist= NULL;
				t1->moveAuth= ((TTOSim*)sim)->dispatcher.
				  requestAuth(t1->consist);
				if (train->getReadDown() != t1->getReadDown())
					t1->consist->reverse();
				t1->targetSpeed= t1->consist->targetSpeed;
				int r= t1->getCurrentRow();
				if (r >= 0) {
					t1->setArrival(r,time);
					int t= time+t1->getWait(r);
					if (t < t1->getSchedLv(r))
						t= t1->getSchedLv(r);
					t1->takeSiding= 0;
					sim->schedule(
					  new PathStart(t,t1,r,false));
				}
			}
			return;
		}
		sim->schedule(new PathStart(
		  time+train->moveAuth.waitTime,train,row,false));
		return;
	} else {
		int nextRow= train->getNextRow(0);
		train->consist->nextStopDist= train->moveAuth.distance;
		train->consist->nextStopTime= 0;
		((TTOSim*)sim)->movingTrains.insert(train);
		train->consist->moving= 5;
		if (row >= 0)
			train->recordOnSheet(row,time,true);
		checkNextStop(train,nextRow);
	}
}

//	handles departure of a train from a station
//	event is reschuled if train must wait
void Departure::handleAI(tt::EventSim<double>* sim)
{
	fprintf(stderr,"departure %s at %f\n",train->getName().c_str(),time);
	int nextRow= train->getNextRow(1);
	double eta= calcETA(time,train,row,nextRow);
	string reason;
	if (train->mustWait(time,train->getRow(),nextRow,eta,&reason)) {
		train->message= string("waiting for ")+reason;
		fprintf(stderr,"waiting for %s\n",reason.c_str());
		sim->schedule(new Departure(time+60,train,row));
		return;
	}
	nextRow= train->getNextRow(0);
	if (nextRow < 0) {
		train->recordOnSheet(row,time,true);
		if (train->getNextTrain() == NULL) {
			ChangeLog::instance()->addDestroy(train->consist);
			if (!train->endVisible) {
				trainList.remove(train->consist);
				listener.removeTrain(train->consist);
				train->consist->setModelsOff();
				train->consist->clearOccupied();
			}
			train->consist= NULL;
		} else {
			AITrain* t1= (AITrain*) train->getNextTrain();
			t1->consist= train->consist;
			train->consist= NULL;
			if (train->getReadDown() != t1->getReadDown()) {
				t1->consist->setModelsOff();
				t1->consist->reverse();
				t1->consist->setModelsOn();
			}
			t1->targetSpeed= t1->consist->targetSpeed;
			int r= t1->getCurrentRow();
			if (r >= 0) {
				t1->setArrival(r,time);
				int t= time+t1->getWait(r);
				if (t < t1->getSchedLv(r))
					t= t1->getSchedLv(r);
				t1->takeSiding= 0;
				sim->schedule(new Departure(t,t1,r));
			}
		}
		return;
	}
	if (!train->hasBlock(train->getRow(),nextRow)) {
		promptForBlock(train);
		train->message= "waiting for block";
		fprintf(stderr,"waiting for block\n");
		sim->schedule(new Departure(time+60,train,row));
		return;
	}
	float d= train->findNextStop(nextRow,1);
	if (d > 0) {
		train->message= "** track occupied";
		fprintf(stderr,"track occupied %f\n",d);
		sim->schedule(new Departure(time+60,train,row));
		return;
	}
	if (train->consist->nextStopDist == 0) {
		train->message= "** next stop not found";
		fprintf(stderr,"next stop not found %f\n");
		sim->schedule(new Departure(time+10,train,row));
		return;
	}
	fprintf(stderr,"nextstopdist %f\n",train->consist->nextStopDist);
	train->recordOnSheet(row,time,true);
	train->consist->nextStopTime= train->getSchedAr(nextRow)-time;
	if (train->getWait(nextRow) == 0)
		train->approachTest= 1;
	if (train->consist->nextStopDist != 0) {
		((TTOSim*)sim)->movingTrains.insert(train);
		train->consist->moving= 5;
		train->consist->speed= 1e-10;;
		if (train->takeSiding == 2) {
			float d= train->consist->nextStopDist;
			if (train->findNextStop(row,2) < 0) {
				train->takeSiding= 0;
				train->consist->nextStopDist= d;
			} else {
				train->takeSiding= 3;
				train->consist->nextStopTime= 0;
			}
		} else {
			train->takeSiding= 0;
		}
	}
}

//	event scheduled when train stop is detected
//	determines reason and schedules another event
void Stopped::handleAI(tt::EventSim<double>* sim)
{
	fprintf(stderr,"stopped %s at %f\n",train->getName().c_str(),time);
	((TTOSim*)sim)->movingTrains.erase(train);
	int row= train->getNextRow(0);
	if (train->takeSiding == 1) {
		sim->schedule(new TakeSiding(time+30,train,row));
	} else if (train->takeSiding == 3) {
		sim->schedule(new LeaveSiding(time+10,train,row));
		if (train->sidingSwitch != NULL) {
			Track::SwVertex* sw= train->sidingSwitch;
			sw->throwSwitch(sw->swEdges[sw->mainEdge],false);
			train->sidingSwitch= NULL;
		}
	} else if (row>=0 && train->testArrival(row)) {
		train->setArrival(row,time);
		double t= time + train->getWait(row);
		if (t < train->getSchedLv(row))
			t= train->getSchedLv(row);
		if (train->getWait(row) == 999)
			sim->schedule(new LocalSwitching(time,train,row));
		else if (train->path != NULL)
			sim->schedule(new PathStart(t,train,row,true));
		else
			sim->schedule(new Departure(t,train,row));
		if (train->takeSiding==2 && train->sidingSwitch!=NULL) {
			Track::SwVertex* sw= train->sidingSwitch;
			sw->throwSwitch(sw->swEdges[sw->mainEdge],false);
			train->sidingSwitch= NULL;
		}
	} else if (train->path != NULL) {
		sim->schedule(new
		  PathStart(time+train->moveAuth.waitTime,train,-1,false));
	} else {
		fprintf(stderr,"%s needs help\n",train->getName().c_str());
		float d= train->findNextStop(row,1);
		if (d > 0) {
			fprintf(stderr,"track occupied %f\n",d);
			sim->schedule(new Stopped(time+300,train));
			return;
		}
		fprintf(stderr,"nextstopdist %f\n",
		  train->consist->nextStopDist);
		train->consist->nextStopTime= train->getSchedAr(row)-time;
		if (train->consist->nextStopDist != 0) {
			((TTOSim*)sim)->movingTrains.insert(train);
			train->consist->moving= 5;
		}
	}
}

//	event for train restarting after stopping to take siding
void TakeSiding::handleAI(tt::EventSim<double>* sim)
{
	fprintf(stderr,"takesiding %s at %f\n",train->getName().c_str(),time);
	Track::SwVertex* sw= NULL;
	train->takeSiding= 2;
	train->approachTest= 0;
	train->consist->nextStopDist= 0;
	Track* track= train->consist->location.edge->track;
	track->findSPT(train->consist->location,true);
	Station* s= (Station*) timeTable->getRow(row);
	float sd= 0;
	for (int i=0; i<s->sidingSwitches.size(); i++) {
		Track::Vertex* v= s->sidingSwitches[i];
		if (sw==NULL || sw->dist<v->dist)
			sw= (Track::SwVertex*) v;
		sd+= v->dist;
		fprintf(stderr," %d %f %d\n",i,v->dist,v->occupied);
	}
	sd/= s->sidingSwitches.size();
	float cl2= train->consist->getLength()/2;
	float sl2= s->longestSiding()/3.281/2;
	if (train->getClass()<3 && s->locations.size()>0) {
		float d= 0;
		int m= 0;
		for (int i=0; i<s->locations.size(); i++) {
			float dist= s->locations[i].getDist();
			if (dist>=0 && dist<1e10) {
				d+= dist;
				m++;
			}
		}
		if (m > 0)
			d/= m;
		fprintf(stderr,"%f %f %f %f\n",d,sd,sl2,cl2);
		if (d<sd+sl2 && d>sd+sl2-cl2)
			sd= sd+sl2-cl2;
		else if (d>sd-sl2 && d<sd-sl2+cl2)
			sd= sd-sl2+cl2;
	}
	train->consist->nextStopDist= sd+cl2;
	Track::Edge* e= sw->swEdges[1-sw->mainEdge];
	Track::Vertex* v= e->v1==sw ? e->v2 : e->v1;
	float d= train->checkOccupied(v);
	if (d > 0) {
		fprintf(stderr,"track occupied %f\n",d);
		train->consist->nextStopDist= 0;
		sim->schedule(new TakeSiding(time+60,train,row));
		return;
	}
	train->alignSwitches(v);
	fprintf(stderr,"nextStopDist=%f\n",train->consist->nextStopDist);
	((TTOSim*)sim)->movingTrains.insert(train);
	train->consist->moving= 5;
}

//	event for restarting train after leaving siding
void LeaveSiding::handleAI(tt::EventSim<double>* sim)
{
	fprintf(stderr,"leavesiding %s at %f\n",train->getName().c_str(),time);
	train->takeSiding= 0;
	int nextRow= train->getNextRow(0);
	train->findNextStop(nextRow,1);
	if (train->consist->nextStopDist != 0) {
		train->consist->nextStopTime= train->getSchedAr(nextRow)-time;
		((TTOSim*)sim)->movingTrains.insert(train);
		train->consist->moving= 5;
	}
}

//	event for restarting train after leaving siding
void LocalSwitching::handleAI(tt::EventSim<double>* sim)
{
//	fprintf(stderr,"localswitching %s at %f\n",
//	  train->getName().c_str(),time);
	if (train->switcher == NULL) {
		train->switcher= new Switcher(train->consist);
		train->consist->modelCouplerSlack= 1;
		train->consist->signalList.clear();
	}
	train->switcher->update();
	if (train->switcher->train) {
		sim->schedule(new LocalSwitching(time+10,train,row));
	} else {
		delete train->switcher;
		train->switcher= NULL;
		fprintf(stderr,"locel switching done\n");
		double t= time;
		if (t < train->getSchedLv(row))
			t= train->getSchedLv(row);
		if (train->path != NULL)
			sim->schedule(new PathStart(t,train,row,true));
		else
			sim->schedule(new Departure(t,train,row));
	}
}

//	examines track to match track locations with timetable stations
//	finds siding switches in the process
void TTOSim::findStations(Track* track)
{
	for (int i=0; i<timeTable->getNumRows(); i++) {
		Station* si= (Station*) timeTable->getRow(i);
		Track::Location loc;
		for (int j=0; j<2; j++) {
			if (track->findLocation(si->getName(),j,&loc) == 0)
				break;
			si->locations.push_back(loc);
		}
		si->nDownLocations= 0;
		si->length= 0;
		for (int j=0; j<si->getNumAltNames(); j++) {
			for (int k=0; k<2; k++) {
				if (track->findLocation(si->getAltName(j),
				  k,&loc) == 0)
					break;
				si->locations.push_back(loc);
			}
			if (si->getNumTracks()>1 && j<si->getNumTracks()/2)
				si->nDownLocations= si->locations.size();
		}
	}
	for (int i=0; i<timeTable->getNumRows(); i++) {
		Station* si= (Station*) timeTable->getRow(i);
		if (si->locations.size() == 0)
			continue;
		fprintf(stderr,"spt from %s\n",si->getName().c_str());
		track->findSPT(si->locations[0],true);
		for (int j=1; j<si->locations.size(); j++) {
			float d= si->locations[j].getDist();
			if (d<1e5 && si->length<d)
				si->length= d;
		}
		for (int j=0; j<timeTable->getNumRows(); j++) {
			if (i == j)
				continue;
			Station* sj= (Station*) timeTable->getRow(j);
			float len= sj->longestSiding()/3.281;
			if (sj->locations.size()==0 ||
			  sj->sidingSwitches.size()!=0 || len==0)
				continue;
			WLocation wlocj;
			sj->locations[0].getWLocation(&wlocj);
			Track::Vertex* v= track->findSiding(wlocj.coord,len);
			if (v == NULL)
				continue;
			Track::Vertex* p= track->findSidingParent(v);
			sj->sidingSwitches.push_back(v);
			sj->sidingSwitches.push_back(p);
			fprintf(stderr," to %s %d %.1f %.1f %.1f\n",
			  sj->getName().c_str(),j,v->dist-p->dist,len,
			  .5*(v->dist+p->dist)*3.281/5280);
		}
	}
	for (int i=1; i<timeTable->getNumRows()-1; i++) {
		Station* si= (Station*) timeTable->getRow(i);
		if (si->locations.size() > 0)
			continue;
		int j= i-1;
		Station* sj= (Station*) timeTable->getRow(j);
		while (j>0 && sj->locations.size()==0) {
			j--;
			sj= (Station*) timeTable->getRow(j);
		}
		if (sj->locations.size() == 0)
			continue;
		int k= i+1;
		Station* sk= (Station*) timeTable->getRow(k);
		while (k<timeTable->getNumRows()-1 && sk->locations.size()==0) {
			k++;
			sk= (Station*) timeTable->getRow(k);
		}
		if (sk->locations.size() == 0)
			continue;
		float d= (sj->getDistance()-si->getDistance())*5280/3.281;
		if (d < 0)
			d= -d;
		for (int i1=0; i1<si->getNumTracks(); i1++) {
			fprintf(stderr,"spt for %s track %d %f\n",
			  si->getName().c_str(),i1+1,d);
			if (i1>0 && sj->getNumTracks()>1)
				track->findSPT(sj->locations[
				  sj->nDownLocations],true);
			else
				track->findSPT(sj->locations[0],true);
			Track::Vertex* v= (i1>0 && sk->getNumTracks()>1) ?
			  sk->locations[sk->nDownLocations].edge->v1 :
			  sk->locations[0].edge->v1;
			fprintf(stderr,"%f\n",v->dist);
			Track::Edge* e= v->inEdge;
			while (e != NULL) {
				if (v == e->v1)
					v= e->v2;
				else
					v= e->v1;
				if (v->dist <= d)
					break;
				e= v->inEdge;
			}
			if (e == NULL)
				return;
			Track::Location loc;
			loc.edge= e;
			if (v == e->v1) {
				loc.offset= d-v->dist;
				loc.rev= 0;
			} else {
				loc.offset= e->length - (d-v->dist);
				loc.rev= 1;
			}
			fprintf(stderr,"found %p %f\n",e,loc.getDist());
			si->locations.push_back(loc);
		}
		if (si->getNumTracks() > 1)
			si->nDownLocations= 1;
	}
	for (int i=0; i<timeTable->getNumRows(); i++) {
		Station* si= (Station*) timeTable->getRow(i);
		fprintf(stderr,"%s %d %d %d %f\n",
		  si->getName().c_str(),(int)si->locations.size(),
		  si->nDownLocations,si->getNumTracks(),si->length);
	}
}

//	initializes sim by scheduling createTrain events for each train
//	returns the starting time of earliest train
double TTOSim::init(bool isClient)
{
	dispatcher.ignoreOtherTrains= timeTable->getIgnoreOther();
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i)
		findStations(i->second);
	double start= 1e10;
	for (int i=0; i<timeTable->getNumTrains(); i++) {
		AITrain* t= (AITrain*) timeTable->getTrain(i);
		t->init(0);
		if (t->getPrevTrain() != NULL)
			continue;
		int r= t->getCurrentRow();
		if (r < 0)
			continue;
		TrainMap::iterator j= trainMap.find(t->getName());
		if (j == trainMap.end()) {
			fprintf(stderr,"cannot find train %s\n",
			  t->getName().c_str());
			continue;
		}
		t->consist= j->second;
		t->targetSpeed= j->second->targetSpeed;
		fprintf(stderr,"schedule create %s %s\n",t->getName().c_str(),
		 j->second->name.c_str());
		if (!isClient) {
			schedule(new
			  CreateTrain(t->getSchedAr(r)+t->startDelay,t,r));
			if (t->path != NULL)
				dispatcher.registerPath(t->consist,t->path);
		}
		if (start > t->getSchedAr(r))
			start= t->getSchedAr(r);
		if (!t->startVisible) {
			trainList.remove(t->consist);
			listener.removeTrain(t->consist);
			t->consist->setModelsOff();
			t->consist->clearOccupied();
		}
	}
	if (start == 1e10)
		start= 0;
	return start;
}

//	converts a train from AI to user control
bool TTOSim::takeControlOfAI(Consist* consist)
{
	for (int i=0; i<timeTable->getNumTrains(); i++) {
		AITrain* t= (AITrain*) timeTable->getTrain(i);
		if (t->consist != consist)
			continue;
		t->event= NULL;
		t->consist= NULL;
		consist->nextStopDist= 0;
		movingTrains.erase(t);
		return true;
	}
	return false;
}

//	converts a train from user to AI control
bool TTOSim::convertToAI(Consist* consist)
{
	for (int i=0; i<timeTable->getNumTrains(); i++) {
		AITrain* t= (AITrain*) timeTable->getTrain(i);
		if (t->consist!=NULL ||  t->getName()!=consist->name)
			continue;
		int r= t->getCurrentRow();
		fprintf(stderr,"current row %d\n",r);
		if (r < 0)
			continue;
		t->consist= consist;
		consist->targetSpeed= t->targetSpeed;
		t->osDist= 0;
		consist->convertFromAirBrakes();
		if (t->getActualLv(r) < 0) {
			consist->stop();
			schedule(new Departure(t->getSchedLv(r),t,r));
			if (t->takeSiding != 0)
				t->takeSiding= 2;
			fprintf(stderr,"departure scheduled\n");
		} else {
			t->takeSiding= 0;
			int nextRow= t->getNextRow(0);
			fprintf(stderr,"next row %d\n",nextRow);
			if (nextRow >= 0)
				t->findNextStop(nextRow,1);
			if (consist->nextStopDist == 0)
				consist->stop();
			t->consist->nextStopTime=
			  t->getSchedAr(nextRow)-simTime;
			movingTrains.insert(t);
			t->consist->moving= 5;
		}
		return true;
	}
	return false;
}

//	records time user train passed station on train sheet 
bool TTOSim::osUserTrain(Consist* consist, double time)
{
	for (int i=0; i<timeTable->getNumTrains(); i++) {
		AITrain* t= (AITrain*) timeTable->getTrain(i);
		if (t->consist!=NULL ||  t->getName()!=consist->name)
			continue;
		int r= t->getCurrentRow();
		fprintf(stderr,"current row %d\n",r);
		if (r < 0)
			continue;
		if (t->getActualLv(r) < 0) {
			t->recordOnSheet(r,time,false);
			fprintf(stderr,"os %s left %s at %f\n",
			  t->getName().c_str(),
			  timeTable->getRow(r)->getName().c_str(),time);
		} else {
			r= t->getNextRow(0);
			if (r < 0)
				continue;
			t->setArrival(r,time);
			fprintf(stderr,"os %s arrived %s at %f\n",
			  t->getName().c_str(),
			  timeTable->getRow(r)->getName().c_str(),time);
		}
		return true;
	}
	return false;
}

//	processes events that should occur before specified time
//	checks moving trains for movement related events and then checks queue
void TTOSim::processEvents(double time)
{
	for (set<AITrain*>::iterator i= movingTrains.begin();
	  i!=movingTrains.end(); ++i) {
		AITrain* t= *i;
		if (t->consist->nextStopDist == 0) {
			schedule(new Stopped(time,t));
		} else if (t->approachTest && t->consist->nextStopDist<500) {
			t->approach(time);
		} else if (t->path!=NULL &&
		  t->consist->nextStopDist>0 &&
		  t->consist->nextStopDist<t->moveAuth.updateDistance) {
			t->moveAuth= dispatcher.requestAuth(t->consist);
			fprintf(stderr,"auth update %s %f %d %f\n",
			  t->getName().c_str(),t->moveAuth.distance,
			  t->moveAuth.waitTime,t->moveAuth.updateDistance);
			if (t->consist->nextStopDist < t->moveAuth.distance)
				t->consist->nextStopDist= t->moveAuth.distance;
			checkNextStop(t,t->getNextRow(0));
		} else if (t->osDist>0 && t->consist->nextStopDist<t->osDist) {
			t->osDist= 0;
			int row= t->getNextRow(0);
			if (timeTable->getRow(row)->getCallSign()!=
			  userOSCallSign) {
				t->setArrival(row,time);
				t->recordOnSheet(row,time,true);
				fprintf(stderr,"os %s at %f\n",
				  t->getName().c_str(),time);
			}
		}
	}
	tt::EventSim<double>::processEvents(time);
}

//	returns distance squared between location loc and nearest moving train
float TTOSim::movingTrainDist2(osg::Vec3d loc)
{
	float mind= 1e30;
	for (set<AITrain*>::iterator i= movingTrains.begin();
	  i!=movingTrains.end(); ++i) {
		AITrain* t= *i;
//		if (-.1<t->consist->speed && t->consist->speed<.1)
		if (t->consist->speed == 0)
			continue;
		WLocation wl;
		t->consist->location.getWLocation(&wl);
		float d= (loc-wl.coord).length2();
		if (mind > d)
			mind= d;
	}
	return mind;
}

tt::Train* TimeTable::addTrain(std::string name)
{
	tt::Train* t= (tt::Train*) new AITrain(name);
	tt::TimeTable::addTrain(t);
	return t;
};

tt::Station* TimeTable::addStation(std::string name)
{
	tt::Station* s= (tt::Station*) new Station(name);
	tt::TimeTable::addStation(s);
	return s;
};

//	finds a path to the next stop and updates nextStopDist
//	if siding>0 takes siding else holds main
//	normally returns zero
//	returns a positive distance if the track ahead is occupied
//	returns -1 if stop to take siding is not necessary because is
//	controlled by interlocking
float AITrain::findNextStop(int nextRow, int siding)
{
	Track* track= consist->location.edge->track;
	track->findSPT(consist->location,siding!=2);
	Station* s= (Station*) timeTable->getRow(nextRow);
	Track::Vertex* farv= NULL;
	if (siding && s->sidingSwitches.size() > 0) {
		approachTest= siding==1;
		consist->nextStopDist= 1e10;
		for (int i=0; i<s->sidingSwitches.size(); i++) {
			Track::Vertex* v= s->sidingSwitches[i];
			if (consist->nextStopDist > v->dist) {
				consist->nextStopDist= v->dist;
				farv= v;
			}
		}
		if (siding==2 && farv!=NULL) {
			Track::SwVertex* sw= (Track::SwVertex*) farv;
			if (sw->hasInterlocking ||
			  sw->swEdges[sw->mainEdge]==sw->edge2)
				return -1;
		}
		if (siding == 2) 
			consist->nextStopDist+= 1 + consist->getLength();
		else
			consist->nextStopDist-= 1;
		sidingSwitch= (Track::SwVertex*) farv;
		fprintf(stderr,"nextStopDist=%f approach\n",
		  consist->nextStopDist);
	} else if (s->locations.size() > 0) {
		approachTest= 0;
		consist->nextStopDist= 0;
		int i= 0;
		int n= s->locations.size();
		if (s->getNumTracks()>1 && readDown && s->nDownLocations>0)
			n= s->nDownLocations;
		else if (s->getNumTracks()>1 && !readDown &&
		  s->nDownLocations<n)
			i= s->nDownLocations;
		fprintf(stderr," %d %d %d\n",i,n,s->getNumTracks());
		int m= 0;
		for (; i<n; i++) {
			float dist= s->locations[i].getDist();
			if (dist<0 || dist>1e10)
				continue;
			fprintf(stderr," %d %f\n",i,dist);
			consist->nextStopDist+= dist;
			m++;
			Track::Edge* e= s->locations[i].edge;
			if (farv==NULL || farv->dist<e->v1->dist)
				farv= e->v1;
			if (farv->dist < e->v2->dist)
				farv= e->v2;
		}
		if (m > 0)
			consist->nextStopDist/= m;
		float cl2= consist->getLength()/2;
		if (s->sidingSwitches.size() > 0) {
			float sd= 0;
			float sl2= s->longestSiding()/3.281/2;
			for (int i=0; i<s->sidingSwitches.size(); i++)
				sd+= s->sidingSwitches[i]->dist;
			sd/= s->sidingSwitches.size();
			fprintf(stderr,"%f %f %f %f\n",
			  consist->nextStopDist,sd,sl2,cl2);
			if (consist->nextStopDist<sd+sl2 &&
			  consist->nextStopDist>sd+sl2-cl2)
				consist->nextStopDist= sd+sl2-cl2;
			if (consist->nextStopDist>sd-sl2 &&
			  consist->nextStopDist<sd-sl2+cl2)
				consist->nextStopDist= sd-sl2+cl2;
		}
		consist->nextStopDist+= cl2;
		fprintf(stderr,"nextStopDist=%f\n",
		  consist->nextStopDist);
	} else {
		approachTest= 0;
		consist->nextStopDist= 0;
		return 0;
	}
	if (farv == NULL) {
		consist->nextStopDist= 0;
		return 0;
	}
	float d= checkOccupied(farv);
	if (d > 0) {
		consist->nextStopDist= 0;
		return d;
	}
	alignSwitches(farv);
	if (consist->endLocation.getDist() < 0) {
		for (SigDistList::iterator i=consist->signalList.begin();
		  i!=consist->signalList.end(); ++i)
			i->second-= consist->getLength();
		consist->nextStopDist=
		  -consist->nextStopDist + consist->getLength();
		fprintf(stderr,"reverse\n");
	}
	return 0;
}

//	checks to see if stopped train has arrived at its next stop
bool AITrain::testArrival(int row)
{
	Track* track= consist->location.edge->track;
	track->findSPT(consist->location,true);
	Station* s= (Station*) timeTable->getRow(row);
	if (s->sidingSwitches.size() > 0) {
		float d= 0;
		for (int i=0; i<s->sidingSwitches.size(); i++) {
			Track::Vertex* v= s->sidingSwitches[i];
			if (d < v->dist)
				d= v->dist;
		}
		fprintf(stderr,"testArrival %f %f\n",d,s->longestSiding());
		if (d < s->longestSiding())
			return true;
		else if (takeSiding)
			return false;
	}
	float maxd= 0;
	int i= 0;
	int n= s->locations.size();
	if (s->getNumTracks()>1 && readDown && s->nDownLocations>0)
		n= s->nDownLocations;
	else if (s->getNumTracks()>1 && !readDown && s->nDownLocations>n)
		i= s->nDownLocations;
	for (; i<n; i++) {
		float d= s->locations[i].getDist();
		if (d<1e10 && maxd < d)
			maxd= d;
	}
	fprintf(stderr,"testArrival %f %f %f\n",maxd,s->length,
	  consist->getLength());
	return maxd<consist->getLength() || maxd<s->length;
}

//	checks to see if the track a train is about to start moving to is
//	occupied
float AITrain::checkOccupied(Track::Vertex* farv)
{
	Track::Vertex* v= farv;
	Track::Edge* pe= NULL;
	float oDist= 0;
	for (;;) {
		Track::Edge* e= v->inEdge;
		if (e==NULL || e==pe)
			break;
		if (pe!=NULL && pe->occupied && v->dist>0) {
			oDist= v->dist;
			fprintf(stderr,"occupied %p %f %f %d\n",
			  pe,pe->v1->dist,pe->v2->dist,pe->occupied);
		}
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
	}
	return oDist;
}

//	aligns switch for the path a train is about to take
void AITrain::alignSwitches(Track::Vertex* farv)
{
	Track::Vertex* v= farv;
	for (SigDistList::iterator i=consist->signalList.begin();
	  i!=consist->signalList.end(); ++i)
		i->first->trainDistance= 0;
	consist->signalList.clear();
	Track::Edge* pe= NULL;
	v->dist= -v->dist;
	for (;;) {
		if (v->type==Track::VT_SWITCH && pe!=NULL)
			((Track::SwVertex*)v)->throwSwitch(pe,false);
		Track::Edge* e= v->inEdge;
		if (e==NULL || e==pe || e==consist->location.edge)
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
//				  s,consist->nextStopDist-loc.getDist());
				consist->signalList.push_front(
				  make_pair(s,
				   consist->nextStopDist-loc.getDist()));
			}
		}
		v->dist= -v->dist;
		if (v->type==Track::VT_SWITCH)
			((Track::SwVertex*)v)->throwSwitch(e,false);
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
	}
	Track::Edge* e= consist->location.edge;
	for (list<Signal*>::iterator i=e->signals.begin();
	  i!=e->signals.end(); ++i) {
		Signal* s= *i;
		for (int j=0; j<s->getNumTracks(); j++) {
			Track::Location& loc= s->getTrack(j);
			if (loc.edge != e)
				continue;
//			fprintf(stderr,"start signal %p %p %f %f %d %d\n",
//			  s,e,e->v1->dist,e->v2->dist,loc.rev,
//			  consist->location.rev);
			float d= consist->location.dDistance(&loc);
			if (d<0 || consist->location.rev!=loc.rev)
				continue;
//			fprintf(stderr,"signal %p %f\n",
//			  s,consist->nextStopDist-d);
			consist->signalList.push_front(
			  make_pair(s,consist->nextStopDist-d));
		}
	}
}

//	find signals on the path a train is about to take
void AITrain::findSignals(Track::Vertex* farv)
{
//	fprintf(stderr,"findsignals %p %f\n",farv,farv->dist);
	Track::Vertex* v= farv;
	for (SigDistList::iterator i=consist->signalList.begin();
	  i!=consist->signalList.end(); ++i)
		i->first->trainDistance= 0;
	consist->signalList.clear();
	Track::Edge* pe= NULL;
	v->dist= -v->dist;
	for (;;) {
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
//				  s,consist->nextStopDist-loc.getDist());
				consist->signalList.push_front(
				  make_pair(s,
				   consist->nextStopDist-loc.getDist()));
			}
		}
		v->dist= -v->dist;
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
	}
	Track::Edge* e= consist->location.edge;
	for (list<Signal*>::iterator i=e->signals.begin();
	  i!=e->signals.end(); ++i) {
		Signal* s= *i;
		for (int j=0; j<s->getNumTracks(); j++) {
			Track::Location& loc= s->getTrack(j);
			if (loc.edge != e)
				continue;
//			fprintf(stderr,"start signal %p %p %f %f %d %d\n",
//			  s,e,e->v1->dist,e->v2->dist,loc.rev,
//			  consist->location.rev);
			float d= consist->location.dDistance(&loc);
			if (d<0 || consist->location.rev!=loc.rev)
				continue;
//			fprintf(stderr,"signal %p %f\n",
//			  s,consist->nextStopDist-d);
			consist->signalList.push_front(
			  make_pair(s,consist->nextStopDist-d));
		}
	}
}

//	test performed when a train approaches a station to see if
//	train needs to stop or if it can continue
void AITrain::approach(double time)
{
	if (consist->signalList.size() > 0) {
		SigDistList::iterator i=consist->signalList.begin();
		if (i->first->getState()!=Signal::CLEAR &&
		  i->first->trainDistance<consist->nextStopDist)
			return;
	}
	fprintf(stderr,"approach %s at %f %f\n",getName().c_str(),time/3600,
	  consist->nextStopDist);
	approachTest= 0;
	int row1= getNextRow(0);
	int row2= getNextRow(1,row1);
	if (time < getSchedLv(row1))
		time= getSchedLv(row1);
	float etaRow1= 1000/consist->targetSpeed;
	double eta= time + etaRow1;
	//eta+= getActualLv(row)-getSchedLv(row);
	if (row2 >= 0) {
		eta+= getSchedAr(row2)-getSchedLv(row1);
		double eta1= time + timeTable->distance(row1,row2)*5280/3.281/
		  consist->targetSpeed + etaRow1;
		fprintf(stderr,"eta %f %f %d %d %f\n",
		  eta/3600,eta1/3600,row2,row1,etaRow1);
		if (eta < eta1)
			eta= eta1;
	}
	takeSiding= 0;
	string reason;
	int meetAhead= mustWait(time,row1,row2,eta,&reason);
	if (meetAhead == 1) {
		fprintf(stderr,"take siding %s\n",reason.c_str());
		takeSiding= 1;
		Station* s= (Station*) timeTable->getRow(row1);
		int hasInterlocking= 0;
		for (int i=0; i<s->sidingSwitches.size(); i++) {
			Track::SwVertex* sw=
			  (Track::SwVertex*)s->sidingSwitches[i];
			hasInterlocking|= sw->hasInterlocking;
		}
		if (hasInterlocking) {
			fprintf(stderr,"has interlocking\n");
			Track* track= consist->location.edge->track;
			track->findSPT(consist->location,true);
			Track::Vertex* bv= NULL;
			for (int i=0; i<s->sidingSwitches.size(); i++) {
				Track::Vertex* v= s->sidingSwitches[i];
				fprintf(stderr," %p %f %d\n",
				  v,v->dist,
				  ((Track::SwVertex*)v)->hasInterlocking);
				if (bv==NULL || v->dist<bv->dist)
					bv= v;
			}
			Track::SwVertex* sw= (Track::SwVertex*)bv;
			fprintf(stderr," %p\n",sw);
			if (sw!=NULL && sw->hasInterlocking) {
				fprintf(stderr,"interlocking siding\n");
				takeSiding= 2;
				findNextStop(row1,0);
				consist->nextStopTime= getSchedAr(row1)-time;
			}
		}
	} else if (meetAhead==2 || getWait(row1)>0 || row2<0 ||
	  getSchedLv(row1)>time+etaRow1) {
		fprintf(stderr,"stop %d %f %s\n",getSchedLv(row1),
		  time+etaRow1,reason.c_str());
		findNextStop(row1,0);
		consist->nextStopTime= getSchedAr(row1)-time;
	} else if (findNextStop(row2,1) > 0 ||
	  consist->nextStopDist==0) {
		fprintf(stderr,"stop occupied ahead\n");
		findNextStop(row1,0);
		consist->nextStopTime= getSchedAr(row1)-time;
	} else {
		fprintf(stderr,"don't stop %d\n",meetAhead);
		consist->nextStopTime= getSchedAr(row2)-time;
		Station* s= (Station*) timeTable->getRow(row1);
		osDist= 0;
		int n= 0;
		for (int i=0; i<s->locations.size(); i++) {
			fprintf(stderr," %d %f\n",i,s->locations[i].getDist());
			float d= s->locations[i].getDist();
			if (d < 0)
				d= -d;
			if (d > 1e4)
				continue;
			osDist+= d;
			n++;
		}
		if (n > 0)
			osDist/= n;
		else
			osDist= 500;
		osDist+= consist->getLength();
		osDist= consist->nextStopDist - osDist;
		if (getWait(row2) == 0)
			approachTest= 1;
		fprintf(stderr,"osDist=%f %d\n",osDist,approachTest);
	}
}

//	records a train's time on the train sheet
void AITrain::recordOnSheet(int row, int time, bool autoOS)
{
	if (autoOS && timeTable->getRow(row)->getCallSign()==userOSCallSign)
		return;
	setDeparture(row,time);
	string msg= string("OS OS ")+getName();
	int a= getActualAr(row)/60;
	int d= time/60;
	if (a == d) {
		char buf[10];
		int h= d/60;
		if (h > 13)
			h-= 12;
		sprintf(buf,"%d%2.2d",h,d%60);
		msg+= " by ";
		msg+= buf;
	} else {
		char buf[10];
		int h= a/60;
		if (h > 13)
			h-= 12;
		sprintf(buf,"%d%2.2d",h,a%60);
		msg+= " a ";
		msg+= buf;
		h= d/60;
		if (h > 13)
			h-= 12;
		sprintf(buf,"%d%2.2d",h,d%60);
		msg+= " d ";
		msg+= buf;
	}
	msg+= " ";
	msg+= timeTable->getRow(row)->getCallSign();
	msg+= "  ";
	listener.playMorse(msg.c_str());
}
