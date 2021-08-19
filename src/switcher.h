#ifndef SWITCHER_H
#define SWITCHER_H

struct Switcher {
	Track* track;
	Train* train;
	RailCarInst* targetCar;
	Train* targetTrain;
	Track::Location destination;
	int moves;
	float targetSpeed;
	Switcher(Train* t) {
		moves= 0;
		train= t;
		track= t->location.edge->track;
		targetCar= NULL;
		targetSpeed= t->targetSpeed;
	};
	void update();
	void findSetoutCar();
	void findPickupCar();
	float uncoupledLength();
	void uncouplePower(bool rear);
	void uncoupleTarget(bool rear);
	float targetDistance();
	float checkCapacity(Train* testTrain, string& dest);
	float findCarDist(Track::Location& loc, RailCarInst* car, Train* train);
	bool isEngine(RailCarInst* car) {
		return car->engine!=NULL ||
		  car->waybill && car->waybill->priority>=200;
	};
	void findSPT(Track::Vertex* avoid, bool fix);
};
extern Switcher* autoSwitcher;

#endif
