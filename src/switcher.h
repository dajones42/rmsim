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
