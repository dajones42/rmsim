//	classes for timetable simulation and AI train control
//
//Copyright 2009 Doug Jones
#ifndef TTOSIM_H
#define TTOSIM_H

#include "eventsim.h"
#include "track.h"
#include "dispatcher.h"

//	this typedef used to handle two different Train structures
typedef Train Consist;

struct Switcher;

struct AIEvent;

struct AITrain : public tt::Train {
	Consist* consist;
	AIEvent* event;
	int approachTest;
	int takeSiding;
	float osDist;
	float targetSpeed;
	Track::SwVertex* sidingSwitch;
	MoveAuth moveAuth;
	Switcher* switcher;
	std::string message;
	AITrain(std::string name) : tt::Train(name) {
		consist= NULL;
		approachTest= 0;
		takeSiding= 0;
		osDist= 0;
		sidingSwitch= NULL;
		switcher= NULL;
	};
	void approach(double time);
	float findNextStop(int nextRow, int siding);
	float checkOccupied(Track::Vertex* v);
	void alignSwitches(Track::Vertex* v);
	void findSignals(Track::Vertex* v);
	bool testArrival(int row);
	void recordOnSheet(int row, int time, bool autoOS);
};

struct Station : public tt::Station {
	vector<Track::Location> locations;
	float length;
	int nDownLocations;
	vector<Track::Vertex*> sidingSwitches;
	Station(std::string name) : tt::Station(name) { };
};

struct TimeTable : public tt::TimeTable {
	virtual tt::Station* addStation(std::string name);
	virtual tt::Train* addTrain(std::string name);
};

struct TTOSim : public tt::EventSim<double> {
	Dispatcher dispatcher;
	set<AITrain*> movingTrains;
	double init(bool isClient);
	void findStations(Track* track);
	void processEvents(double time);
	float movingTrainDist2(osg::Vec3d loc);
	bool takeControlOfAI(Consist* train);
	bool convertToAI(Consist* train);
	bool osUserTrain(Consist* train, double time);
};

#endif
