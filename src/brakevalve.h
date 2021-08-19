//	Classes for modeling automatic brake valves
//
//Copyright 2018 Doug Jones
#ifndef BRAKEVALVE_H
#define BRAKEVALVE_H

#include <string>
#include <vector>
#include <map>

#include "airtank.h"

struct BrakeValve {
	struct Passage {
		int tank1;
		int tank2;
		float area;
		float maxPressure2;
		bool oneway;
	};
	struct State {
		std::string name;
		std::string nextNameUp;
		std::string nextNameDown;
		int index;
		float upThreshold;
		float downThreshold;
		State* nextStateUp;
		State* nextStateDown;
		std::vector<Passage*> passages;
		State() {
			upThreshold= 100*AirTank::PSI2PA;
			downThreshold= -100*AirTank::PSI2PA;
			nextStateUp= this;
			nextStateDown= this;
		};
	};
	struct Piston {
		int upTank;
		int downTank;
		std::vector<State*> states;
	};
	struct Tank {
		std::string name;
		int index;
		float volume;
		float pipeLength;
		Tank(std::string nm) {
			name= nm;
			volume= 0;
			pipeLength= 0;
		}
	};
	struct RetainerSetting {
		std::string name;
		float area;
		float pressure;
		RetainerSetting(std::string n, float a, float p) {
			name= n;
			area= a;
			pressure= p;
		};
	};
	typedef std::map<std::string,Tank*> TankMap;
	typedef std::map<std::string,State*> StateMap;
	std::string name;
	std::vector<Tank*> tanks;
//	std::vector<State*> states;
	std::vector<Piston*> pistons;
	std::vector<RetainerSetting*> retainerSettings;
	TankMap tankMap;
	StateMap stateMap;
	BrakeValve(std::string nm) {
		name= nm;
	};
	void addTank(std::string name, float vol);
	void addPipe(std::string name, float length);
	void addState(std::string name, std::string upName,
	  std::string downName, float upThreshold, float downThreshold);
	void addPiston(std::string t1, std::string t2);
	void addPassage(std::string state, std::string t1, std::string t2,
	  float area, bool oneway=false, float maxPressure2=0);
	void addRetainerSetting(std::string name, float area, float pressure);
	void matchStates();
	static BrakeValve* get(std::string name);
	int updateState(int state, std::vector<AirTank*>& tanks);
	void updatePressures(int state, float timeStep,
	  std::vector<AirTank*>& tanks, int retainerControl);
	int getTankIndex(std::string name) {
		TankMap::iterator i= tankMap.find(name);
		return i==tankMap.end() ? -1 : i->second->index;
	};
};

#endif
