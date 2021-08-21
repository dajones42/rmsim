//	class for user controled interlocking
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
#ifndef INTERLOCKING_H
#define INTERLOCKING_H

#include <stdio.h>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <osg/Vec3>

#include "track.h"
#include "signal.h"
#include "commandreader.h"

class Interlocking {
 public:
	enum LeverState { NORMAL=01, REVERSE=02 };
	enum SwitchState { NOTOCCUPIED, OCCUPIED, ROUTELOCK };
	typedef std::list<std::pair<Track::SwVertex*,int> > SwitchList;
	typedef std::list<std::pair<int,int> > ConditionList;
 private:
	struct RouteLock {
		int lockTime;
		std::list<Track::Edge*> edges;
	};
	typedef std::list<RouteLock*> RouteLockList;
	struct Interlock {
		int lever1;
		int states1;
		int lever2;
		int states2;
		ConditionList when;
		Interlock(int l1, int s1, int l2, int s2) {
			lever1= l1;
			states1= s1;
			lever2= l2;
			states2= s2;
		};
	};
	struct Lever {
		LeverState state;
		osg::Vec3f color;
		SwitchList switches;
		Signal* signal;
		int lockTime;
		RouteLock* routeLock;
		Lever() {
			state= NORMAL;
			signal= NULL;
			lockTime= 0;
			routeLock= NULL;
		};
	};
	std::vector<Lever> levers;
	std::list<Interlock> interlocks;
	RouteLockList routeLocks;
	std::map<Track::SwVertex*,int> switch2LeverMap;
	int lockDelay;
 public:
	Interlocking(int nLevers);
	int getNumLevers() { return levers.size(); };
	void addInterlock(int l1, int s1, int l2, int s2) {
		interlocks.push_back(Interlock(l1,s1,l2,s2));
	};
	void addCondition(int l, int s) {
		if (interlocks.size() > 0) {
			interlocks.rbegin()->when.push_back(
			  std::make_pair(l,s));
		}
	};
	LeverState getState(int lever) {
		return levers[lever].state;
	};
	void setColor(int lever, float r, float g, float b) {
		levers[lever].color= osg::Vec3f(r,g,b);
	};
	osg::Vec3f getColor(int lever) {
		return levers[lever].color;
	};
	void addSwitch(int lever, Track::SwVertex* sw, int rev) {
		levers[lever].switches.push_back(std::make_pair(sw,rev));
		sw->hasInterlocking= 1;
		switch2LeverMap[sw]= lever;
	};
	void setSignal(int lever, Signal* sig) {
		levers[lever].signal= sig;
	};
	Signal* getSignal(int lever) {
		return levers[lever].signal;
	};
	bool isLocked(int lever);
	bool toggleState(int lever, int timeS);
	bool setState(int lever, LeverState state, int timeS);
	SwitchState getSwitchOccupied(int lever);
	int getLockDurationS(int lever, int timeS) {
		if (levers[lever].lockTime == 0)
			return 0;
		int dt= lockDelay - (timeS-levers[lever].lockTime);
		return dt>0 ? dt : 0;
	};
	void addRouteLock(Signal* sig, int timeS, int sigLever);
	void updateRouteLocks(int timeS);
	void setRouteLock(Track::Vertex* v, RouteLock* rl);
	int countUnclearedSignals() {
		if (this == NULL) return 0;
		int n= 0;
		for (int i=0; i<levers.size(); i++)
			if (levers[i].signal && levers[i].state!=REVERSE &&
			  levers[i].signal->trainDistance>0)
				n++;
		return n;
	};
	bool hasLock(int sigLever, Track::SwVertex* sw);
	std::string image;
};

struct InterlockingParser: public CommandBlockHandler {
	Interlocking* interlocking;
	InterlockingParser() {
		interlocking= NULL;
	};
	virtual bool handleCommand(CommandReader& reader);
	virtual void handleBeginBlock(CommandReader& reader);
	virtual void handleEndBlock(CommandReader& reader);
	int getLeverNum(CommandReader& reader, int idx);
	int getLeverStates(CommandReader& reader, int idx);
};

extern Interlocking* interlocking;
extern osg::Node* interlockingModel;

#endif
