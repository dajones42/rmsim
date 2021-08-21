//	code for user controled interlocking
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
#include "interlocking.h"
#include <stdexcept>
#include <stdlib.h>

//	creates an interlocking with the specified number of levers
Interlocking::Interlocking(int nLevers)
{
	levers.resize(nLevers);
	for (int i=0; i<nLevers; i++)
		levers[i].state= NORMAL;
	lockDelay= 120;
};

//	returns true if the indicated lever has switches that are occupied
Interlocking::SwitchState Interlocking::getSwitchOccupied(int lever)
{
	for (SwitchList::iterator i=levers[lever].switches.begin();
	  i!=levers[lever].switches.end(); ++i)
		if (i->first->occupied)
			return OCCUPIED;
	if (levers[lever].routeLock != NULL)
		return ROUTELOCK;
	return NOTOCCUPIED;
}

//	returns true if the indicated lever cannot be moved
bool Interlocking::isLocked(int lever)
{
	if (levers[lever].routeLock != NULL)
		return true;
	for (SwitchList::iterator i=levers[lever].switches.begin();
	  i!=levers[lever].switches.end(); ++i)
		if (i->first->occupied)
			return true;
	LeverState s= levers[lever].state;
	for (std::list<Interlock>::iterator i=interlocks.begin();
	  i!=interlocks.end(); i++) {
		std::list<std::pair<int,int> >::iterator j= i->when.begin();
		for (; j!=i->when.end(); ++j)
			if ((levers[j->first].state&j->second)==0)
				break;
		if (j != i->when.end())
			continue;
		if (i->lever1==lever && (i->states1&s)==0 &&
		  (levers[i->lever2].state&i->states2)==0)
			return true;
		if (i->lever2==lever && (i->states2&s)!=0 &&
		  (levers[i->lever1].state&i->states1)!=0)
			return true;
	}
	return false;
}

//	changes the state of the indicated lever
//	returns true if changed, false if lever cannot move
//	throws switches and changes signal state as appropriate
bool Interlocking::toggleState(int lever, int timeS)
{
	updateRouteLocks(timeS);
	if (isLocked(lever))
		return false;
	Signal* sig= levers[lever].signal;
	if (levers[lever].state == NORMAL) {
		levers[lever].state= REVERSE;
		for (SwitchList::iterator i=levers[lever].switches.begin();
		  i!=levers[lever].switches.end(); ++i) {
			Track::SwVertex* sw= i->first;
			if (i->second)
				sw->throwSwitch(sw->swEdges[sw->mainEdge],true);
			else
				sw->throwSwitch(sw->swEdges[1-sw->mainEdge],
				  true);
		}
		if (sig)
			sig->setState(Signal::CLEAR);
	} else if (getLockDurationS(lever,timeS) <= 0) {
		if (levers[lever].lockTime==0 && sig!=NULL &&
		  sig->trainDistance>0 && !sig->isDistant()) {
			levers[lever].state= (LeverState)(REVERSE|NORMAL);
			levers[lever].lockTime= timeS;
			//addRouteLock(sig,timeS);
		} else {
			levers[lever].state= NORMAL;
			if (sig!=NULL && levers[lever].lockTime==0)
				addRouteLock(sig,timeS,lever);
			levers[lever].lockTime= 0;
		}
		for (SwitchList::iterator i=levers[lever].switches.begin();
		  i!=levers[lever].switches.end(); ++i) {
			Track::SwVertex* sw= i->first;
			if (i->second)
				sw->throwSwitch(sw->swEdges[1-sw->mainEdge],
				  true);
			else
				sw->throwSwitch(sw->swEdges[sw->mainEdge],true);
		}
		if (sig) {
			sig->setState(Signal::STOP);
		}
	} else {
		levers[lever].state= REVERSE;
		levers[lever].lockTime= 0;
		if (sig)
			sig->setState(Signal::CLEAR);
	}
	return true;
}; 

//	sets the state of the indicated lever
bool Interlocking::setState(int lever, LeverState state, int timeS)
{
	if (levers[lever].state == state)
		return true;
	return toggleState(lever,timeS);
};

bool Interlocking::hasLock(int sigLever, Track::SwVertex* sw)
{
	std::map<Track::SwVertex*,int>::iterator i= switch2LeverMap.find(sw);
	if (i == switch2LeverMap.end())
		return false;
	int swLever= i->second;
	for (std::list<Interlock>::iterator i=interlocks.begin();
	  i!=interlocks.end(); i++) {
		if (i->lever1==sigLever && i->lever2==swLever &&
		  (i->states1&REVERSE)!=0)
			return true;
		if (i->lever2==sigLever && i->lever1==swLever &&
		  (i->states2&REVERSE)!=0)
			return true;
	}
	return false;
}

void Interlocking::addRouteLock(Signal* sig, int timeS, int sigLever)
{
	for (int j=0; j<sig->getNumTracks(); j++) {
		Track::Location& loc= sig->getTrack(j);
		Track::Vertex* v= loc.rev ? loc.edge->v2 : loc.edge->v1;
		float x= loc.rev ? loc.offset-loc.edge->length : loc.offset;
		Track::Vertex* lastSwitch= NULL;
		for (Track::Edge* e=loc.edge; e!=NULL && x<1000;
		  e=v->nextEdge(e)) {
			x+= e->length;
			v= v==e->v1 ? e->v2 : e->v1;
			if (v->type == Track::VT_SWITCH) {
				Track::SwVertex* sw= (Track::SwVertex*)v;
				if (sw->hasInterlocking && hasLock(sigLever,sw))
					lastSwitch= sw;
			}
		}
		if (lastSwitch == NULL)
			continue;
		RouteLock* rl= new RouteLock;
		routeLocks.push_back(rl);
		rl->lockTime= timeS;
		v= loc.rev ? loc.edge->v2 : loc.edge->v1;
		x= loc.rev ? loc.offset-loc.edge->length : loc.offset;
		for (Track::Edge* e=loc.edge; e!=NULL && x<1000;
		  e=v->nextEdge(e)) {
			rl->edges.push_back(e);
			x+= e->length;
			v= v==e->v1 ? e->v2 : e->v1;
			if (v->type == Track::VT_SWITCH) {
				Track::SwVertex* sw= (Track::SwVertex*)v;
				if (lastSwitch == sw)
					break;
			}
		}
		fprintf(stderr,"route lock %d %d\n",
		  rl->lockTime,rl->edges.size());
	}
}
void Interlocking::updateRouteLocks(int timeS)
{
	if (routeLocks.size() == 0)
		return;
	for (RouteLockList::iterator i= routeLocks.begin();
	  i!=routeLocks.end();) {
		RouteLock* rl= *i;
		if (timeS-rl->lockTime > lockDelay)
			rl->edges.clear();
		while (rl->edges.size() > 0) {
			Track::Edge* e= rl->edges.front();
			if (e->occupied)
				break;
			rl->edges.pop_front();
		}
//		fprintf(stderr,"route lock update %d %d\n",
//		  rl->lockTime,rl->edges.size());
		if (rl->edges.size() == 0) {
			i= routeLocks.erase(i);
			delete rl;
		} else {
			i++;
		}
	}
	for (int i=0; i<levers.size(); i++)
		levers[i].routeLock= NULL;
	for (RouteLockList::iterator i= routeLocks.begin();
	  i!=routeLocks.end(); i++) {
		RouteLock* rl= *i;
		for (std::list<Track::Edge*>::iterator j=rl->edges.begin();
		  j!=rl->edges.end(); j++) {
			Track::Edge* e= *j;
			setRouteLock(e->v1,rl);
			setRouteLock(e->v2,rl);
		}
	}
	for (std::list<Interlock>::iterator i=interlocks.begin();
	  i!=interlocks.end(); i++) {
		if (levers[i->lever1].routeLock!=NULL &&
		  i->states1==(NORMAL|REVERSE) &&
		  levers[i->lever2].signal==NULL &&
		  levers[i->lever2].switches.size()==0)
			levers[i->lever2].routeLock=
			  levers[i->lever1].routeLock;
		if (levers[i->lever2].routeLock!=NULL &&
		  i->states2==(NORMAL|REVERSE) &&
		  levers[i->lever1].signal==NULL &&
		  levers[i->lever1].switches.size()==0)
			levers[i->lever1].routeLock=
			  levers[i->lever2].routeLock;
	}
}

void Interlocking::setRouteLock(Track::Vertex* v, RouteLock* rl)
{
	if (v->type != Track::VT_SWITCH)
		return;
	Track::SwVertex* sw= (Track::SwVertex*) v;
	std::map<Track::SwVertex*,int>::iterator i= switch2LeverMap.find(sw);
	if (i == switch2LeverMap.end())
		return;
//	fprintf(stderr,"set route lock %d\n",i->second);
	levers[i->second].routeLock= rl;
}

//	parser for creating interlocking
void InterlockingParser::handleBeginBlock(CommandReader& reader)
{
	interlocking= new Interlocking(reader.getInt(1,1,1000));
}

//	parser for populating interlocking parts
bool InterlockingParser::handleCommand(CommandReader& reader)
{
	if (reader.getString(0) == "lever") {
		int i= reader.getInt(1,1,interlocking->getNumLevers())-1;
		interlocking->setColor(i,reader.getDouble(2,0,1),
		  reader.getDouble(3,0,1),reader.getDouble(4,0,1));
		return true;
	} else if (reader.getString(0) == "siglever") {
		int i= reader.getInt(1,1,interlocking->getNumLevers())-1;
		interlocking->setColor(i,reader.getDouble(2,0,1),
		  reader.getDouble(3,0,1),reader.getDouble(4,0,1));
		SignalMap::iterator j= signalMap.find(reader.getString(5));
		if (j == signalMap.end())
			throw std::invalid_argument("cannot find signal");
		interlocking->setSignal(i,j->second);
		return true;
	} else if (reader.getString(0) == "swlever") {
		int i= reader.getInt(1,1,interlocking->getNumLevers())-1;
		interlocking->setColor(i,reader.getDouble(2,0,1),
		  reader.getDouble(3,0,1),reader.getDouble(4,0,1));
		for (TrackMap::iterator j=trackMap.begin();
		  j!=trackMap.end(); ++j) {
			Track::SwVertex* sw= j->second->findSwitch(
			  reader.getDouble(5,-1e10,1e10),
			  reader.getDouble(6,-1e10,1e10),
			  reader.getDouble(7,-1e10,1e10));
			if (sw != NULL) {
				int rev= reader.getInt(8,0,1,0);
				interlocking->addSwitch(i,sw,rev);
				if (rev)
					sw->throwSwitch(
					  sw->swEdges[1-sw->mainEdge],true);
				else
					sw->throwSwitch(
					  sw->swEdges[sw->mainEdge],true);
				break;
			}
		}
		return true;
	} else if (reader.getString(0) == "locking") {
		int l1= getLeverNum(reader,1);
		int s1= getLeverStates(reader,1);
		for (int i=2; i<reader.getNumTokens(); i++) {
			interlocking->addInterlock(l1,s1,
			  getLeverNum(reader,i),getLeverStates(reader,i));
		}
		return true;
	} else if (reader.getString(0) == "when") {
		Interlocking::ConditionList conditions;
		bool locking= false;
		int l1= -1;
		int s1;
		for (int i=1; i<reader.getNumTokens(); i++) {
			if (reader.getString(i)=="locking") {
				locking= true;
			} else if (!locking) {
				int l= getLeverNum(reader,i);
				int s= getLeverStates(reader,i);
				conditions.push_back(std::make_pair(l,s));
			} else if (l1 < 0) {
				l1= getLeverNum(reader,i);
				s1= getLeverStates(reader,i);
			} else {
				interlocking->addInterlock(l1,s1,
				  getLeverNum(reader,i),
				  getLeverStates(reader,i));
				for (Interlocking::ConditionList::iterator
				  j=conditions.begin(); j!=conditions.end();
				  ++j)
					interlocking->addCondition(j->first,
					  j->second);
			}
		}
		return true;
	} else if (reader.getString(0) == "image") {
		interlocking->image= reader.getString(1);
		return true;
	}
	return false;
};

int InterlockingParser::getLeverNum(CommandReader& reader, int idx)
{
	int n= atoi(reader.getString(idx).c_str());
	if (n<1 || n>interlocking->getNumLevers())
		throw std::out_of_range("invalid lever number");
	return n-1;
}

int InterlockingParser::getLeverStates(CommandReader& reader, int idx)
{
	std::string s= reader.getString(idx);
	if (s.find("N") != std::string::npos)
		return Interlocking::NORMAL;
	if (s.find("R") != std::string::npos)
		return Interlocking::REVERSE;
	if (s.find("B") != std::string::npos)
		return Interlocking::NORMAL|Interlocking::REVERSE;
	throw std::out_of_range("unknown lever state");
}

void InterlockingParser::handleEndBlock(CommandReader& reader)
{
};
