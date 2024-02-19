//	automatic brake valve code
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

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "brakevalve.h"

using namespace std;
#include <string>
#include <map>
#include <vector>

void BrakeValve::addTank(std::string name, float vol)
{
	Tank* t= new Tank(name);
	t->index= tanks.size();
	t->volume= vol;
	tanks.push_back(t);
	tankMap[name]= t;
}

void BrakeValve::addPipe(std::string name, float length)
{
	Tank* t= new Tank(name);
	t->index= tanks.size();
	t->pipeLength= length;
	tanks.push_back(t);
	tankMap[name]= t;
}

void BrakeValve::addPiston(std::string up, std::string down)
{
	TankMap::iterator i= tankMap.find(up);
	if (i == tankMap.end()) {
		fprintf(stderr,"cannot find tank %s\n",up.c_str());
		return;
	}
	TankMap::iterator j= tankMap.find(down);
	if (j == tankMap.end()) {
		fprintf(stderr,"cannot find tank %s\n",down.c_str());
		return;
	}
	Piston* p= new Piston;
	p->upTank= i->second->index;
	p->downTank= j->second->index;
	pistons.push_back(p);
}

void BrakeValve::addState(std::string name, std::string upName,
  std::string downName, float upThreshold, float downThreshold)
{
	State* s= new State();
	s->name= name;
	s->nextNameUp= upName;
	s->nextNameDown= downName;
	s->upThreshold= upThreshold*AirTank::PSI2PA;
	s->downThreshold= downThreshold*AirTank::PSI2PA;
	Piston* p= pistons[pistons.size()-1];
	s->index= p->states.size();
	p->states.push_back(s);
//	states.push_back(s);
	stateMap[name]= s;
}

void BrakeValve::addPassage(std::string state, std::string t1, std::string t2,
  float area, bool oneway, float maxPressure2)
{
	StateMap::iterator i= stateMap.find(state);
	if (i == stateMap.end()) {
		fprintf(stderr,"cannot find state %s\n",state.c_str());
		return;
	}
	TankMap::iterator j= tankMap.find(t1);
	if (j == tankMap.end()) {
		fprintf(stderr,"cannot find tank %s\n",t1.c_str());
		return;
	}
	TankMap::iterator k= tankMap.find(t2);
	if (t2.size()>0 && k==tankMap.end()) {
		fprintf(stderr,"cannot find tank %s\n",t2.c_str());
		return;
	}
	Passage* p= new Passage();
	p->tank1= j->second->index;
	p->tank2= t2.size()==0 ? -1 : k->second->index;
	p->area= area;
	p->oneway= oneway;
	if (maxPressure2 > 0)
		p->maxPressure2= maxPressure2*AirTank::PSI2PA+AirTank::STDATM;
	else
		p->maxPressure2= 0;
	i->second->passages.push_back(p);
}

void BrakeValve::matchStates()
{
	for (int i=0; i<pistons.size(); i++) {
		Piston* piston= pistons[i];
		for (int j=0; j<piston->states.size(); j++) {
			State* s= piston->states[j];
			StateMap::iterator k= stateMap.find(s->nextNameUp);
			if (k == stateMap.end()) {
				fprintf(stderr,"cannot find state %s\n",
				  s->nextNameUp.c_str());
			} else {
				s->nextStateUp= k->second;
			}
			k= stateMap.find(s->nextNameDown);
			if (k == stateMap.end()) {
				fprintf(stderr,"cannot find state %s\n",
				  s->nextNameDown.c_str());
			} else {
				s->nextStateDown= k->second;
			}
		}
	}
}

void BrakeValve::addRetainerSetting(std::string name, float area, float psig)
{
	retainerSettings.push_back(
	  new RetainerSetting(name,area,psig*AirTank::PSI2PA+AirTank::STDATM));
}

float radius2Area(float r)
{
	return r*r*M_PI;
}

float chokeArea(float diam, float cContract)
{
	return .25*cContract*diam*diam*M_PI;
}

float chokeAreaI(float diam, float cContract)
{
	return chokeArea(.0254*diam,cContract);
}

BrakeValve* BrakeValve::get(std::string name)
{
	typedef map<string,BrakeValve*> ValveMap;
	static ValveMap valveMap;
	ValveMap::iterator i= valveMap.find(name);
	if (i != valveMap.end())
		return i->second;
	if (name == "K") {
		BrakeValve* valve= new BrakeValve(name);
		valve->addPipe("BP",50*.3048);
		valve->addTank("AR",2500*pow(.0254,3));
		valve->addTank("BC",1000*pow(.0254,3));
		valve->addPiston("AR","BP");
		valve->addState("LAP","SERV","REL",.64,-.84);
		valve->addState("REL","QS","RR",.64,-2.8);
		valve->addState("RR","REL","RR",-.63,-100);
		valve->addState("QS","SERV","QSLAP",1.18,.2);
		valve->addState("SERV","SERV","LAP",100,.2);
		valve->addState("QSLAP","QS","REL",.64,-.84);
		valve->addPassage("REL","BP","AR",chokeAreaI(.082,1));
		valve->addPassage("REL","BC","",radius2Area(.001));
		valve->addPassage("RR","BP","AR",chokeArea(.00143,1));
		valve->addPassage("RR","BC","",radius2Area(.001));
		valve->addPassage("QS","AR","BC",.5*radius2Area(.0016));
		valve->addPassage("QS","BP","BC",chokeAreaI(1./32,1));
		valve->addPassage("SERV","AR","BC",radius2Area(.0016));
		valve->matchStates();
		valve->addRetainerSetting("EX",radius2Area(.001),0);
		valve->addRetainerSetting("HP",.1*radius2Area(.001),20);
		valve->addRetainerSetting("LP",.25*radius2Area(.001),10);
		valve->addRetainerSetting("SD",.2*radius2Area(.001),0);
		valveMap[name]= valve;
		return valve;
	}
	if (name == "AB") {
		BrakeValve* valve= new BrakeValve(name);
		valve->addPipe("BP",50*.3048);
		valve->addTank("AR",2500*pow(.0254,3));
		valve->addTank("BC",1000*pow(.0254,3));
		valve->addTank("ER",3500*pow(.0254,3));
		valve->addTank("QSV",.0006);
		valve->addTank("QAC",145*pow(.0254,3));
		valve->addPiston("AR","BP");
		valve->addState("LAP","SERV","REL",.64,-.84);
		valve->addState("REL","IQS","RR",.64,-2.8);
		valve->addState("RR","REL","RR",-.63,-100);
		valve->addState("IQS","SERV","REL",1.18,.2);
		valve->addState("SERV","SERV","LAP",100,.2);
		valve->addPassage("REL","BP","AR",2*chokeAreaI(.043,1));
		valve->addPassage("REL","AR","ER",radius2Area(.0012));
		valve->addPassage("REL","BC","",radius2Area(.001));
		valve->addPassage("REL","QSV","",chokeAreaI(1./32,.6));
		valve->addPassage("RR","BP","AR",chokeArea(.00143,.9));
		valve->addPassage("RR","AR","ER",radius2Area(.0012));
		valve->addPassage("RR","BC","",radius2Area(.001));
		valve->addPassage("RR","QSV","",chokeAreaI(1./32,.6));
		valve->addPassage("IQS","BP","QSV",radius2Area(.015));
		valve->addPassage("IQS","QSV","",chokeAreaI(1./32,.6));
		valve->addPassage("SERV","AR","BC",radius2Area(.0016));
		valve->addPassage("SERV","BP","BC",chokeAreaI(1./32,1),true,10);
		valve->addPassage("SERV","QSV","",chokeAreaI(1./32,.6));
		valve->addPassage("LAP","BP","BC",chokeAreaI(1./32,1),true,10);
		valve->addPassage("LAP","QSV","",chokeAreaI(1./32,.6));
		valve->addPiston("QAC","BP");
		valve->addState("QA1","QA2","QA1",.8,-100);
		valve->addState("QA2","QA3","QA1",2.9,.7);
		valve->addState("QA3","EMERG","QA2",4,2.8);
		valve->addState("EMERG","EMERG","QA3",100,3);
		valve->addPassage("QA1","BP","QAC",chokeArea(.000515,.7));
		valve->addPassage("QA2","BP","QAC",chokeArea(.000515,.7));
		valve->addPassage("QA2","QAC","",chokeArea(.0003,1));
		valve->addPassage("QA3","BP","QAC",chokeArea(.000515,.7));
		valve->addPassage("QA3","QAC","",chokeArea(.0003,1));
		valve->addPassage("QA3","QAC","",chokeArea(.00246,.6));
		valve->addPassage("EMERG","QAC","",chokeArea(.00246,.6));
		valve->addPassage("EMERG","BP","",chokeArea(.00246,.6));
		valve->addPassage("EMERG","ER","BC",radius2Area(.0016));
		valve->matchStates();
		valve->addRetainerSetting("EX",radius2Area(.001),0);
		valve->addRetainerSetting("HP",.1*radius2Area(.001),20);
		valve->addRetainerSetting("LP",.25*radius2Area(.001),10);
		valve->addRetainerSetting("SD",.2*radius2Area(.001),0);
		valveMap[name]= valve;
		return valve;
	}
	if (name == "H6") {
		BrakeValve* valve= new BrakeValve(name);
		valve->addPipe("BP",50*.3048);
		valve->addTank("AR",1000*pow(.0254,3));
		valve->addTank("BC",1000*pow(.0254,3));
		valve->addTank("AC",400*pow(.0254,3));
		valve->addTank("MR",2*5000*pow(.0254,3));
		valve->addPiston("AR","BP");
		valve->addState("LAP","SERV","REL",.64,-.84);
		valve->addState("REL","SERV","REL",.64,-100);
		valve->addState("SERV","SERV","LAP",100,.2);
		valve->addPassage("REL","BP","AR",.4*chokeAreaI(.082,1));
		valve->addPassage("REL","AC","",.4*radius2Area(.001));
		valve->addPassage("SERV","AR","AC",.4*radius2Area(.0016));
		valve->addPiston("AC","BC");
		valve->addState("ALAP","ASERV","AREL",.5,-.5);
		valve->addState("AREL","ASERV","AREL",.5,-100);
		valve->addState("ASERV","ASERV","ALAP",100,0);
		valve->addPassage("AREL","BC","",radius2Area(.001));
		valve->addPassage("ASERV","MR","BC",radius2Area(.0016));
		valve->matchStates();
		valveMap[name]= valve;
		return valve;
	}
	if (name == "L") {
		BrakeValve* valve= new BrakeValve(name);
		valve->addPipe("BP",50*.3048);
		valve->addTank("AR",4476*pow(.0254,3));
		valve->addTank("BC",1847*pow(.0254,3));//14x12
		valve->addTank("ER",10158*pow(.0254,3));
		valve->addPiston("AR","BP");
		valve->addState("LAP","SERV","REL",.64,-.84);
		valve->addState("REL","QS","REL",.64,-100);
		valve->addState("QS","SERV","QSLAP",1.18,.2);
		valve->addState("SERV","SERV","LAP",100,.2);
		valve->addState("QSLAP","QS","REL",.64,-.84);
		valve->addPassage("REL","BP","AR",chokeAreaI(.082,1));
		valve->addPassage("REL","BC","",chokeAreaI(.1875,1));
		valve->addPassage("REL","ER","AR",3*chokeAreaI(.073,1));
		valve->addPassage("QS","AR","BC",.5*chokeAreaI(.12,1));
		valve->addPassage("QS","BP","BC",chokeAreaI(1./32,1));
		valve->addPassage("SERV","AR","BC",chokeAreaI(.12,1));
		valve->matchStates();
		valveMap[name]= valve;
		return valve;
	}
	if (name == "AMM") {
		BrakeValve* valve= new BrakeValve(name);
		valve->addPipe("BP",50*.3048);
		valve->addPipe("CP",50*.3048);
		valve->addTank("AR",4476*pow(.0254,3));
		valve->addTank("BC",1847*pow(.0254,3));//14x12
		valve->addPiston("AR","BP");
		valve->addState("LAP","SERV","REL",.64,-.84);
		valve->addState("REL","QS","REL",.64,-100);
		valve->addState("QS","SERV","QSLAP",1.18,.2);
		valve->addState("SERV","SERV","LAP",100,.2);
		valve->addState("QSLAP","QS","REL",.64,-.84);
		valve->addPassage("REL","BP","AR",chokeAreaI(.082,1));
		valve->addPassage("REL","BC","",chokeAreaI(.1875,1));
		valve->addPassage("REL","CP","AR",3*chokeAreaI(.073,1));
		valve->addPassage("QS","AR","BC",.5*chokeAreaI(.12,1));
		valve->addPassage("QS","BP","BC",chokeAreaI(1./32,1));
		valve->addPassage("SERV","AR","BC",chokeAreaI(.12,1));
		valve->matchStates();
		valveMap[name]= valve;
		return valve;
	}
	return get("K");
}

int BrakeValve::updateState(int state, vector<AirTank*>& tanks)
{
	int newState= 0;
	for (int i=0; i<pistons.size(); i++) {
		Piston* p= pistons[i];
		float dp= tanks[p->upTank]->getPa() -
		  tanks[p->downTank]->getPa();
		State* s= p->states[(state>>(4*i))&0xf];
		if (dp > s->upThreshold)
			newState|= s->nextStateUp->index<<(4*i);
		else if (dp < s->downThreshold)
			newState|= s->nextStateDown->index<<(4*i);
		else
			newState|= s->index<<(4*i);
	}
	return newState;
}

void BrakeValve::updatePressures(int state, float timeStep,
  vector<AirTank*>& tanks, int retainerControl)
{
	for (int i=0; i<pistons.size(); i++) {
		Piston* piston= pistons[i];
		State* valveState= piston->states[(state>>(4*i))&0xf];
		for (int j=0; j<valveState->passages.size(); j++) {
			BrakeValve::Passage* passage= valveState->passages[j];
			AirTank* tank1= tanks[passage->tank1];
			if (passage->tank2 >= 0) {
				AirTank* tank2= tanks[passage->tank2];
				if (passage->maxPressure2>0 &&
				  tank2->getPa()>=passage->maxPressure2)
					continue;
				if (passage->oneway &&
				  tank1->getPa()<tank2->getPa())
					continue;
				tank1->moveAir(tank2,passage->area,timeStep);
			} else if (passage->tank1==2 &&
			  retainerSettings.size()>0) {
				BrakeValve::RetainerSetting* r=
				  retainerSettings[retainerControl];
				if (tank1->getPa() < r->pressure)
					continue;
				tank1->vent(r->area,timeStep);
			} else {
				tank1->vent(passage->area,timeStep);
			}
		}
	}
}
