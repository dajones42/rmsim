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
#include <iostream>
#include <sstream>

using namespace std;

ChangeLog* ChangeLog::inst= 0;

ChangeLog::ChangeLog()
{
}

ChangeLog* ChangeLog::instance()
{
	if (inst == 0)
		inst= new ChangeLog();
	return inst;
}

void ChangeLog::add(string s)
{
	changeList.insert(make_pair(simTime,s));
//	cerr<<"logadd "<<simTime<<" "<<s<<"\n";
}

void ChangeLog::addThrow(Track::SwVertex* sw)
{
	ostringstream os;
	os<<"throw|"<<sw->id<<"|"<<(sw->edge2==sw->swEdges[1]);
	add(os.str());
}

void ChangeLog::addCreate(Train* t)
{
	ostringstream os;
	os<<"create|"<<t->id;
	add(os.str());
}

void ChangeLog::addDestroy(Train* t)
{
	ostringstream os;
	os<<"destroy|"<<t->id;
	add(os.str());
}

void ChangeLog::addCouple(Train* t1, Train* t2)
{
	ostringstream os;
	os<<"couple|"<<t1->id<<"|"<<t2->id;
	add(os.str());
}

void ChangeLog::addUncouple(Train* t1, Train* t2, bool keepRear)
{
	int n= 0;
	for (RailCarInst* c=t1->firstCar; c!=NULL; c=c->next)
		n++;
	ostringstream os;
	os<<"uncouple|"<<t1->id<<"|"<<n<<"|"<<keepRear<<"|"<<t2->id;
	add(os.str());
}

void ChangeLog::addRandomSelection(Train* t, int nEng, int nCab,
  std::list<int>& carList)
{
	int n= 0;
	for (RailCarInst* c=t->firstCar; c!=NULL; c=c->next)
		n++;
	ostringstream os;
	os<<"randomselection|"<<t->id<<"|"<<n<<"|"<<nEng<<"|"<<nCab;
	for (std::list<int>::iterator i=carList.begin(); i!=carList.end(); ++i)
		os<<"|"<<*i;
	add(os.str());
}


void ChangeLog::print()
{
	for (iterator i=begin(); i!=end(); i++)
		cerr<<i->first<<"|"<<i->second<<"\n";
}
