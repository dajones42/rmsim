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
#ifndef CHANGELOG_H
#define CHANGELOG_H

#include <map>
#include <list>
#include <string>
#include "track.h"
#include "train.h"

class ChangeLog {
	static ChangeLog* inst;
 protected:
	ChangeLog();
	void add(std::string);
 public:
	static ChangeLog* instance();
	void addThrow(Track::SwVertex* sw);
	void addCreate(Train* t);
	void addDestroy(Train* t);
	void addCouple(Train* t1, Train* t2);
	void addUncouple(Train* t1, Train* t2, bool keepRear);
	void addRandomSelection(Train* t, int nEng, int nCab,
	  std::list<int>& carList);
	void print();
	typedef std::multimap<double,std::string> ChangeList;
 private:
	ChangeList changeList;
 public:
	typedef ChangeList::iterator iterator;
	iterator begin() { return changeList.begin(); };
	iterator end() { return changeList.end(); };
	iterator find(double simTime) {
		return changeList.lower_bound(simTime);
	};
};

#endif
