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
