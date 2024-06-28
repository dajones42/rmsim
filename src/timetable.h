//	Classes for storing a train time table
//	Also used to store time sheet info.
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
#ifndef TIMETABLE_H
#define TIMETABLE_H

#include <string>
#include <map>
#include <vector>
#include <list>
#include <queue>
#include "commandreader.h"
#include "track.h"

namespace tt {

#define EXTRACLASS	8

class TimeTable;
class Train;

class Station {
	friend class TimeTable;
	friend class Train;
	std::string name;
	std::string callSign;
	std::vector<std::string> altNames;
	std::vector<float> sidings;
	int nTracks;
	float distance;
	bool promptForBlock;
 public:
	std::list<int> rows;
	Station(std::string& s) { name= s; nTracks= 1; promptForBlock= false; };
	void addSiding(float length) { sidings.push_back(length); };
	void setCallSign(std::string s) { callSign= s; };
	void addAltName(std::string s) { altNames.push_back(s); };
	void setDistance(float d) { distance= d; };
	float getDistance() { return distance; }
	void setNumTracks(int n) { nTracks= n; };
	int getNumTracks() { return nTracks; }
	bool getPromptForBlock() { return promptForBlock; }
	void parse(CommandReader& reader);
	std::string& getName() { return name; };
	int getNumAltNames() { return altNames.size(); };
	std::string& getAltName(int i) { return altNames[i]; };
	std::string& getCallSign() { return callSign; };
	float longestSiding() {
		float max= 0;
		for (int i=0; i<sidings.size(); i++)
			if (max < sidings[i])
				max= sidings[i];
		return max;
	}
};
typedef std::map<std::string,Station*> StationMap;

struct TrainTime {
	int schedAr;
	int schedLv;
	int wait;
	int actualAr;
	int actualLv;
	TrainTime() {
		schedAr= -1;
		schedLv= -1;
		wait= 0;
		actualAr= -1;
		actualLv= -1;
	}
};

typedef std::map<Train*,Station*> MeetMap;

class Train {
 protected:
	friend class TimeTable;
	std::string name;
	int clas;
	bool readDown;
	int row;
	bool active;
	std::vector<TrainTime> times;
	Train* prevTrain;
	Train* nextTrain;
 public:
	int startDelay;
	float minSpeed;
	TimeTable* timeTable;
	Track::Path* path;
	bool startVisible;
	bool endVisible;
	std::string route;
	Train(std::string s) {
		name= s;
		prevTrain= nextTrain= NULL;
		path= NULL;
		minSpeed= 0;
		startDelay= 0;
		active= false;
		startVisible= false;
		endVisible= false;
	};
	MeetMap meets;
	void setSchedTime(Station* s, int ar, int lv, int wait);
	void copyTimes(std::string& name, int offset);
	void init(int startTime);
	void setArrival(int row, int time);
	void setDeparture(int row, int time);
	void parse(CommandReader& reader);
	void addWait(int row, int w) { times[row].wait+= w; };
	int getSchedAr(int row) { return times[row].schedAr; };
	int getSchedLv(int row) { return times[row].schedLv; };
	int getWait(int row) { return times[row].wait; };
	int getActualAr(int row) { return times[row].actualAr; };
	int getActualLv(int row) { return times[row].actualLv; };
	int getCurrentRow() { return row; };
	int getAnyRow(int r);
	int getNextRow(float length, int fromRow= -1);
	int getActive() { return active; };
	int getClass() { return clas; };
	bool getReadDown() { return readDown; };
	Train* getNextTrain() { return nextTrain; };
	Train* getPrevTrain() { return prevTrain; };
	void makePlayer() { prevTrain= this; endVisible= true; };
	int mustWait(int time, int fromRow, int toRow, int eta,
	  std::string* reason);
	std::string& getName() { return name; };
	int getRow() { return row; };
	Station* getRow(int idx);
	TimeTable* getTimeTable() { return timeTable; };
	bool hasBlock(int fromRow, int toRow);
};
typedef std::map<std::string,Train*> TrainMap;

struct BlockTimes {
	Train* train;
	int timeGiven;
	int timeEntered;
	int timeCleared;
	BlockTimes(Train* t, int tm) {
		train= t;
		timeGiven= tm;
		timeEntered= -1;
		timeCleared= -1;
	};
};

struct Block {
	Station* station1;
	Station* station2;
	int nTracks;
	std::vector<BlockTimes> trainTimes;
	Block(Station* s1, Station* s2, int n) {
		station1= s1;
		station2= s2;
		nTracks= n;
	}
};

class TimeTable {
	friend class Train;
 protected:
	std::string name;
	std::vector<Station*> rows;
	std::vector<Train*> trains;
	TrainMap trainMap;
	StationMap stationMap;
	bool downSuperior;
	bool ignoreOtherTrains;
	int rule91Delay;
	std::vector<Block*> blocks;
 public:
	TimeTable() {
		downSuperior= true;
		ignoreOtherTrains= false;
		rule91Delay= 10*60;
	}
	~TimeTable();
	void setSuperior(int down) { downSuperior= down; };
	void setIgnoreOther(int ignore) { ignoreOtherTrains= ignore; };
	Station* findStation(std::string name);
	Train* findTrain(std::string name);
	void addStation(Station* s);
	void addTrain(Train* t);
	virtual Station* addStation(std::string name);
	virtual Train* addTrain(std::string name);
	void addRow(Station* s) {
		s->rows.push_back(rows.size());
		rows.push_back(s);
	};
	void parse(CommandReader& reader);
	int getNumTrains() { return trains.size(); }
	Train* getTrain(int idx) { return trains[idx]; }
	int getNumRows() { return rows.size(); }
	Station* getRow(int idx) { return rows[idx]; }
	std::string& getName() { return name; }
	bool getDownSuperior() { return downSuperior; }
	bool getIgnoreOther() { return ignoreOtherTrains; }
	void print(FILE* out);
	void printTimeSheet(FILE* out, bool activeOnly=false);
	void printTimeSheet2(FILE* out, bool activeOnly=false);
	void printHorz(FILE* out);
	void printTimeSheetHorz(FILE* out, bool activeOnly=false);
	void printTimeSheetHorz2(FILE* out, bool activeOnly=false);
	void printBlocks(FILE* out);
	int getRule91Delay() { return rule91Delay; }
	void setRule91Delay(int d) { rule91Delay= d; }
	float distance(int row1, int row2);
	void printTimeSheet2(std::string& s);
	void printTimeSheetHtml(std::string& s);
	bool addMeet(Train* t1, Train* t2, Station* s);
	Block* getBlockFor(Train* train, int time);
	Block* findBlock(int row1, int row2);
};

}
#endif
