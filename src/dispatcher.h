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
#ifndef DISPATHER_H
#define DISPATHER_H

#include <set>

struct PathAuth {
	Track::Path::Node *endNode;
	Track::Path::Node *sidingNode;
	bool takeSiding;
	PathAuth(Track::Path::Node* n) {
		endNode= n;
		sidingNode= NULL;
		takeSiding= false;
	};
	PathAuth() {
		PathAuth(NULL);
	};
};

struct MoveAuth {
	float distance;
	float updateDistance;
	int waitTime;
	Track::Path::Node *nextNode;
	Track::Vertex* farVertex;
	MoveAuth(float d, int w) {
		distance= d;
		updateDistance= -1;
		waitTime= w;
		nextNode= NULL;
		farVertex= NULL;
	};
	MoveAuth() {
		MoveAuth(0,0);
	};
};

class Dispatcher {
	enum { BETWEEN, HOLDMAIN, TAKESIDING };
	std::vector<int> blockReservations;
	struct BlockList {
		typedef std::set<int>::iterator iterator;
		std::set<int> list;
		int last;
	 public:
		void add(int block);
		void add(BlockList& other);
		int size() { return list.size(); };
		void clear() { list.clear(); };
		iterator begin() { return list.begin(); };
		iterator end() { return list.end(); };
		bool contains(int block);
	};
	bool canReserve(int id, BlockList* bl, bool checkOtherTrains);
	void reserve(int id, BlockList* bl);
	void unreserve(BlockList* bl);
	struct TrainInfo {
		int id;
		Track::Path* path;
		BlockList blocks;
		Track::SwVertex* nextSwitch;
		int state;
		Track::Path::Node* firstNode;
		Track::Path::Node* stopNode;
		TrainInfo(int n, Track::Path* p) {
			id= n;
			path= p;
			nextSwitch= NULL;
			state= BETWEEN;
			stopNode= NULL;
			firstNode= p->firstNode;
		};
		Track::Path::Node* findBlock(int b);
	};
	typedef std::map<Train*,TrainInfo> TrainInfoMap;
	TrainInfoMap trainInfoMap;
	void findBlocks();
	Track* track;
 public:
	bool ignoreOtherTrains;
	Dispatcher() { track= NULL; ignoreOtherTrains= false; };
	void registerPath(Train* train, Track::Path* path);
	MoveAuth requestAuth(Train* train);
	bool isOnReservedBlock(Train* train);
	void release(Train* train);
	PathAuth requestAuth(Train* train, Track::Path* path,
	  Track::Path::Node* node);
};

#endif
