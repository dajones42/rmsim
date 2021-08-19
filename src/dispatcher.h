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
