#ifndef TRACKPATH_H
#define TRACKPATH_H

struct TrackPDP {
	int tx;
	int tz;
	float x;
	float y;
	float z;
	int type1;
	int type2;
};

struct TrPathNode {
	int flags;
	TrackPDP* pdp;
	TrPathNode* next;
	TrPathNode* prev;
	TrPathNode* nextSiding;
};

struct TrackPath {
	TrPathNode* nodes;
	int nNodes;
	TrackPDP* pdps;
	int nPDPs;
 public:
	TrackPath();
	~TrackPath();
	void readFile(const char* path);
	TrPathNode* getFirstNode() { return &nodes[0]; }
};

#endif
