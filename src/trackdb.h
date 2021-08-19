#ifndef TRACKDB_H
#define TRACKDB_H

#include "mstsfile.h"
#include "tsection.h"

struct TrSection {
	int sectionID;
	int shapeID;
	int tx;
	int tz;
	float x;
	float y;
	float z;
	float radius;
	float angle;
	float length;
	float grade;
	double cx;
	double cz;
	TrSection* next;
};

struct TrItem {
	int id;
	int otherID;
	int type;
	int tx;
	int tz;
	float x;
	float y;
	float z;
	float offset;
	int flags;
	int dir;
	string* name;
	TrItem* next;
};

struct TrackNode {
	int shape;
	int id;
	int tx;
	int tz;
	float x;
	float y;
	float z;
	int nSections;
	TrSection* sections;
	TrItem* trItems;
	int nPins;
	TrackNode* pin[3];
	int pinEnd[3];
};

struct TrackDB {
	void saveTrackNode(MSTSFileNode* list);
	void saveTrItem(int type, MSTSFileNode* list);
	void freeMem();
 public:
	TrackNode* nodes;
	int nNodes;
	TrItem* trItems;
	int nTrItems;
	TrackDB();
	~TrackDB();
	void readFile(const char* path, TSection* tSection);
	void readFile(const char* path, int readGlobalTSection,
	  int readRouteTSection);
};

#endif
