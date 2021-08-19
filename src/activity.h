#ifndef ACTIVITY_H
#define ACTIVITY_H

#include "mstsfile.h"

struct Wagon {
	std::string dir;
	std::string name;
	bool isEngine;
	Wagon* next;
};

struct LooseConsist {
	int id;
	int direction;
	int tx;
	int tz;
	float x;
	float z;
	Wagon* wagons;
	LooseConsist* next;
};

struct Traffic {
	std::string service;
	int startTime;
	Traffic* next;
};

struct Activity {
	void saveConsist(MSTSFileNode* list);
	void saveTrItem(int type, MSTSFileNode* list);
 public:
	LooseConsist* consists;
	Traffic* traffic;
	std::string playerService;
	int startTime;
	Activity();
	~Activity();
	void readFile(const char* path);
	void clear();
};

#endif
