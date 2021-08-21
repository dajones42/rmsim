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
