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
#ifndef ACTIVITY_H
#define ACTIVITY_H

#include "mstsfile.h"

struct Wagon {
	std::string dir;
	std::string name;
	bool isEngine;
	int id;
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
	int id;
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
