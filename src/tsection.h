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
#ifndef TSECTION_H
#define TSECTION_H

#include "mstsfile.h"

using namespace std;
#include <map>
#include <vector>

struct Curve {
	float radius;
	float angle;
};

struct MSTSTrackShape {
	struct Path {
		float start[3];
		float angle;
		vector<int> sections;
	};
	string name;
	vector<Path*> paths;
	~MSTSTrackShape();
};

typedef map<int,Curve*> CurveMap;
typedef map<int,float> LengthMap;
typedef map<int,int> MainRouteMap;
typedef map<int,MSTSTrackShape*> MSTSTrackShapeMap;

struct TSection {
 public:
	CurveMap curveMap;
	MainRouteMap mainRouteMap;
	LengthMap lengthMap;
	MSTSTrackShapeMap shapeMap;
	TSection();
	~TSection();
	void readGlobalFile(const char* path, bool saveShapes=false);
	void readRouteFile(const char* path);
	Curve* findCurve(int id);
	int findMainRoute(int id);
};

#endif
