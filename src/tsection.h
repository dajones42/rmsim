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
