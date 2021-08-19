//	structures for navigable water
//	ships use this to keep track of where they are
//
//Copyright 2009 Doug Jones
#ifndef WATER_H
#define WATER_H

#include <map>
#include <list>

struct MSTSRoute;

struct Water {
	struct Vertex {
		double xy[2];
		float depth;
		float color[3];
		int label;
		Vertex* next;
		Vertex* prev;
	};
	inline static double area(Vertex* v1, Vertex* v2, Vertex* v3) {
		return (v3->xy[1]-v1->xy[1])*(v2->xy[0]-v1->xy[0]) -
		  (v3->xy[0]-v1->xy[0])*(v2->xy[1]-v1->xy[1]);
	};
	inline static int ccw(Vertex* v1, Vertex* v2, Vertex* v3) {
		return area(v1,v2,v3)>0;
	};
	inline static int inside(Vertex* vi, Vertex* v1, Vertex* v2,
	  Vertex* v3) {
		return ccw(vi,v1,v2) && ccw(vi,v2,v3) && ccw(vi,v3,v1);
	};
	struct Triangle {
		Vertex* v[3];
		Triangle* adj[3];
		float area;
		float len[3];
		inline int inside(Vertex* vi) {
			return Water::inside(vi,v[0],v[1],v[2]);
		};
	};
	struct Location {  // movable ship location
		float depth;
		Triangle* triangle;
		Water* water;
		int set(double x, double y);
		int update(double x, double y);
		int update(const double* position) {
			update(position[0],position[1]);
		}
		Location() { water= NULL; triangle= NULL; };
	};
	typedef map<int,Vertex*> VertexMap;
	typedef list<Triangle*> TriangleList;
	VertexMap vertexMap;
	TriangleList triangleList;
	float waterLevel;
	MSTSRoute* mstsRoute;
	Water() {
		waterLevel= 0;
		mstsRoute= NULL;
	};
	Vertex* findVertex(int id);
	void addVertex(int id, double x, double y, double depth, int edgeID);
	void addTriangle(int v1id, int v2id, int v3id);
	void matchTriangles();
	void setVColor(float* color0, float* color1, float depth);
	void saveShoreLine(const char* filename);
};
typedef list<Water*> WaterList;
extern WaterList waterList;

#endif
