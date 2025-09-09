//	structures for navigable water
//	ships use this to keep track of where they are
//
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
			return update(position[0],position[1]);
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
