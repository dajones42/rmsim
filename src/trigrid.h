//	triangle grid used for simplifying terrain
//
//Copyright 2009 Doug Jones
#ifndef TRIGRID_H
#define TRIGRID_H

#include <list>
#include <set>

class TriangleGrid {
	int edgeDetail;
	int size;
	std::list<int> vList;
	std::set<int> hiddenVerts;
	void makeTriangle(int depth, int v1, int v2, int v3, int ccw);
	bool needDetail(int v1, int v3);
 public:
	TriangleGrid(int sz, int ed) {
		size= sz;
		edgeDetail= ed;
	};
	enum { SEDGE= 01, EEDGE= 02, NEDGE= 04, WEDGE= 010 };
	std::list<int>& getList() { return vList; };
	void makeList(int depth);
	void hide(int i, int j) { hiddenVerts.insert(i*size+j); };
};

#endif
