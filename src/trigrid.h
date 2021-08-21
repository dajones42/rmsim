//	triangle grid used for simplifying terrain
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
