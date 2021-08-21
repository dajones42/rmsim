//	code to make a simplified grid of triangles
//	used to simplify terrain not near track
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
#include <stdio.h>

#include "trigrid.h"

void TriangleGrid::makeTriangle(int depth, int v1, int v2, int v3, int ccw)
{
	if (depth > 0) {
		int v= (v1+v3)/2;
		makeTriangle(depth-1,v1,v,v2,1-ccw);
		makeTriangle(depth-1,v2,v,v3,1-ccw);
	} else if (ccw) {
//		fprintf(stderr,"ccw triangle %d,%d,%d\n",v1,v2,v3);
		if (hiddenVerts.find(v1) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v2) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v3) != hiddenVerts.end())
			return;
		vList.push_back(v1);
		vList.push_back(v2);
		vList.push_back(v3);
	} else if (edgeDetail && needDetail(v1,v3)) {
//		fprintf(stderr,"detail %d,%d,%d\n",v1,v2,v3);
		int v= (v1+v3)/2;
		if (hiddenVerts.find(v) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v1) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v2) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v3) != hiddenVerts.end())
			return;
		vList.push_back(v1);
		vList.push_back(v);
		vList.push_back(v2);
		vList.push_back(v2);
		vList.push_back(v);
		vList.push_back(v3);
	} else {
//		fprintf(stderr,"cw %d,%d,%d\n",v1,v2,v3);
		if (hiddenVerts.find(v1) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v2) != hiddenVerts.end())
			return;
		if (hiddenVerts.find(v3) != hiddenVerts.end())
			return;
		vList.push_back(v1);
		vList.push_back(v3);
		vList.push_back(v2);
	}
}

bool TriangleGrid::needDetail(int v1, int v3)
{
	int i1= v1/size;
	int i3= v3/size;
	if ((edgeDetail&NEDGE)!=0 && i1==0 && i3==0)
		return true;
	if ((edgeDetail&SEDGE)!=0 && i1==size-1 && i3==size-1)
		return true;
	int j1= v1%size;
	int j3= v3%size;
	if ((edgeDetail&WEDGE)!=0 && j1==0 && j3==0)
		return true;
	if ((edgeDetail&EEDGE)!=0 && j1==size-1 && j3==size-1)
		return true;
	return false;
}

void TriangleGrid::makeList(int depth)
{
//	fprintf(stderr,"makeList(%d) %d %d\n",depth,size,edgeDetail);
	if (edgeDetail && depth%2==0)
		depth++;
	vList.clear();
	int vnw= 0;
	int vne= size-1;
	int vsw= size*size-size;
	int vse= size*size-1;
	makeTriangle(depth,vnw,vsw,vse,1);
	makeTriangle(depth,vse,vne,vnw,1);
}
