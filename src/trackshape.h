//	class for creating track models
//
//Copyright 2009 Doug Jones
#ifndef TRACKSHAPE_H
#define TRACKSHAPE_H

#include <string>
#include <vector>
#include <map>
#include "texture.h"
#include <osg/Vec4>

struct TrackShape {
	struct Offset {
		float x;	// distance from center line
		float y;	// distance below rail head
		int other;
		Offset(float x, float y) {
			this->x= x;
			this->y= y;
			this->other= 0;
		};
	};
	struct Surface {
		short vo1;
		short vo2;
		float u1;
		float u2;
		float meters;
		int nTies;
		int flags;
		Surface(short i, short j, float u1, float u2, float m, int n,
		  int f=0) {
			this->vo1= i;
			this->vo2= j;
			this->u1= u1;
			this->u2= u2;
			this->meters= m;
			this->nTies= n;
			this->flags= f;
		};
	};
	struct EndVert {
		short offset;
		float u;
		float v;
		EndVert(short o, float u, float v) {
			this->offset= o;
			this->u= u;
			this->v= v;
		};
	};
	typedef std::vector<Offset> Offsets;
	typedef std::vector<Surface> Surfaces;
	typedef std::vector<EndVert> EndVerts;
	Offsets offsets;
	Surfaces surfaces;
	EndVerts endVerts;
	Texture* texture;
	osg::Vec4 color;
	TrackShape() {
		texture= NULL;
		color= osg::Vec4(1,1,1,1);
	}
	void matchOffsets();
};
typedef std::map<std::string,TrackShape*> TrackShapeMap;
extern TrackShapeMap trackShapeMap;

#endif
