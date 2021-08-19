//	2D model information
//	mostly old, but still used for ship collision detection
//
//Copyright 2009 Doug Jones
#ifndef MODEL2D_H
#define MODEL2D_H

struct Model2D {
	struct Vertex {
		float x;
		float y;
		float u;
		float v;
	};
	Vertex* vertices;
	int nVertices;
	float radius;
	float color[3];
	int primitive;
	Texture* texture;
	Model2D() {
		texture= NULL;
		primitive= GL_POLYGON;
		radius= 0;
	};
};
typedef map<string,Model2D*> Model2DMap;
extern Model2DMap model2DMap;

#endif
