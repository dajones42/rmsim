//	2D model information
//	mostly old, but still used for ship collision detection
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
		vertices= NULL;
	};
	~Model2D() {
		if (vertices)
			free(vertices);
	};
	void computeRadius();
};
typedef map<string,Model2D*> Model2DMap;
extern Model2DMap model2DMap;

#endif
