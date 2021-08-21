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
#include "rmsim.h"
#include "waterdrawable.h"

osg::BoundingSphere WaterDrawable::computeBound() const
{
	osg::BoundingBox bbox;
	for (Water::TriangleList::iterator j=water->triangleList.begin();
	  j!=water->triangleList.end(); ++j) {
		Water::Triangle* t= *j;
		for (int k=0; k<3; k++) {
			Water::Vertex* v= t->v[k];
			bbox.expandBy(v->xy[0],v->xy[1],0);
		}
	}
	return bbox;
}

void WaterDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	glBegin(GL_TRIANGLES);
	glNormal3f(0,0,1);
	for (Water::TriangleList::iterator j=water->triangleList.begin();
	  j!=water->triangleList.end(); ++j) {
		Water::Triangle* t= *j;
		for (int k=0; k<3; k++) {
			Water::Vertex* v= t->v[k];
			glColor3f(v->color[0],v->color[1],v->color[2]);
			glVertex3d(v->xy[0],v->xy[1],0);
		}
	}
	glEnd();
	if (drawTriangles) {
		glColor3f(.4,.9,.9);
		for (Water::TriangleList::iterator
		  j=water->triangleList.begin();
		  j!=water->triangleList.end(); ++j) {
			Water::Triangle* t= *j;
			glBegin(GL_LINE_LOOP);
			for (int k=0; k<3; k++)
				glVertex3d(t->v[k]->xy[0],t->v[k]->xy[1],0);
			glEnd();
		}
	}
}
