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
