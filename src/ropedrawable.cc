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
#include "ropedrawable.h"

osg::BoundingSphere RopeDrawable::computeBound() const
{
	osg::BoundingBox bbox;
	for (RopeList::iterator j=ropeList.begin(); j!=ropeList.end(); ++j) {
		Rope* r= *j;
		r->calcPosition();
		bbox.expandBy(r->xy1[0],r->xy1[1],r->xy1[2]);
		bbox.expandBy(r->xy2[0],r->xy2[1],r->xy2[2]);
	}
	return bbox;
}

void RopeDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	const float sz= .03;
	for (RopeList::iterator j=ropeList.begin(); j!=ropeList.end(); ++j) {
		Rope* r= *j;
		r->calcPosition();
		if (false)//r == selectedRope)
			glColor3f(1,1,1);
		else if (r->adjust < 0)
			glColor3f(.8,0,0);
		else if (r->adjust > 0)
			glColor3f(0,.8,0);
		else
			glColor3f(.8,.8,0);
		float dx= sz*(r->xy1[0]-r->xy2[0])/r->length;
		float dy= sz*(r->xy1[1]-r->xy2[1])/r->length;
		glBegin(GL_QUADS);
		glVertex3d(r->xy1[0]+dy,r->xy1[1]-dx,r->xy1[2]);
		glVertex3d(r->xy1[0],r->xy1[1],r->xy1[2]+sz);
		glVertex3d(r->xy2[0],r->xy2[1],r->xy2[2]+sz);
		glVertex3d(r->xy2[0]+dy,r->xy2[1]-dx,r->xy2[2]);
		glVertex3d(r->xy1[0],r->xy1[1],r->xy1[2]-sz);
		glVertex3d(r->xy1[0]+dy,r->xy1[1]-dx,r->xy1[2]);
		glVertex3d(r->xy2[0]+dy,r->xy2[1]-dx,r->xy2[2]);
		glVertex3d(r->xy2[0],r->xy2[1],r->xy2[2]-sz);
		glVertex3d(r->xy1[0]-dy,r->xy1[1]+dx,r->xy1[2]);
		glVertex3d(r->xy1[0],r->xy1[1],r->xy1[2]-sz);
		glVertex3d(r->xy2[0],r->xy2[1],r->xy2[2]-sz);
		glVertex3d(r->xy2[0]-dy,r->xy2[1]+dx,r->xy2[2]);
		glVertex3d(r->xy1[0],r->xy1[1],r->xy1[2]+sz);
		glVertex3d(r->xy1[0]-dy,r->xy1[1]+dx,r->xy1[2]);
		glVertex3d(r->xy2[0]-dy,r->xy2[1]+dx,r->xy2[2]);
		glVertex3d(r->xy2[0],r->xy2[1],r->xy2[2]+sz);
		glEnd();
	}
}

void RopeDrawable::accept(osg::PrimitiveFunctor& pf) const
{
#if 0
	const float sz= .03;
	for (RopeList::iterator j=ropeList.begin(); j!=ropeList.end(); ++j) {
		Rope* r= *j;
		r->calcPosition();
		float dx= sz*(r->xy1[0]-r->xy2[0])/r->length;
		float dy= sz*(r->xy1[1]-r->xy2[1])/r->length;
		pf.begin(GL_QUADS);
		pf.vertex(r->xy1[0]+dy,r->xy1[1]-dx,r->xy1[2]);
		pf.vertex(r->xy1[0],r->xy1[1],r->xy1[2]+sz);
		pf.vertex(r->xy2[0],r->xy2[1],r->xy2[2]+sz);
		pf.vertex(r->xy2[0]+dy,r->xy2[1]-dx,r->xy2[2]);
		pf.vertex(r->xy1[0],r->xy1[1],r->xy1[2]-sz);
		pf.vertex(r->xy1[0]+dy,r->xy1[1]-dx,r->xy1[2]);
		pf.vertex(r->xy2[0]+dy,r->xy2[1]-dx,r->xy2[2]);
		pf.vertex(r->xy2[0],r->xy2[1],r->xy2[2]-sz);
		pf.vertex(r->xy1[0]-dy,r->xy1[1]+dx,r->xy1[2]);
		pf.vertex(r->xy1[0],r->xy1[1],r->xy1[2]-sz);
		pf.vertex(r->xy2[0],r->xy2[1],r->xy2[2]-sz);
		pf.vertex(r->xy2[0]-dy,r->xy2[1]+dx,r->xy2[2]);
		pf.vertex(r->xy1[0],r->xy1[1],r->xy1[2]+sz);
		pf.vertex(r->xy1[0]-dy,r->xy1[1]+dx,r->xy1[2]);
		pf.vertex(r->xy2[0]-dy,r->xy2[1]+dx,r->xy2[2]);
		pf.vertex(r->xy2[0],r->xy2[1],r->xy2[2]+sz);
		pf.end();
	}
#endif
}

RopeDrawable* ropeDrawable;
