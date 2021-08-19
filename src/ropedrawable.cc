
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
