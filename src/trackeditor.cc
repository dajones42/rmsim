#include <stdio.h>
#include <stdlib.h>
#include <iostream>

using namespace std;
#include <string>
#include <list>
#include <vector>

#include <osg/Notify>
#include <osg/Switch>
#include <osg/PositionAttitudeTransform>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osg/CullFace>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include "rmsim.h"
#include "parser.h"
#include "trackeditor.h"

extern string command;
extern Person currentPerson;

double lineSegDistSq(osg::Vec3d p, osg::Vec3d p1, osg::Vec3d p2)
{
	osg::Vec3d dp= p2-p1;
	double d= dp*dp;
	double n= dp*(p-p1);
	if (d==0 || n<=0 || n>=d)
		return -1;
	dp*= n/d;
	dp+= p1 - p;
	return dp*dp;
}

float addAngles(float a1, float a2)
{
	float a= a1+a2;
	if (a < -M_PI)
		a+= 2*M_PI;
	if (a > M_PI)
		a-= 2*M_PI;
	return a;
}

void movePointAway(osg::Vec3d& p, double dist, osg::Vec3d p1, osg::Vec3d p2)
{
	osg::Vec3d dp= p2-p1;
	double d= dp*dp;
	double n= dp*(p-p1);
	if (d==0 || n<=0 || n>=d)
		return;
	dp*= n/d;
	dp+= p1;
	d= (p-dp).length();
	if (d<dist && d>0)
		p+= (p-dp)*dist/d;
	return;
}

VertexDragger::VertexDragger(osg::Vec3d pos)
{
	setPosition(pos);
	osg::Geode* geode= new osg::Geode;
	osg::Sphere* sphere= new osg::Sphere(osg::Vec3(0,0,0),4);
	shapeDrawable= new osg::ShapeDrawable(sphere);
	geode->addDrawable(shapeDrawable.get());
	osg::StateSet* stateSet= geode->getOrCreateStateSet();
	stateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
	stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	stateSet->setRenderBinDetails(15,"RenderBin");
	osg::AutoTransform* at= new osg::AutoTransform;
	at->setAutoScaleToScreen(true);
	at->addChild(geode);
	addChild(at);
	setCullingActive(false);
	setColor(osg::Vec4f(0,1,1,1));
	heading= -1;
	elevation= 0;
	grade= -100;
	setNodeMask(4);
};

string VertexDragger::toString()
{
	osg::Vec3d p= getPosition();
	char buf[100];
	osg::Vec3f d= dir;
	d[2]= 0;
	d.normalize();
	float h= 90-atan2(d[1],d[0])*180/M_PI;
	if (h < 0)
		h+= 360;
	snprintf(buf,100,"%.3f %.3f %.3f h: %.1f g: %.1f",
	  p[0],p[1],p[2],h,dir[2]*100);
	return buf;
}

EdgeDrawable::EdgeDrawable(VertexDragger* v1, VertexDragger* v2) {
	vertex1= v1;
	vertex2= v2;
	v1->addEdge(this);
	v2->addEdge(this);
	setDataVariance(osg::Object::DYNAMIC);
	setUseDisplayList(false);
	type= CURVE;
	selected= false;
	radius= 0;
	angle= 0;
}

string EdgeDrawable::toString()
{
	char buf[100];
	if (type == STRAIGHT)
		snprintf(buf,100,"length: %.1f grade: %.1f",
		  length,grade);
	else
		snprintf(buf,100,"length: %.1f grade: %.1f radius: %.1f",
		  length,grade,radius);
	return buf;
}

osg::BoundingSphere EdgeDrawable::computeBound() const
{
	osg::BoundingSphere bbox;
	bbox.expandBy(vertex1->getPosition());
	bbox.expandBy(vertex2->getPosition());
	return bbox;
}

void EdgeDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (selected)
		glColor3f(1,1,0);
	else if (type == STRAIGHT)
		glColor3f(0,1,1);
	else
		glColor3f(1,0,1);
	glBegin(GL_LINE_STRIP);
	osg::Vec3d p= vertex1->getPosition();
	glVertex3d(p.x(),p.y(),p.z());
	for (list<osg::Vec3d>::const_iterator i=points.begin(); i!=points.end();
	  i++) {
		p= *i;
		glVertex3d(p.x(),p.y(),p.z());
	}
	p= vertex2->getPosition();
	glVertex3d(p.x(),p.y(),p.z());
	glEnd();
}

void EdgeDrawable::addStraightPoints(float spacing)
{
	if (type != STRAIGHT)
		return;
	points.clear();
	osg::Vec3d p1= vertex1->getPosition();
	osg::Vec3d p2= vertex2->getPosition();
	osg::Vec3f d= p2 - p1;
	int ni= (int)(length/spacing);
	d/= ni+1;
	osg::Vec3d p= p1;
	for (int i=0; i<ni; i++) {
		p+= d;
		points.push_back(p);
	}
}

inline double triArea(osg::Vec3f& p1, osg::Vec3f& p2, osg::Vec3f& p3)
{
	return triArea(p1[0],p1[1],p2[0],p2[1],p3[0],p3[1]);
}

void EdgeDrawable::update()
{
	points.clear();
	osg::Vec3d p1= vertex1->getPosition();
	osg::Vec3d p2= vertex2->getPosition();
	osg::Vec3f d= p2 - p1;
	length= d.length();
	grade= length>0 ? d[2]/length*100 : 0;
	radius= 0;
//	cerr<<"dist "<<d.length()/.3048<<endl;
	if (isStraight())
		return;
	osg::Vec3f d1= vertex1->getDirection(true);
	osg::Vec3f d2= vertex2->getDirection(false);
	if (d*d1 < 0)
		d1= -d1;
	if (d*d2 < 0)
		d2= -d2;
	float a1= triArea(p1[0]-d1[0],p1[1]-d1[1],p1[0],p1[1],p2[0],p2[1]);
	float a2= triArea(p1[0],p1[1],p2[0],p2[1],p2[0]+d2[0],p2[1]+d2[1]);
	if ((a1<0 && a2>0) || (a1>0 && a2<0)) {
		osg::Vec3f perp1= a1>0 ? osg::Vec3f(-d1[1],d1[0],0) :
		  osg::Vec3f(d1[1],-d1[0],0);
		osg::Vec3f perp2= a2>0 ? osg::Vec3f(-d2[1],d2[0],0) :
		  osg::Vec3f(d2[1],-d2[0],0);
//		fprintf(stderr,"reverse %f %f %f\n",a1,a2,length);
//		fprintf(stderr," %f %f %f %f\n",d1[0],d1[1],perp1[0],perp1[1]);
//		fprintf(stderr," %f %f %f %f\n",d2[0],d2[1],perp2[0],perp2[1]);
		float lo= length/4;
		float hi= 10*length;
		for (int i=0; i<15; i++) {
			float r= .5*(hi+lo);
			osg::Vec3f c1= p1+perp1*r;
			osg::Vec3f c2= p2+perp2*r;
			float r1= (c2-c1).length();
//			fprintf(stderr,"  %d %f %f %f %f\n",i,r,r1/2,lo,hi);
			if (r1 < 2*r)
				hi= r;
			else
				lo= r;
		}
		float r= .5*(hi+lo);
		osg::Vec3d c1= p1+perp1*r;
		osg::Vec3d c2= p2+perp2*r;
		osg::Vec3d c= (c1+c2)*.5;
		c[2]= .5*(p1[2]+p2[2]);
		osg::Vec3f dc= a1>0 ? osg::Vec3f(-(c-c1)[1],(c-c1)[0],0) :
		  osg::Vec3f((c-c1)[1],-(c-c1)[0],0);
		dc.normalize();
		addCurvePoints(p1,c,d1,dc);
		points.push_back(c);
		addCurvePoints(c,p2,dc,d2);
	} else {
		addCurvePoints(p1,p2,d1,d2);
	}
	float dz= p2[2]-p1[2];
	length= 0;
	for (list<osg::Vec3d>::const_iterator i=points.begin();
	  i!=points.end(); i++) {
		osg::Vec3d p= *i;
		length+= (p-p1).length();
		p1= p;
	}
	length+= (p2-p1).length();
	grade= length>0 ? dz/length*100 : 0;
}

//	finds the intersection between two lines
osg::Vec3d lineLineInt(osg::Vec3d p11, osg::Vec3d p12, osg::Vec3d p21,
  osg::Vec3d p22)
{
	double d= p11[0]*(p22[1]-p21[1]) + p12[0]*(p21[1]-p22[1]) +
	  p21[0]*(p11[1]-p12[1]) + p22[0]*(p12[1]-p11[1]);
	if (d == 0) {//colinear or parallel
		return p11+p21;
	}
	double ns= p11[0]*(p22[1]-p21[1]) + p21[0]*(p11[1]-p22[1]) +
	  p22[0]*(p21[1]-p11[1]);
	double s= ns/d;
	double nt= -(p11[0]*(p21[1]-p12[1]) + p12[0]*(p11[1]-p21[1]) +
	  p21[0]*(p12[1]-p11[1]));
	double t= nt/d;
	return osg::Vec3d(p11[0] + s*(p12[0]-p11[0]),
	  p11[1] + s*(p12[1]-p11[1]),
	  p11[2] + s*(p12[2]-p11[2]));
}

void EdgeDrawable:: addCurvePoints(osg::Vec3d p1, osg::Vec3d p2, osg::Vec3f d1,
  osg::Vec3f d2)
{
	d1[2]= 0;
	d2[2]= 0;
	float dz= p2[2]-p1[2];
	osg::Vec3f up= d1^d2;
	up.normalize();
	osg::Vec3f dsn= up^d1;
	dsn.normalize();
	double ang= acos(d1*d2);
	osg::Vec3d pi= lineLineInt(p1,p1+d1,p2,p2+d2);
	double t1= (pi-p1).length();
	double t2= (pi-p2).length();
	double t;
	double slen;
	if (t1 > t2) {
		t= t2;
		slen= t1 - t;
	} else {
		t= t1;
		slen= t2 - t;
	}
	double r= t/tan(ang/2);
	double clen= r*ang;
	radius= r;
	if (t1 > t2) {
		p1+= d1*slen + up*(slen/(slen+clen)*dz);
		points.push_back(p1);
	}
	int m= (int)ceil(fabs(ang)*180/3.14159);
	ang/= m;
	double ddz= dz*clen/(slen+clen)/m;
	t= fabs(r*tan(ang/2));
	double h= 0;
	double cs= 1;
	double sn= 0;
	for (int i=1; i<m; i++) {
		p1+= d1*t*cs;
		p1+= dsn*t*sn;
		h+= ang;
		cs= cos(h);
		sn= sin(h);
		p1+= d1*t*cs;
		p1+= dsn*t*sn;
		p1+= up*ddz;
		points.push_back(p1);
	}
	if (t1 < t2) {
		p2-= d2*slen + up*(slen/(slen+clen)*dz);
		points.push_back(p2);
	}
}

void VertexDragger::moveTo(osg::Vec3d pos)
{
	if (elevation > 0)
		pos[2]= elevation;
	setPosition(pos);
}

void VertexDragger::moveTo(osg::Vec3d p, EdgeDrawable* e)
{
	osg::Vec3d p0= e->vertex1->getPosition();
	osg::Vec3d p2= e->vertex2->getPosition();
	double mind= 1e10;
	float dist= 0;
	for (list<osg::Vec3d>::const_iterator j=e->points.begin();
	  j!=e->points.end(); j++) {
		const osg::Vec3d& p1= *j;
		moveTo(p,p0,p1,mind,dist);
		dist+= (p1-p0).length();
		p0= p1;
	}
	moveTo(p,p0,p2,mind,dist);
}

void VertexDragger::moveTo(osg::Vec3d p, osg::Vec3d p1, osg::Vec3d p2,
  double& mind, float dist, bool onSegment)
{
	osg::Vec3d dp= p2-p1;
	double d= dp*dp;
	double n= dp*(p-p1);
	if (d==0)
		return;
	if (onSegment && (n<=0 || n>=d))
		return;
	dp*= n/d;
	d= (p1-p)*(p1-p);
	if (mind > d) {
		mind= d;
		moveTo(p1 + dp);
		dir= p2 - p1;
		dir.normalize();
		edgeDistance= dist+(getPosition()-p1).length();
	}
}

void TrackEditor::updateEdges()
{
	osg::Vec3f zero(0,0,0);
	for (int i=0; i<rootNode->getNumChildren(); i++) {
		VertexDragger* v= dynamic_cast<VertexDragger*>
		  (rootNode->getChild(i));
		if (v == NULL)
			continue;
		v->setDirection(zero);
		int ns= 0;
		for (EdgeList::iterator j=v->edges.begin(); j!=v->edges.end();
		  j++) {
			EdgeDrawable* e= *j;
			if (e->isStraight())
				ns++;
		}
		for (EdgeList::iterator j=v->edges.begin(); j!=v->edges.end();
		  j++) {
			EdgeDrawable* e= *j;
			if (ns==1 && !e->isStraight())
				continue;
			osg::Vec3d p1= e->vertex1->getPosition();
			osg::Vec3d p2= e->vertex2->getPosition();
			osg::Vec3f d= p2 - p1;
			d.normalize();
			if (v==(j==v->edges.begin() ? e->vertex1 : e->vertex2))
				v->dir-= d;
			else
				v->dir+= d;
		}
		v->dir.normalize();
		if (v->heading >= 0) {
			float a= (90-v->heading)*M_PI/180;
			float cs= cos(a);
			float sn= sin(a);
			float g= v->grade>-100 ? .01*v->grade : v->dir[2];
			float x= sqrt(1-g*g);
			v->dir= osg::Vec3f(x*cs,x*sn,g);
		} else if (v->grade > -100) {
			osg::Vec3f d= v->dir;
			d.normalize();
			float g= .01*v->grade;
			float x= sqrt(1-g*g);
			v->dir= osg::Vec3f(x*d[0],x*d[1],g);
		}
		osg::Vec3d p= v->getPosition();
//		fprintf(stderr,"%p %f %f %f %d %f %f %f\n",
//		  v,p[0],p[1],p[2],ns,v->dir[0],v->dir[1],v->dir[2]);
	}
	for (int i=0; i<edgeGeode->getNumDrawables(); i++) {
		EdgeDrawable* e= dynamic_cast<EdgeDrawable*>
		  (edgeGeode->getDrawable(i));
		if (e == NULL)
			continue;
		e->update();
	}
}

TrackEditor::TrackEditor(osg::Group* root)
{
	rootNode= new osg::Switch();
	root->addChild(rootNode);
	edgeGeode= new EdgeGeode();
	osg::StateSet* stateSet= edgeGeode->getOrCreateStateSet();
	stateSet->setRenderBinDetails(15,"RenderBin");
	osg::PositionAttitudeTransform* pat=
	  new osg::PositionAttitudeTransform;
	pat->addChild(edgeGeode);
	pat->setPosition(osg::Vec3d(0,0,0));
	rootNode->addChild(pat);
	editing= false;
	commandMode= false;
	profileGeode= NULL;
};

void TrackEditor::startEditing()
{
	Track::Location loc;
	findTrackLocation(currentPerson.location[0],currentPerson.location[1],
	  currentPerson.location[2],&loc);
	VertexDragger* vd1= new VertexDragger(loc.edge->v1->location.coord);
	VertexDragger* vd2= new VertexDragger(loc.edge->v2->location.coord);
	rootNode->addChild(vd1);
	rootNode->addChild(vd2);
	Track::Vertex* v= loc.edge->v1;
	Track::Edge* e= loc.edge;
	EdgeDrawable* ed= edgeGeode->addEdge(vd1,vd2);
	ed->type= e->curvature!=0 ? EdgeDrawable::CURVE :
	  EdgeDrawable::STRAIGHT;
	for (;;) {
		if (v->type == Track::VT_SWITCH)
			break;
		e= v->edge1==e ? v->edge2 : v->edge1;
		if (e == NULL)
			break;
		v= v==e->v1 ? e->v2 : e->v1;
		VertexDragger* vd= new VertexDragger(v->location.coord);
		rootNode->addChild(vd);
		ed= edgeGeode->addEdge(vd1,vd);
		ed->type= e->curvature!=0 ? EdgeDrawable::CURVE :
		  EdgeDrawable::STRAIGHT;
		vd1= vd;
	}
	v= loc.edge->v2;
	e= loc.edge;
	for (;;) {
		if (v->type == Track::VT_SWITCH)
			break;
		e= v->edge1==e ? v->edge2 : v->edge1;
		if (e == NULL)
			break;
		v= v==e->v1 ? e->v2 : e->v1;
		VertexDragger* vd= new VertexDragger(v->location.coord);
		rootNode->addChild(vd);
		ed= edgeGeode->addEdge(vd2,vd);
		ed->type= e->curvature!=0 ? EdgeDrawable::CURVE :
		  EdgeDrawable::STRAIGHT;
		vd2= vd;
	}
	editing= true;
	rootNode->setAllChildrenOn();
}

void TrackEditor::stopEditing()
{
	editing= false;
	rootNode->setAllChildrenOff();
}

osg::Vec3d TrackEditor::mouseHitCoord(osgViewer::Viewer* viewer,
  float x, float y, osg::Vec3d dflt)
{
	if (viewer == NULL)
		return dflt;
	typedef osgUtil::LineSegmentIntersector::Intersections Hits;
	Hits hits;
	if (!viewer->computeIntersections(x,y,hits,~3))
		return dflt;
	double bestr= 1;
	Hits::iterator bestHit= hits.end();
	for (Hits::iterator hit=hits.begin(); hit!=hits.end(); ++hit) {
		if (bestr > hit->ratio) {
			bestr= hit->ratio;
			bestHit= hit;
		}
	}
	if (bestHit == hits.end())
		return dflt;
	return bestHit->getWorldIntersectPoint();
}

VertexDragger* TrackEditor::findSelection(osgViewer::Viewer* viewer,
  float x, float y, bool add)
{
	if (viewer == NULL)
		return NULL;
//	cerr<<"find "<<x<<" "<<y<<" "<<add<<std::endl;
	typedef osgUtil::LineSegmentIntersector::Intersections Hits;
	Hits hits;
	if (!viewer->computeIntersections(x,y,hits,~3))
		return NULL;
	double bestr= 1;
	Hits::iterator bestHit= hits.end();
	for (Hits::iterator hit=hits.begin(); hit!=hits.end(); ++hit) {
		if (bestr > hit->ratio) {
			bestr= hit->ratio;
			bestHit= hit;
		}
	}
	if (bestHit == hits.end())
		return NULL;
	if (selectedE.valid()) {
		selectedE->selected= false;
		selectedE= NULL;
	}
	for (int k=0; k<bestHit->nodePath.size(); k++) {
		VertexDragger* vd= dynamic_cast<VertexDragger*>
		  (bestHit->nodePath[k]);
		if (vd != NULL)
			return vd;
	}
	osg::Vec3d point= bestHit->getWorldIntersectPoint();
	VertexDragger* vd= NULL;
	if (add) {
		vd= new VertexDragger(point);
		rootNode->addChild(vd);
		rootNode->setAllChildrenOn();
	} else {
		EdgeDrawable* e= closestEdge(point,NULL);
		if (e != NULL) {
			e->selected= true;
			selectedE= e;
			command= e->toString();
		}
	}
	return vd;
}

void TrackEditor::alignStraight(VertexDragger* vd)
{
	vector<EdgeDrawable*> straights;
	for (EdgeList::iterator i= vd->edges.begin(); i!=vd->edges.end(); i++) {
		EdgeDrawable* e= *i;
		if (e->type == EdgeDrawable::STRAIGHT)
			straights.push_back(e);
	}
	if (straights.size() == 1) {
		EdgeDrawable* e1= straights[0];
		VertexDragger* v1= e1->otherEnd(vd);
		straights.clear();
		for (EdgeList::iterator i= v1->edges.begin();
		  i!=v1->edges.end(); i++) {
			EdgeDrawable* e= *i;
			if (e!=e1 && e->type==EdgeDrawable::STRAIGHT)
				straights.push_back(e);
		}
		if (straights.size() >= 1) {
			EdgeDrawable* e2= straights[0];
			VertexDragger* v2= e2->otherEnd(v1);
			osg::Vec3d p1= v1->getPosition();
			osg::Vec3d p2= v2->getPosition();
			double mind= 1e10;
			vd->moveTo(vd->getPosition(),p1,p2,mind,0,false);
		}
	} else if (straights.size() >= 2) {
		EdgeDrawable* e1= straights[0];
		EdgeDrawable* e2= straights[1];
		VertexDragger* v1= e1->otherEnd(vd);
		VertexDragger* v2= e2->otherEnd(vd);
		osg::Vec3d p1= v1->getPosition();
		osg::Vec3d p2= v2->getPosition();
		double mind= 1e10;
		vd->moveTo(vd->getPosition(),p1,p2,mind,0);
	}
}

void TrackEditor::moveAway(VertexDragger* vd, float dist)
{
	osg::Vec3d pos= vd->getPosition();
	for (int i=0; i<edgeGeode->getNumDrawables(); i++) {
		EdgeDrawable* e= dynamic_cast<EdgeDrawable*>
		  (edgeGeode->getDrawable(i));
		if (e == NULL)
			continue;
		if (e->vertex1==vd || e->vertex2==vd)
			continue;
		osg::Vec3d p0= e->vertex1->getPosition();
		osg::Vec3d p2= e->vertex2->getPosition();
		for (list<osg::Vec3d>::const_iterator j=e->points.begin();
		  j!=e->points.end(); j++) {
			const osg::Vec3d p1= *j;
			movePointAway(pos,dist,p0,p1);
			p0= p1;
		}
		movePointAway(pos,dist,p0,p2);
	}
	vd->setPosition(pos);
}

EdgeDrawable* TrackEditor::closestEdge(osg::Vec3d point, VertexDragger* avoid)
{
	EdgeDrawable* bestE= NULL;
	double bestD= 1e30;
	for (int i=0; i<edgeGeode->getNumDrawables(); i++) {
		EdgeDrawable* e= dynamic_cast<EdgeDrawable*>
		  (edgeGeode->getDrawable(i));
		if (e == NULL)
			continue;
		if (e->vertex1==avoid || e->vertex2==avoid)
			continue;
		osg::Vec3d p0= e->vertex1->getPosition();
		osg::Vec3d p2= e->vertex2->getPosition();
		double mind= 1e10;
		for (list<osg::Vec3d>::const_iterator j=e->points.begin();
		  j!=e->points.end(); j++) {
			const osg::Vec3d& p1= *j;
			double d= lineSegDistSq(point,p0,p1);
			if (d>=0 && d<bestD) {
				bestD= d;
				bestE= e;
			}
			p0= p1;
		}
		double d= lineSegDistSq(point,p0,p2);
		if (d>=0 && d<bestD) {
			bestD= d;
			bestE= e;
		}
	}
	return bestE;
}

void TrackEditor::clearSelected()
{
	if (selectedV.valid()) {
		selectedV->setColor(osg::Vec4f(0,1,1,1));
		selectedV= NULL;
	}
	if (selectedE.valid()) {
		selectedE->selected= false;
		selectedE= NULL;
	}
}

void TrackEditor::setSelected(VertexDragger* v)
{
	clearSelected();
	v->setColor(osg::Vec4f(1,1,0,1));
	selectedV= v;
}

bool TrackEditor::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled() || !editing)
		return false;
	if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
		if ((ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_SHIFT)) {
			currentPerson.setLocation(mouseHitCoord(
			  dynamic_cast<osgViewer::Viewer*>(&aa),
			  ea.getX(),ea.getY(),
			  currentPerson.location),NULL,osg::Vec3d(0,0,0));
			return true;
		}
//		cerr<<"press "<<ea.getButton()<<" "<<ea.getModKeyMask()<<endl;
		bool add= (ea.getButton()&osgGA::GUIEventAdapter::
		  RIGHT_MOUSE_BUTTON)!=0 ||
		  (ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL)!=0;
		VertexDragger* vd= findSelection(
		  dynamic_cast<osgViewer::Viewer*>(&aa),ea.getX(),ea.getY(),
		  add);
		dragging= vd;
		if (vd)
			vd->setNodeMask(2);
		if (selectedV.valid()) {
			selectedV->setColor(osg::Vec4f(0,1,1,1));
			if (vd!=NULL && selectedV!=vd && add) {
				edgeGeode->addEdge(selectedV,vd);
			}
		}
		if (selectedV == vd) {
			selectedV= NULL;
		} else {
			selectedV= vd;
			if (vd != NULL) {
				vd->setColor(osg::Vec4f(1,1,0,1));
				command= selectedV->toString();
			}
		}
	} else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE) {
		updateEdges();
		if (dragging.valid())
			dragging->setNodeMask(4);
		dragging= NULL;
	} else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
		if (dragging.valid()) {
			dragging->moveTo(mouseHitCoord(
			  dynamic_cast<osgViewer::Viewer*>(&aa),
			  ea.getX(),ea.getY(),dragging->getPosition()));
		}
	} else if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
		if (commandMode) {
			switch (ea.getKey()) {
			 case osgGA::GUIEventAdapter::KEY_Return:
				{
				fprintf(stderr,"handle %s\n",command.c_str());
				string r= handleCommand(command.c_str());
				fprintf(stderr,"returned %s\n",r.c_str());
				commandMode= false;
				command= r;
				}
				return true;
			 case osgGA::GUIEventAdapter::KEY_Delete:
			 case osgGA::GUIEventAdapter::KEY_BackSpace:
				if (command.size() > 0)
					command.resize(command.size()-1);
				return true;
			 default:
				if (ea.getKey()>=' ' && ea.getKey()<0x7f) {
					command+= (char)ea.getKey();
					return true;
				}
			}
		} else {
			command.clear();
		}
		switch (ea.getKey()) {
		 case osgGA::GUIEventAdapter::KEY_Delete:
		 case osgGA::GUIEventAdapter::KEY_BackSpace:
			if (selectedV.valid()) {
				rootNode->removeChild(selectedV.get());
				for (EdgeList::iterator i=
				  selectedV->edges.begin();
				  i!= selectedV->edges.end(); i++)
					edgeGeode->removeEdge(*i);
			}
			if (selectedE.valid())
				edgeGeode->removeEdge(selectedE.get());
			clearSelected();
			return true;
		 case 'o':
			if (selectedV.valid()) {
				moveAway(selectedV,15*.3048);
				updateEdges();
				command= selectedV->toString();
			}
			return true;
		 case 's':
			if (selectedV.valid()) {
				//addSwitch(selectedV);
				updateEdges();
			} else if (selectedE.valid()) {
				selectedE->type= EdgeDrawable::STRAIGHT;
				updateEdges();
				command= selectedE->toString();
				//clearSelected();
			}
			return true;
		 case 'c':
			if (selectedE.valid()) {
				selectedE->type= EdgeDrawable::CURVE;
				updateEdges();
				command= selectedE->toString();
			}
			//clearSelected();
			return true;
		 case 'l':
			if (selectedV.valid()) {
				alignStraight(selectedV);
				updateEdges();
			}
			return true;
		 case 'p':
			if (profileGeode) {
				rootNode->removeChild(profileGeode);
				profileGeode= NULL;
			} else {
				makeProfile();
			}
			return true;
		 case 'q':
			stopEditing();
			return true;
		 case '!':
			commandMode= true;
			return true;
		 default:
			break;
		}
	}
	return false;
}

//	multi character command parser
//	commands from network or typed by user
string TrackEditor::handleCommand(const char* cmd)
{
	string result;
	Parser parser;
	parser.setDelimiters("|");
	parser.setCommand(cmd);
	try {
		if (parser.getString(0)=="quit") {
			stopEditing();
		} else if (parser.getString(0)=="e" && selectedV.valid()) {
			selectedV->elevation= parser.getDouble(1,1,1e5,
			  selectedV->elevation);
		} else if (parser.getString(0)=="h" && selectedV.valid()) {
			selectedV->heading= parser.getDouble(1,0,360,
			  selectedV->heading);
		} else if (parser.getString(0)=="g" && selectedV.valid()) {
			selectedV->grade= parser.getDouble(1,-10,10,
			  selectedV->grade);
		} else {
			result= "?";
		}
	} catch (const char* msg) {
		result= msg;
	} catch (const std::exception& error) {
		result= error.what();
	}
	return result;
}

void TrackEditor::makeProfile()
{
	double minZ= 1e10;
	double maxZ= -1e10;
	for (int i=0; i<rootNode->getNumChildren(); i++) {
		VertexDragger* v= dynamic_cast<VertexDragger*>
		  (rootNode->getChild(i));
		if (v == NULL)
			continue;
		osg::Vec3d p= v->getPosition();
		if (minZ > p[2])
			minZ= p[2];
		if (maxZ < p[2])
			maxZ= p[2];
	}
	minZ-= 5;
	maxZ+= 5;
	int nvi= 0;
	osg::Vec3Array* verts= new osg::Vec3Array;
	for (int i=0; i<edgeGeode->getNumDrawables(); i++) {
		EdgeDrawable* e= dynamic_cast<EdgeDrawable*>
		  (edgeGeode->getDrawable(i));
		if (e == NULL)
			continue;
		osg::Vec3d p1= e->vertex1->getPosition();
		for (list<osg::Vec3d>::const_iterator i=e->points.begin();
		  i!=e->points.end(); i++) {
			osg::Vec3d p2= *i;
			verts->push_back(osg::Vec3(p1[0],p1[1],minZ));
			verts->push_back(osg::Vec3(p2[0],p2[1],minZ));
			verts->push_back(osg::Vec3(p1[0],p1[1],maxZ));
			verts->push_back(osg::Vec3(p1[0],p1[1],maxZ));
			verts->push_back(osg::Vec3(p2[0],p2[1],minZ));
			verts->push_back(osg::Vec3(p2[0],p2[1],maxZ));
			nvi+= 6;
			p1= p2;
		}
		osg::Vec3d p2= e->vertex2->getPosition();
		verts->push_back(osg::Vec3(p1[0],p1[1],minZ));
		verts->push_back(osg::Vec3(p2[0],p2[1],minZ));
		verts->push_back(osg::Vec3(p1[0],p1[1],maxZ));
		verts->push_back(osg::Vec3(p1[0],p1[1],maxZ));
		verts->push_back(osg::Vec3(p2[0],p2[1],minZ));
		verts->push_back(osg::Vec3(p2[0],p2[1],maxZ));
		nvi+= 6;
	}
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(.9,.9,.9,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(new osg::DrawArrays(
	  osg::PrimitiveSet::TRIANGLES,0,nvi));
	osg::Geode* geode= new osg::Geode;
	geode->addDrawable(geometry);
	osg::StateSet* stateSet= geode->getOrCreateStateSet();
	stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	stateSet->setAttributeAndModes(
	  new osg::CullFace(osg::CullFace::FRONT_AND_BACK),
	  osg::StateAttribute::OFF);
	rootNode->addChild(geode);
	profileGeode= geode;
}
