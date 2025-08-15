//	track data structure
//
//	multiple Track structures may be used to model moving track
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
#include "rmsim.h"
#include "changelog.h"

#include <osg/Switch>
#include <osg/Billboard>
#include <osgText/Text>

TrackMap trackMap;

Track::Track()
{
	vQueue= NULL;
	shape= NULL;
	geode= NULL;
	matrix= NULL;
	updateSignals= false;
}

Track::~Track()
{
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i)
		delete *i;
	for (EdgeList::iterator i=edgeList.begin(); i!=edgeList.end(); ++i)
		delete *i;
	if (vQueue != NULL)
		free(vQueue);
	if (matrix != NULL)
		delete matrix;
}

Track::Vertex* Track::addVertex(int type, double x, double y, float z)
{
//	fprintf(stderr,"addvertex %d %f %f %f\n",type,x,y,z);
	Vertex* v;
	switch (type) {
	 case VT_SIMPLE:
		v= new Vertex;
		break;
	 case VT_SWITCH:
		v= (Vertex*) new SwVertex;
		break;
	 default:
		throw "unknown vertex type";
	}
	v->type= type;
	v->location.coord[0]= x;
	v->location.coord[1]= y;
	v->location.coord[2]= z;
	v->location.up[0]= 0;
	v->location.up[1]= 0;
	v->location.up[2]= 1;
	v->edge1= NULL;
	v->edge2= NULL;
	v->occupied= 0;
	v->grade= 0;
	v->elevation= z;
	vertexList.push_back(v);
	return v;
}

void Track::Vertex::saveEdge(int n, Edge* e)
{
//	fprintf(stderr,"saveedge %p %d %p\n",this,n,e);
	switch (n) {
	 case 0:
		edge1= e;
		break;
	 case 1:
		if (type == VT_SWITCH) {
			SwVertex* sv= (SwVertex*) this;
			sv->swEdges[0]= e;
			if (sv->mainEdge == 0)
				edge2= e;
		} else {
			edge2= e;
		}
		break;
	 case 2:
		if (type == VT_SWITCH) {
			SwVertex* sv= (SwVertex*) this;
			sv->swEdges[1]= e;
			if (sv->mainEdge == 1)
				edge2= e;
		}
		break;
	}
}

Track::Edge* Track::addEdge(int type, Vertex* v1, int n1, Vertex* v2, int n2)
{
//	fprintf(stderr,"addEdge %.2f %.2f %.2f %d %.2f %.2f %.2f %d\n",
//	  v1->location.coord[0],v1->location.coord[1],v1->location.coord[2],n1,
//	  v2->location.coord[0],v2->location.coord[1],v2->location.coord[2],n2);
	Edge* e;
	switch (type) {
	 case ET_STRAIGHT:
		e= new Edge;
		break;
	 case ET_SPLINE:
		e= (Edge*) new SplineEdge;
		break;
	 default:
		throw "unknown edge type";
	}
	e->type= type;
	e->track= this;
	e->v1= v1;
	e->v2= v2;
	e->ssEdge= NULL;
	v1->saveEdge(n1,e);
	v2->saveEdge(n2,e);
	edgeList.push_back(e);
	double dx= v1->location.coord[0] - v2->location.coord[0];
	double dy= v1->location.coord[1] - v2->location.coord[1];
	double dz= v1->location.coord[2] - v2->location.coord[2];
	e->length= sqrt(dx*dx+dy*dy+dz*dz);
	if (e->length < .001)
		e->length= .001;
	e->occupied= 0;
	e->curvature= 0;
}

void Track::Edge::updateSignals()
{
//	fprintf(stderr,"updatesignals %p %d\n",this,occupied);
	int flags= 0;
	for (SignalList::iterator i=signals.begin(); i!=signals.end(); i++) {
		Signal* s= *i;
		int bit= s->getTrack(0).rev ? 1 : 2;
//		fprintf(stderr,"updatesig %p %d %d\n",s,flags,bit);
		if ((flags&bit) == 0) {
			s->update();
			flags|= bit;
		}
	}
	Signal* s= findSignal(v1,this);
	if (s)
		s->update();
	s= findSignal(v2,this);
	if (s)
		s->update();
}

void Track::calcMinMax()
{
	minVertexX= 1e20;
	maxVertexX= -1e20;
	minVertexY= 1e20;
	maxVertexY= -1e20;
	minVertexZ= 1e20;
	maxVertexZ= -1e20;
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		if (minVertexX > v->location.coord[0])
			minVertexX= v->location.coord[0];
		if (maxVertexX < v->location.coord[0])
			maxVertexX= v->location.coord[0];
		if (minVertexY > v->location.coord[1])
			minVertexY= v->location.coord[1];
		if (maxVertexY < v->location.coord[1])
			maxVertexY= v->location.coord[1];
		if (minVertexZ > v->location.coord[2])
			minVertexZ= v->location.coord[2];
		if (maxVertexZ < v->location.coord[2])
			maxVertexZ= v->location.coord[2];
	}
}

//	finds the nearest track location to the given coordinates
float Track::findLocation(double x, double y, double z, Track::Location *loc)
{
	float bestd= 1e30;
	for (EdgeList::iterator i=edgeList.begin();
	  i!=edgeList.end(); ++i) {
		Edge* e= *i;
		double dx= e->v2->location.coord[0] - e->v1->location.coord[0];
		double dy= e->v2->location.coord[1] - e->v1->location.coord[1];
		double dz= e->v2->location.coord[2] - e->v1->location.coord[2];
		double d= dx*dx + dy*dy + dz*dz;
		double n= dx*(x-e->v1->location.coord[0]) +
		  dy*(y-e->v1->location.coord[1]) +
		  dz*(z-e->v1->location.coord[2]);
		if (d==0 || n<=0) {
			dx= e->v1->location.coord[0]-x;
			dy= e->v1->location.coord[1]-y;
			dz= e->v1->location.coord[2]-z;
			n= 0;
		} else if (n >= d) {
			dx= e->v2->location.coord[0]-x;
			dy= e->v2->location.coord[1]-y;
			dz= e->v2->location.coord[2]-z;
			n= e->length;
		} else {
			dx= e->v1->location.coord[0] + dx*n/d - x;
			dy= e->v1->location.coord[1] + dy*n/d - y;
			dz= e->v1->location.coord[2] + dz*n/d - z;
			if (e->type == ET_SPLINE) {
				Track::SplineEdge* sp= (Track::SplineEdge*) e;
				float a= n/d;
				float b= 1-a;
				float a3= a*a*a-a;
				float b3= b*b*b-b;
				dx+= (b3*sp->dd1[0] + a3*sp->dd2[0]) *
				  sp->splineMult;
				dy+= (b3*sp->dd1[1] + a3*sp->dd2[1]) *
				  sp->splineMult;
				dz+= (b3*sp->dd1[2] + a3*sp->dd2[2]) *
				  sp->splineMult;
			}
			n= e->length*n/d;
		}
		d= dx*dx + dy*dy + dz*dz;
		if (bestd > d) {
			bestd= d;
			loc->edge= e;
			loc->offset= n;
			loc->rev= 0;
		}
	}
	return bestd;
}

//	finds the nearest track location to the given coordinates
float Track::findLocation(double x, double y, Track::Location *loc)
{
	float bestd= 1e30;
	for (EdgeList::iterator i=edgeList.begin();
	  i!=edgeList.end(); ++i) {
		Edge* e= *i;
		double dx= e->v2->location.coord[0] - e->v1->location.coord[0];
		double dy= e->v2->location.coord[1] - e->v1->location.coord[1];
		double d= dx*dx + dy*dy;
		double n= dx*(x-e->v1->location.coord[0]) +
		  dy*(y-e->v1->location.coord[1]);
		if (d==0 || n<=0) {
			dx= e->v1->location.coord[0]-x;
			dy= e->v1->location.coord[1]-y;
			n= 0;
		} else if (n >= d) {
			dx= e->v2->location.coord[0]-x;
			dy= e->v2->location.coord[1]-y;
			n= e->length;
		} else {
			dx= e->v1->location.coord[0] + dx*n/d - x;
			dy= e->v1->location.coord[1] + dy*n/d - y;
			if (e->type == ET_SPLINE) {
				Track::SplineEdge* sp= (Track::SplineEdge*) e;
				float a= n/d;
				float b= 1-a;
				float a3= a*a*a-a;
				float b3= b*b*b-b;
				dx+= (b3*sp->dd1[0] + a3*sp->dd2[0]) *
				  sp->splineMult;
				dy+= (b3*sp->dd1[1] + a3*sp->dd2[1]) *
				  sp->splineMult;
			}
			n= e->length*n/d;
		}
		d= dx*dx + dy*dy;
		if (bestd > d) {
			bestd= d;
			loc->edge= e;
			loc->offset= n;
			loc->rev= 0;
		}
	}
	return bestd;
}

//	finds the switch nearest to the given coordinates
Track::SwVertex* Track::findSwitch(double x, double y, double z)
{
	double bestd= 1000;
	SwVertex* bestsw= NULL;
	for (VertexList::iterator i=vertexList.begin();
	  i!=vertexList.end(); ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		WLocation loc= v->location;
		if (matrix != NULL)
			loc.coord= matrix->preMult(loc.coord);
		double dx= loc.coord[0] - x;
		double dy= loc.coord[1] - y;
		double dz= loc.coord[2] - z;
		double d= dx*dx + dy*dy + dz*dz;
		if (bestd > d) {
			bestd= d;
			bestsw= (SwVertex*) v;
		}
	}
//	fprintf(stderr,"throw distsq %f\n",bestd);
	return bestsw;
}

int Track::throwSwitch(double x, double y, double z)
{
	SwVertex* sw= findSwitch(x,y,z);
	if (sw != NULL) {
		sw->throwSwitch(NULL,false);
		return 1;
	} else {
		return 0;
	}
}

int Track::lockSwitch(double x, double y, double z)
{
	SwVertex* sw= findSwitch(x,y,z);
	if (sw != NULL) {
		sw->locked= true;
		return 1;
	} else {
		return 0;
	}
}

//	converts a track location into world coordinates
//	if !local adjusts the result to handle moving track (on ships)
void Track::Location::getWLocation(WLocation* loc, int local, bool useElevation)
{
	float a= offset/edge->length;
	float b= 1-a;
	WLocation* l1= &edge->v1->location;
	WLocation* l2= &edge->v2->location;
	for (int i=0; i<3; i++) {
		loc->coord[i]= b*l1->coord[i] + a*l2->coord[i];
		loc->up[i]= b*l1->up[i] + a*l2->up[i];
	}
	if (useElevation)
		loc->coord[2]= b*edge->v1->elevation + a*edge->v2->elevation;
	if (edge->type == ET_SPLINE) {
		Track::SplineEdge* sp= (Track::SplineEdge*) edge;
		float a3= a*a*a-a;
		float b3= b*b*b-b;
		for (int i=0; i<3; i++)
			loc->coord[i]+= (b3*sp->dd1[i] + a3*sp->dd2[i]) *
			  sp->splineMult;
	}
	if (!local && edge->track->matrix!=NULL) {
		osg::Matrixd& m= *(edge->track->matrix);
		loc->coord= m.preMult(loc->coord);
//		loc->up= edge->track->matrix->preMult(loc->up);
		osg::Vec3f& u= loc->up;
		float ux= m(0,0)*u[0] + m(1,0)*u[1] + m(2,0)*u[2];
		float uy= m(0,1)*u[0] + m(1,1)*u[1] + m(2,1)*u[2];
		float uz= m(0,2)*u[0] + m(1,2)*u[1] + m(2,2)*u[2];
		u[0]= ux;
		u[1]= uy;
		u[2]= uz;
	}
}

float Track::Location::grade()
{
	if (edge->track->matrix != NULL) {
		osg::Vec3d p1=
		  edge->track->matrix->preMult(edge->v1->location.coord);
		osg::Vec3d p2=
		  edge->track->matrix->preMult(edge->v2->location.coord);
		float g= (p1[2]-p2[2])/edge->length;
		return rev ? -g : g;
	}
#if 1
#if 0
	float g= (edge->v1->location.coord[2]-edge->v2->location.coord[2])/
	  edge->length;
#else
	float g= (edge->v1->elevation-edge->v2->elevation)/edge->length;
#endif
#else
	float a= offset/edge->length;
	float b= 1-a;
	float g= b*edge->v1->grade + a*edge->v2->grade;
	g= -g;
#endif
	return rev ? -g : g;
}

//	moves a track location the specified distance
//	throws switches as needed if selected
//	adjusts the occupied count by dOccupied
int Track::Location::move(float dist, int throwSwitches, int dOccupied)
{
	int reverse= dist<0;
	if (reverse) {
		dist= -dist;
		dOccupied= -dOccupied;
	}
	for (;;) {
		float max= reverse==this->rev ? edge->length-offset : offset;
		if (dist <= max) {
			if (reverse == this->rev)
				offset+= dist;
			else
				offset-= dist;
			return 0;
		}
		dist-= max;
		Vertex* v= reverse==this->rev ? edge->v2 : edge->v1;
		if (dOccupied < 0) {
			v->occupied--;
			edge->occupied--;
			if (edge->occupied==0 && edge->track->updateSignals &&
			  edge->signals.size()>0)
				edge->updateSignals();
#if 0
//			fprintf(stderr,"%d %d-%d %d %d %d %d\n",
//			  v->id,edge->v1->id,edge->v2->id,
			fprintf(stderr," %d %d %d %d\n",
			  edge->occupied,dOccupied,
			  edge->track->updateSignals,edge->signals.size());
#endif
		}
		if (edge != v->edge1) {
			if (v->type==VT_SWITCH && throwSwitches)
				((SwVertex*)v)->throwSwitch(edge,false);
			edge= v->edge1;
		} else if (v->edge2 == NULL) {
			if (v == edge->v1)
				offset= 0;
			else
				offset= edge->length;
			return 1;
		} else {
			edge= v->edge2;
		}
		if (dOccupied > 0) {
			v->occupied++;
			edge->occupied++;
			if (edge->occupied>0 && edge->track->updateSignals &&
			  edge->signals.size()>0)
				edge->updateSignals();
#if 0
//			fprintf(stderr,"%d %d-%d %d %d %d %d\n",
//			  v->id,edge->v1->id,edge->v2->id,
			fprintf(stderr," %d %d %d %d\n",
			  edge->occupied,dOccupied,
			  edge->track->updateSignals,edge->signals.size());
#endif
		}
		if (v == edge->v1) {
			offset= 0;
			this->rev= reverse;
		} else {
			offset= edge->length;
			this->rev= !reverse;
//			if (throwSwitches)
//				edge->v1->throwSwitch(edge,false);
		}
	}
}

//	calculates the distance between two locations on the same edge
float Track::Location::distance(Track::Location* other)
{
	if (edge != other->edge)
		return 1e30;
	if (offset > other->offset)
		return offset - other->offset;
	return other->offset - offset;
}

//	calculates the distance between two locations on no matter
//	what edge they are on
float Track::Location::dDistance(Track::Location* other)
{
//	if (edge == other->edge)
//		fprintf(stderr,"samee %d %f %d %f %f\n",
//		  rev,offset,other->rev,other->offset,edge->length);
	if (edge == other->edge)
		return rev ? offset-other->offset : other->offset-offset;
	float d= offset;
	Vertex* v= edge->v1;
	Edge* e= edge;
	for (;;) {
//		if (v->type == VT_SWITCH)
//			break;
		e= v->edge1==e ? v->edge2 : v->edge1;
		if (e == NULL)
			break;
		if (e == other->edge) {
			d+= v==e->v1 ? other->offset : e->length-other->offset;
//			fprintf(stderr,"v1 %d %f\n",rev,d);
			return rev ? d : -d;
		}
		d+= e->length;
		v= v==e->v1 ? e->v2 : e->v1;
	}
	d= edge->length-offset;
	v= edge->v2;
	e= edge;
	for (;;) {
//		if (v->type == VT_SWITCH)
//			break;
		e= v->edge1==e ? v->edge2 : v->edge1;
		if (e == NULL)
			break;
		if (e == other->edge) {
			d+= v==e->v1 ? other->offset : e->length-other->offset;
//			fprintf(stderr,"v2 %d %f\n",rev,d);
			return rev ? -d : d;
		}
		d+= e->length;
		v= v==e->v1 ? e->v2 : e->v1;
	}
	return 1e30;
}

//	calculates the distance between location and end of track
float Track::Location::maxDistance(bool behind, float alignTol)
{
	if (rev)
		behind= !behind;
	float d= behind ? offset : edge->length-offset;
	Vertex* v= behind ? edge->v1 : edge->v2;
	Edge* e= edge;
	for (;;) {
		Edge* pe= e;
		e= v->edge1==e ? v->edge2 : v->edge1;
		if (e == NULL)
			break;
//		if (alignTol>=0 && v->type==VT_SWITCH)
//			fprintf(stderr,"%f %p %p %p %p\n",
//			  d,e,pe,v->edge1,v->edge2);
		if (alignTol>=0 && v->type==VT_SWITCH &&
		  e==v->edge1 && pe!=v->edge2) {
			if (d > alignTol)
				return d-alignTol/2;
			((SwVertex*)v)->throwSwitch(pe,false);
		}
		d+= e->length;
		v= v==e->v1 ? e->v2 : e->v1;
	}
	return d;
}

//	calculates the distance between location and targetV.
float Track::Location::vDistance(Track::Vertex* targetV, bool behind,
  bool* facing)
{
	if (rev)
		behind= !behind;
	float d= behind ? offset : edge->length-offset;
	Vertex* v= behind ? edge->v1 : edge->v2;
	Edge* e= edge;
	for (;;) {
		if (v == targetV) {
			if (facing != NULL)
				*facing= e==v->edge1;
			return d;
		}
		Edge* pe= e;
		e= v->edge1==e ? v->edge2 : v->edge1;
		if (e == NULL)
			break;
		d+= e->length;
		v= v==e->v1 ? e->v2 : e->v1;
	}
	return -1;
}

//	throws a switch to the selected edge
//	with override interlocking if force
void Track::SwVertex::throwSwitch(Track::Edge* edge, bool force)
{
	if (occupied>0 || edge1==edge || edge2==edge)
		return;
	if (hasInterlocking && !force)
		return;
	edge2= edge2==swEdges[0] ? swEdges[1] : swEdges[0];
	ChangeLog::instance()->addThrow(this);
//	fprintf(stderr,"throw0 %p %p %p %p %f %f %f\n",
//	  this,edge2,swEdges[0],swEdges[1],
//	  location.coord[0],location.coord[1],location.coord[2]);
	if (edge1->track->updateSignals) {
		Signal* s= findSignal(edge1->otherV(this),edge1);
//		fprintf(stderr,"swe0 %p %d\n",s,mainEdge);
		if (s)
			s->update();
		for (int i=0; i<2; i++) {
			s= findSignal(swEdges[i]->otherV(this),swEdges[i]);
//			fprintf(stderr,"swe %d %p\n",i+1,s);
			if (s)
				s->update();
		}
	}
}

void Track::translate(double dx, double dy, double dz)
{
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		v->location.coord[0]+= dx;
		v->location.coord[1]+= dy;
		v->location.coord[2]+= dz;
	}
}

void Track::rotate(double angle)
{
	double cs= cos(angle*M_PI/180);
	double sn= sin(angle*M_PI/180);
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		double x= cs*v->location.coord[0] - sn*v->location.coord[1];
		double y= sn*v->location.coord[0] + cs*v->location.coord[1];
		v->location.coord[0]= x;
		v->location.coord[1]= y;
	}
	for (Track::EdgeList::iterator i=edgeList.begin();
	  i!=edgeList.end(); ++i) {
		Track::Edge* e= *i;
		if (e->curvature>0 && e->type==ET_SPLINE) {
			SplineEdge* se= (SplineEdge*)e;
			float r= 1746.4/e->curvature;
			float a= se->angle;
			se->setCircle(r,a);
		}
	}
}

//	finds the shortest path from startLocation to any place reachable
//	without changing direction
//	if path is defined reachablility is only propagated through
//	switches on the path
int Track::findSPT(Track::Location& startLocation, bool bothDirections,
  Path* path)
{
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		v->dist= 1e30;
		v->inEdge= NULL;
	}
	if (path != NULL) {
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			if (v->type == VT_SWITCH)
				v->dist= -1;
		}
		for (Path::Node* p=path->firstNode; p!=NULL; p=p->next) {
			if (p->sw != NULL)
				p->sw->dist= 1e30;
			for (Path::Node* p1=p->nextSiding; p1!=NULL;
			  p1=p1->nextSiding)
				if (p1->sw != NULL)
					p1->sw->dist= 1e30;
		}
	}
	if (vQueue == NULL)
		vQueue= (Vertex**) malloc(vertexList.size()*sizeof(Vertex**));
	Edge* e= startLocation.edge;
	int vQueueSize= 0;
	e->v1->dist= startLocation.offset;
	e->v1->inEdge= e;
	e->v2->dist= e->length-startLocation.offset;
	e->v2->inEdge= e;
	if (bothDirections || startLocation.rev)
		vQueue[vQueueSize++]= e->v1;
	if (bothDirections || !startLocation.rev)
		vQueue[vQueueSize++]= e->v2;
	while (vQueueSize > 0) {
		int besti= 0;
		for (int i=1; i<vQueueSize; i++)
			if (vQueue[besti]->dist > vQueue[i]->dist)
				besti= i;
		Vertex* v= vQueue[besti];
		vQueue[besti]= vQueue[--vQueueSize];
//		if (!bothDirections)
//		fprintf(stderr,"%p %d %f\n",
//		  v,v->inEdge,v->dist);
		for (int i=0; i<2; i++) {
			Edge* e= NULL;
			if (v->type==VT_SWITCH && v->inEdge==v->edge1 &&
			  (v->dist>1e3 || ((SwVertex*)v)->hasInterlocking==0))
				e= ((SwVertex*)v)->swEdges[i];
			else if (i > 0)
				break;
			else if (v->type==VT_SWITCH && v->inEdge!=v->edge1)
				e= v->edge1;
			else
				e= v->nextEdge(v->inEdge);
			if (e == NULL)
				break;
//			if (!bothDirections)
//			fprintf(stderr," %d %p-%p %f\n",i,
//			  e->v1,e->v2,e->length);
			Vertex* v2= v==e->v1 ? e->v2 : e->v1;
			float d= v->dist + e->length;
			if (v2->dist > d) {
				v2->dist= d;
				if (v2->inEdge == NULL)
					vQueue[vQueueSize++]= v2;
				v2->inEdge= e;
			}
		}
	}
	if (path != NULL) {
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			if (v->dist < 0)
				v->dist= 1e30;
		}
	}
}

//	finds the shortest path from startLocation to any place
//	chgPenalty is a penalty for changing direction
int Track::findSPT(Track::Location& startLocation, float chgPenalty,
  float occupiedPenalty, Track::Vertex* avoid)
{
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		v->dist= 1e30;
		v->inEdge= NULL;
	}
	if (vQueue == NULL)
		vQueue= (Vertex**) malloc(vertexList.size()*sizeof(Vertex**));
	Edge* e= startLocation.edge;
	int vQueueSize= 0;
	e->v1->dist= startLocation.offset;
	e->v1->inEdge= e;
	e->v2->dist= e->length-startLocation.offset;
	e->v2->inEdge= e;
	if (e->v1 != avoid)
		vQueue[vQueueSize++]= e->v1;
	if (e->v2 != avoid)
		vQueue[vQueueSize++]= e->v2;
	while (vQueueSize > 0) {
		int besti= 0;
		for (int i=1; i<vQueueSize; i++)
			if (vQueue[besti]->dist > vQueue[i]->dist)
				besti= i;
		Vertex* v= vQueue[besti];
		vQueue[besti]= vQueue[--vQueueSize];
		for (int i=0; i<2; i++) {
			Edge* e= NULL;
			float d= v->dist;
			if (v->type==VT_SWITCH) {
				SwVertex* sw= (SwVertex*)v;
				if (v->inEdge == v->edge1) {
					e= sw->swEdges[i];
				} else if (i == 0) {
					e= v->edge1;
				} else {
					d+= chgPenalty;
					if (v->inEdge == sw->swEdges[0])
						e= sw->swEdges[1];
					else
						e= sw->swEdges[0];
				}
				if (sw->locked && e!=v->edge1 && e!=v->edge2)
					continue;
			} else if (i == 0) {
				e= v->nextEdge(v->inEdge);
			}
			if (e == NULL)
				break;
			Vertex* v2= v==e->v1 ? e->v2 : e->v1;
			if (e->occupied)
				d+= e->length * occupiedPenalty;
			else
				d+= e->length;
			if (v2->dist > d) {
				if (v2->type==VT_SWITCH &&
				  e!=v2->edge1 && e!=v2->edge2)	{
					SwVertex* sw= (SwVertex*)v2;
					if (sw->locked)
						continue;
					if (v2->occupied)
						d+= 1000;
				}
				v2->dist= d;
				if (v2->inEdge == NULL && v2!=avoid)
					vQueue[vQueueSize++]= v2;
				v2->inEdge= e;
			}
		}
	}
#if 0
	if (avoid) {
		fprintf(stderr," avoid %p %f %p\n",
		  avoid,avoid->dist,avoid->inEdge);
		for (int i=0; i<2; i++) {
			Edge* e= i==0 ? avoid->edge1 : avoid->edge2;
			if (e) {
				Vertex* v= e->otherV(avoid);
				fprintf(stderr,"  edge%d %p %p %p %f %f\n",
				  i+1,e,v,v->inEdge,v->dist,e->length);
			}
		}
	}
#endif
}

void findTrackLocation(double x, double y, double z, Track::Location* locp)
{
	float bestd= 1e30;
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i) {
		Track::Location loc;
		float d= i->second->findLocation(x,y,z,&loc);
		if (bestd > d) {
			bestd= d;
			*locp= loc;
		}
	}
}

Track::SwVertex* findTrackSwitch(int id)
{
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i) {
		Track::SwitchMap::iterator j= i->second->switchMap.find(id);
		if (j != i->second->switchMap.end())
			return j->second;
	}
}

Track::SSEdge* findTrackSSEdge(int id)
{
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i) {
		Track::SSEdgeMap::iterator j= i->second->ssEdgeMap.find(id);
		if (j != i->second->ssEdgeMap.end())
			return j->second;
	}
}

//	connects two different Track structures
//	used to connect float bridges to land and flaot bridges to car floats
TrackConn::TrackConn(Track* t1, Track* t2)
{
	fprintf(stderr,"connecting track %p and %p\n",t1,t2);
	track1= t1;
	track2= t2;
	if (t1->vertexList.size() > t2->vertexList.size()) {
		Track* t= t1;
		t1= t2;
		t2= t;
	}
	for (Track::VertexList::iterator i=t1->vertexList.begin();	
	  i!=t1->vertexList.end(); ++i) {
		Track::Vertex* v1= *i;
		if (v1->edge1!=NULL && v1->edge2!=NULL)
			continue;
		WLocation loc1= v1->location;
		if (t1->matrix != NULL)
			loc1.coord= t1->matrix->preMult(loc1.coord);
		fprintf(stderr," v1 %p %lf %lf %f\n",v1,
		  loc1.coord[0],loc1.coord[1],loc1.coord[2]);
		Track::Vertex* bestv2= NULL;
		float bestd= 1;
		for (Track::VertexList::iterator i=t2->vertexList.begin();	
		  i!=t2->vertexList.end(); ++i) {
			Track::Vertex* v2= *i;
			if (v2->edge1!=NULL && v2->edge2!=NULL)
				continue;
			WLocation loc2= v2->location;
			if (t2->matrix != NULL)
				loc2.coord= t2->matrix->preMult(loc2.coord);
			float dx= loc1.coord[0]-loc2.coord[0];
			float dy= loc1.coord[1]-loc2.coord[1];
			float dz= loc1.coord[2]-loc2.coord[2];
			float d= dx*dx + dy*dy + dz+dz;
			if (bestd > d) {
				bestd= d;
				bestv2= v2;
			}
		}
		if (bestv2 == NULL)
			continue;
		fprintf(stderr," edge %p %p %f\n",v1,bestv2,bestd);
		Track::Edge* e= new Track::Edge;
		e->track= &track;
		e->v1= v1;
		e->v2= bestv2;
		if (v1->edge1 == NULL)
			v1->edge1= e;
		else
			v1->edge2= e;
		if (bestv2->edge1 == NULL)
			bestv2->edge1= e;
		else
			bestv2->edge2= e;
		track.edgeList.push_back(e);
		e->length= .00001;
		e->occupied= 0;
	}
}

TrackConn::~TrackConn()
{
	for (Track::EdgeList::iterator i=track.edgeList.begin();
	  i!=track.edgeList.end(); ++i) {
		Track::Edge* e= *i;
		if (e->v1->edge1 == e)
			e->v1->edge1= NULL;
		else if (e->v1->edge2 == e)
			e->v1->edge2= NULL;
		if (e->v2->edge1 == e)
			e->v2->edge1= NULL;
		else if (e->v2->edge2 == e)
			e->v2->edge2= NULL;
//		delete e;
	}
}

int TrackConn::occupied()
{
	for (Track::EdgeList::iterator i=track.edgeList.begin();
	  i!=track.edgeList.end(); ++i) {
		Track::Edge* e= *i;
		if (e->occupied)
			return 1;
	}
	return 0;
}

//	calculates spline edge parameters
void Track::calcSplines(Track::EdgeList& splines, Track::Edge* edge1,
  Track::Edge* edge2)
{
	for (int i=0; i<3; i++) {
		Spline<double> spline;
		float d= 0;
		for (EdgeList::iterator j=splines.begin(); j!=splines.end();
		  ++j) {
			SplineEdge* se= (SplineEdge*) *j;
			if (d == 0)
				spline.add(d,se->v1->location.coord[i]);
			d+= se->length;
			spline.add(d,se->v2->location.coord[i]);
		}
		float yp1= 1e30;
		if (edge1 != NULL)
			yp1= (edge1->v2->location.coord[i] -
			  edge1->v1->location.coord[i]) / edge1->length;
		float yp2= 1e30;
		if (edge2 != NULL)
			yp2= (edge2->v2->location.coord[i] -
			  edge2->v1->location.coord[i]) / edge2->length;
		fprintf(stderr,"i=%d yp1=%f yp2=%f\n",i,yp1,yp2);
		spline.compute(yp1,yp2);
		int k= 0;
		for (EdgeList::iterator j=splines.begin(); j!=splines.end();
		  ++j) {
			SplineEdge* se= (SplineEdge*) *j;
			se->dd1[i]= spline.getY2(k);
			k++;
			se->dd2[i]= spline.getY2(k);
			fprintf(stderr,"se %d %lf %lf %lf %lf\n",
			 i,se->v1->location.coord[i],se->v2->location.coord[i],
			 se->dd1[i],se->dd2[i]);
		}
	}
	for (EdgeList::iterator j=splines.begin(); j!=splines.end(); ++j) {
		SplineEdge* se= (SplineEdge*) *j;
		se->splineMult= (se->length*se->length)/6;
		Track::Location loc;
		loc.edge= se;
		loc.rev= 0;
		for (float d=.5; d>.001; d*=.5) {
			float s= 0;
			WLocation wl1= se->v1->location;
			for (float o=d; o<1.0001; o+=d) {
				loc.offset= se->length*(o>1 ? 1 : o);
				WLocation wl2;
				loc.getWLocation(&wl2,1);
				float dx= wl1.coord[0]-wl2.coord[0];
				float dy= wl1.coord[1]-wl2.coord[1];
				float dz= wl1.coord[2]-wl2.coord[2];
				s+= sqrt(dx*dx+dy*dy+dz*dz);
				wl1= wl2;
			}
			float e= s-se->length;
//			fprintf(stderr,"%f %f %f %f\n",d,e,s,se->length);
			se->length= s;
			if (e < .01)
				break;
		}
	}
	splines.clear();
}

//	sets pline edge parameters to match a circle as close as possible
void Track::SplineEdge::setCircle(float radius, float angle)
{
	this->angle= angle;
	float x= -4*radius*(1-cos(.5*angle))/3;
	if (angle < 0)
		x= -x;
	float dx= v1->location.coord[0] - v2->location.coord[0];
	float dy= v1->location.coord[1] - v2->location.coord[1];
	float r= sqrt(dx*dx+dy*dy);
//	fprintf(stderr,"x=%f %f %f dx=%f dy=%f r=%f %f\n",
//	  x,radius,angle,dx,dy,r,
//	  2*radius*sin(.5*angle));
	splineMult= 1;
	dd1[0]= dd2[0]= x*dy/r;
	dd1[1]= dd2[1]= -x*dx/r;
	dd1[2]= dd2[2]= 0;
//	fprintf(stderr," %f %f %f\n",dd1[0],dd1[1],length);
	if (radius > 0)
		curvature= 1746.4/radius;
}

Track::Vertex* Track::findSiding(osg::Vec3d& coord, float len)
{
	Track::Vertex* bestV= NULL;
	float bestD= 9*len*len;
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		Edge* e= v->inEdge;
		if (e==NULL || e==v->edge1)
			continue;
		Vertex* p= findSidingParent(v);
		if (p==NULL || v->dist-p->dist<len)
			continue;
		float d= (v->location.coord-coord).length2();
		if (d < bestD) {
			bestV= v;
			bestD= d;
		}
	}
	return bestV;
}

//	tries to find a siding by analysing the findSPT results
Track::Vertex* Track::findSiding(float distance, float tol)
{
	Track::Vertex* bestV= NULL;
	float bestD= tol;
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		Edge* e= v->inEdge;
		if (e==NULL || e==v->edge1)
			continue;
		SwVertex* sw= (SwVertex*)v;
		e= sw->swEdges[0]==e ? sw->swEdges[1] : sw->swEdges[0];
		Track::Vertex* v1= e->v1==v ? e->v2 : e->v1;
		if (v1->dist < 1e30) {
			float d= v->dist-distance;
			if (d < 0)
				d= -d;
			if (d < bestD) {
				bestV= v;
				bestD= d;
			}
		}
	}
	return bestV;
}

//	tries to find the other end of a siding
Track::Vertex* Track::findSidingParent(Track::Vertex* v)
{
	Edge* e= v->inEdge;
	if (e==NULL || v->edge1==e || v->type!=VT_SWITCH)
		return NULL;
	SwVertex* sw= (SwVertex*)v;
	e= sw->swEdges[0]==e ? sw->swEdges[1] : sw->swEdges[0];
	Track::Vertex* v1= e->v1==v ? e->v2 : e->v1;
	return findCommonParent(v,v1);
}

//	tries to find the other end of a siding
Track::Vertex* Track::findCommonParent(Track::Vertex* v1, Track::Vertex* v2)
{
	if (v1->inEdge==NULL || v2->inEdge==NULL)
		return NULL;
	while (v1 != v2) {
		if (v1->dist > v2->dist) {
			Edge* e= v1->inEdge;
			v1= e->v1==v1 ? e->v2 : e->v1;
		} else {
			Edge* e= v2->inEdge;
			v2= e->v1==v2 ? e->v2 : e->v1;
		}
	}
	return v1;
}

void Track::saveLocation(double x, double y, double z, std::string& name,
  int rev)
{
	Location loc;
	float d= findLocation(x,y,z,&loc);
	loc.rev= rev;
	locations.insert(make_pair(name,loc));
//	printf("%.3lf|%.3lf|%.3lf|%s|%f\n",x,y,z,name.c_str(),d);
}

int Track::findLocation(std::string& name, int index, Location* loc)
{
	LocationMap::iterator i= locations.find(name);
	for (;;) {
		if (i==locations.end() || i->first!=name)
			return 0;
		if (index == 0) {
			*loc= i->second;
			return 1;
		}
		++i;
		index--;
	}
}

int Track::findLocation(std::string& name, Location* loc)
{
	Location loc1;
	Location loc2;
	if (findLocation(name,0,&loc1) == 0)
		return 0;
	if (findLocation(name,1,&loc2) == 0) {
		*loc= loc1;
		return 1;
	}
	if (loc1.edge == loc2.edge) {
		*loc= loc1;
		loc->offset= (loc1.offset+loc2.offset)/2;
		return 1;
	}
	findSPT(loc1,true);
	Vertex* v= loc2.edge->v1;
	float d= (v->dist + loc2.offset) / 2;
//	fprintf(stderr,"%f %f %f %f %f\n",d,
//	  loc1.edge->v1->dist,loc1.edge->v2->dist,
//	  loc2.edge->v1->dist,loc2.edge->v2->dist);
	if (v->dist > loc2.edge->v2->dist) {
		v= loc2.edge->v2;
		d= (v->dist + loc2.edge->length - loc2.offset) / 2;
	}
	if (v->inEdge == NULL) {
		*loc= loc2;
		return 1;
	}
	if (v->dist < d) {
		loc->rev= 0;
		loc->edge= loc2.edge;
		if (v == loc2.edge->v1)
			loc->offset= d - v->dist;
		else
			loc->offset= loc2.edge->length - (d - v->dist);
		return 1;
	}
	Edge* e= loc2.edge;
	while (v->dist>d && e!=v->inEdge) {
		e= v->inEdge;
		v= v==e->v1 ? e->v2 : e->v1;
	}
	loc->rev= 0;
	loc->edge= e;
	if (e == v->inEdge && v==e->v1)
		loc->offset= v->dist + d;
	else if (e == v->inEdge)
		loc->offset= e->length - (d + v->dist);
	else if (v == e->v1)
		loc->offset= d - v->dist;
	else
		loc->offset= e->length - (d - v->dist);
	return 1;
}

float Track::Location::getDist()
{
	float d= edge->v2->dist - edge->v1->dist;
	if (d>0 && (edge->v1->dist>edge->length || edge->v2->dist>edge->length))
		d= edge->v1->dist + offset;
	else if (d<0 && (edge->v1->dist>edge->length ||
	  edge->v2->dist>edge->length))
		d= edge->v1->dist - offset;
	else if (edge->v1->dist > offset)
		d= edge->v1->dist - offset;
	else
		d= offset-edge->v1->dist;
//	fprintf(stderr,"getdist %p %p %.3f %.3f %.3f %.3f %.3f %f\n",
//	  edge->v1,edge->v2,edge->v1->dist,edge->v2->dist,edge->length,
//	  offset,d,edge->v2->dist-edge->v1->dist);
	return d;
}

//	checks to see if the track is occupied between farv and the start of
//	the last findSPT
float Track::checkOccupied(Track::Vertex* farv)
{
	Track::Vertex* v= farv;
	Track::Edge* pe= NULL;
	float oDist= 0;
	for (;;) {
		Track::Edge* e= v->inEdge;
		if (e==NULL || e==pe)
			break;
		if (pe!=NULL && pe->occupied && v->dist>0) {
			oDist= v->dist;
			fprintf(stderr,"occupied %p %f %f %d\n",
			  pe,pe->v1->dist,pe->v2->dist,pe->occupied);
		}
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
	}
	return oDist;
}

void Track::orient(Path* path)
{
	findSPT(path->firstNode->loc,true,NULL);
	for (Path::Node* p= path->firstNode->next; p!=NULL; p=p->next) {
		if (p->sw != NULL)
			p->loc.edge= p->sw->edge1;
		Edge* e= p->loc.edge;
		p->loc.rev= e->v1->dist<e->v2->dist;
		if (p->sw != NULL)
			p->loc.offset= e->v1==p->sw ? 0 : e->length;
		for (Path::Node* p1= p->nextSiding;
		  p1!=NULL && p1->nextSiding!=NULL; p1=p1->nextSiding) {
			if (p1->sw != NULL)
				p1->loc.edge= p1->sw->edge1;
			Edge* e= p1->loc.edge;
			p1->loc.rev= e->v1->dist<e->v2->dist;
			if (p1->sw != NULL)
				p1->loc.offset= e->v1==p1->sw ? 0 : e->length;
		}
	}
	Edge* e= path->firstNode->next->loc.edge;
	if (e == path->firstNode->loc.edge) {
		Path::Node* n1= path->firstNode;
		Path::Node* n2= path->firstNode->next;
		n1->loc.rev= n1->loc.offset>n2->loc.offset;
		n2->loc.rev= n1->loc.rev;
//		path->firstNode->loc.rev= 
//		  path->firstNode->next->loc.rev;
	} else {
		Vertex* v= e->v1->dist>e->v2->dist ? e->v1 : e->v2;
		if (v->inEdge == NULL) {
			fprintf(stderr," bad path\n");
			return;
		}
		while (v->inEdge != path->firstNode->loc.edge) {
			e= v->inEdge;
			v= e->v1==v ? e->v2 : e->v1;
		}
		e= v->inEdge;
		path->firstNode->loc.rev= e->v1==v;
	}
	for (Path::Node* p= path->firstNode; p!=NULL; p=p->next) {
		if (p->next != NULL) {
			if (p->sw == NULL)
				p->nextSSEdge= p->loc.edge->ssEdge;
			else if (p->next->sw == NULL)
				p->nextSSEdge= p->next->loc.edge->ssEdge;
			else
				p->nextSSEdge= p->sw->findSSEdge(p->next->sw);
		}
		if (p->nextSiding != NULL) {
			if (p->sw == NULL)
				p->nextSidingSSEdge= p->loc.edge->ssEdge;
			else if (p->nextSiding->sw == NULL)
				p->nextSidingSSEdge=
				  p->nextSiding->loc.edge->ssEdge;
			else
				p->nextSidingSSEdge=
				  p->sw->findSSEdge(p->nextSiding->sw);
		}
//		fprintf(stderr,"main %f %p %p %p\n",
//		  p->loc.getDist(),p,p->next,p->nextSiding);
		Path::Node* p1= p->nextSiding;
		for (; p1!=NULL && p1->nextSiding!=NULL; p1=p1->nextSiding) {
			if (p1->sw == NULL)
				p1->nextSidingSSEdge= p1->loc.edge->ssEdge;
			else if (p1->nextSiding->sw == NULL)
				p1->nextSidingSSEdge=
				  p1->nextSiding->loc.edge->ssEdge;
			else
				p1->nextSidingSSEdge=
				  p1->sw->findSSEdge(p1->nextSiding->sw);
//			fprintf(stderr," siding %f %p %p %p\n",
//			  p1->loc.getDist(),p1,p1->next,p1->nextSiding);
		}
		if (p1 != NULL)
			p1->type= Path::SIDINGEND;
		if (p->next!=NULL && p->nextSiding!=NULL)
			p->type= Path::SIDINGSTART;
	}
}

void Track::makeSSEdges()
{
	int max= -1;
	for (VertexList::iterator i=vertexList.begin();
	  i!=vertexList.end(); ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		SwVertex* sw= (SwVertex*)v;
		if (max < sw->id)
			max= sw->id;
	}
	for (VertexList::iterator i=vertexList.begin();
	  i!=vertexList.end(); ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		SwVertex* sw= (SwVertex*)v;
		if (sw->id < 0)
			sw->id= ++max;
		switchMap[sw->id]= sw;
		for (int j=0; j<3; j++) {
			if (sw->ssEdges[j] != NULL)
				continue;
			SSEdge* sse= new SSEdge;
			sse->block= ssEdgeMap.size()+1;
			ssEdgeMap[sse->block]= sse;
			sse->length= 0;
			sse->v1= sw;
			sse->v2= NULL;
			sse->track= this;
			sw->ssEdges[j]= sse;
			Edge* e= j==0 ? sw->edge1 : sw->swEdges[j-1];
			v= sw;
			while (e != NULL) {
				e->ssEdge= sse;
				e->ssOffset= sse->length;
				sse->length+= e->length;
				v= e->v1==v ? e->v2 : e->v1;
				if (v->type == VT_SWITCH)
					break;
				e= v->edge1==e ? v->edge2 : v->edge1;
			}
			sse->v2= v;
			if (v->type != VT_SWITCH)
				continue;
			SwVertex* sw1= (SwVertex*)v;
			if (sw == sw1)
				continue;
			if (e == sw1->edge1)
				sw1->ssEdges[0]= sse;
			else if (e == sw1->swEdges[0])
				sw1->ssEdges[1]= sse;
			else
				sw1->ssEdges[2]= sse;
		}
	}
}

void Track::alignSwitches(Path::Node* from, Path::Node* to, bool takeSiding)
{
	SwVertex* prevSw= NULL;
	SSEdge* sse1= from->loc.edge->ssEdge;
	while (from != to) {
		Path::Node* next;
		SSEdge* sse2;
		if (from->next==NULL ||
		  (takeSiding && from->nextSiding!=NULL)) {
			next= from->nextSiding;
			sse2= from->nextSidingSSEdge;
		} else {
			next= from->next;
			sse2= from->nextSSEdge;
		}
		if (sse1 != sse2 && from->sw) {
			SwVertex* sw= from->sw;
			if (sw == prevSw)
				;// don't change twice if reverse
			else if (sw->ssEdges[1]==sse1 || sw->ssEdges[1]==sse2)
				sw->throwSwitch(sw->swEdges[0],false);
			else
				sw->throwSwitch(sw->swEdges[1],false);
			prevSw= sw;
		}
		from= next;
		sse1= sse2;
	}
}

void Track::Location::set(SSEdge* ssEdge, float ssOffset, int r)
{
	edge= NULL;
	Vertex* v= ssEdge->v1;
	if (v->edge1->ssEdge == ssEdge)
		edge= v->edge1;
	if (v->edge2!=NULL && v->edge2->ssEdge==ssEdge)
		edge= v->edge2;
	if (edge == NULL)
		return;
	offset= ssOffset;
	rev= r;
	while (edge!=NULL && offset>edge->length) {
		offset-= edge->length;
		v= v==edge->v1 ? edge->v2 : edge->v1;
		edge= edge==v->edge1 ? v->edge2 : v->edge1;
	}
}

void Track::makeSwitchCurves()
{
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		SwVertex* sw= (SwVertex*)v;
		v= sw->edge1->otherV(sw);
		osg::Vec3d d1= sw->location.coord - v->location.coord;
		d1.normalize();
		Edge* e3= sw->swEdges[1-sw->mainEdge];
		v= e3->otherV(sw);
		osg::Vec3d d2= v->location.coord - sw->location.coord;
		d2.normalize();
		float h= atan2(d1[1],d1[0]);
		float dh= atan2(d2[1],d2[0]) - h;
		if (dh < -3.14156)
			dh+= 2*3.14159;
		if (dh > 3.14156)
			dh-= 2*3.14159;
		float y= (dh<0 ? -4 : 4) / 3.281;
		float x= y/sin(dh);
//		fprintf(stderr,"sw %f %f %f %f\n",h,dh,x,y);
		if (x > sw->edge1->length || x > e3->length)
			continue;
		sw->edge1->length-= x;
		e3->length-= x;
		sw->swEdges[sw->mainEdge]->length+= x;
		sw->location.coord-= d1*x;
		float r= x/tan(fabs(dh/2));
		int m= (int)ceil(2*fabs(dh)*180/3.14159);
		dh/= m;
		float t= fabs(r*tan(dh/2));
//		fprintf(stderr," %d %f %f %f\n",m,dh,t,r);
		float cs= cos(h);
		float sn= sin(h);
		float dz= x/m*(d1[2]+d2[2]);
		osg::Vec3d p= sw->location.coord;
		v= sw;
		for (int j=0; j<m; j++) {
			p+= osg::Vec3d(t*cs,t*sn,dz);
			h+= dh;
			cs= cos(h);
			sn= sin(h);
			p+= osg::Vec3d(t*cs,t*sn,0);
			Vertex* v1= addVertex(VT_SIMPLE,p[0],p[1],p[2]);
			Edge* e= addEdge(ET_STRAIGHT,
			  v,j==0?2-sw->mainEdge:1,v1,0);
			e->length= fabs(r*dh);
			v= v1;
		}
		v->edge2= e3;
		if (e3->v1 == sw)
			e3->v1= v;
		else
			e3->v2= v;
	}
}

Track::Edge* Track::addCurve(Vertex* v1, int n1, Vertex* v2, int n2)
{
//	fprintf(stderr,"c %d %p %p\n",n1,v1->edge1,v1->edge2);
	Edge* e0= n1==0 ? v1->edge2 : v1->edge1;
	if (e0 == NULL || e0->length<1)
		return addEdge(ET_STRAIGHT,v1,n1,v2,n2);
	Vertex* v0= e0->otherV(v1);
	osg::Vec3d d1= v1->location.coord - v0->location.coord;
	d1.normalize();
	osg::Vec3d d2= v2->location.coord - v1->location.coord;
	float x= d2.length();
	d2.normalize();
	if (x > e0->length)
		x= e0->length;
	x-= .1;
	float h= atan2(d1[1],d1[0]);
	float dh= atan2(d2[1],d2[0]) - h;
	if (dh < -3.14156)
		dh+= 2*3.14159;
	if (dh > 3.14156)
		dh-= 2*3.14159;
//	fprintf(stderr,"curv %f %f %f\n",h,dh,x);
	e0->length-= x;
	v1->location.coord-= d1*x;
	float r= x/tan(fabs(dh/2));
	int m= (int)ceil(2*fabs(dh)*180/3.14159);
	dh/= m;
	float t= fabs(r*tan(dh/2));
//	fprintf(stderr," %d %f %f %f\n",m,dh,t,r);
	float cs= cos(h);
	float sn= sin(h);
	float dz= x/m*(d1[2]+d2[2]);
	osg::Vec3d p= v1->location.coord;
	for (int j=0; j<m; j++) {
		p+= osg::Vec3d(t*cs,t*sn,dz);
		h+= dh;
		cs= cos(h);
		sn= sin(h);
		p+= osg::Vec3d(t*cs,t*sn,0);
		Vertex* v= addVertex(VT_SIMPLE,p[0],p[1],p[2]);
		Edge* e= addEdge(ET_STRAIGHT,v1,n1,v,0);
		e->length= fabs(r*dh);
		v1= v;
		n1= 1;
	}
	return addEdge(ET_STRAIGHT,v1,n1,v2,n2);
}

void Track::addSwitchStand(int swid, double offset, double zoffset,
  osg::Node* model, osg::Group* rootNode, double fOffset)
{
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		if (v->type != VT_SWITCH)
			continue;
		SwVertex* sw= (SwVertex*) v;
		if (sw->id==swid && sw->edge1==NULL)
			fprintf(stderr,"no point %d %f %f %f\n",
			  sw->id,sw->location.coord[0],sw->location.coord[1],
			  sw->location.coord[2]);
		if (sw->id != swid || sw->edge1==NULL)
			continue;
		v= sw->edge1->otherV(sw);
		osg::Vec3d fwd= sw->location.coord - v->location.coord;
		fwd.normalize();
		osg::Vec3d side= osg::Vec3d(0,0,1)^fwd;
		side.normalize();
		osg::Vec3d pos= sw->location.coord + side*offset + fwd*fOffset;
		Location loc;
		float d= sqrt(findLocation(pos[0],pos[1],pos[2],&loc));
		if (d < .9*fabs(offset)) {
			osg::Vec3d pos1= sw->location.coord + side*(-offset) +
			  fwd*fOffset;
			float d1=
			  sqrt(findLocation(pos1[0],pos1[1],pos1[2],&loc));
			if (.9*d1 > d) {
				pos= pos1;
				offset= -offset;
			}
//			fprintf(stderr,"switchstand %f %f %f %f %f\n",
//			  pos[0],pos[1],pos[2],d,d1);
		}
		if (offset > 0) {
			fwd= -fwd;
			side= -side;
		}
		osg::MatrixTransform* mt= new osg::MatrixTransform;
		mt->setMatrix(osg::Matrixd(fwd[0],fwd[1],fwd[2],0,
		  side[0],side[1],side[2],0,0,0,1,0,
		  pos[0],pos[1],pos[2]+zoffset,1));
		osg::Node* copy= (osg::Node*)
		  model->clone(osg::CopyOp::DEEP_COPY_NODES);
		SetSwVertexVisitor visitor(sw);
		copy->accept(visitor);
		mt->addChild(copy);
		rootNode->addChild(mt);
		mt->addUpdateCallback(
		  new SwitchStandZUpdateCB(sw,zoffset));
	}
}

void printTrackLocations()
{
	typedef multimap<float,Track::LocationMap::iterator> SortMap;
	SortMap sortMap;
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i) {
		Track* t= i->second;
		Track::Location loc;
		float d= t->findLocation(currentPerson.location[0],
		  currentPerson.location[1],currentPerson.location[2],&loc);
		t->findSPT(loc,0.,0.);
		for (Track::LocationMap::iterator j=t->locations.begin();
		  j!=t->locations.end(); j++) {
			sortMap.insert(make_pair(j->second.getDist()+d,j));
		}
	}
	for (SortMap::iterator i=sortMap.begin(); i!=sortMap.end(); ++i) {
		WLocation wl;
		i->second->second.getWLocation(&wl,1);
		printf("%.3lf|%.3lf|%.3lf|%s|%f|%d\n",
		  wl.coord[0],wl.coord[1],wl.coord[2],i->second->first.c_str(),
		  i->first*3.281/5280,i->second->second.rev);
	}
}

osg::Switch* addTrackLabels()
{
	osg::Switch* labels= new osg::Switch();
	osg::Billboard* bb= new osg::Billboard();
	bb->setMode(osg::Billboard::POINT_ROT_EYE);
	bb->setNormal(osg::Vec3f(0,0,1));
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i) {
		Track* t= i->second;
		for (Track::LocationMap::iterator j=t->locations.begin();
		  j!=t->locations.end(); j++) {
			WLocation wl;
			j->second.getWLocation(&wl,1);
			osgText::Text* text= new osgText::Text;
			bb->addDrawable(text,wl.coord+osg::Vec3(0,0,5));
			text->setFont("fonts/arial.ttf");
			text->setCharacterSize(25);
			text->setPosition(osg::Vec3(0,0,0));
			text->setColor(osg::Vec4(1,0,0,1));
			text->setText(j->first);
			text->setAlignment(osgText::Text::CENTER_BOTTOM);
			text->setCharacterSizeMode(
			  osgText::Text::SCREEN_COORDS);
		}
	}
	labels->addChild(bb);
	labels->setAllChildrenOff();
	return labels;
}

void Track::alignSwitches(std::string from, std::string to)
{
	Location f,t;
	if (findLocation(from,&f) == 0)
		fprintf(stderr,"cannot find track location %s\n",from.c_str());
	else if (findLocation(to,&t) == 0)
		fprintf(stderr,"cannot find track location %s\n",to.c_str());
	else
		alignSwitches(f,t);
}

//	aligns switch from from to to.
void Track::alignSwitches(Location& from, Location& to)
{
	findSPT(from,true);
	Track::Vertex* v= to.edge->v1->dist > to.edge->v2->dist ?
	  to.edge->v1 : to.edge->v2;
	Track::Edge* pe= NULL;
	for (;;) {
		if (v->type==Track::VT_SWITCH && pe!=NULL)
			((Track::SwVertex*)v)->throwSwitch(pe,false);
		Track::Edge* e= v->inEdge;
		if (e==NULL || e==pe)
			break;
		if (v->type==Track::VT_SWITCH)
			((Track::SwVertex*)v)->throwSwitch(e,false);
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
	}
}

void Track::calcGrades()
{
	const float maxCrest= .5*.2/100/(.3048*100);// .2% per 100ft section
	const float maxSag= .5*.1/100/(.3048*100);// .1% per 100ft section
	int nCrest= 0;
	int nSag= 0;
	for (VertexList::iterator i=vertexList.begin();
	  i!=vertexList.end(); ++i) {
		Vertex* v= *i;
		v->grade= 0;
		if (v->type != VT_SWITCH) {
			if (v->edge1)
				v->grade= -v->edge1->grade(v);
			if (v->edge2)
				v->grade= .5*v->grade + .5*v->edge2->grade(v);
			if (v->edge1 && v->edge2) {
				float g1= v->edge1->grade();
				float g2= v->edge2->grade();
				if (v == v->edge1->v1)
					g1= -g1;
				if (v == v->edge2->v2)
					g2= -g2;
				float dg= (g2-g1)/
				  (v->edge2->length+v->edge1->length);
				if (dg<-maxCrest)
					nCrest++;
				if (dg>maxSag)
					nSag++;
//					fprintf(stderr,
//					  "dg %p %f %f %f %f %f\n",
//					  v,dg,g1,g2,-maxCrest,maxSag);
			}
		} else {
			SwVertex* sw= (SwVertex*)v;
			if (v->edge1)
				v->grade= -v->edge1->grade(v)/3;
			for (int j=0; j<2; j++) {
				if (sw->swEdges[j])
					v->grade+= sw->swEdges[j]->grade(v)/3;
			}
		}
	}
//	fprintf(stderr,"crest %d sag %d\n",nCrest,nSag);
}

float Track::averageElevation(Edge* edge, Vertex* vertex, float dist)
{
	Vertex* v= edge->otherV(vertex);
	float len= 0;
	float area= 0;
	while (len < dist) {
//		if (v->type == VT_SWITCH)
//			return v->elevation;
		area+= .5*(edge->v1->elevation+edge->v2->elevation)*
		  edge->length;
		len+= edge->length;
		Edge* e= v->nextEdge(edge);
		if (e == NULL)
			break;
		v= e->otherV(v);
		edge= e;
	}
//	fprintf(stderr,"aelev %f %f %f\n",area/len,area,len);
	return area/len;
}

void Track::calcSmoothGrades(int nIterations, float distance)
{
	calcGrades();
	const float maxCrest= .5*.2/100/(.3048*100);// .2% per 100ft section
	const float maxSag= .5*.1/100/(.3048*100);// .1% per 100ft section
	fprintf(stderr,"maxCrest %e maxSag %e\n",maxCrest,maxSag);
	for (VertexList::iterator i=vertexList.begin();
	  i!=vertexList.end(); ++i) {
		Vertex* v= *i;
		v->inEdge= NULL;
	}
	for (int iter=0; iter<nIterations; iter++) {
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			v->dist= v->elevation;
		}
		int n= 0;
		double se= 0;
		float minDE= 1e10;
		float maxDE= -1e10;
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			float de= v->elevation-v->location.coord[2];
			se+= de*de;
			if (minDE > de)
				minDE= de;
			if (maxDE < de)
				maxDE= de;
			if (v->inEdge)
				continue;
			Edge* edge1= v->edge1;
			Edge* edge2= v->edge2;
			if (edge1 && !edge2) {
				Vertex* v1= edge1->otherV(v);
				v->dist= v1->dist;
			}
			if (!edge1 || !edge2)
				continue;
			float dist= distance;
			float e1= averageElevation(edge1,v,dist);
			float e2= averageElevation(edge2,v,dist);
			float g1= (v->elevation-e1)/(.5*dist);
			float g2= (e2-v->elevation)/(.5*dist);
			float dg= (g2-g1)/dist;
			if (-maxCrest<=dg && dg<=maxSag)
				continue;
			v->dist= .5 * (e1+e2);
			if (dg < -maxCrest)
				v->dist+= .5*maxCrest*dist;
			if (maxSag < dg)
				v->dist-= .5*maxSag*dist;
			n++;
		}
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			v->elevation= v->dist;
		}
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			if (v->type != VT_SWITCH)
				continue;
			SwVertex* sw= (SwVertex*)v;
			Edge* edge1= v->edge2;
			Edge* edge2= edge1==sw->swEdges[0] ? sw->swEdges[1] :
			  sw->swEdges[0];
			Vertex* v1= edge1->otherV(v);
			float g= (v1->elevation-v->elevation)/edge1->length;
			Vertex* v2= edge2->otherV(v);
			//v2->elevation= g*edge2->length + v->elevation;
			//v2->inEdge= edge2;
		}
		fprintf(stderr,"iter %d %d %f %f %f\n",
		  iter,n,se,minDE,maxDE);
		if (n == 0) {
			break;
		}
	}
	calcGrades();
}

void Track::split(std::string& newName, double x1, double y1,
  double x2, double y2)
{
	double xy1[2]= { x1,y1 };
	double xy2[2]= { x2,y2 };
	double pi[2];
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		v->dist= 0;
		v->inEdge= NULL;
	}
//	fprintf(stderr,"splittrack %s %.3f %.3f %.3f %.3f\n",
//	  newName.c_str(),x1,y1,x2,y2);
	EdgeList splitEdges;
	VertexList splitVerts;
	for (EdgeList::iterator i=edgeList.begin(); i!=edgeList.end(); ++i) {
		Edge* e= *i;
		char code= segSegInt(xy1,xy2,e->v1->location.coord.ptr(),
		  e->v2->location.coord.ptr(),pi);
		if (code == 'e')
			throw std::invalid_argument(
			  "split line colinear with track");
		if (code == '0')
			continue;
		if (triArea(x1,y1,x2,y2,
		  e->v1->location.coord[0],e->v1->location.coord[1]) > 0) {
			e->v1->dist= 1;
			e->v1->inEdge= e;
			splitVerts.push_back(e->v1);
		} else {
			e->v2->dist= 1;
			e->v2->inEdge= e;
			splitVerts.push_back(e->v2);
		}
//		fprintf(stderr,
//		  "splitpoint %c %.3f %.3f %.3f %.3f %.3f %.3f %.0f %.0f\n",
//		  code,pi[0],pi[1],
//		  e->v1->location.coord[0],e->v1->location.coord[1],
//		  e->v2->location.coord[0],e->v2->location.coord[1],
//		  e->v1->dist,e->v2->dist);
		splitEdges.push_back(e);
	}
	for (VertexList::iterator i=splitVerts.begin(); i!=splitVerts.end();
	  ++i) {
		Vertex* v= *i;
		for (int i=0; i<2; i++) {
			Edge* e= NULL;
			if (v->type==VT_SWITCH && v->inEdge==v->edge1)
				e= ((SwVertex*)v)->swEdges[i];
			else if (i > 0)
				break;
			else if (v->type==VT_SWITCH && v->inEdge!=v->edge1)
				e= v->edge1;
			else
				e= v->nextEdge(v->inEdge);
			if (e == NULL)
				break;
			if (v!=e->v1 && e->v1->inEdge==NULL) {
				e->v1->inEdge= e;
				splitVerts.push_back(e->v1);
			} else if (v!=e->v2 && e->v2->inEdge==NULL) {
				e->v2->inEdge= e;
				splitVerts.push_back(e->v2);
			}
		}
	}
//	fprintf(stderr,"%d splitverts\n",splitVerts.size());
	Track* newTrack= new Track();
	trackMap[newName]= newTrack;
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		if (v->inEdge)
			newTrack->vertexList.push_back(v);
	}
	for (VertexList::iterator i=newTrack->vertexList.begin();
	  i!=newTrack->vertexList.end(); ++i) {
		Vertex* v= *i;
		vertexList.remove(v);
	}
	for (EdgeList::iterator i=edgeList.begin(); i!=edgeList.end(); ++i) {
		Edge* e= *i;
		if (e->v1->inEdge && e->v2->inEdge) {
			newTrack->edgeList.push_back(e);
			e->track= newTrack;
		}
	}
	for (EdgeList::iterator i=newTrack->edgeList.begin();
	  i!=newTrack->edgeList.end(); ++i) {
		Edge* e= *i;
		edgeList.remove(e);
	}
	for (EdgeList::iterator i=splitEdges.begin(); i!=splitEdges.end();
	  ++i) {
		Edge* e= *i;
		double s,t;
		char code= segSegInt(xy1,xy2,e->v1->location.coord.ptr(),
		  e->v2->location.coord.ptr(),pi,&s,&t);
		Location loc;
		loc.edge= e;
		loc.offset= t*e->length;
		loc.rev= 0;
		WLocation wloc;
		loc.getWLocation(&wloc);
		int n1= e==e->v1->edge1 ? 0 : 1;
		if (n1==1 && e->v1->type==VT_SWITCH &&
		  e==((SwVertex*)e->v1)->swEdges[1])
			n1= 2;
		int n2= e==e->v2->edge1 ? 0 : 1;
		if (n2==1 && e->v2->type==VT_SWITCH &&
		  e==((SwVertex*)e->v2)->swEdges[1])
			n2= 2;
//		fprintf(stderr,
//		  "splitedge %c %.3f %.3f %.3f %.3f %.3f %.3f %.0f %.0f\n",
//		  code,pi[0],pi[1],
//		  e->v1->location.coord[0],e->v1->location.coord[1],
//		  e->v2->location.coord[0],e->v2->location.coord[1],
//		  e->v1->dist,e->v2->dist);
//		fprintf(stderr," %f %f %.3f %.3f %.3f %d %d\n",
//		  s,t,wloc.coord[0],wloc.coord[1],wloc.coord[2],n1,n2);
		Vertex* nv1= addVertex(VT_SIMPLE,
		  wloc.coord[0],wloc.coord[1],wloc.coord[2]);
		Vertex* nv2= newTrack->addVertex(VT_SIMPLE,
		  wloc.coord[0],wloc.coord[1],wloc.coord[2]);
		Edge* ne1;
		Edge* ne2;
		if (e->v1->inEdge) {
			ne1= addEdge(e->type,nv1,0,e->v2,n2);
			ne2= newTrack->addEdge(e->type,e->v1,n1,nv2,0);
			t= 1-t;
		} else {
			ne1= addEdge(e->type,e->v1,n1,nv1,0);
			ne2= newTrack->addEdge(e->type,nv2,0,e->v2,n2);
		}
		if (e->curvature>0 && e->type==ET_SPLINE) {
			float r= 1746.4/e->curvature;
			float a= ((SplineEdge*)e)->angle;
//			fprintf(stderr," %f %f %f\n",r,a,t);
			((SplineEdge*)ne1)->setCircle(r,a*t);
			((SplineEdge*)ne2)->setCircle(r,a*(1-t));
		}
//		fprintf(stderr," len %f %f %f\n",
//		  e->length,ne1->length,ne2->length);
		edgeList.remove(e);
	}
}

Track* Track::expand()
{
	typedef map<Vertex*,Vertex*> VMap;
	VMap vmap;
	Track* copy= new Track();
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		Vertex* cv= copy->addVertex(v->type,v->location.coord[0],
		  v->location.coord[1],v->location.coord[2]);
		vmap[v]= cv;
	}
	for (EdgeList::iterator i=edgeList.begin(); i!=edgeList.end(); ++i) {
		Edge* e= *i;
		Vertex* v1= vmap.find(e->v1)->second;
		Vertex* v2= vmap.find(e->v2)->second;
		int n1= e==e->v1->edge1 ? 0 : 1;
		if (n1==1 && e->v1->type==VT_SWITCH &&
		  e==((SwVertex*)e->v1)->swEdges[1])
			n1= 2;
		int n2= e==e->v2->edge1 ? 0 : 1;
		if (n2==1 && e->v2->type==VT_SWITCH &&
		  e==((SwVertex*)e->v2)->swEdges[1])
			n2= 2;
		if (e->length>1 && e->curvature>0 && e->type==ET_SPLINE) {
			Location loc;
			loc.edge= e;
			loc.rev= 0;
			SplineEdge* se= (SplineEdge*)e;
			float a= fabs(((SplineEdge*)e)->angle);
			int n= (int)ceil(180*a/M_PI)+1;
			for (int j=1; j<n; j++) {
				loc.offset= j*e->length/n;
				WLocation wloc;
				loc.getWLocation(&wloc,true);
				Vertex* v= copy->addVertex(VT_SIMPLE,
				  wloc.coord[0],wloc.coord[1],wloc.coord[2]);
				copy->addEdge(ET_STRAIGHT,v1,n1,v,0);
				v1= v;
				n1= 1;
			}
		}
		copy->addEdge(ET_STRAIGHT,v1,n1,v2,n2);
	}
	return copy;
}
