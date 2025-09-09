//	ship model and movement code
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

#include <osg/Matrix>
#include <osgUtil/PlaneIntersector>
#include <osgUtil/IntersectionVisitor>
#include <osg/ComputeBoundsVisitor>

ShipMap shipMap;
ShipList shipList;
Ship* myShip= NULL;
RopeList ropeList;

Ship::Ship()
{
	model= NULL;
	model2D= NULL;
	boundary= NULL;
	track= NULL;
	cleats= NULL;
	modelOffset= 0;
	position[0]= position[1]= position[2]= 0;
	heading[0]= 1;
	heading[1]= 0;
	linVel[0]= linVel[1]= 0;
	angVel= 0;
	draft= 0;
	mass= massInv= 0;
	inertia= inertiaInv= 0;
	boundaryXY= NULL;
	boundaryLocs= NULL;
	calcBoundary= 1;
	trim= 0;
	list= 0;
	ddepth= 0;
}

Ship::~Ship()
{
	while (cleats != NULL) {
		Cleat* t= cleats;
		cleats= t->next;
		delete t;
	}
	if (boundaryXY != NULL)
		delete[] boundaryXY;
	if (boundaryLocs != NULL)
		delete[] boundaryLocs;
}

//	sets the ship mass and moment of inertia
void Ship::setMass(float m, float sx, float sy, float sz)
{
	if (m <= 1) {
		if (model && sx==0) {
			osg::ComputeBoundsVisitor cbv;
			model->accept(cbv);
			osg::BoundingBox bb= cbv.getBoundingBox();
			Model2D* waterLine= computeBoundary(-modelOffset);
			if (waterLine) {
				osg::BoundingBox wlbb;
				for (int i=0; i<waterLine->nVertices; i++) {
					Model2D::Vertex* v=
					  &waterLine->vertices[i];
					wlbb.expandBy(osg::Vec3(v->x,v->y,0));
				}
				sx= wlbb.xMax() - wlbb.xMin();
				sy= wlbb.yMax() - wlbb.yMin();
				lwl= sx;
				delete waterLine;
			} else {
				sx= bb.xMax() - bb.xMin();
				sy= bb.yMax() - bb.yMin();
			}
			if (sz == 0)
				sz= sy/2.4;
			m*= sx*sy*sz*WDENSITY;
			fprintf(stderr,"calc mass %.0f %.3f %.3f %.3f\n",
			  m,sx,sy,sz);
			rudder.location= sx/2;
			rudder.area= .025*sx*sz;
			sx= bb.xMax() - bb.xMin();
			sy= bb.yMax() - bb.yMin();
		} else {
			m*= sx*sy*sz*WDENSITY;
			fprintf(stderr,"calc mass %f\n",m);
		}
	}
	mass= m;
	massInv= 1/m;
	inertia= m*(sx*sx+sy*sy)/12;
	inertiaInv= 1/inertia;
//	fprintf(stderr,"mass %f %e %f %e\n",mass,massInv,inertia,inertiaInv);
}

void Ship::setInertia(float i)
{
	inertia= i;
	inertiaInv= 1/inertia;
}

void Ship::setHeading(float deg)
{
	float h= (90-deg)*3.14159/180;
	if (h < -3.14159)
		h+= 2*3.14159;
	if (h > 3.14159)
		h-= 2*3.14159;
	heading[0]= cos(h);
	heading[1]= sin(h);
}

float Ship::getHeading()
{
	float h= atan2(heading[1],heading[0]);
	float deg= 90-h*180/3.14159;
	if (deg < 0)
		deg+= 360;
	if (deg > 360)
		deg-= 360;
	return deg;
}

void Ship::setPosition(double x, double y)
{
	position[0]= x;
	position[1]= y;
	location.set(x,y);
	if (location.water != NULL)
		position[2]= location.water->waterLevel;
	else
		position[2]= 0;
}

double* Ship::getBoundaryXY()
{
	if (boundary==NULL || calcBoundary==0)
		return boundaryXY;
	if (boundaryXY == NULL) {
		boundaryXY= new double[2*boundary->nVertices];
		boundaryLocs= new Water::Location[2*boundary->nVertices];
	}
	for (int i=0; i<boundary->nVertices; i++)
		getRelPointPos(boundary->vertices[i].x,
		  boundary->vertices[i].y,boundaryXY+2*i);
	calcBoundary= 0;
	return boundaryXY;
}

Water::Location* Ship::getBoundaryLocs()
{
	return boundaryLocs;
}

void Ship::setSpeed(float s)
{
	linVel[0]= s*heading[0];
	linVel[1]= s*heading[1];
	angVel= 0;
}

//	get speed over ground (no current yet)
float Ship::getSOG()
{
	return sqrt(linVel[0]*linVel[0]+linVel[1]*linVel[1]);
}

//	get course over ground (no current yet)
float Ship::getCOG()
{
	float deg= 90-atan2(linVel[1],linVel[0])*180/3.14159;
	if (deg < 0)
		deg+= 360;
	if (deg > 360)
		deg-= 360;
	return deg;
}

float Ship::getSpeed()
{
	return linVel[0]*heading[0] + linVel[1]*heading[1];
}

//	move the ship using forces already calculated
void Ship::move(double dt)
{
//	if (force[0]!=0 || force[1]!=0 || torque!=0)
//		fprintf(stderr,"%s %lf %lf %lf\n",
//		  name.c_str(),force[0],force[1],torque);
	calcBoundary= 1;
	linVel[0]+= dt*force[0]*massInv;
	linVel[1]+= dt*force[1]*massInv;
	angVel+= dt*torque*inertiaInv;
	if (location.depth < draft) {
		double p[2];
		p[0]= position[0]+ dt*linVel[0];
		p[1]= position[1]+ dt*linVel[1];
		Water::Location loc= location;
		loc.update(p);
		if (loc.depth<location.depth ||
		  (loc.depth==0 && location.depth==0)) {
			linVel[0]= linVel[1]= 0;
			angVel= 0;
		}
	}
	position[0]+= dt*linVel[0];
	position[1]+= dt*linVel[1];
	float x= dt*angVel*heading[1];
	heading[1]+= dt*angVel*heading[0];
	heading[0]-= x;
	x= .5 + .5*(heading[0]*heading[0]+heading[1]*heading[1]);
	heading[0]/= x;
	heading[1]/= x;
	location.update(position);
	if (model != NULL) {
		osg::Vec3f fwd= osg::Vec3f(heading[0],heading[1],-trim);
		osg::Vec3f side= osg::Vec3f(-heading[1],heading[0],-list);
		fwd.normalize();
		side.normalize();
		osg::Vec3f up= fwd^side;
		osg::Matrixd m(
		  fwd[0],fwd[1],fwd[2],0,
		  side[0],side[1],side[2],0,
		  up[0],up[1],up[2],0,
		  position[0],position[1],position[2]+modelOffset,1);
		model->setMatrix(m);
		if (track != NULL)
			track->setMatrix(m);
//		fprintf(stderr,"setMat %s %p %f %f\n",
//		  name.c_str(),track,position[2],modelOffset);
	}
}

//	calculated forces acting on ship other than collisions and ropes
void Ship::calcForces()
{
	float sfwd= linVel[0]*heading[0] + linVel[1]*heading[1];
	float sside= linVel[0]*heading[1] - linVel[1]*heading[0];
	float thrust= engine.getThrust(sfwd);
	force[0]= thrust*heading[0];
	force[1]= thrust*heading[1];
	float fd= sfwd>0 ? fwdDrag(sfwd) : -backDrag(-sfwd);
	float sd= sside>0 ? sideDrag(sside) : -sideDrag(-sside);
	float fd1= sside==0 ? fd : sfwd*sd/sside;
	float sd1= sfwd==0 ? sd : sside*fd/sfwd;
//	fprintf(stderr,"%f %f %f %f %f %f\n",sfwd,fd,fd1,sside,sd,sd1);
//	if ((sfwd>0 && fd1>fd) || (sfwd<0 && fd1<fd))
//		fd= fd1;
	if ((sside>0 && sd1>sd) || (sside<0 && sd1<sd))
		sd= sd1;
	force[0]-= heading[0]*fd;
	force[1]-= heading[1]*fd;
	force[0]-= heading[1]*sd;
	force[1]+= heading[0]*sd;
	torque= 0;
	float rsx= linVel[0] + heading[1]*angVel*rudder.location;
	float rsy= linVel[1] - heading[0]*angVel*rudder.location;
	float len= sqrt(rsx*rsx + rsy*rsy);
	if (len == 0)
		return;
	float dot= (heading[0]*rsx + heading[1]*rsy)/len;
	if (dot > 1)
		dot= 1;
	float ang0= acosf(dot);
	float area= triArea(0,0,heading[0],heading[1],rsx,rsy);
//	float drift=
//	  acosf((heading[0]*linVel[0] + heading[1]*linVel[1])/getSOG());
//	if (rudder.angle != 0)
//		fprintf(stderr,"%.1f %.1f %d %f %f %f\n",
//		  ang0*180/M_PI,drift*180/M_PI,rudder.angle,area,dot,angVel);
	float rsfwd= sfwd;
	if (rsfwd>0 && rsfwd < engine.prop.speed)
		rsfwd= engine.prop.speed;
	float rs= rudder.sn;
	if (area > 0)
		rs+= ang0;
	else
		rs-= ang0;
	torque= rs*rsfwd*rsfwd*rudder.location*rudder.area*WDENSITY;
	if (rsfwd < 0)
		torque*= -.5;
	torque+= angVel>0 ? -rotDrag(angVel) : rotDrag(-angVel);
//	fprintf(stderr,"%f %f %f %f %d\n",
//	  torque,rs,rsfwd,rudder.sn,rudder.angle);
#if 0
	float rsside= sside+rudder.location*angVel;
	float rs= rsfwd*rudder.sn;// - rudder.cs*rsside;
	torque= rudder.sn*rsfwd*rsfwd*rudder.location*rudder.area*WDENSITY;
	if (rsfwd < 0)
		torque*= -.5;
//	if (thrust > 0)
//		torque+= thrust*rudder.sn*rudder.location*rudder.cFwdThrust;
#endif
#if 0
	float targetAngVel= 0;
	if (rsfwd > 0)
		targetAngVel= rudder.cFwdSpeed*rsfwd*rudder.angle;
	else if (rsfwd < 0)
		targetAngVel= rudder.cBackSpeed*rsfwd*rudder.angle;
	if (angVel-targetAngVel > .0001) {
		torque= -.5*rsfwd*rsfwd*rudder.location*rudder.area*WDENSITY;
	} else if (angVel-targetAngVel < -.0001) {
		torque= .5*rsfwd*rsfwd*rudder.location*rudder.area*WDENSITY;
	} else {
		torque= 0;
	}
#endif
//	if (angVel != 0)
//	fprintf(stderr,"angvel %f %f %f\n",angVel,targetAngVel,torque);
//	torque+= sside>0 ? -sideRotDrag(sside) : sideRotDrag(-sside);
//	fprintf(stderr,"%f %f %f\n",force[0],force[1],torque);
}

//	adds an impulse force to handle collisions
void Ship::addImpulse(double f, float* dir, double* point)
{
	linVel[0]+= f*dir[0]*massInv;
	linVel[1]+= f*dir[1]*massInv;
	angVel+= ((point[0]-position[0])*f*dir[1] -
	  (point[1]-position[1])*f*dir[0]) * inertiaInv;
}

//	adds a force at the specified poinrt
void Ship::addForceAt(float fx, float fy, double px, double py)
{
	force[0]+= fx;
	force[1]+= fy;
	torque+= (px-position[0])*fy - (py-position[1])*fx;
}

//	creates a rope between two cleats
void addRope(Cleat* c1, Cleat* c2)
{
	if (c1==NULL || c2==NULL || c1->ship==c2->ship)
		return;
	for (RopeList::iterator i=ropeList.begin(); i!=ropeList.end(); ++i) {
		Rope* r= *i;
		if ((r->cleat1==c1 && r->cleat2==c2) ||
		  (r->cleat2==c1 && r->cleat1==c2))
			return;
	}
	Rope* rope= new Rope;
	rope->cleat1= c1;
	rope->cleat2= c2;
	c1->getPosition(rope->xy1);
	c2->getPosition(rope->xy2);
	float dx= rope->xy1[0]-rope->xy2[0];
	float dy= rope->xy1[1]-rope->xy2[1];
	float dz= rope->xy1[2]-rope->xy2[2];
	rope->dist= rope->length= sqrt(dx*dx+dy*dy+dz*dz);
	rope->adjust= 0;
	ropeList.push_back(rope);
}

//	creates a rope between a cleat and an arbitrary point
void addRope(Cleat* cleat, double x, double y, double z)
{
	Rope* rope= new Rope;
	rope->cleat1= cleat;
	rope->cleat2= NULL;
	cleat->getPosition(rope->xy1);
	rope->xy2[0]= x;
	rope->xy2[1]= y;
	rope->xy2[2]= z;
	float dx= rope->xy1[0]-rope->xy2[0];
	float dy= rope->xy1[1]-rope->xy2[1];
	rope->dist= rope->length= sqrt(dx*dx+dy*dy);
	rope->adjust= 0;
	ropeList.push_back(rope);
}

void Rope::calcPosition()
{
	cleat1->getPosition(xy1);
	if (cleat2)
		cleat2->getPosition(xy2);
}

Cleat* Ship::findCleat(float x, float y, float maxDist)
{
//	fprintf(stderr,"findcleat %f %f %f\n",x,y,maxDist);
	float bestD= maxDist*maxDist;
	Cleat* bestC= NULL;
	for (Cleat* c=cleats; c!=NULL; c=c->next) {
		double dx= c->x-x;
		double dy= c->y-y;
		double d= dx*dx+dy*dy;
//		fprintf(stderr," %p %f %f %f\n",c,dx,dy,d);
		if (bestD > d) {
			bestD= d;
			bestC= c;
		}
	}
	return bestC;
}

//	adjusts a car float's mass based on the mass of cars on its track
//	also adjusts the way the car float floats
void Ship::adjustMass()
{
	if (track==NULL || mass<=0 || lwl<=0 || bwl<=0)
		return;
	ddepth= track->sumMass/(lwl*bwl*WDENSITY);
	track->sumMass+= mass;
	float mi= 1/track->sumMass;
	track->sumXMass*= mi;
	track->sumYMass*= mi;
	track->sumZMass*= mi;
	massInv= mi;
	float depth= track->sumMass/(lwl*bwl*WDENSITY);
	float bmt= bwl*bwl/(12*depth);
	float bml= lwl*lwl/(12*depth);
	float gmt= bmt - depth/2 + ddepth - track->sumZMass;
	float gml= bml - depth/2 + ddepth - track->sumZMass;
//	fprintf(stderr,"%f %f %f %f %f %f\n",lwl,bwl,ddepth,depth,gmt,gml);
	trim= .99*trim + .01*track->sumXMass/gml;
	list= .99*list + .01*track->sumYMass/gmt;
	float d= position[2];
	if (location.water != NULL)
		d-= location.water->waterLevel;
	d= .99*d + -.01*ddepth;
	if (location.water != NULL)
		d+= location.water->waterLevel;
	position[2]= d;
//	fprintf(stderr,"cf %f %f %f\n",trim,list,position[2]);
#if 0
	if (mi != massInv) {
		fprintf(stderr,"%s %e %e %lf %lf %lf %lf\n",
		  name.c_str(),massInv,mi,
		  track->sumMass,track->sumXMass,
		  track->sumYMass,track->sumZMass);
		massInv= mi;
	}
#endif
}

void Ship::calcDrag(float factor, float speed, float power, float propDiameter)
{
	float thrust;
	if (power==0 && lwl>0) {
		float knots= 1.944*speed;
		float slr= knots/sqrt(3.281*lwl);
		float shp= mass*.4536/pow(8.11*(2.3-slr),3);
		float kw= shp*.7457;
		thrust= .8*1e3*kw/speed;
		float d= sqrt(thrust/(.625*speed*speed)/
		  (3.14159*.25*1000*64/62.5));
		Prop prop;
		prop.setSize(d,0);
		float t= prop.getThrust(speed,kw*1e3,4*speed);
		fprintf(stderr,"slr %f %f %f %f\n",slr,knots,3.281*lwl,speed);
		fprintf(stderr,"shp %f %f %f %f %f\n",shp,kw,thrust,d,t);
		engine.setPower(kw);
		engine.setProp(d,0);
	} else {
		Prop prop;
		prop.setSize(propDiameter,0);
		thrust= factor*prop.getThrust(speed,power*1e3,4*speed);
	}
	float turningRadius= lwl>0 ? 1.5*lwl : 20;
	float driftAngle= 10;
	float sideF= mass*speed/turningRadius/sin(driftAngle*M_PI/180);
	fwdDrag.add(0,0);
	backDrag.add(0,0);
	sideDrag.add(0,0);
	for (float s=1; s<2*speed; s+=1) {
		fwdDrag.add(s,thrust/speed/speed*s*s);
		backDrag.add(s,1.5*thrust/speed/speed*s*s);
		sideDrag.add(s,sideF/speed*s);
	}
	fwdDrag.compute();
	backDrag.compute();
	sideDrag.compute();
	float torque= .1*speed*speed*rudder.location*rudder.area*WDENSITY;
	float r= speed/turningRadius;
	rotDrag.add(0,0);
	rotDrag.add(r,torque);
	rotDrag.add(2*r,2*torque);
	rudder.cFwdSpeed= 1/turningRadius/30;
	rudder.cBackSpeed= .5*rudder.cFwdSpeed;
//	rotDrag.compute();
//	for (float s=0; s<2*speed; s+=1)
//		fprintf(stderr,"fwddrag %f %f\n",s,fwdDrag(s));
//	for (float s=0; s<2*speed; s+=1)
//		fprintf(stderr,"sidedrag %f %f\n",s,sideDrag(s));
}

static Model2D::Vertex* verts;

int triComp(const void* p1, const void* p2)
{
	Model2D::Vertex* v1= (Model2D::Vertex*) p1;
	Model2D::Vertex* v2= (Model2D::Vertex*) p2;
	double a= triArea(verts[0].x,verts[0].y,v1->x,v1->y,v2->x,v2->y);
	if (a>0)
		return -1;
	if (a<0)
		return 1;
	float x= fabs(v1->x-verts[0].x) - fabs(v2->x-verts[0].x);
	float y= fabs(v1->y-verts[0].y) - fabs(v2->y-verts[0].y);
	if (x<0 || y<0) {
		v1->u= 1;
		return -1;
	} else if (x>0 || y>0) {
		v2->u= 1;
		return 1;
	}
	v1->u= 1;
	return 0;
}

Model2D* Ship::computeBoundary(float height)
{
	if (!model)
		return NULL;
	osg::ComputeBoundsVisitor cbv;
	model->accept(cbv);
	osg::BoundingBox bb= cbv.getBoundingBox();
	fprintf(stderr,"model bb %.3f %.3f %.3f %.3f %.3f %.3f\n",
	  bb.xMin(),bb.xMax(),
	  bb.yMin(),bb.yMax(),bb.zMin(),bb.zMax());
	osgUtil::PlaneIntersector* pi= new osgUtil::PlaneIntersector(
	  osg::Plane(osg::Vec3f(0,0,1),-height));
	osgUtil::IntersectionVisitor iv(pi);
	model->getChild(0)->accept(iv);
	typedef osgUtil::PlaneIntersector::Intersections Hits;
	Hits hits= pi->getIntersections();
//	fprintf(stderr,"%d hits\n",hits.size());
	if (hits.size() == 0)
		return NULL;
	int nPoints= 0;
	bb.init();
	for (Hits::iterator hit=hits.begin(); hit!=hits.end(); ++hit) {
		nPoints+= hit->polyline.size();
		for (int j=0; j<hit->polyline.size(); j++) {
			osg::Vec3d v= hit->polyline[j];
			bb.expandBy(v);
		}
	}
	fprintf(stderr,"plane bb %.3f %.3f %.3f %.3f %.3f %.3f\n",
	  bb.xMin(),bb.xMax(),
	  bb.yMin(),bb.yMax(),bb.zMin(),bb.zMax());
	float dx= bb.xMax()-bb.xMin();
	float dy= bb.yMax()-bb.yMin();
	float dz= bb.zMax()-bb.zMin();
	fprintf(stderr,"dx %.3f dy %.3f dz %.3f\n",dx,dy,dz);
	verts= (Model2D::Vertex*) calloc(nPoints,sizeof(Model2D::Vertex));
	int k= 0;
	int bestk= 0;
	for (Hits::iterator hit=hits.begin(); hit!=hits.end(); ++hit) {
//		fprintf(stderr," hit size %d %p\n",hit->polyline.size(),
//		  hit->drawable);
		for (int j=0; j<hit->polyline.size(); j++) {
			osg::Vec3d v= hit->polyline[j];
			if (dx>dy && dy>dz) {
				verts[k].x= v.x();
				verts[k].y= v.y();
			} else if (dx>dz && dz>dy) {
				verts[k].x= v.x();
				verts[k].y= v.z();
			} else if (dy>dx && dx>dz) {
				verts[k].x= v.y();
				verts[k].y= v.x();
			} else if (dy>dz && dz>dx) {
				verts[k].x= v.y();
				verts[k].y= v.z();
			} else if (dz>dx && dx>dy) {
				verts[k].x= v.z();
				verts[k].y= v.x();
			} else {
				verts[k].x= v.z();
				verts[k].y= v.y();
			}
			if (k>bestk && (verts[k].x<verts[bestk].x ||
			  (verts[k].x==verts[bestk].x &&
			   verts[k].y<verts[bestk].y)))
				bestk= k;
			k++;
//			fprintf(stderr,"  %d %.3f %.3f %.3f\n",
//			  j,v.x(),v.y(),v.z());
		}
	}
	float t= verts[bestk].x;
	verts[bestk].x= verts[0].x;
	verts[0].x= t;
	t= verts[bestk].y;
	verts[bestk].y= verts[0].y;
	verts[0].y= t;
	qsort(verts+1,nPoints-1,sizeof(Model2D::Vertex),triComp);
	k= 1;
	for (int i=1; i<nPoints; i++) {
		if (i>k && verts[i].u==0) {
			verts[k]= verts[i];
			k++;
		}
	}
	nPoints= k;
	k= 0;
	int j= 1;
	verts[k].v= -1;
	verts[j].v= k;
	for (int i=2; i<nPoints; ) {
		double a= triArea(verts[k].x,verts[k].y,verts[j].x,verts[j].y,
		  verts[i].x,verts[i].y);
		if (a > 0) {
			verts[i].v= j;
			k= j;
			j= i;
			i++;
		} else {
			verts[j].u= 1;
			j= k;
			k= (int)verts[k].v;
			if (k < 0) {
				fprintf(stderr,"convex hull error %d\n",i);
				break;
			}
		}
	}
	k= 1;
	for (int i=1; i<nPoints; i++) {
		if (i>k && verts[i].u==0) {
			verts[k]= verts[i];
			k++;
		}
	}
	nPoints= k;
//	for (int i=0; i<nPoints; i++)
//		fprintf(stderr,"%d %.3f %.3f %.3f %.3f\n",
//		  i,verts[i].x,verts[i].y,verts[i].u,verts[i].v);
	Model2D* model= new Model2D();
	model->nVertices= nPoints;
	model->vertices= verts;
	model->computeRadius();
	return model;
}
