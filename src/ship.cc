//	ship model and movement code
//
//Copyright 2009 Doug Jones

#include "rmsim.h"

#include <osg/Matrix>

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
		m*= sx*sy*sz*WDENSITY;
		fprintf(stderr,"calc mass %f\n",m);
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
	float rsfwd= sfwd;
	float rsside= sside+rudder.location*angVel;
	float rs= rsfwd*rudder.sn - rudder.cs*rsside;
	float rt= rs*rs*rudder.location*rudder.area*WDENSITY;
	torque= 0;
	if (thrust > 0)
		torque+= thrust*rudder.sn*rudder.location*rudder.cFwdThrust;
	if (rs > 0)
		torque+= rt;
	else
		torque-= rt;
	torque+= angVel>0 ? -rotDrag(angVel) : rotDrag(-angVel);
	torque+= sside>0 ? -sideRotDrag(sside) : sideRotDrag(-sside);
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
	if (track==NULL || mass==0)
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
	Prop prop;
	prop.setSize(propDiameter,0);
	float thrust= factor*prop.getThrust(speed,power*1e3,4*speed);
	fwdDrag.add(0,0);
	backDrag.add(0,0);
	sideDrag.add(0,0);
	rotDrag.add(0,0);
	for (float s=1; s<2*speed; s+=1) {
		fwdDrag.add(s,thrust/speed/speed*s*s);
		backDrag.add(s,1.5*thrust/speed/speed*s*s);
		sideDrag.add(s,1e5*s*s);
		rotDrag.add(s,1e6*s*s);
	}
	fwdDrag.compute();
	backDrag.compute();
	sideDrag.compute();
	rotDrag.compute();
//	for (float s=0; s<2*speed; s+=1)
//		fprintf(stderr,"fwddrag %f %f\n",s,fwdDrag(s));
}
