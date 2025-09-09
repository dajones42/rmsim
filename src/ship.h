//	classes for ship model
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
#ifndef SHIP_H
#define SHIP_H

#include "spline.h"

typedef Spline<float> ForceTable;

#include "engine.h"
#include "prop.h"

struct Rudder {
	float location;
	float area;
	int angle;
	float cs;
	float sn;
	float cFwdSpeed;
	float cBackSpeed;
	float cFwdThrust;
	float cBackThrust;
	void set(int degrees) {
		angle= degrees;
		if (angle > 45)
			angle= 45;
		if (angle < -45)
			angle= -45;
		float rad= angle*3.14159/180;
		cs= cos(rad);
		sn= sin(rad);
	}
	void move(int degrees) { set(angle+degrees); }
	Rudder() {
		location= 16;
		area= 7.4;
		angle= 0;
		cs= 1;
		sn= 0;
		cFwdSpeed= 1e5;
		cBackSpeed= 1e4;
		cFwdThrust= 1;
		cBackThrust= 1;
	}
};

struct Cleat;

struct Ship {
	string name;
	osg::MatrixTransform* model;
	Track* track;
	string model3D;
	Model2D* model2D;
	Model2D* boundary;
	Water::Location location;
	double position[3];
	float modelOffset;
	float linVel[2];
	float heading[2];
	float angVel;
	float force[2];
	float torque;
	float mass;
	float inertia;
	float massInv;
	float inertiaInv;
	float draft;
	float lwl;
	float bwl;
	float ddepth;
	float trim;
	float list;
	Water::Location* boundaryLocs;
	double* boundaryXY;
	int calcBoundary;
	ForceTable fwdDrag;
	ForceTable backDrag;
	ForceTable sideDrag;
	ForceTable rotDrag;
	ForceTable sideRotDrag;
	Rudder rudder;
	Engine engine;
	Cleat* cleats;
	Ship();
	~Ship();
	void calcForces();
	void setMass(float m, float sx=1, float sy=1, float sz=1);
	void setInertia(float i);
	void calcDrag(float factor, float speed, float power,
	  float propDiameter);
	void setHeading(float deg);
	float getHeading();
	const float* getHeadingV() { return heading; };
	void setPosition(double x, double y);
	const double* getPosition() { return position; };
	const float* getLinVel() { return linVel; };
	float getAngVel() { return angVel; };
	void setSpeed(float s);
	float getSpeed();
	float getSOG();
	float getCOG();
	void addImpulse(double f, float* dir, double* p);
	void addForceAt(float fx, float fy, double px, double py);
	void getRelPointPos(float x, float y, double* xy) {
		xy[0]= position[0] + heading[0]*x - heading[1]*y;
		xy[1]= position[1] + heading[1]*x + heading[0]*y;
	};
	void getBoundaryPos(int i, double* xy) {
		getRelPointPos(boundary->vertices[i].x,
		  boundary->vertices[i].y,xy);
	};
	void move(double dt);
	double* getBoundaryXY();
	Water::Location* getBoundaryLocs();
	Cleat* findCleat(float x, float y, float maxDist=1);
	void adjustMass();
	Model2D* computeBoundary(float height);
};
typedef map<string,Ship*> ShipMap;
typedef list<Ship*> ShipList;
extern ShipMap shipMap;
extern ShipList shipList;
extern Ship* myShip;

struct Cleat {
	Ship* ship;
	float x;
	float y;
	float z;
	float standingZ;
	Cleat* next;
	void getPosition(double* xy) {
		if (ship->model != NULL) {
			const osg::Matrixd& m= ship->model->getMatrix();
			osg::Vec3d p= m.preMult(osg::Vec3d(x,y,z));
			xy[0]= p[0];
			xy[1]= p[1];
			xy[2]= p[2];
		} else {
			ship->getRelPointPos(x,y,xy);
			xy[2]= z+ship->position[2];
		}
	};
};

struct Rope {
	Cleat* cleat1;
	Cleat* cleat2;
	double xy1[3];
	double xy2[3];
	float length;
	float dist;
	int adjust;
	void calcPosition();
};
typedef list<Rope*> RopeList;
extern RopeList ropeList;
void addRope(Cleat* c1, Cleat* c2);
void addRope(Cleat* cleat, double x, double y, double z);

void moveShips(double dt);
void adjustShipMass();

struct ShipContact {
	int index;
	Ship* ship1;
	Ship* ship2;
	double point1[2];
	double point2[2];
	float dir[2];
	float dist;
	float stretch;
	int isRope;
	ShipContact* next;
};
void moveShips(double dt);

#endif
