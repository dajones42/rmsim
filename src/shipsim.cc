//	ship collision detection code
//	ropes are models as inverted collisions
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
#include "lcpsolver.h"
#include "geometry.h"

ShipContact* freeContacts= NULL;
ShipContact* contacts= NULL;
int nContacts= 0;
int maxContacts= 0;
double* lcpMatrix;
double* lcpQ;
double* lcpW;
double* lcpZ;

LCPSolver lcpSolver;

int shipShoreCount= 0;

void allocSpace()
{
	if (nContacts <= maxContacts)
		return;
	if (maxContacts > 0) {
		delete[] lcpMatrix;
		delete[] lcpQ;
		delete[] lcpW;
		delete[] lcpZ;
	}
	maxContacts= nContacts;
	lcpMatrix= new double[maxContacts*maxContacts];
	lcpQ= new double[maxContacts];
	lcpW= new double[2*maxContacts];
	lcpZ= new double[2*maxContacts];
}

void freeContactList()
{
	while (contacts != NULL) {
		ShipContact* t= contacts;
		contacts= t->next;
		t->next= freeContacts;
		freeContacts= t;
	}
}

void testShipShipContact(Ship* ship1, Ship* ship2)
{
	if (ship2<=ship1 || ship1->boundary==NULL || ship2->boundary==NULL ||
	  (ship1->massInv==0 && ship2->massInv==0))
		return;
	const double* xyz1= ship1->getPosition();
	const double* xyz2= ship2->getPosition();
	float dx= xyz2[0]-xyz1[0];
	float dy= xyz2[1]-xyz1[1];
	float d= ship1->boundary->radius + ship2->boundary->radius;
//	fprintf(stderr,"ss %f %f %f\n",dx,dy,d);
	if (dx*dx+dy*dy > d*d)
		return;
	Model2D* b1= ship1->boundary;
	Model2D* b2= ship2->boundary;
	double s1p1[2],s1p2[2],s2p1[2],s2p2[2];
	ship1->getBoundaryPos(b1->nVertices-1,s1p1);
	ship2->getBoundaryPos(b2->nVertices-1,s2p1);
	ship1->getBoundaryPos(0,s1p2);
	ship2->getBoundaryPos(0,s2p2);
//	fprintf(stderr," %.4lf,%.4lf %.4lf,%.4lf\n",
//	  s1p1[0],s1p1[1],s2p1[0],s2p1[1]);
	int i1= 0;
	int i2= 0;
	int n= 0;
	int nx= 0;
	int inside= 0;
	double p[20][2];
	int m= 0;
	do {
		if (n >= 20)
			n= 19;
		char code= segSegInt(s1p1,s1p2,s2p1,s2p2,p[n]);
		double cross= triArea(0,0,s1p2[0]-s1p1[0],s1p2[1]-s1p1[1],
		  s2p2[0]-s2p1[0],s2p2[1]-s2p1[1]);
		double h12= triArea(s2p1,s2p2,s1p2);
		double h21= triArea(s1p1,s1p2,s2p2);
		if (code=='1' || code=='v') {
			if (n==0 || p[n-1][0]!=p[n][0] || p[n-1][1]!=p[n][1]) {
				if (nx==0 && n>0)
					nx= n;
				n++;
			}
			if (h12 > 0)
				inside= 1;
			else if (h21 > 0)
				inside= 2;
		}
//		fprintf(stderr,
//		  "n=%d %d %d %d %.4lf,%.4lf %.4lf,%.4lf %.4lf,%.4lf\n",
//		  n,i1,i2,inside,s1p2[0],s1p2[1],s2p2[0],s2p2[1],
//		  p[n-1][0],p[n-1][1]);
//		fprintf(stderr,"ssi %c %d %d %d %d %d\n",
//		  code,i1,i2,cross,h12,h21);
		if ((cross>=0 && h21>0) || (cross<0 && h12<=0)) {
			i1= (i1+1)%b1->nVertices;
			if (i1 == 0)
				m++;
			if (inside == 1) {
				p[n][0]= s1p2[0];
				p[n][1]= s1p2[1];
				n++;
			}
			s1p1[0]= s1p2[0];
			s1p1[1]= s1p2[1];
			ship1->getBoundaryPos(i1,s1p2);
		} else {
			i2= (i2+1)%b2->nVertices;
			if (i2 == 0)
				m++;
			if (inside==2) {
				p[n][0]= s2p2[0];
				p[n][1]= s2p2[1];
				n++;
			}
			s2p1[0]= s2p2[0];
			s2p1[1]= s2p2[1];
			ship2->getBoundaryPos(i2,s2p2);
		}
	} while (m < 3);
	if (nx == 0)
		return;
//	fprintf(stderr,"%s and %s are close %d %lf,%lf %lf,%lf\n",
//	  ship1->name.c_str(),ship2->name.c_str(),n,
//	  p[0][0],p[0][1],p[1][0],p[1][1]);
	ShipContact* c= freeContacts;
	if (c != NULL)
		freeContacts= c->next;
	else
		c= new ShipContact;
	c->point1[0]= c->point2[0]= .5*(p[0][0]+p[nx][0]);
	c->point1[1]= c->point2[1]= .5*(p[0][1]+p[nx][1]);
	c->dir[0]= p[0][1]-p[nx][1];
	c->dir[1]= p[nx][0]-p[0][0];
	d= sqrtIter((c->dir[0]>0?c->dir[0]:-c->dir[0])+
	  (c->dir[1]>0?c->dir[1]:-c->dir[1]),
	  c->dir[0]*c->dir[0]+c->dir[1]*c->dir[1]);
	if (d != 0) {
		c->dir[0]/= d;
		c->dir[1]/= d;
	}
//	fprintf(stderr," %lf %.3lf,%.3lf %.3lf,%.3lf %.3lf,%.3lf\n",
//	  triArea(p[0][0],p[0][1],p[nx][0],p[nx][1],xyz1[0],xyz1[1]),
//	  p[0][0],p[0][1],p[nx][0],p[nx][1],xyz1[0],xyz1[1]);
	if (triArea(p[0],p[nx],xyz1) > 0) {
		c->ship1= ship1;
		c->ship2= ship2;
	} else {
		c->ship1= ship2;
		c->ship2= ship1;
	}
	c->dist= 0;
	c->stretch= 0;
	for (int i=1; i<n; i++) {
		int i1= (i+1)%n;
		double a= triArea(p[0],p[i],p[i1]);
		c->stretch+= a>0 ? a : -a;
//		fprintf(stderr,"area=%lf %.4lf,%.4lf %.4lf,%.4lf %.4lf,%.4lf\n",
//		  a,p[0][0],p[0][1],p[i][0],p[i][1],p[i1][0],p[i1][1]);
	}
	c->isRope= 0;
	c->index= nContacts++;
	c->next= contacts;
	contacts= c;
//	fprintf(stderr,"%s %s %.3f,%.3f %.3f,%.3f %lf %d\n",
//	  c->ship1->name.c_str(),c->ship2->name.c_str(),
//	  c->point1[0],c->point1[1],c->dir[0],c->dir[1],c->stretch,c->index);
}

void testRopeContact(Rope* rope)
{
	if (rope->cleat1->ship->massInv==0 &&
	  (rope->cleat2==NULL || rope->cleat2->ship->massInv==0)) {
		fprintf(stderr,"null rope %s %s %d %lf %lf %d\n",
		rope->cleat1->ship->name.c_str(),
		rope->cleat2==NULL?"ground":rope->cleat2->ship->name.c_str(),
		rope->adjust,rope->length,rope->dist,nContacts);
		return;
	}
	rope->calcPosition();
//	fprintf(stderr,"rope %s %s %d %lf %lf %d\n",
//	  rope->cleat1->ship->name.c_str(),
//	  rope->cleat2==NULL?"ground":rope->cleat2->ship->name.c_str(),
//	  rope->adjust,rope->length,rope->dist,nContacts);
	float dx= rope->xy2[0]-rope->xy1[0];
	float dy= rope->xy2[1]-rope->xy1[1];
	rope->dist= sqrtIter(rope->dist,dx*dx+dy*dy);
	if (rope->dist < rope->length) {
		if (rope->adjust < 0)
			rope->length= rope->dist;
		else
			return;
	}
	if (rope->adjust > 0) {
		rope->length= rope->dist;
		return;
	}
	ShipContact* c= freeContacts;
	if (c != NULL)
		freeContacts= c->next;
	else
		c= new ShipContact;
	c->ship1= rope->cleat1->ship;
	c->point1[0]= rope->xy1[0];
	c->point1[1]= rope->xy1[1];
	c->ship2= rope->cleat2==NULL?NULL:rope->cleat2->ship;
	c->point2[0]= rope->xy2[0];
	c->point2[1]= rope->xy2[1];
	c->dir[0]= (rope->xy2[0]-rope->xy1[0])/rope->dist;
	c->dir[1]= (rope->xy2[1]-rope->xy1[1])/rope->dist;
	c->dist= rope->dist;
	c->stretch= rope->dist-rope->length;
	c->isRope= 1;
	c->index= nContacts++;
	c->next= contacts;
	contacts= c;
	if (rope->adjust < -1) {
		float f= 1e4;
		c->ship1->addForceAt(f*c->dir[0],f*c->dir[1],
		  c->point1[0],c->point1[1]);
		if (c->ship2 != NULL)
			c->ship2->addForceAt(-f*c->dir[0],-f*c->dir[1],
			  c->point2[0],c->point2[1]);
	}
}

void testShipShoreContact(Ship* ship, double* ep1, double* ep2)
{
	if (ship->boundary==NULL || ship->massInv==0)
		return;
	int bn= ship->boundary->nVertices;
	double* bxy= ship->getBoundaryXY();
	double* p1= bxy + 2*(bn-1);
	double cp1[2];
	double cp2[2];
	int n= 0;
	int left1= triArea(ep1,ep2,p1) > 0;
	for (int i=0; i<bn && n<2; i++) {
		double* p2= bxy + 2*i;
		int left2= triArea(ep1,ep2,p2) > 0;
		if (left1 && !left2) {
			char code= segSegInt(p1,p2,ep1,ep2,cp1);
			if (code=='1' || code=='v') {
				n++;
			} else if (triArea(p1,p2,ep1) > 0) {
				cp1[0]= ep1[0];
				cp1[1]= ep1[1];
				n++;
			} else {
				return;
			}
		} else if (!left1 && left2) {
			char code= segSegInt(p1,p2,ep1,ep2,cp2);
			if (code=='1' || code=='v') {
				n++;
			} else if (triArea(p1,p2,ep2) > 0) {
				cp2[0]= ep2[0];
				cp2[1]= ep2[1];
				n++;
			} else {
				return;
			}
		}
		left1= left2;
		p1= p2;
	}
	if (n < 2)
		return;
	ShipContact* c= freeContacts;
	if (c != NULL)
		freeContacts= c->next;
	else
		c= new ShipContact;
	c->point1[0]= c->point2[0]= .5*(cp1[0]+cp2[0]);
	c->point1[1]= c->point2[1]= .5*(cp1[1]+cp2[1]);
	c->dir[0]= cp1[1]-cp2[1];
	c->dir[1]= cp2[0]-cp1[0];
	float d= sqrtIter((c->dir[0]>0?c->dir[0]:-c->dir[0])+
	  (c->dir[1]>0?c->dir[1]:-c->dir[1]),
	  c->dir[0]*c->dir[0]+c->dir[1]*c->dir[1]);
	if (d != 0) {
		c->dir[0]/= d;
		c->dir[1]/= d;
	}
	c->ship1= ship;
	c->ship2= NULL;
	c->dist= 0;
	c->stretch= 0;
	c->isRope= 0;
	c->index= nContacts++;
	c->next= contacts;
	contacts= c;
//	fprintf(stderr,"%s land %.3f,%.3f %.3f,%.3f %lf %d\n",
//	  c->ship1->name.c_str(),
//	  c->point1[0],c->point1[1],c->dir[0],c->dir[1],c->stretch,c->index);
}

int testShipShoreContact(Ship* ship, Water::Vertex* v)
{
	double* vxy= v->xy;
	double* vnxy= v->next->xy;
	double* vpxy= v->prev->xy;
	double va= triArea(vpxy[0],vpxy[1],vxy[0],vxy[1],vnxy[0],vnxy[1]);
	int bn= ship->boundary->nVertices;
	double* bxy= ship->getBoundaryXY();
	double* p1= bxy + 2*(bn-1);
	double* p2= bxy;
	double ap= triArea(vpxy[0],vpxy[1],vxy[0],vxy[1],p1[0],p1[1]);
	double an= triArea(vxy[0],vxy[1],vnxy[0],vnxy[1],p1[0],p1[1]);
	int water1= va>=0 ? (ap>0 && an>0) : (ap>0 || an>0);
	int n= 0;
	double p[20][2];
	int i1= 0;
	int i2= 0;
	for (int j=0; j<bn; j++) {
		double* p2= bxy + 2*j;
		ap= triArea(vpxy[0],vpxy[1],vxy[0],vxy[1],p2[0],p2[1]);
		an= triArea(vxy[0],vxy[1],vnxy[0],vnxy[1],p2[0],p2[1]);
		int water2= va>=0 ? (ap>0 && an>0) : (ap>0 || an>0);
		if (n >= 20)
			n= 19;
		if (water1 != water2) {
			char code= segSegInt(p1,p2,vpxy,vxy,p[n]);
			if (code!='1' && code!='v')
				code= segSegInt(p1,p2,vxy,vnxy,p[n]);
			if ((code=='1' || code=='v') &&
			  (n==0 || p[n-1][0]!=p[n][0] || p[n-1][1]!=p[n][1])) {
				if (water2 == 0)
					i1= n;
				else
					i2= n;
				n++;
			}
		}
//		fprintf(stderr,
//		  "n=%d %d %d %d %.4lf,%.4lf %.4lf,%.4lf %.4lf,%.4lf\n",
//		  n,i1,i2,inside,p2[0],p2[1],s2p2[0],s2p2[1],
//		  p[n-1][0],p[n-1][1]);
		if (water2 == 0) {
			p[n][0]= p2[0];
			p[n][1]= p2[1];
			n++;
		}
		p1= p2;
		water1= water2;
	}
	if (n<2 || i1==i2)
		return 0;
//	fprintf(stderr,"%s and %s are close %d %lf,%lf %lf,%lf\n",
//	  ship->name.c_str(),ship2->name.c_str(),n,
//	  p[0][0],p[0][1],p[1][0],p[1][1]);
	ShipContact* c= freeContacts;
	if (c != NULL)
		freeContacts= c->next;
	else
		c= new ShipContact;
	c->point1[0]= c->point2[0]= .5*(p[0][0]+p[n-1][0]);
	c->point1[1]= c->point2[1]= .5*(p[0][1]+p[n-1][1]);
	c->dir[0]= p[i1][1]-p[i2][1];
	c->dir[1]= p[i2][0]-p[i1][0];
	float d= sqrtIter((c->dir[0]>0?c->dir[0]:-c->dir[0])+
	  (c->dir[1]>0?c->dir[1]:-c->dir[1]),
	  c->dir[0]*c->dir[0]+c->dir[1]*c->dir[1]);
	if (d != 0) {
		c->dir[0]/= d;
		c->dir[1]/= d;
	}
//	fprintf(stderr," %lf %.3lf,%.3lf %.3lf,%.3lf %.3lf,%.3lf\n",
//	  triArea(p[0][0],p[0][1],p[n-1][0],p[n-1][1],xyz1[0],xyz1[1]),
//	  p[0][0],p[0][1],p[n-1][0],p[n-1][1],xyz1[0],xyz1[1]);
	c->ship1= ship;
	c->ship2= NULL;
	c->dist= 0;
	c->stretch= 0;
	for (int i=1; i<n; i++) {
		int i1= (i+1)%n;
		double a= triArea(p[0][0],p[0][1],
		  p[i][0],p[i][1],p[i1][0],p[i1][1]);
		c->stretch+= a>0 ? a : -a;
//		fprintf(stderr,"area=%lf %.4lf,%.4lf %.4lf,%.4lf %.4lf,%.4lf\n",
//		  a,p[0][0],p[0][1],p[i][0],p[i][1],p[i1][0],p[i1][1]);
	}
	c->isRope= 0;
	c->index= nContacts++;
	c->next= contacts;
	contacts= c;
//	fprintf(stderr,"%s land %.3f,%.3f %.3f,%.3f %lf\n",
//	  c->ship1->name.c_str(),
//	  c->point1[0],c->point1[1],c->dir[0],c->dir[1],c->stretch);
	return 1;
}

void testShipShoreContact(Ship* ship)
{
	if (ship->boundary == NULL)
		return;
	Water::Triangle* t= ship->location.triangle;
	if (t == NULL)
		return;
	shipShoreCount++;
	const double* xy= ship->getPosition();
	double threshold= 1.1*ship->boundary->radius*ship->boundary->radius;
	for (int j=0; j<3; j++) {
		Water::Vertex* v= t->v[j];
		if (v->next==NULL && v->prev!=NULL)
			v= v->prev;
		if (v->next==NULL || v->prev==NULL)
			continue;
		for (Water::Vertex* v1=v; v1!=NULL && v1->next!=NULL;
		  v1=v1->next) {
			if (v1->label == shipShoreCount)
				break;
			v1->label= shipShoreCount;
			if (lineSegDistSq(xy[0],xy[1],v1->xy[0],v1->xy[1],
			  v1->next->xy[0],v1->next->xy[1]) > threshold)
				break;
			testShipShoreContact(ship,v1->xy,v1->next->xy);
		}
		for (Water::Vertex* v1=v->prev; v1!=NULL && v1->next!=NULL;
		  v1=v1->prev) {
			if (v1->label == shipShoreCount)
				break;
			v1->label= shipShoreCount;
			if (lineSegDistSq(xy[0],xy[1],v1->xy[0],v1->xy[1],
			  v1->next->xy[0],v1->next->xy[1]) > threshold)
				break;
			testShipShoreContact(ship,v1->xy,v1->next->xy);
		}
	}
}

void findContacts()
{
	nContacts= 0;
	for (RopeList::iterator i=ropeList.begin(); i!=ropeList.end(); ++i)
		testRopeContact(*i);
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i)
		for (ShipList::iterator j=shipList.begin(); j!=shipList.end();
		  ++j)
			testShipShipContact(*i,*j);
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i)
		testShipShoreContact(*i);
}

float relPosCrossDir(Ship* ship, double* point, float* dir)
{
	const double* xyz= ship->getPosition();
	return (point[0]-xyz[0])*dir[1] - (point[1]-xyz[1])*dir[0];
}

inline float v2dot(float* v1, float* v2)
{
	return v1[0]*v2[0] + v1[1]*v2[1];
}

void calcLCPMatrix()
{
	for (ShipContact* ci=contacts; ci!=NULL; ci=ci->next) {
		double* a= lcpMatrix+nContacts*ci->index;
		float r1ni= relPosCrossDir(ci->ship1,ci->point1,ci->dir);
		float r2ni= ci->ship2==NULL ? 0 :
		  relPosCrossDir(ci->ship2,ci->point2,ci->dir);
		for (ShipContact* cj=contacts; cj!=NULL; cj=cj->next) {
			float r1nj=
			  relPosCrossDir(cj->ship1,cj->point1,cj->dir);
			float r2nj= cj->ship2==NULL ? 0 :
			  relPosCrossDir(cj->ship2,cj->point2,cj->dir);
			float sum= 0;
			if (ci->ship1 == cj->ship1) {
				sum+= v2dot(ci->dir,cj->dir)*ci->ship1->massInv;
				sum+= r1ni*r1nj*ci->ship1->inertiaInv;
			} else if (ci->ship1 == cj->ship2) {
				sum-= v2dot(ci->dir,cj->dir)*ci->ship1->massInv;
				sum-= r1ni*r2nj*ci->ship1->inertiaInv;
			}
			if (ci->ship2 == NULL) {
				a[cj->index]= sum;
				continue;
			}
			if (ci->ship2 == cj->ship2) {
				sum+= v2dot(ci->dir,cj->dir)*ci->ship2->massInv;
				sum+= r2ni*r2nj*ci->ship2->inertiaInv;
			} else if (ci->ship2 == cj->ship1) {
				sum-= v2dot(ci->dir,cj->dir)*ci->ship2->massInv;
				sum-= r2ni*r1nj*ci->ship2->inertiaInv;
			}
			a[cj->index]= sum;
		}
	}
}

void relPosVel(Ship* ship, double* point, float* r, float* rv, float* ra)
{
	const double* xyz= ship->getPosition();
	const float* v= ship->getLinVel();
	float w= ship->getAngVel();
	r[0]= point[0]-xyz[0];
	r[1]= point[1]-xyz[1];
	rv[0]= v[0] - w*r[1];
	rv[1]= v[1] + w*r[0];
	if (ra != NULL) {
		float t= ship->torque*ship->inertiaInv;
		ra[0]= ship->force[0]*ship->massInv - t*r[1] - w*w*r[0];
		ra[1]= ship->force[1]*ship->massInv + t*r[0] - w*w*r[1];
	}
}

void calcPreSpeed(int addStretch)
{
	for (ShipContact* c=contacts; c!=NULL; c=c->next) {
		float x1[2],v1[2];
		relPosVel(c->ship1,c->point1,x1,v1,NULL);
		if (c->ship2 != NULL) {
			float x2[2],v2[2];
			relPosVel(c->ship2,c->point2,x2,v2,NULL);
//			fprintf(stderr,"%lf %lf %lf %lf\n",
//			  v1[0],v1[1],v2[0],v2[1]);
			v1[0]-= v2[0];
			v1[1]-= v2[1];
		}
		lcpQ[c->index]= v2dot(c->dir,v1);
#if 0
		if (c->isRope || addStretch==0)
			continue;
		if (lcpQ[c->index]<.1 && c->stretch>1)
			lcpQ[c->index]-= .1;
		else if (lcpQ[c->index]<.1 && c->stretch>.1)
			lcpQ[c->index]-= .1*c->stretch;
#endif
	}
}

void 	doImpulse()
{
#if 0
	double q0[10];
	for (ShipContact* c=contacts; c!=NULL; c=c->next)
		q0[c->index]= lcpQ[c->index];
	for (ShipContact* c=contacts; c!=NULL; c=c->next) {
		double f= lcpZ[c->index];
		c->ship1->addImpulse(100,c->dir,c->point1);
		if (c->ship2 != NULL)
			c->ship2->addImpulse(-100,c->dir,c->point2);
		calcPreSpeed(0);
		fprintf(stderr,"%d %lf:",c->index,f);
		for (ShipContact* c1=contacts; c1!=NULL; c1=c1->next)
			fprintf(stderr," %lf",lcpQ[c1->index]-q0[c1->index]);
		fprintf(stderr,"\n");
		c->ship1->addImpulse(-100,c->dir,c->point1);
		if (c->ship2 != NULL)
			c->ship2->addImpulse(100,c->dir,c->point2);
	}
#endif
	for (ShipContact* c=contacts; c!=NULL; c=c->next) {
		double f= lcpZ[c->index];
		c->ship1->addImpulse(f,c->dir,c->point1);
		if (c->ship2 != NULL)
			c->ship2->addImpulse(-f,c->dir,c->point2);
//		fprintf(stderr," %lf",f);
	}
//	fprintf(stderr,"\n");
#if 0
	calcPreSpeed(0);
	for (ShipContact* c=contacts; c!=NULL; c=c->next)
		fprintf(stderr," %lf",lcpQ[c->index]);
	fprintf(stderr,"\n");
#endif
//	for (ShipContact* c=contacts; c!=NULL; c=c->next)
//		if (lcpQ[c->index] < -.1)
//			exit(0);
}

void 	calcPreAccel()
{
	for (ShipContact* c=contacts; c!=NULL; c=c->next) {
		float x1[2],v1[2],a1[2];
		relPosVel(c->ship1,c->point1,x1,v1,a1);
		if (c->ship2 != NULL) {
			float x2[2],v2[2],a2[2];
			relPosVel(c->ship2,c->point2,x2,v2,a2);
			x1[0]-= x2[0];
			x1[1]-= x2[1];
			v1[0]-= v2[0];
			v1[1]-= v2[1];
			a1[0]-= a2[0];
			a1[1]-= a2[1];
		}
		lcpQ[c->index]= v2dot(c->dir,a1);
		if (c->dist > 0) {
			lcpQ[c->index]+= v2dot(v1,v1)/c->dist;
			float xdotv= v2dot(x1,v1);
			lcpQ[c->index]-= .5*xdotv*xdotv/(v2dot(x1,x1)*c->dist);
		}
	}
}

void 	adjustForces()
{
	for (ShipContact* c=contacts; c!=NULL; c=c->next) {
		double f= lcpZ[c->index];
//		fprintf(stderr," %lf",f);
		if (f <= 0)
			continue;
	//	if (c->stretch > .2)
	//		f+= (c->stretch-.2)*(10*f+1e4);
		c->ship1->addForceAt(f*c->dir[0],f*c->dir[1],
		  c->point1[0],c->point1[1]);
		if (c->ship2 != NULL)
			c->ship2->addForceAt(-f*c->dir[0],-f*c->dir[1],
			  c->point2[0],c->point2[1]);
		if (c->isRope)
			continue;
#if 0
		float ndir[2];
		ndir[0]= -c->dir[1];
		ndir[1]= c->dir[0];
		float x1[2],v1[2];
		relPosVel(c->ship1,c->point1,x1,v1,NULL);
		if (c->ship2 != NULL) {
			float x2[2],v2[2];
			relPosVel(c->ship2,c->point2,x2,v2,NULL);
			v1[0]-= v2[0];
			v1[1]-= v2[1];
		}
		float v= v2dot(ndir,v1);
		fprintf(stderr," fv(%f)",v);
		if (v > .1)
			f*= -.2;
		else if (v < -.1)
			f*= .2;
		else
			continue;
		c->ship1->addForceAt(f*ndir[0],f*ndir[1],
		  c->point1[0],c->point1[1]);
		if (c->ship2 != NULL)
			c->ship2->addForceAt(-f*ndir[0],-f*ndir[1],
			  c->point2[0],c->point2[1]);
#endif
	}
//	fprintf(stderr,"\n");
}

void findImpulse()
{
#if 0
	lcpSolver.setNEquations(2*nContacts);
	for (int i=0; i<nContacts; i++) {
		lcpSolver.setq(-1,i);
		lcpSolver.setq(lcpQ[i],i+nContacts);
		for (int j=0; j<nContacts; j++) {
			double a= lcpMatrix[i*nContacts+j];
			lcpSolver.seta(a,j,i+nContacts);
			lcpSolver.seta(-a,i+nContacts,j);
		}
	}
#endif
	lcpSolver.setEquations(nContacts,lcpMatrix,lcpQ);
//	lcpSolver.printEquations(stderr);
	if (lcpSolver.solve() == 0) {
//		lcpSolver.printEquations(stderr);
		for (int i=0; i<nContacts; i++)
			lcpZ[i]= 0;
	} else {
//		lcpSolver.printEquations(stderr);
		lcpSolver.getResults(lcpW,lcpZ);
	}
}

void moveShips(double dt)
{
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i)
		(*i)->calcForces();
	findContacts();
	if (nContacts > 0) {
		allocSpace();
		calcLCPMatrix();
		calcPreSpeed(1);
		findImpulse();
		doImpulse();
		calcPreAccel();
		lcpSolver.setEquations(nContacts,lcpMatrix,lcpQ);
//		lcpSolver.printEquations(stderr);
		lcpSolver.solve();
//		lcpSolver.printEquations(stderr);
		lcpSolver.getResults(lcpW,lcpZ);
		adjustForces();
		freeContactList();
//		timeMult= 0;
	}
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i) {
		Ship* ship= *i;
		ship->move(dt);
	}
}

//	adjusts ship masses to account for railcars moving onto ships
void adjustShipMass()
{
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i) {
		Ship* ship= *i;
		if (ship->track == NULL)
			continue;
		ship->track->sumMass= 0;
		ship->track->sumXMass= 0;
		ship->track->sumYMass= 0;
		ship->track->sumZMass= 0;
	}
	for (FBList::iterator i=fbList.begin(); i!=fbList.end(); ++i) {
		FloatBridge* fb= *i;
		if (fb->carFloat == NULL)
			continue;
		for (FloatBridge::FBTrackList::iterator j=fb->tracks.begin();
		  j!=fb->tracks.end(); ++j) {
			FloatBridge::FBTrack* fbt= *j;
			fbt->track->sumMass= 0;
			fbt->track->sumXMass= 0;
			fbt->track->sumYMass= 0;
			fbt->track->sumZMass= 0;
		}
	}
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t->moving<=0 || (t->location.edge->track->matrix==NULL &&
		  t->endLocation.edge->track->matrix==NULL))
			continue;
		for (RailCarInst* c=t->firstCar; c!=NULL; c=c->next) {
			for (int j=0; j<c->wheels.size(); j++) {
				Track* track= c->wheels[j].location.edge->track;
				if (track->matrix != NULL) {
					float m= c->mass/c->wheels.size();
					WLocation loc;
					c->wheels[j].location.getWLocation(
					  &loc,1);
					track->sumMass+= m;
					track->sumXMass+= m*loc.coord[0];
					track->sumYMass+= m*loc.coord[1];
					track->sumZMass+= m*loc.coord[2];
				}
			}
		}
	}
	for (FBList::iterator i=fbList.begin(); i!=fbList.end(); ++i) {
		FloatBridge* fb= *i;
		if (fb->carFloat == NULL)
			continue;
		for (FloatBridge::FBTrackList::iterator j=fb->tracks.begin();
		  j!=fb->tracks.end(); ++j) {
			FloatBridge::FBTrack* fbt= *j;
			Track* track= fbt->track;
			if (track->sumMass == 0)
				continue;
			Track* cftrack= fb->carFloat->track;
			float x= (track->sumXMass/track->sumMass+fb->length/2)/
			  fb->length;
			cftrack->sumMass+= x*track->sumMass;
			cftrack->sumXMass+= x*track->sumXMass;
			cftrack->sumYMass-= x*track->sumYMass*fbt->offset;
			cftrack->sumZMass+= x*track->sumZMass;
		}
	}
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i)
		(*i)->adjustMass();
}
