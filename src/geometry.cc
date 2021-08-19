//	misc geometry functions
//
//Copyright 2009 Doug Jones
#include <stdio.h>

//	returns twice the area of a triangle
//	positive if the points are listed in clockwise order, else neative
double triArea(double x1, double y1, double x2, double y2, double x3, double y3)
{
	return (x2-x1)*(y3-y1) - (x3-x1)*(y2-y1);
}

inline int between(double* a, double* b, double*c)
{
	if (a[0]==b[0])
		return (a[1]<=c[1] && c[1]<=b[1]) ||
		  (a[1]>=c[1] && c[1]>=b[1]);
	else
		return (a[0]<=c[0] && c[0]<=b[0]) ||
		  (a[0]>=c[0] && c[0]>=b[0]);
}

//	finds the intersection between two line segments
//	TODO: say what return codes are
char segSegInt(double* p11, double* p12, double* p21, double* p22, double* p,
  double* sp, double* tp)
{
//	fprintf(stderr,"%f,%f %f,%f\n",p11[0],p11[1],p12[0],p12[1]);
//	fprintf(stderr,"%f,%f %f,%f\n",p21[0],p21[1],p22[0],p22[1]);
	double d= p11[0]*(p22[1]-p21[1]) + p12[0]*(p21[1]-p22[1]) +
	  p21[0]*(p11[1]-p12[1]) + p22[0]*(p12[1]-p11[1]);
	if (d == 0) {
		if (triArea(p11[0],p11[1],p12[0],p12[1],p21[0],p21[1]) != 0)
			return '0';
//		fprintf(stderr,"collinear\n");
		if (between(p11,p12,p21)) {
			p[0]= p21[0];
			p[1]= p21[1];
			return 'e';
		}
		if (between(p11,p12,p22)) {
			p[0]= p22[0];
			p[1]= p22[1];
			return 'e';
		}
		if (between(p21,p22,p11)) {
			p[0]= p11[0];
			p[1]= p11[1];
			return 'e';
		}
		if (between(p21,p22,p12)) {
			p[0]= p12[0];
			p[1]= p12[1];
			return 'e';
		}
		return '0';
	}
	double ns= p11[0]*(p22[1]-p21[1]) + p21[0]*(p11[1]-p22[1]) +
	  p22[0]*(p21[1]-p11[1]);
	double s= ns/d;
	if (s<0 || s>1)
		return '0';
	double nt= -(p11[0]*(p21[1]-p12[1]) + p12[0]*(p11[1]-p21[1]) +
	  p21[0]*(p12[1]-p11[1]));
	double t= nt/d;
	if (t<0 || t>1)
		return '0';
	p[0]= p11[0] + s*(p12[0]-p11[0]);
	p[1]= p11[1] + s*(p12[1]-p11[1]);
	if (sp)
		*sp= s;
	if (tp)
		*tp= t;
	if (ns==0 || ns==d || nt==0 || nt==d)
		return 'v';
	return '1';
}

//	returns square of distance between a point and line segment in 2D
double lineSegDistSq(double x, double y, double x1, double y1,
  double x2, double y2)
{
	double dx= x2-x1;
	double dy= y2-y1;
	double d= dx*dx + dy*dy;
	double n= dx*(x-x1) + dy*(y-y1);
	if (d==0 || n<=0) {
		dx= x1-x;
		dy= y1-y;
	} else if (n >= d) {
		dx= x2-x;
		dy= y2-y;
	} else {
		dx= x1 + dx*n/d - x;
		dy= y1 + dy*n/d - y;
	}
	return dx*dx + dy*dy;
}

//	returns square of distance between a point and line segment in 3D
double lineSegDistSq(double x, double y, double z,
  double x1, double y1, double z1, double x2, double y2, double z2)
{
	double dx= x2-x1;
	double dy= y2-y1;
	double dz= z2-z1;
	double d= dx*dx + dy*dy + dz*dz;
	double n= dx*(x-x1) + dy*(y-y1) + dz*(z-z1);
	if (d==0 || n<=0) {
		dx= x1-x;
		dy= y1-y;
		dz= z1-z;
	} else if (n >= d) {
		dx= x2-x;
		dy= y2-y;
		dz= z2-z;
	} else {
		dx= x1 + dx*n/d - x;
		dy= y1 + dy*n/d - y;
		dz= z1 + dz*n/d - z;
	}
	return dx*dx + dy*dy + dz*dz;
}

float segAxisInt(float x1, float y1, float x2, float y2, float x)
{
	double d= x2-x1;
	if (d == 0)
		return y1;
	double n= y2-y1;
	return y1 + (x-x1)*n/d;
}
