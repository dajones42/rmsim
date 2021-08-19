//	misc functions for geometric calculation
//
//Copyright 2009 Doug Jones
#ifndef GEOMETRY_H
#define GEOMETRY_H

//	finds the point of intersection between two line sements
char segSegInt(double* p11, double* p12, double* p21, double* p22, double* p,
  double* sp=NULL, double* tp=NULL);

//	finds the point of intersection between two line sements
float segAxisInt(float x1, float y1, float x2, float y2, float x);

//	returns the signed area of a triangle
//	positive if the points are listed in clockwise order
double triArea(double x1, double y1, double x2, double y2,
  double x3, double y3);

//	returns square of distance between a point and line in 2D
double lineSegDistSq(double x, double y, double x1, double y1,
  double x2, double y2);

//	returns square of distance between a point and line in 3D
double lineSegDistSq(double x, double y, double z,
  double x1, double y1, double z1, double x2, double y2, double z2);

//	returns the signed area of a triangle
inline double triArea(const double* p1, const double* p2, const double* p3)
{
	return triArea(p1[0],p1[1],p2[0],p2[1],p3[0],p3[1]);
}

//	returns an estimated square root
//	uses a single interation of newtons method
inline float sqrtIter(float est, float sq)
{
	return est<=0 ? sqrt(sq) : .5*(est+sq/est);
}

#endif
