//	Gode homolosine map projection functions
//
//Copyright 2009 Doug Jones
#ifndef GHPROJ_H
#define GHPROJ_H

struct GHProjection {
	double R;
	double lon_center[12];
	double feast[12];
	GHProjection(double r=6370997.);
	void ll2xy(double lat, double lng, double* x, double *y);
	int xy2ll(double x, double y, double* lat, double *lng);
};

#endif
