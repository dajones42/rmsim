//	Template for cubic spline table lookup
//
//Copyright 2009 Doug Jones
#ifndef SPLINE_H
#define SPLINE_H

#include <vector>
#include "commandreader.h"

template <class T> class Spline {
	std::vector<T> x;
	std::vector<T> y;
	std::vector<T> y2;
	int loIndex;
 public:
	Spline() { loIndex= 0; }
	void clear() {
		x.clear();
		y.clear();
	}
	void add(T nx, T ny) {
		x.push_back(nx);
		y.push_back(ny);
	}
	void compute(T yp1=1e30, T yp2=1e30) {
		int n= x.size();
		if (n <= 0)
			return;
		std::vector<T> u(n);
		y2.resize(n);
		if (yp1 >= 1e30) {
			y2[0]= 0;
			u[0]= 0;
		} else {
			y2[0]= -.5;
			T d= x[1]-x[0];
			u[0]= 3/d * ((y[1]-y[0])/d - yp1);
		}
		for (int i=1; i<n-1; i++) {
			T sig= (x[i]-x[i-1]) / (x[i+1]-x[i-1]);
			T p= sig*y2[i-1] + 2;
			y2[i]= (sig-1)/p;
			u[i]= (6*((y[i+1]-y[i])/(x[i+1]-x[i]) -
			  (y[i]-y[i-1])/(x[i]-x[i-1])) / (x[i+1]-x[i-1]) -
			  sig*u[i-1]) / p;
		}
		if (yp2 >= 1e30) {
			y2[n-1]= 0;
		} else {
			T d= x[n-1]-x[n-2];
			y2[n-1]= (3/d*(yp2-(y[n-1]-y[n-2])/d) - .5*u[n-2]) /
			  (.5*y2[n-2]+1);
		}
		for (int i=n-2; i>=0; i--)
			y2[i]= y2[i]*y2[i+1] + u[i];
	}
	int size() { return x.size(); }
	T getMinX() { return x[0]; }
	T getMaxX() { return x[x.size()-1]; }
	T operator()(T vx) {
		int i= 0;
		int j= x.size()-1;
		if (j < i)
			return 0;
		if (j == i)
			return y[0];
		while (j-i > 1) {
			int k= (i+j)/2;
			if (x[k] > vx)
				j= k;
			else
				i= k;
		}
		T d= x[j]-x[i];
		T a= (x[j]-vx)/d;
		T b= (vx-x[i])/d;
		T vy= a*y[i] + b*y[j];
		if (y2.size()>0 && a>=0 && b>=0)
			vy+= ((a*a*a-a)*y2[i] + (b*b*b-b)*y2[j])*(d*d)/6;
		return vy;
	}
	void copy(Spline<T>& from, T xMult, T yMult) { 
		int n= from.x.size();
		x.reserve(n);
		x.reserve(n);
		for (int i=0; i<n; i++) {
			T x1= xMult*from.x[i];
			T y1= yMult*from.y[i];
			T y2= from.getY(x1);
			if (y2 > y1)
				add(x1,y2);
			else
				add(x1,y1);
		}
	}
	T getX(int i) {
		return x[i];
	}
	T getY(int i) {
		return y[i];
	}
	T getY2(int i) {
		return y2[i];
	}
	void scaleX(T m) {
		for (int i=0; i<x.size(); i++)
			x[i]*= m;
	}
	void scaleY(T m) {
		for (int i=0; i<y.size(); i++)
			y[i]*= m;
		for (int i=0; i<y2.size(); i++)
			y2[i]*= m;
	}
};

template <class T> class SplineParser : public CommandBlockHandler {
	Spline<T>* spline;
	bool linear;
 public:
	SplineParser(Spline<T>* s) { spline= s; linear= false; };
	virtual bool handleCommand(CommandReader& reader) {
		if (reader.getString(0) == "linear") {
			linear= true;
			return true;
		}
		T x= reader.getDouble(0,-1e10,1e10);
		T y= reader.getDouble(1,-1e10,1e10);
		spline->add(x,y);
		return true;
	}
	virtual void handleBeginBlock(CommandReader& reader) {
		spline->clear();
	};
	virtual void handleEndBlock(CommandReader& reader) {
		if (!linear)
			spline->compute();
	};
};

#endif
