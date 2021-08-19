//	equation solver for ship collision system
//
//Copyright 2009 Doug Jones
#ifndef LCPSOLVER_H
#define LCPSOLVER_H

class LCPSolver {
	struct Equation {
		int varID; // w<0, z>=0
		double* coef;
	};
	Equation* equations;
	int nEq;
	int maxEq;
 public:
	LCPSolver();
	~LCPSolver();
	void setEquations(int n, double* a, double* q);
	void setNEquations(int n);
	void setq(double q, int row);
	void seta(double a, int row, int col);
	int solve();
	void getResults(double* w, double* z);
	void printEquations(FILE*);
};

#endif
