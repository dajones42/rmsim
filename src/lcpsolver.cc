//	Linear Complementarity Problem solver
//	used to compute ship collision forces
//	see Game Physics by David Eberly
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
#include <stdio.h>
#include <string.h>
#include "lcpsolver.h"

LCPSolver::LCPSolver()
{
	maxEq= 0;
}

LCPSolver::~LCPSolver()
{
	for (int i=0; i<maxEq; i++)
		delete equations[i].coef;
	if (maxEq > 0)
		delete equations;
}

void LCPSolver::setNEquations(int n)
{
	if (n > maxEq) {
		for (int i=0; i<maxEq; i++)
			delete equations[i].coef;
		if (maxEq > 0)
			delete equations;
		maxEq= n;
		equations= new Equation[maxEq];
		for (int i=0; i<maxEq; i++)
			equations[i].coef= new double[2+2*maxEq];
	}
	nEq= n;
	for (int i=0; i<nEq; i++) {
		Equation* eq= &equations[i];
		eq->varID= -1-i;
		memset(eq->coef,0,(2+nEq*2)*sizeof(double));
	}
}

void LCPSolver::setEquations(int n, double* a, double* q)
{
	setNEquations(n);
	for (int i=0; i<nEq; i++) {
		Equation* eq= &equations[i];
		eq->coef[0]= q[i];
		double* ai= a+i*nEq;
		for (int j=0; j<nEq; j++) {
			eq->coef[j+2]= ai[j];
			if (ai[j] != 0)
				eq->coef[1]= 1;
		}
	}
}

void LCPSolver::setq(double q, int row)
{
	Equation* eq= &equations[row];
	eq->coef[0]= q;
}

void LCPSolver::seta(double a, int row, int col)
{
	Equation* eq= &equations[row];
	eq->coef[col+2]= a;
	if (a != 0)
		eq->coef[1]= 1;
}

int LCPSolver::solve()
{
	int besti= -1;
	for (int i=0; i<nEq; i++) {
		Equation* eq= &equations[i];
		if (eq->coef[0] >= 0)
			continue;
		if (eq->coef[1] == 0)
			continue;
		if (besti<0 || eq->coef[0]<equations[besti].coef[0])
			besti= i;
	}
	if (besti < 0)
		return 1;
	int entering= 0;
	int n2= 2*nEq+2;
	for (;;) {
		Equation* eq1= &equations[besti];
		int col1= entering >= 0 ? 1+entering : 1+nEq-entering;
		double x= -1/eq1->coef[col1];
		for (int i=0; i<n2; i++)
			eq1->coef[i]*= x;
		int leaving= eq1->varID;
		int col2= leaving >= 0 ? 1+leaving : 1+nEq-leaving;
		eq1->coef[col2]= -x;
		eq1->varID= entering;
		eq1->coef[col1]= 0;
#if 0
		fprintf(stderr,"%d %d %d %d %lf\n",
		  entering,leaving,col1,col2,x);
		fprintf(stderr,"%3d:",eq1->varID);
		for (int j=0; j<n2; j++)
			fprintf(stderr," %lf",eq1->coef[j]);
		fprintf(stderr,"\n");
#endif
		for (int i=0; i<nEq; i++) {
			if (i == besti)
				continue;
			Equation* eq2= &equations[i];
			double x= eq2->coef[col1];
			if (x == 0)
				continue;
			for (int j=0; j<n2; j++)
				eq2->coef[j]+= x*eq1->coef[j];
			eq2->coef[col1]= 0;
		}
//		printEquations(stderr);
		if (leaving == 0)
			return 1;
		entering= -leaving;
		col1= entering >= 0 ? 1+entering : 1+nEq-entering;
		besti= -1;
		double bestr= 0;
		for (int i=0; i<nEq; i++) {
			Equation* eq= &equations[i];
			if (eq->coef[col1] >= 0)
				continue;
			double r= eq->coef[0]/eq->coef[col1];
			if (besti<0 || r>bestr) {
				besti= i;
				bestr= r;
			}
		}
		if (besti<0)
			return 0;
	}
}

void LCPSolver::getResults(double* w, double* z)
{
	memset(w,0,nEq*sizeof(double));
	memset(z,0,nEq*sizeof(double));
	for (int i=0; i<nEq; i++) {
		Equation* eq= &equations[i];
		if (eq->varID < 0)
			w[-eq->varID-1]= eq->coef[0];
		else if (eq->varID > 0)
			z[eq->varID-1]= eq->coef[0];
	}
}

void LCPSolver::printEquations(FILE* out)
{
	int n2= 2*nEq+2;
	for (int i=0; i<nEq; i++) {
		Equation* eq= &equations[i];
		fprintf(out,"%3d:",eq->varID);
		for (int j=0; j<n2; j++)
			fprintf(out," %lg",eq->coef[j]);
		fprintf(out,"\n");
	}
}
