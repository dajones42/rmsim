//	air brake model code
//
//Copyright 2018 Doug Jones

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

#include "airtank.h"

//	1 atmoshpere in Pa
const float AirTank::STDATM= 101325;

//	conversion factor for psi to Pa
const float AirTank::PSI2PA= 6894.8;

//	air density in kg per cubic meter at 20 degrees C and 1 atmosphere
const float AirTank::DENSITY= 1.204;

//	dynamic viscosity of air at 20 degrees C
const float AirTank::VISCOSITY= 1.825-5;

//	ratio of specific heat at constant pressure and
//	specific heat at constant volume for air
const float AirTank::HEATRATIO= 1.4;

float AirTank::getPsig()
{
	return (pressure-STDATM) / PSI2PA;
}

void AirTank::setPsig(float psig)
{
	pressure= psig*PSI2PA + STDATM;
}

//	Returns air density in tank in kg per cubic meter
float AirTank::getDensity()
{
	return pressure*DENSITY/STDATM;
}

void AirTank::addAir(float kg)
{
	pressure+= kg/volume*STDATM/DENSITY;
}

void AirTank::moveAir(AirTank* other, float kg)
{
	float p0= pressure;
	float p= (pressure*volume + other->pressure*other->volume) /
	  (volume+other->volume);
	addAir(-kg);
	other->addAir(kg);
	if ((p>p0 && pressure>p) || (p<p0 && pressure<p)) {
		pressure= p;
		other->pressure= p;
	}
}

float AirTank::moveAir(AirTank* other, float area, float timeStep)
{
	float kg= timeStep*massFlowRate(pressure,other->pressure,area);
	moveAir(other,kg);
	return kg;
}

float AirTank::vent(float area, float timeStep)
{
	float kg= timeStep*massFlowRate(pressure,STDATM,area);
	addAir(-kg);
	if (pressure < STDATM)
		pressure= STDATM;
	return kg;
}

void AirPipe::addAir(float kg)
{
	float d= getDensity();
	AirTank::addAir(kg);
	if (kg > 0)
		airSpeed*= d/getDensity();
}

//	Updates the speed of air moving in the pipe based on momentum
//	conservation.
//	Differential equations from "Railway Air Brake Model and Parallel
//	Computing Scheme", Qing Wu et al, Journal of Computational and
//	Nonlinear Dynamics, Sept 2017, Vol 12.
//	Positive air speed is from next to prev.
//	Negative air speed is limited to steady flow rate to prevent
//	oscillations that cause accidental brake release.
void AirPipe::updateAirSpeed(float timeStep)
{
//	fprintf(stderr,"uas %p %p %p %d %d\n",this,next,prev,nextOpen,prevOpen);
	float dpdx= 0;
	float accel= 0;
	if (prev && prevOpen) {
		dpdx+= (pressure-prev->pressure) / (.5*(length+prev->length));
		accel-= airSpeed*(airSpeed-prev->airSpeed)/length;
	} else if (prevOpen) {
		dpdx+= (pressure-STDATM) / (.5*length);
	} else {
		accel-= airSpeed*airSpeed/length;
	}
	if (next && nextOpen) {
		dpdx+= (next->pressure-pressure) / (.5*(next->length+length));
		accel-= airSpeed*(next->airSpeed-airSpeed)/length;
	} else if (nextOpen) {
		dpdx+= (STDATM-pressure) / (.5*length);
	} else {
		accel-= -airSpeed*airSpeed/length;
	}
	float density= getDensity();
	accel-= dpdx/density;
	if (airSpeed > 0)
		accel-= friction*airSpeed*airSpeed/(2*diameter);
	else
		accel+= friction*airSpeed*airSpeed/(2*diameter);
	airSpeed+= timeStep*accel;
	float maxSpeed= sqrt(fabs(dpdx)/density*2*diameter/friction);
	if (airSpeed < -maxSpeed)
		airSpeed= -maxSpeed;
	airFlow= timeStep*airSpeed*density*diameter*diameter/4*M_PI;
}

//	Returns air mass flow rate between two volumes with absolute
//	pressures p1 and p2 connected by an opening with area area.
//	The result will be positive when p1 is upstream (larger) and
//	otherwise negative.
//	Equations from "Railway Air Brake Model and Parallel
//	Computing Scheme", Qing Wu et al, Journal of Computational and
//	Nonlinear Dynamics, Sept 2017, Vol 12.
//	Based on steady state flow in a converging nozzle.
float AirTank::massFlowRate(float p1, float p2, float area)
{
	static float threshold= 0;
	static float chokedMult= 0;
	static float speedOfSound= 0;
	if (threshold == 0) {	// calculate constants once to save time
		threshold= pow(2/(HEATRATIO+1),HEATRATIO/(HEATRATIO-1));
		speedOfSound= sqrt(HEATRATIO*STDATM/DENSITY);
		chokedMult= HEATRATIO*
		  pow(2/(HEATRATIO+1),.5*(HEATRATIO+1)/(HEATRATIO-1))/
		  speedOfSound;
	}
	if (p1 == p2)
		return 0;
	if (p2 > p1)
		return -massFlowRate(p2,p1,area);
	float r= p2/p1;
	if (r > threshold) { // subsonic flow
		return p1*area*
		  sqrt(2*HEATRATIO*HEATRATIO/(HEATRATIO-1)*
		  pow(r,2/HEATRATIO) *
		  (1-pow(r,(HEATRATIO-1)/HEATRATIO))) / speedOfSound;
	} else {	// choked flow
		return p1*area*chokedMult;
	}
}
