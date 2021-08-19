//	air brake model code
//
//Copyright 2009,2018 Doug Jones

#include "stdio.h"
#include "stdlib.h"
#include "math.h"

using namespace std;
#include <string>

#include "airbrake.h"

AirBrake::AirBrake(string brakeValve)
{
	next= NULL;
	prev= NULL;
	nextOpen= false;
	prevOpen= false;
	cutOut= false;
	retainerControl= 0;
	valve= BrakeValve::get(brakeValve);
	valveState= 0;
	bpIndex= 0;
	arIndex= 1;
	bcIndex= 2;
	if (valve) {
		for (int i=0; i<valve->tanks.size(); i++) {
			BrakeValve::Tank* t= valve->tanks[i];
			if (t->volume > 0) {
				tanks.push_back(new AirTank(t->volume));
			} else {
				AirPipe* p= new AirPipe(t->pipeLength,.032);
				pipes.push_back(p);
				tanks.push_back(p);
			}
		}
		bpIndex= valve->getTankIndex("BP");
		arIndex= valve->getTankIndex("AR");
		bcIndex= valve->getTankIndex("BC");
	}
}

void AirBrake::setNext(AirBrake* p)
{
	next= p;
	for (int i=0; i<pipes.size(); i++) {
		if (p && i<p->pipes.size())
			pipes[i]->setNext(p->pipes[i]);
		else
			pipes[i]->setNext(0);
	}
};

void AirBrake::setPrev(AirBrake* p)
{
	prev= p;
	for (int i=0; i<pipes.size(); i++) {
		if (p && i<p->pipes.size())
			pipes[i]->setPrev(p->pipes[i]);
		else
			pipes[i]->setPrev(0);
	}
};

void AirBrake::setNextOpen(bool open)
{
	nextOpen= open;
	for (int i=0; i<pipes.size(); i++)
		pipes[i]->setNextOpen(open);
};

void AirBrake::setPrevOpen(bool open)
{
	prevOpen= open;
	for (int i=0; i<pipes.size(); i++)
		pipes[i]->setPrevOpen(open);
};

void AirBrake::incRetainer()
{
	if (retainerControl < valve->retainerSettings.size()-1)
		retainerControl++;
}

void AirBrake::decRetainer()
{
	if (retainerControl > 0)
		retainerControl--;
}

void AirBrake::updateAirSpeeds(float dt)
{
	for (int i=0; i<pipes.size(); i++) {
		pipes[i]->updateAirSpeed(dt);
	}
}

void AirBrake::updatePressures(float dt)
{
	for (int i=0; i<pipes.size(); i++) {
		AirPipe* pipe= pipes[i];
		if (pipe->prev && pipe->prevOpen) {
			float flow= .5*(pipe->airFlow+pipe->prev->airFlow);
			pipe->addAir(flow);
		} else if (pipe->prevOpen) {
			pipe->addAir(pipe->airFlow);
		}
		if (pipe->next && pipe->nextOpen) {
			float flow= .5*(pipe->airFlow+pipe->next->airFlow);
			pipe->addAir(-flow);
		} else if (pipe->nextOpen) {
			pipe->addAir(-pipe->airFlow);
		}
	}
	valveState= valve->updateState(valveState,tanks);
	valve->updatePressures(valveState,dt,tanks,retainerControl);
}

EngAirBrake::EngAirBrake(float maxEqRes, string brakeValve) :
  AirBrake(brakeValve)
{
	autoControl= 0;
	indControl= 0;
	pumpOn= false;
	engCutOut= true;
	pumpOnThreshold= maxEqRes+20;
	pumpOffThreshold= maxEqRes+30;
	pumpChargeRate= 1;
	feedThreshold= maxEqRes;
	serviceOpeningArea= .00333*.00333*M_PI;
	chargeOpeningArea= .4*.00312*.00312*M_PI;
	mainRes= new AirTank(4*50000*pow(.0254,3));
	eqRes= new AirTank(145*pow(.0254,3)); // 812?
	mainRes->setPsig(pumpOffThreshold);
	eqRes->setPsig(0);
	mrIndex= -1;
	cpIndex= -1;
	if (valve) {
		mrIndex= valve->getTankIndex("MR");
		cpIndex= valve->getTankIndex("CP");
	}
}

void EngAirBrake::updatePressures(float dt)
{
	if (mrIndex>=0)
		tanks[mrIndex]->setPa(mainRes->getPa());
	AirBrake::updatePressures(dt);
	if (mrIndex>=0)
		mainRes->setPa(tanks[mrIndex]->getPa());
	float flow= 0;
	if (engCutOut)
		return;
	if (mainRes->getPsig() < pumpOnThreshold)
		pumpOn= true;
	else if (mainRes->getPsig() > pumpOffThreshold)
		pumpOn= false;
	if (pumpOn)
		mainRes->setPsig(mainRes->getPsig()+dt*pumpChargeRate);
	if (autoControl > 0)
		eqRes->setPsig(eqRes->getPsig()-6.5*dt);
	if (autoControl<0 && tanks[bpIndex]->getPsig()<feedThreshold) {
		float kg= mainRes->moveAir(tanks[bpIndex],chargeOpeningArea,dt);
		flow= kg/dt/tanks[bpIndex]->getDensity();
	}
	if (autoControl>=0 && tanks[bpIndex]->getPa()>eqRes->getPa()) {
		float kg= tanks[bpIndex]->vent(serviceOpeningArea,dt);
		flow= kg/dt/tanks[bpIndex]->getDensity();
	}
	if (autoControl < 0)
		eqRes->setPa(tanks[bpIndex]->getPa());
	airFlow= .98*airFlow + .02*flow;
	if (cpIndex>=0 && tanks[cpIndex]->getPsig()<feedThreshold)
		mainRes->moveAir(tanks[cpIndex],chargeOpeningArea,dt);
}

//	instant bail off, might be better to do this over time
void EngAirBrake::bailOff()
{
	setAuxResPressure(getPipePressure());
	setCylPressure(0);
}

AirBrake* AirBrake::create(bool engine, float maxEqRes, string brakeValve)
{
	if (engine) {
		return new EngAirBrake(maxEqRes,brakeValve);
	} else {
		return new AirBrake(brakeValve);
	}
}
