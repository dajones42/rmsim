//	Classes for modeling air brakes
//
//Copyright 2009 Doug Jones
#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <string>
#include <vector>
#include <map>

#include "airtank.h"
#include "brakevalve.h"

class AirBrake;

//	air brake state information
class AirBrake {
 protected:
	AirBrake* next;
	AirBrake* prev;
	bool nextOpen;
	bool prevOpen;
	bool cutOut;
	int retainerControl;
	std::vector<AirPipe*> pipes;
	std::vector<AirTank*> tanks;
	BrakeValve* valve;
	int valveState;
	int bpIndex;
	int arIndex;
	int bcIndex;
 public:
	AirBrake(std::string brakeValve);
	void setNext(AirBrake* p);
	void setPrev(AirBrake* p);
	AirBrake* getNext() { return next; };
	AirBrake* getPrev() { return prev; };
	void setNextOpen(bool v);
	void setPrevOpen(bool v);
	bool getNextOpen() { return nextOpen; };
	bool getPrevOpen() { return prevOpen; };
	void setCutOut(bool v) { cutOut= v; };
	bool getCutOut() { return cutOut; };
	void setRetainer(int v) {
		if (v>=0 && v<valve->retainerSettings.size())
			retainerControl= v;
	};
	int getRetainer() { return retainerControl; };
	void incRetainer();
	void decRetainer();
	std::string getRetainerName() {
		return valve->retainerSettings.size() > 0 ?
		  valve->retainerSettings[retainerControl]->name : "";
	};
	float getPipePressure() { return tanks[bpIndex]->getPsig(); };
	float getPressure(std::string name) {
		int i= valve->getTankIndex(name);
		return i<0 ? 0 : tanks[i]->getPsig();
	};
	float setPressure(std::string name, float p) {
		int i= valve->getTankIndex(name);
		if (i >= 0)
			tanks[i]->setPsig(p);
	}
	float setPipePressure(float p) {
		setPressure("BP",p);
		setPressure("ER",p);
		setPressure("QAC",p);
	};
	float getAuxResPressure() { return tanks[arIndex]->getPsig(); };
	void setAuxResPressure(float p) {
		setPressure("AR",p);
	};
	void setCylPressure(float p) {
		setPressure("BC",p);
		setPressure("AC",p);
	};
	int getTripleValveState() { return valveState&0xf; };
	virtual float getCylPressure() { return tanks[bcIndex]->getPsig(); };
	virtual float getForceMult() {
		return tanks[bcIndex]->getPsig()>5 ?
		  (tanks[bcIndex]->getPsig()-5)/45 : 0;
	};
	void setVolume(std::string name, float v) {
		int i= valve->getTankIndex(name);
		if (i >= 0)
			tanks[i]->volume= v;
	}
//	virtual void calcDeltas(float dt);
//	virtual void applyDeltas();
	virtual void updateAirSpeeds(float dt);
	virtual void updatePressures(float dt);
	static AirBrake* create(bool engine, float maxEqRes,
	  std::string brakeValve);
};

//	locomotive air brake state information
class EngAirBrake : public AirBrake {
 protected:
	float autoControl;
	float indControl;
	bool pumpOn;
	bool engCutOut;
	AirTank* mainRes;
	AirTank* eqRes;
	float airFlow;
	float pumpOnThreshold;	// units psi
	float pumpOffThreshold;	// units psi
	float pumpChargeRate;	// units psi/s
	float feedThreshold;	// units psi
	float serviceOpeningArea;
	float chargeOpeningArea;
	int mrIndex;
	int cpIndex;
 public:
	void setAutoControl(float v) { autoControl= v; }
	float getAutoControl() { return autoControl; }
	void setIndControl(float v) { indControl= v; }
	float getIndControl() { return indControl; }
	void setMainResPressure(float p) { mainRes->setPsig(p); }
	float getMainResPressure() { return mainRes->getPsig(); }
	void setEqResPressure(float p) { eqRes->setPsig(p); }
	float getEqResPressure() { return eqRes->getPsig(); }
	float getMaxEqResPressure() { return feedThreshold; }
	void setEngCutOut(bool v) { engCutOut= v; };
	bool getEngCutOut() { return engCutOut; };
	float getAirFlow() { return airFlow; }
	float getAirFlowCFM() {
		return airFlow*pow(3.281,3)*60*tanks[0]->getPa()/
		  AirTank::STDATM;
	}
	bool getPumpOn() { return pumpOn; }
	void bailOff();
	EngAirBrake(float maxEqRes, std::string brakeValve);
//	virtual void calcDeltas(float dt);
//	virtual void applyDeltas();
	virtual void updatePressures(float dt);
};

#endif
