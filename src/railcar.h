//	classes for rail car parameters and state
//
//Copyright 2009 Doug Jones
#ifndef RAILCAR_H
#define RAILCAR_H

#include "airbrake.h"
#include "locoeng.h"

#include <osgParticle/RadialShooter>
#include <osgParticle/RandomRateCounter>
#include <osg/Image>

struct HeadLight {
	float x;
	float y;
	float z;
	float radius;
	int unit;
	unsigned int color;
	HeadLight(float x, float y, float z, float r, int u, unsigned int c) {
		this->x= x;
		this->y= y;
		this->z= z;
		radius= r;
		unit= u;
		color= c;
	};
};

struct RailCarPart {
	float xoffset;		// distance forward of center
	float zoffset;		// distance above rail head
	osg::Node* model;
	int parent;
	RailCarPart(int p, float xo, float zo) {
		xoffset= xo;
		zoffset= zo;
		model= NULL;
		parent= p;
	};
};

struct RailCarSmoke {
	osg::Vec3f position;
	osg::Vec3f normal;
	float size;
	float minRate;
	float maxRate;
	float minSpeed;
	float maxSpeed;
	osgParticle::RadialShooter* shooter;
	osgParticle::RandomRateCounter* counter;
	RailCarSmoke() {
		shooter= NULL;
		counter= NULL;
	};
};

struct RailCarInside {
	osg::Vec3f position;
	float angle;
	float vAngle;
	osg::Image* image;
	RailCarInside(float px, float py, float pz, float a, float va,
	  osg::Image* img) {
		position= osg::Vec3f(px,py,pz);
		angle= a;
		vAngle= va;
		image= img;
		if (image)
			image->ref();
	};
};

struct RailCarDef {
	string name;
	string soundFile;
	string brakeValve;
	int axles;
	float mass0;		// empty static mass
	float mass1;		// loaded static mass
	float mass2;		// rotating mass equivalent
	float drag0a;
	float drag0b;
	float drag1;
	float drag2;
	float drag2a;
	float area;
	float width;		// bounding box width
	float length;
	float offset;		// offset of model center from car center
	float maxSlack;
	float couplerGap;
	vector<RailCarPart> parts;
	vector<RailCarSmoke> smoke;
	float maxBForce;
	float soundGain;
	LocoEngine* engine;
	vector<RailCarInside> inside;
	std::list<HeadLight> headlights;
	RailCarDef();
	void copy(RailCarDef* other);
	void copyWheels(RailCarDef* other);
};

struct Waybill {
	std::string destination;
	osg::Vec3f color;
	osg::Switch* label;
	int priority;
	Waybill() { label= NULL; priority= 100; };
};

struct RailCarWheel {
	Track::Location location;
	float cs;
	float sn;
	float state;
	float mult;
	RailCarWheel(float radius);
	void move(float distance, int rev);
};

struct RailCarInst {
	struct LinReg {
		double sw;
		double so;
		double soo;
		double sx;
		double sy;
		double sz;
		double sxo;
		double syo;
		double szo;
		double ax;
		double ay;
		double az;
		double bx;
		double by;
		double bz;
		osg::Vec3f up;
		void init();
		void calc();
		void sum(double w, double o, double x, double y, double z,
		  osg::Vec3f& u);
	};
	RailCarDef* def;
	LocoEngine* engine;
	vector<RailCarWheel> wheels;
	osg::Switch* modelsSw;
	vector<osg::MatrixTransform*> models;
	vector<LinReg*> linReg;
	int mainWheel;
	float mass;
	float massInv;
	float drag0;
	float drag1;
	float grade;
	float curvature;
	float speed;
	float force;
	float drag;
	float slack;
	float maxSlack;
	float couplerState[2];
	float cA;
	float cB;
	float cC;
	float cR;
	float cU;
	float cG;
	int rev;
	float distance;
	AirBrake* airBrake;
	float handBControl;
	RailCarInst* next;
	RailCarInst* prev;
	Waybill* waybill;
	int animState;
	RailCarInst(RailCarDef* def, osg::Group* group, float maxEqRes,
	  std::string brakeValve="");
	~RailCarInst();
	void move(float distance);
	void setLocation(float offset, Track::Location* loc);
	void calcForce(float tControl, float dControl, float engBMult,
	  float dt);
	void setLoad(float f);
	float calcBrakes(float dt, int n);
	void setModelsOn() { modelsSw->setAllChildrenOn(); };
	void setModelsOff() { modelsSw->setAllChildrenOff(); };
	void decHandBrakes() {
		handBControl-= .25;
		if (handBControl < 0)
			handBControl= 0;
	}
	void incHandBrakes() {
		handBControl+= .25;
		if (handBControl > 1)
			handBControl= 1;
	}
	void addWaybill(std::string& dest, float r, float g, float b, int p);
	float getMainWheelState() {
		return wheels[mainWheel].state;
	};
	float getMainWheelRadius() {
		return def->parts[mainWheel].zoffset;
	};
	float getCouplerState(bool front) {
		if (rev)
			front= !front;
		return couplerState[front?0:1];
	};
	void addSmoke();
	int getAnimState(int index) {
		if (index<0)
			return 1;
		return (animState&(1<<index))!=0;
	};
	void setHeadLight(int unit, bool rev, bool on);
};

struct HeadLightVisitor : public osg::NodeVisitor {
	RailCarDef* model;
	int unit;
	bool rev;
	bool on;
	HeadLightVisitor(RailCarDef* m, int u, bool r, bool o) :
	  osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
		model= m;
		unit= u;
		rev= r;
		on= o;
	};
	virtual void apply(osg::Node& node);
};

typedef map<string,RailCarDef*> RailCarDefMap;
extern RailCarDefMap railCarDefMap;
extern RailCarDef* findRailCarDef(string name, bool random);

#endif
