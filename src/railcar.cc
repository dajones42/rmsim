//	rail car data and movement code
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
#include "rmsim.h"

#include <osgText/Text>
#include <osg/Billboard>
#include <osg/LOD>
#if 1
#include <osgParticle/Particle>
#include <osgParticle/ParticleSystem>
#include <osgParticle/ParticleSystemUpdater>
#include <osgParticle/ModularEmitter>
#include <osgParticle/ModularProgram>
#include <osgParticle/RandomRateCounter>
#include <osgParticle/SectorPlacer>
#include <osgParticle/RadialShooter>
#include <osgParticle/AccelOperator>
#include <osgParticle/FluidFrictionOperator>
#endif
#include <osgSim/LightPointNode>

RailCarDefMap railCarDefMap;

RailCarDef* findRailCarDef(string name, bool random)
{
	RailCarDefMap::iterator i= railCarDefMap.find(name);
	if (i != railCarDefMap.end())
		return i->second;
	if (!random)
		return NULL;
	int n= (int)(railCarDefMap.size()*drand48());
//	fprintf(stderr,"find random car %d %d\n",n,(int)railCarDefMap.size());
	for (i=railCarDefMap.begin(); i!=railCarDefMap.end(); i++) {
		if (n-- <= 0)
			return i->second;
	}
//	fprintf(stderr,"no random car\n");
	return NULL;
}

RailCarDef::RailCarDef()
{
	axles= 4;
	mass0= 30;
	mass1= 30;
	mass2= 3;
	drag0a= 6.3743;
	drag0b= 128.998;
	drag1= .49358;
	drag2= .11979;
	drag2a= .57501;
	area= 9;
	width= 3;
	length= 12;
	offset= 0;
	maxBForce= 0;
	engine= NULL;
	maxSlack= .1;
	couplerGap= .02;
	brakeValve= "K";
};

void RailCarDef::copy(RailCarDef* other)
{
	axles= other->axles;
	mass0= other->mass0;
	mass1= other->mass1;
	mass2= other->mass2;
	drag0a= other->drag0a;
	drag0b= other->drag0b;
	drag1= other->drag1;
	drag2= other->drag2;
	drag2a= other->drag2a;
	area= other->area;
	width= other->width;
	length= other->length;
	offset= other->offset;
	maxBForce= other->maxBForce;
	maxSlack= other->maxSlack;
	couplerGap= other->couplerGap;
	engine= other->engine;
	for (int i=0; i<other->inside.size(); i++)
		inside.push_back(other->inside[i]);
	for (int i=0; i<other->parts.size(); i++)
		parts.push_back(other->parts[i]);
	soundFile= other->soundFile;
	brakeValve= other->brakeValve;
}

void RailCarDef::copyWheels(RailCarDef* other)
{
//	fprintf(stderr,"copywheels %p %p\n",this,other);
	for (int i=0; i<axles; i++) {
		int j= parts[i].parent;
//		fprintf(stderr," %d %p %f %f %f %f\n",
//		  i,parts[i].model,parts[i].zoffset,other->parts[i].zoffset,
//		  parts[i].xoffset,other->parts[i].xoffset);
		parts[i].model= other->parts[i].model;
		parts[i].zoffset= other->parts[i].zoffset;
		float dx= parts[j].xoffset-parts[i].xoffset;
		float odx= other->parts[j].xoffset-other->parts[i].xoffset;
//		fprintf(stderr,"   %f %f\n",dx,odx);
		parts[i].xoffset+= dx-odx;
//		fprintf(stderr," %d %p %f %f\n",
//		  i,parts[i].model,parts[i].zoffset,parts[i].xoffset);
	}
	for (int i=0; i<axles; i++) {
		int j= parts[i].parent;
//		fprintf(stderr," %d %p %f %f\n",
//		  j,parts[j].model,parts[j].zoffset,parts[j].xoffset);
		parts[j].model= other->parts[j].model;
		parts[j].zoffset= other->parts[j].zoffset;
//		parts[j].xoffset= other->parts[j].xoffset;
//		fprintf(stderr," %d %p %f %f\n",
//		  j,parts[j].model,parts[j].zoffset,parts[j].xoffset);
	}
}

//	creates an instance of a rail car
RailCarInst::RailCarInst(RailCarDef* def, osg::Group* group, float maxEqRes,
  std::string brakeValve)
{
	modelsSw= new osg::Switch;
#if 0
	osg::LOD* lod= new osg::LOD;
	lod->addChild(modelsSw,0,2000);
	group->addChild(lod);
#else
	group->addChild(modelsSw);
#endif
	modelsSw->setAllChildrenOn();
	this->def= def;
	engine= NULL;
	if (def->engine)
		engine= def->engine->copy();
	float maxr= 0;
	mainWheel= 0;
	for (int i=0; i<def->axles; i++) {
		float r= def->parts[i].zoffset;
		if (maxr < r) {
			maxr= r;
			mainWheel= i;
		}
	}
	animState= 0;
	SetCarVisitor visitor(this);
	for (int i=0; i<def->parts.size(); i++) {
		if (i < def->axles) {
			float r= def->parts[i].zoffset;
			if (r > .99*maxr)
				r= maxr;
			wheels.push_back(RailCarWheel(r));
		}
		models.push_back(NULL);
		linReg.push_back(new LinReg);
		if (def->parts[i].model == NULL)
			continue;
		osg::MatrixTransform* node= new osg::MatrixTransform;
		node->addChild((osg::Node*)def->parts[i].model->clone(
		  osg::CopyOp::DEEP_COPY_NODES));
		modelsSw->addChild(node);
		models[i]= node;
		node->accept(visitor);
	}
//	if (visitor.nRods > 0)
//		fprintf(stderr,"%d rods\n",visitor.nRods);
	setLoad(0);
	grade= 0;
	curvature= 0;
	speed= 0;
	force= 0;
	slack= 0;
	maxSlack= def->maxSlack;
	couplerState[0]= couplerState[1]= 0;
	next= NULL;
	prev= NULL;
	rev= 0;
	handBControl= 0;
	airBrake= AirBrake::create(def->engine,maxEqRes,
	  def->brakeValve!=""?def->brakeValve:brakeValve);
	distance= 0;
	waybill= NULL;
	addSmoke();
}

RailCarInst::~RailCarInst()
{
	if (waybill)
		delete waybill;
	//remove model transforms from graph
	osg::Group* p= modelsSw->getParent(0);
	if (p != NULL)
		p->removeChild(modelsSw);
}

//	sets a rail cars location by setting the location of each wheelset
void RailCarInst::setLocation(float offset, Track::Location* location)
{
	float s= rev ? -1 : 1;
	offset+= -s*def->offset;
	int n= 0;
	for (int i=0; i<wheels.size(); i++) {
		Track::Location loc= *location;
		loc.move(offset+s*def->parts[i].xoffset,0,0);
		wheels[n].location= loc;
		n++;
	}
}

//	linear regression used to convert wheelset locations up to higher parts
void RailCarInst::LinReg::init()
{
	sw= 0;
	so= 0;
	soo= 0;
	sx= 0;
	sxo= 0;
	sy= 0;
	syo= 0;
	sz= 0;
	szo= 0;
	up[0]= up[1]= up[2]= 0;
}

void RailCarInst::LinReg::sum(double w, double o, double x, double y, double z,
  osg::Vec3f& u)
{
	sw+= w;
	so+= w*o;
	soo+= w*o*o;
	sx+= w*x;
	sxo+= w*x*o;
	sy+= w*y;
	syo+= w*y*o;
	sz+= w*z;
	szo+= w*z*o;
	up[0]+= w*u[0];
	up[1]+= w*u[1];
	up[2]+= w*u[2];
}

void RailCarInst::LinReg::calc()
{
	double d= sw*soo - so*so;
	if (d > 1e-20) {
		ax= (soo*sx-so*sxo)/d;
		ay= (soo*sy-so*syo)/d;
		az= (soo*sz-so*szo)/d;
		bx= (sw*sxo-so*sx)/d;
		by= (sw*syo-so*sy)/d;
		bz= (sw*szo-so*sz)/d;
	} else {
		ax= sx/sw;
		ay= sy/sw;
		az= sz/sw;
		bx= 0;
		by= 0;
		bz= 0;
	}
	up[0]/= sw;
	up[1]/= sw;
	up[2]/= sw;
}

//	moves a rail car the specified distance
//	also adjusts all the parts to follow the wheelset locations
void RailCarInst::move(float distance)
{
	this->distance+= distance;
	if (next != NULL)
		slack+= distance;
	if (prev != NULL)
		prev->slack-= distance;
	grade= 0;
	curvature= 0;
	int n= wheels.size();
	for (int i=n; i<def->parts.size(); i++)
		linReg[i]->init();
	for (int i=0; i<n; i++) {
		wheels[i].move(distance,rev);
		grade+= wheels[i].location.grade();
		curvature+= wheels[i].location.curvature();
		WLocation wloc;
		wheels[i].location.getWLocation(&wloc);
		LinReg* lr= linReg[i];
		RailCarPart& part= def->parts[i];
		lr->ax= wloc.coord[0];
		lr->ay= wloc.coord[1];
		lr->az= wloc.coord[2];
		lr->up= wloc.up;
		if (part.parent >= 0)
			linReg[part.parent]->sum(1,part.xoffset,
			  wloc.coord[0],wloc.coord[1],wloc.coord[2],wloc.up);
	}
	grade/= n;
	curvature/= n;
	int m= 0;
	for (int i=n; i<def->parts.size(); i++) {
		LinReg* lr= linReg[i];
		if (lr->sw < 1.5) {
			m++;
			continue;
		}
		lr->calc();
		RailCarPart& part= def->parts[i];
		if (part.parent >= 0)
			linReg[part.parent]->sum(1,part.xoffset,
			  lr->ax+part.xoffset*lr->bx,
			  lr->ay+part.xoffset*lr->by,
			  lr->az+part.xoffset*lr->bz,lr->up);
	}
	if (m > 0) {
		for (int i=def->parts.size()-1; i>=n; i--) {
			LinReg* lr= linReg[i];
			if (lr->sw > 1.5)
				continue;
			RailCarPart& part= def->parts[i];
			if (part.parent < 0)
				continue;
			LinReg* plr= linReg[part.parent];
			lr->sum(1,part.xoffset,
			  plr->ax+part.xoffset*plr->bx,
			  plr->ay+part.xoffset*plr->by,
			  plr->az+part.xoffset*plr->bz,plr->up);
			lr->calc();
		}
	}
	for (int i=0; i<def->parts.size(); i++) {
		LinReg* lr= linReg[i];
		RailCarPart& part= def->parts[i];
		if (i < n) {
			lr->bx= linReg[part.parent]->bx;
			lr->by= linReg[part.parent]->by;
			lr->bz= linReg[part.parent]->bz;
		}
		if (models[i] == NULL)
			continue;
		osg::Vec3f fwd= osg::Vec3f(lr->bx,lr->by,lr->bz);
//		osg::Vec3f side= osg::Vec3f(-lr->by,lr->bx,0);
		osg::Vec3f side= lr->up^fwd;
//		if (n>4) {
//		fprintf(stderr,"%d\n",i);
//		fprintf(stderr,"lrup %f %f %f\n",lr->up[0],lr->up[1],lr->up[2]);
//		fprintf(stderr,"fwd %f %f %f\n",fwd[0],fwd[1],fwd[2]);
//		fprintf(stderr,"oldside %f %f %f\n",-lr->by,lr->bx,0);
//		fprintf(stderr,"side %f %f %f\n",side[0],side[1],side[2]);
//		}
		fwd.normalize();
		side.normalize();
		osg::Vec3f up= fwd^side;
//		fprintf(stderr,"up %f %f %f\n",up[0],up[1],up[2]);
		float xo= part.xoffset;
		float zo= part.zoffset;
		if (i < n) {
			RailCarWheel& w= wheels[i];
			models[i]->setMatrix(osg::Matrixd(
			  w.cs,0,w.sn,0,
			  0,1,0,0,
			  -w.sn,0,w.cs,0,
			  0,0,0,1)*osg::Matrixd(
			  fwd[0],fwd[1],fwd[2],0,
			  side[0],side[1],side[2],0,
			  up[0],up[1],up[2],0,
			  lr->ax,
			  lr->ay,
			  lr->az+zo*up[0]+zo*up[2],1));
		} else {
			models[i]->setMatrix(osg::Matrixd(
			  fwd[0],fwd[1],fwd[2],0,
			  side[0],side[1],side[2],0,
			  up[0],up[1],up[2],0,
			  lr->ax+xo*lr->bx,//+xo*lr->bz,
			  lr->ay+xo*lr->by,
			  lr->az+zo*up[0]+zo*up[2]+xo*lr->bz,1));
		}
	}
	if (engine) {
		for (vector<RailCarSmoke>::iterator i=def->smoke.begin();
		  i!=def->smoke.end(); ++i) {
			if (i->shooter && i->counter) {
				float speed= i->minSpeed +
				  (i->maxSpeed-i->minSpeed)*
				  engine->getSmokeSpeed();
				float rate= i->minRate +
				  (i->maxRate-i->minRate)*
				  engine->getSmokeRate();
				i->shooter->setInitialSpeedRange(speed,speed+1);
				i->counter->setRateRange(rate,rate+10);
			}
		}
	}
}

float cosTable[]={
	1., .923880, .707107, .382683, 0, -.382683, -.707107, -.923880,
	-1., -.923880, -.707107, -.382683, 0, .382683, .707107, .923880, 1
};
float sinTable[]={
	0, .382683, .707107, .923880, 1., .923880, .707107, .382683,
	0, -.382683, -.707107, -.923880, -1., -.923880, -.707107, -.382683, 0
};

RailCarWheel::RailCarWheel(float radius)
{
	cs= 1;
	sn= 0;
	state= 0;
	mult= 1./(2.*3.14159*radius);
}

//	moves a rail cars wheel and adjusts its angle
void RailCarWheel::move(float distance, int rev)
{
	location.move(distance,0,0);
	if (rev)
		state+= mult*distance;
	else
		state-= mult*distance;
	if (state >= 1) {
		int i= (int)state;
		state-= i;
	}
	if (state < 0) {
		int i= (int)(-state);
		state+= i+1;
	}
	float s= 16*state;
	int i= (int) s;
	s-= i;
	cs= cosTable[i] + s*(cosTable[i+1]-cosTable[i]);
	sn= sinTable[i] + s*(sinTable[i+1]-sinTable[i]);
	float x= .5 + .5*(cs*cs+sn*sn);
	cs/= x;
	sn/= x;
}

#if 0
float RailCarInst::calcBrakes(float dt, int n)
{
	if (linePres < auxResPres-1)
		tripleValveState= TVS_APPLY;
	else if (linePres > auxResPres+1)
		tripleValveState= TVS_RELEASE;
	else if (tripleValveState==TVS_APPLY && linePres>=auxResPres)
		tripleValveState= TVS_LAP;
	else if (tripleValveState==TVS_RELEASE && linePres<=auxResPres)
		tripleValveState= TVS_LAP;
//	fprintf(stderr," %d %f %f %f\n",tripleValveState,
//	  linePres,auxResPres,cylPres);
	if (tripleValveState == TVS_APPLY) {
		float dp= .015*dt*(auxResPres-cylPres);
		if (dp > auxResPres-linePres) {
			dp= auxResPres-linePres;
			tripleValveState= TVS_LAP;
		}
		if (3.5*dp > auxResPres-cylPres)
			dp= (auxResPres-cylPres)/3.5;
		auxResPres-= dp;
		cylPres+= 2.5*dp;
	}
	if (tripleValveState == TVS_RELEASE) {
		RetainerData& rd= def->retainerData[retainerControl];
		if (cylPres > rd.pressure) {
			cylPres-= dt*rd.rate*cylPres;
			if (cylPres < rd.pressure)
				cylPres= rd.pressure;
		}
		float dp= dt*.015*(linePres-auxResPres);
		if (linePres-auxResPres < .1) {
			dp= linePres-auxResPres;
			tripleValveState= TVS_LAP;
		}
		auxResPres+= dp;
		return dp*2440;
	}
	return 0;
}
#endif

//	calculates total force acting on car except for couple forces
void RailCarInst::calcForce(float tControl, float dControl, float engBMult,
  float dt)
{
	float bMult= 0;
	if (airBrake != NULL)
		bMult= airBrake->getForceMult();
	if (bMult < handBControl)
		bMult= handBControl;
	float s= speed>0 ? speed : -speed;
	float drag2= ((speed>0 && prev==NULL) || (speed<0 && next==NULL)) ?
	  def->drag2a : def->drag2;
	drag= bMult*def->maxBForce + drag0 + s*(drag1 + def->area*drag2*s);
	if (s < 7.5)
		drag+= bMult*def->maxBForce*.2*(7.5-s);
	force= 0;
	if (engine) {
		force= engine->getForce(tControl,dControl,this,dt);
		if (engBMult > bMult)
			drag+= (engBMult-bMult)*def->maxBForce;
	}
	force+= 9.8*mass*grade;
	drag+= .0004*9.8*mass*curvature;
//	fprintf(stderr,"%f %f %f\n",force,drag,force-drag);
	if (speed > 0)
		force-= drag;
	else if (speed < 0)
		force+= drag;
	else if (force > drag)
		force-= drag;
	else if (force < -drag)
		force+= drag;
	else
		force= 0;
}

//	adjusts the mass to set the fraction of load to f
void RailCarInst::setLoad(float f)
{
	mass= f*def->mass1 + (1-f)*def->mass0;
	massInv= 1/(mass+def->mass2);
	drag0= .001*mass*def->drag0a + def->axles*def->drag0b;
	drag1= .001*mass*def->drag1;
//	fprintf(stderr,"%s %.2f %.0f %.0f %.0f %f %f\n",def->name.c_str(),
//	  f,def->mass0,def->mass1,mass,drag0,drag1);
}

//	adds a waybill to the car
//	These are like the f7 car numbers in MSTS except that they show
//	the car's destination
void RailCarInst::addWaybill(string& dest, float r, float g, float b, int p)
{
	if (waybill)
		return;
	waybill= new Waybill;
	waybill->destination= dest;
	waybill->color= osg::Vec3f(r,g,b);
	waybill->label= new osg::Switch();
	waybill->priority= p;
	osg::Billboard* bb= new osg::Billboard();
	bb->setMode(osg::Billboard::POINT_ROT_EYE);
	bb->setNormal(osg::Vec3f(0,0,1));
	osgText::Text* text= new osgText::Text;
	bb->addDrawable(text,osg::Vec3f(0,0,5));
	text->setFont("fonts/arial.ttf");
	text->setCharacterSize(25);
	text->setPosition(osg::Vec3(0,0,0));
	text->setColor(osg::Vec4(r,g,b,1));
	text->setText(dest);
	text->setAlignment(osgText::Text::CENTER_BOTTOM);
	text->setCharacterSizeMode(osgText::Text::SCREEN_COORDS);
	waybill->label->addChild(bb);
	waybill->label->setAllChildrenOff();
	models[def->parts.size()-1]->addChild(waybill->label);
}

#if 1
void RailCarInst::addSmoke()
{
	for (vector<RailCarSmoke>::iterator i=def->smoke.begin();
	  i!=def->smoke.end(); ++i) {
		fprintf(stderr,"addSmoke %f %f %f\n",
		  i->position[0],i->position[1],i->position[2]);
		osgParticle::Particle pt;
		pt.setShape(osgParticle::Particle::QUAD);
		pt.setLifeTime(3);
		pt.setSizeRange(osgParticle::rangef(i->size,2*i->size));
		pt.setAlphaRange(osgParticle::rangef(.8,0));
		pt.setColorRange(osgParticle::rangev4(
		  osg::Vec4(.5,.5,.5,1),osg::Vec4(.4,.4,.4,.2)));
		pt.setRadius(.05);
		pt.setMass(.05);
		osgParticle::ParticleSystem* ps=
		  new osgParticle::ParticleSystem;
//		ps->setDefaultAttributes("",false,false);
		ps->setDefaultAttributes("Images/smoke.rgb",false,false);
		ps->setDefaultParticleTemplate(pt);
		osgParticle::ModularEmitter* emitter=
		  new osgParticle::ModularEmitter;
		emitter->setParticleSystem(ps);
		osgParticle::RandomRateCounter* counter=
		  new osgParticle::RandomRateCounter;
		counter->setRateRange(20,30);
		emitter->setCounter(counter);
		i->counter= counter;
//		osgParticle::SectorPlacer* placer=
//		  new osgParticle::SectorPlacer;
//		placer->setCenter(i->position[0],i->position[1],i->position[2]);
//		placer->setRadiusRange(.5,1);
//		placer->setPhiRange(0,2*osg::PI);
//		emitter->setPlacer(placer);
		osgParticle::RadialShooter* shooter=
		  new osgParticle::RadialShooter;
		shooter->setInitialSpeedRange(1,2);
		shooter->setThetaRange(-osg::PI/8,osg::PI/8);
		shooter->setPhiRange(0,2*osg::PI);
		emitter->setShooter(shooter);
		i->shooter= shooter;
#if 0
		osgParticle::ModularProgram* program=
		 new osgParticle::ModularProgram;
		program->setParticleSystem(ps);
		osgParticle::AccelOperator* op1=
		 new osgParticle::AccelOperator;
		op1->setToGravity();
		program->addOperator(op1);
		osgParticle::FluidFrictionOperator* op2=
		 new osgParticle::FluidFrictionOperator;
		op2->setFluidToAir();
		program->addOperator(op2);
		models[models.size()-1]->addChild(program);
#endif
		osg::MatrixTransform* mt= new osg::MatrixTransform;
		mt->setMatrix(osg::Matrix::translate(
		  i->position[0],i->position[1],i->position[2]));
		mt->addChild(emitter);
		models[models.size()-1]->addChild(mt);
		osg::Geode* g= new osg::Geode;
		g->addDrawable(ps);
//		models[models.size()-1]->addChild(g);
		modelsSw->addChild(g);
		osgParticle::ParticleSystemUpdater* updater=
		 new osgParticle::ParticleSystemUpdater;
		updater->addParticleSystem(ps);
		models[models.size()-1]->addChild(updater);
	}
}
#endif

void HeadLightVisitor::apply(osg::Node& node)
{
	osgSim::LightPointNode* lpn=
	  dynamic_cast<osgSim::LightPointNode*> (&node);
	if (lpn != NULL) {
		osgSim::LightPointNode::LightPointList& lplist=
		  lpn->getLightPointList();
		list<HeadLight>::iterator j= model->headlights.begin();
		for (osgSim::LightPointNode::LightPointList::iterator
		  i=lplist.begin(); i!=lplist.end(); ++i) {
			if (j->unit == unit) {
				i->_on= on;
				if (rev)
					i->_position= osg::Vec3d(
					  i->_position[0],
					  i->_position[1],
					  -i->_position[2]);
			}
			j++;
			if (j == model->headlights.end())
				break;
		}
	}
	traverse(node);
}

void RailCarInst::setHeadLight(int unit, bool rev, bool on)
{
	if (def->headlights.size() > 0) {
		HeadLightVisitor visitor(def,unit,rev,on);
		models[models.size()-1]->accept(visitor);
	}
}
