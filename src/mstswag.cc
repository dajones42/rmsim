//	reads selected info from an MSTS wag file
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
#include "mstswag.h"
#include "mstsfile.h"

//	gets a float value from a named field
//	ignores any unit suffix at the moment
float getFloat(MSTSFileNode* node, const char* name, int child, float dflt)
{
	MSTSFileNode* n= node->children->find(name);
	if (n == NULL) {
		fprintf(stderr,"cannot find %s\n",name);
		return dflt;
	}
	n= n->getChild(child);
	if (n==NULL || n->value==NULL)
		return dflt;
	return atof(n->value->c_str());
}

//	gets a int value from a named field
//	ignores any unit suffix at the moment
int getInt(MSTSFileNode* node, const char* name, int child, float dflt)
{
	MSTSFileNode* n= node->children->find(name);
	if (n == NULL) {
		//fprintf(stderr,"cannot find %s\n",name);
		return dflt;
	}
	n= n->getChild(child);
	if (n==NULL || n->value==NULL)
		return dflt;
	return atoi(n->value->c_str());
}

//	gets a hex value from a named field
//	ignores any unit suffix at the moment
unsigned int getHex(MSTSFileNode* node, const char* name, int child, float dflt)
{
	MSTSFileNode* n= node->children->find(name);
	if (n == NULL) {
		//fprintf(stderr,"cannot find %s\n",name);
		return dflt;
	}
	n= n->getChild(child);
	if (n==NULL || n->value==NULL)
		return dflt;
	return (unsigned int)strtoll(n->value->c_str(),NULL,16);
}

RailCarDef* readMSTSWag(const char* dir, const char* file, bool saveNames)
{
	string path= string(dir)+"/"+file;
	typedef map<string,RailCarDef*> WagMap;
	static WagMap wagMap;
	WagMap::iterator i= wagMap.find(path);
	if (i != wagMap.end())
		return i->second;
	MSTSFile wagFile;
	try {
		wagFile.readFile(path.c_str());
	} catch (const char* msg) {
		fprintf(stderr,"cannot read %s\n",path.c_str());
		return NULL;
	} catch (const std::exception& error) {
		fprintf(stderr,"cannot read %s\n",path.c_str());
		return NULL;
	}
	MSTSFileNode* wagon= wagFile.find("Wagon");
	if (wagon == NULL)
		return NULL;
	MSTSFileNode* wshape= wagon->children->find("WagonShape");
	if (wshape==NULL || wshape->getChild(0)==NULL ||
	  wshape->getChild(0)->value==NULL)
		return NULL;
	RailCarDef* def= new RailCarDef;
	wagMap[path]= def;
	def->name= file;
	MSTSFileNode* lights= wagon->children->find("Lights");
	for (int i=0; lights!=NULL && lights->getChild(i)!=NULL; i++) {
		MSTSFileNode* light= lights->getChild(i);
		MSTSFileNode* type=
		  light->children->find("Type");
		if (type==NULL || type->getChild(0)==NULL ||
		  type->getChild(0)->value==NULL ||
		  *(type->getChild(0)->value)!="0")
			continue;
		MSTSFileNode* conditions= light->children->find("Conditions");
		if (conditions == NULL)
			continue;
		int headlight= getInt(conditions,"Headlight",0,0);
		int unit= getInt(conditions,"Unit",0,0);
		if (headlight < 3)
			continue;
		MSTSFileNode* states= light->children->find("States");
		if (states == NULL)
			continue;
		MSTSFileNode* state= states->children->find("State");
		if (state == NULL)
			continue;
		def->headlights.push_back(HeadLight(
		  getFloat(state,"Position",0,0),
		  getFloat(state,"Position",1,0),
		  getFloat(state,"Position",2,0),
		  getFloat(state,"Radius",0,0),unit,
		  getHex(state,"LightColour",0,0)));
	}
	path= string(dir)+"/"+wshape->getChild(0)->value->c_str();
	MSTSShape shape;
	shape.readFile(path.c_str());
	shape.createRailCar(def,saveNames);
	def->mass0= def->mass1= 1e3*getFloat(wagon,"Mass",0,20);
	def->length= getFloat(wagon,"Size",2,10);
	MSTSFileNode* coupling= wagon->children->find("Coupling");
	if (coupling != NULL) {
		MSTSFileNode* spring= coupling->children->find("Spring");
		if (spring != NULL)
			def->length+= .01*getFloat(spring,"r0",0,0);
	}
	def->maxBForce= getFloat(wagon,"MaxBrakeForce",0,16);
	if (def->maxBForce < 1000)
		def->maxBForce*= 1000;
	if (def->maxBForce < .7*def->mass1)
		def->maxBForce= .7*def->mass1;
	if (def->maxBForce > 2*def->mass0)
		def->maxBForce= 2*def->mass0;
//	fprintf(stderr,"maxbforce %s %f %f %f\n",file,
//	  def->mass0,def->length,def->maxBForce);
	MSTSFileNode* fanim= wagon->children->find("FreightAnim");
	if (fanim!=NULL && fanim->getChild(0)!=NULL &&
	  fanim->getChild(0)->value!=NULL) {
		path= string(dir)+"/"+fanim->getChild(0)->value->c_str();
		try {
		MSTSShape fashape;
		fashape.readFile(path.c_str());
		fashape.fixTop();
		osg::MatrixTransform* mt= (osg::MatrixTransform*)
		  def->parts[def->parts.size()-1].model;
		mt->addChild(fashape.createModel(0,10,saveNames));
		} catch (const char* msg) {
		}
	}
	MSTSFileNode* sound= wagon->children->find("Sound");
	MSTSFileNode* inside= wagon->children->find("Inside");
	if (inside != NULL) {
		MSTSFileNode* pos=
		  inside->children->find("PassengerCabinHeadPos");
		if (pos != NULL)
			def->inside.push_back(RailCarInside(
			  atof(pos->getChild(2)->value->c_str()),
			  atof(pos->getChild(0)->value->c_str()),
			  atof(pos->getChild(1)->value->c_str()),0,0,NULL));
//		fprintf(stderr,"inside %f %f %f\n",def->insideCoord[0],
//		 def->insideCoord[1],def->insideCoord[2]);
	}
	float c1= getFloat(wagon,"Friction",0,0);
	float e1= getFloat(wagon,"Friction",1,0);
	float v2= getFloat(wagon,"Friction",2,0);
	float c2= getFloat(wagon,"Friction",3,0);
	float e2= getFloat(wagon,"Friction",4,0);
//	fprintf(stderr,"friction %s %f %f %f %f %f\n",file,c1,e1,v2,c2,e2);
	if (v2 > 0) {
		// least squares fit to get davis equation values from friction
		// only used if friction looks like its from fcalc
		v2*= .44704;
		double sy= 0;
		double syx= 0;
		double syx2= 0;
		double sx= 0;
		double sx2= 0;
		double sx3= 0;
		double sx4= 0;
		int n= 0;
		if (e1 == 0) {
			sy+= c1;
			n++;
		}
		float y0= c1*pow(v2,e1) + c2*v2;
		for (float x=v2; x<80*.44704; x+=1) {
			float y= y0 + c2*pow(x,e2);
			n++;
			sy+= y;
			syx+= y*x;
			syx2+= y*x*x;
			sx+= x;
			sx2+= x*x;
			sx3+= x*x*x;
			sx4+= x*x*x*x;
		}
//		fprintf(stderr," %d %f %f %f %f %f %f %f\n",
//		  n,sy,syx,syx2,sx,sx2,sx3,sx4);
		double s1= syx - sy*sx/n;
		double s2= sx*sx2/n - sx3;
		double s3= sx2 - sx*sx/n;
		double s4= syx2 - sy*sx2/n;
		double s5= sx2*sx2/n - sx4;
		double s6= sx3 - sx*sx2/n;
		float c= (s1*s6-s3*s4)/(s3*s5-s2*s6);
		float b= (s1+c*s2)/s3;
		float a= (sy-b*sx-c*sx2)/n;
//		fprintf(stderr," %f %f %f\n",a,b,c);
		for (float x=v2; x<80*.44704; x+=4) {
			float y= y0 + c2*pow(x,e2);
			float z= a + b*x + c*x*x;
//			fprintf(stderr," %f %f %f %f\n",x,y,z,y-z);
		}
		def->drag0a= a/(.001*def->mass0);
		def->drag0b= 0;
		def->drag1= b/(.001*def->mass0);
		def->drag2= def->drag2a= c/def->area;
//		fprintf(stderr," %f %f %f\n",def->drag0a,def->drag1,def->drag2);
		for (int j=0; j<2; j++) {
		float x1= 20*.44704;
		float x2= 40*.44704;
		float x3= 60*.44704;
		if (j == 1) {
			if (e1 == 0)
				x2= 40*.44704;
			else
				x2= 50*.44704;
			x3= 80*.44704;
		}
		float y1= y0 + c2*pow(x1,e2);
		float y2= y0 + c2*pow(x2,e2);
		float y3= y0 + c2*pow(x3,e2);
		if (e1 == 0) {
			x1= 0;
			y1= c1;
		}
		float x1sq= x1*x1;
		float x2sq= x2*x2;
		float x3sq= x3*x3;
		float x21= x2-x1;
		float x31= x3-x1;
		c= (x21*(y3-y1)-x31*(y2-y1))/(x21*(x3sq-x1sq)-x31*(x2sq-x1sq));
		b= (y2-y1-c*(x2sq-x1sq))/x21;
		a= y1-b*x1-c*x1sq;
//		fprintf(stderr," %f %f %f\n",a,b,c);
		}
		if (e1 <= 0) {
			float x1= v2;
			float x2= 80*.44704;
			float n= x2-x1;
			double x1p= x1*x1;
			double x2p= x2*x2;
			sx= (x2p-x1p)/2;
			double x1e2= c2*pow(x1,e2)*x1;
			double x2e2= c2*pow(x2,e2)*x2;
			sy= y0*(x2-x1) + (x2e2-x1e2)/(1+e2);
			x1e2*= x1;
			x2e2*= x2;
			syx= y0*(x2p-x1p)/2 + (x2e2-x1e2)/(2+e2);
			x1p*= x1;
			x2p*= x2;
			sx2= (x2p-x1p)/3;
			x1e2*= x1;
			x2e2*= x2;
			syx2= y0*(x2p-x1p)/3 + (x2e2-x1e2)/(3+e2);
			x1p*= x1;
			x2p*= x2;
			sx3= (x2p-x1p)/4;
			x1p*= x1;
			x2p*= x2;
			sx4= (x2p-x1p)/5;
			double s1= syx - sy*sx/n;
			double s2= sx*sx2/n - sx3;
			double s3= sx2 - sx*sx/n;
			double s4= syx2 - sy*sx2/n;
			double s5= sx2*sx2/n - sx4;
			double s6= sx3 - sx*sx2/n;
			float c= (s1*s6-s3*s4)/(s3*s5-s2*s6);
			float b= (s1+c*s2)/s3;
			float a= (sy-b*sx-c*sx2)/n;
//			fprintf(stderr," %f %f %f %f %f %f %f %f\n",
//			  n,sy,syx,syx2,sx,sx2,sx3,sx4);
//			fprintf(stderr," %f %f %f\n",a,b,c);
		} 
		if (e1 == 0) {
			float x1= v2;
			float x2= 80*.44704;
			float n= x2;
			double x1p= x1*x1;
			double x2p= x2*x2;
			sx= x2p/2;
			double x1e2= c2*pow(x1,e2)*x1;
			double x2e2= c2*pow(x2,e2)*x2;
			sy= y0*(x2-x1) + (x2e2-x1e2)/(1+e2) + c1*x1;
			x1e2*= x1;
			x2e2*= x2;
			syx= y0*(x2p-x1p)/2 + (x2e2-x1e2)/(2+e2) + c1*x1p/2;
			x1p*= x1;
			x2p*= x2;
			sx2= x2p/3;
			x1e2*= x1;
			x2e2*= x2;
			syx2= y0*(x2p-x1p)/3 + (x2e2-x1e2)/(3+e2) + c1/x1p/3;
			x1p*= x1;
			x2p*= x2;
			sx3= x2p/4;
			x1p*= x1;
			x2p*= x2;
			sx4= x2p/5;
			double s1= syx - sy*sx/n;
			double s2= sx*sx2/n - sx3;
			double s3= sx2 - sx*sx/n;
			double s4= syx2 - sy*sx2/n;
			double s5= sx2*sx2/n - sx4;
			double s6= sx3 - sx*sx2/n;
			float c= (s1*s6-s3*s4)/(s3*s5-s2*s6);
			float b= (s1+c*s2)/s3;
			float a= (sy-b*sx-c*sx2)/n;
//			fprintf(stderr," %f %f %f %f %f %f %f %f\n",
//			  n,sy,syx,syx2,sx,sx2,sx3,sx4);
//			fprintf(stderr," %f %f %f\n",a,b,c);
		}
	} else {
		float x1= 0;
		float y1= 0;
		float x2= 30*.44704;
		float y2= c1*pow(x2,e1);
		float x3= 60*.44704;
		float y3= c1*pow(x3,e1);
		float x1sq= x1*x1;
		float x2sq= x2*x2;
		float x3sq= x3*x3;
		float x21= x2-x1;
		float x31= x3-x1;
		float c= (x21*(y3-y1)-x31*(y2-y1))/(x21*(x3sq-x1sq)-x31*(x2sq-x1sq));
		float b= (y2-y1-c*(x2sq-x1sq))/x21;
		float a= y1-b*x1-c*x1sq;
//		fprintf(stderr," %f %f %f\n",a,b,c);
	}
	MSTSFileNode* engine= wagFile.find("Engine");
	if (engine == NULL)
		return def;
	if (sound!=NULL && sound->getChild(0)!=NULL &&
	  sound->getChild(0)->value!=NULL) {
		path= string(dir)+"/SOUND/"+sound->getChild(0)->value->c_str();
		def->soundFile= path;
		def->soundGain= 1;
	}
	MSTSFileNode* headout= engine->children->find("HeadOut");
	if (headout!=NULL) {
		def->inside.push_back(RailCarInside(
		  atof(headout->getChild(2)->value->c_str()),
		  -atof(headout->getChild(0)->value->c_str()),
		  atof(headout->getChild(1)->value->c_str()),0,0,NULL));
//		fprintf(stderr,"headout %f %f %f\n",def->insideCoord[0],
//		 def->insideCoord[1],def->insideCoord[2]);
	}
//	fprintf(stderr,"got engine\n");
	MSTSFileNode* tp= engine->children->find("Type");
	if (tp == NULL)
		return def;
	tp= tp->getChild(0);
	if (tp==NULL || tp->value==NULL)
		return def;
	fprintf(stderr,"engine type %s\n",tp->value->c_str());
	if (*tp->value == "Diesel") {
		DieselEngine* e= new DieselEngine;
		def->engine= e;
		e->setMaxPower(1000*getFloat(engine,"MaxPower",0,0));
		e->setMaxForce(1000*getFloat(engine,"MaxForce",0,0));
		fprintf(stderr,"maxforce %f\n",e->getMaxForce());
	}
	if (*tp->value == "Electric") {
		ElectricEngine* e= new ElectricEngine;
		def->engine= e;
		e->setMaxPower(1000*getFloat(engine,"MaxPower",0,0));
		e->setMaxForce(1000*getFloat(engine,"MaxForce",0,0));
		fprintf(stderr,"maxforce %f\n",e->getMaxForce());
	}
	if (*tp->value == "Steam") {
		SteamEngine* e= new SteamEngine;
		def->engine= e;
		e->setNumCylinders(
		  (int)(getFloat(engine,"NumCylinders",0,0)+.5));
		float stroke= getFloat(engine,"CylinderStroke",0,0);
		e->setCylStroke(stroke);
		e->setCylDiameter(getFloat(engine,"CylinderDiameter",0,0));
		float diam= 2*getFloat(engine,"WheelRadius",0,0);
		if (diam < stroke)
			diam*= 39.37;// assume meters
		if (diam > 100)
			diam/= 6;// undo AI animation workaround
		e->setWheelDiameter(diam);
		e->setBoilerVolume(getFloat(engine,"BoilerVolume",0,0));
		e->setMaxBoilerPressure(
		  getFloat(engine,"MaxBoilerPressure",0,0));
		e->setIdealFireMass(getFloat(engine,"IdealFireMass",0,0));
		e->setAuxSteamUsage(getFloat(engine,"BasicSteamUsage",0,0));
		e->setSafetyUsage(
		  getFloat(engine,"SafetyValvesSteamUsage",0,0));
		e->setSafetyDrop(
		  getFloat(engine,"SafetyValvePressureDifference",0,0));
		e->setMaxBoilerOutput(getFloat(engine,"MaxBoilerOutput",0,0));
		e->setExhaustLimit(getFloat(engine,"ExhaustLimit",0,0));
	}
	MSTSFileNode* effects= engine->children->find("Effects");
	if (effects!=NULL) {
		MSTSFileNode* steamEffects=
		  effects->children->find("SteamSpecialEffects");
		if (steamEffects) {
			MSTSFileNode* stackFX=
			  steamEffects->children->find("StackFX");
			if (stackFX) {
				RailCarSmoke smoke;
				smoke.position= osg::Vec3f(
				  atof(stackFX->getChild(2)->value->c_str()),
				  atof(stackFX->getChild(0)->value->c_str()),
				  atof(stackFX->getChild(1)->value->c_str()));
				smoke.normal= osg::Vec3f(
				  atof(stackFX->getChild(5)->value->c_str()),
				  atof(stackFX->getChild(3)->value->c_str()),
				  atof(stackFX->getChild(4)->value->c_str()));
				smoke.size=
				  2*atof(stackFX->getChild(6)->value->c_str());
				smoke.minRate= 20;
				smoke.maxRate= 200;
				smoke.minSpeed= 1;
				smoke.maxSpeed= 5;
				def->smoke.push_back(smoke);
			}
		}
	}
	return def;
}
