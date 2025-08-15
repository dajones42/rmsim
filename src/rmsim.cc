//	main code for rail fan simulator
//	uses three camera controls
//		look from the person location
//		look at the person location
//		look down on the person location
//	most user interaction is controled by single keystrokes
//	some multicharacter commands are also supported
//	these are preceeded by ! and end with return
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
#include "ttosim.h"
#include "parser.h"
#include "rmssocket.h"
#include "changelog.h"
#include "switcher.h"
#include "caboverlay.h"
#include "trackeditor.h"
#include "trackpathdrawable.h"
#include "camera.h"
#include "ropedrawable.h"
#include "waterdrawable.h"
#include "ship.h"

#include <osg/Timer>
#include <osg/Group>
#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/StateSet>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/CameraManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgDB/DatabasePager>
#include <osgText/Text>
#include <osg/Drawable>
#include <osg/BlendFunc>
#include <osg/Material>
#include <osg/ComputeBoundsVisitor>
#include <microhttpd.h>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#include <osgShadow/ViewDependentShadowMap>

double fps= 0;
double simTime= 0;
float rtt= 0;
int startTime= 0;
int endTime= 48*60*60;
int timeMult= 1;
int timeWarp= 0;
int timeWarpSigCount= 0;
int timeWarpMsgCount= 0;
float timeWarpDist= 0;
MSTSRoute* mstsRoute= NULL;
TimeTable* timeTable= NULL;
Interlocking* interlocking= NULL;
osg::Node* interlockingModel= NULL;
std::string userOSCallSign;
std::string saveString;
osg::Vec3d clickLocation;
osg::MatrixTransform* clickMT= NULL;
osg::Vec3f clickOffset;
Ship* selectedShip= NULL;
Train* selectedTrain= NULL;
RailCarInst* selectedRailCar= NULL;
RailCarInst* myRailCar= NULL;
osg::Camera* camera= NULL;
TTOSim ttoSim;
std::string command;
bool commandMode= false;
double commandClearTime= 0;
osg::Switch* rootNode;
Track::SwVertex* deferredThrow= NULL;
Switcher* autoSwitcher= NULL;
TrackEditor* trackEditor= NULL;
int hudState= 1;
osgShadow::ShadowMap* shadowMap= NULL;
osgShadow::ShadowedScene* shadowScene= NULL;
int handleWebRequest(void* cls, MHD_Connection* connection, const char* url,
  const char* method, const char* version, const char* upload_data,
  size_t* upload_data_size, void** ptr);

bool hudMouseOn= false;

void saveState(string filename)
{
	FILE* out= fopen(filename.c_str(),"w");
	if (out == NULL)
		throw "cannot create file";
	fprintf(out,"save %s\n",saveString.c_str());
	fprintf(out,"\ttime %f\n",simTime);
	ChangeLog* log= ChangeLog::instance();
	for (ChangeLog::iterator i=log->begin(); i!=log->end(); i++) {
		string s= i->second;
		for (int j=s.find('|'); j!=string::npos; j=s.find('|',j))
			s[j]= ' ';
		fprintf(out,"\t%s\n",s.c_str());
	}
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		WLocation loc;
		t->location.getWLocation(&loc);
		fprintf(out,
		  "\ttrain %d %.2f %.2f %.2f %d %.2f %.2f %.2f %.2f %.2f\n",
		  t->id,loc.coord[0],loc.coord[1],loc.coord[2],t->location.rev,
		  t->speed,t->dControl,t->tControl,t->bControl,t->engBControl);
	}
	if (Person::stack.size() <= Person::stackIndex)
		Person::stack.push_back(currentPerson);
	Person::stack[Person::stackIndex]= currentPerson;
	for (int i=0; i<Person::stack.size(); i++) {
		Person& rf= Person::stack[i];
		osg::Vec3d loc= rf.getLocation();
		fprintf(out,"\tperson %.2f %.2f %.2f %.2f %.2f %.2f",
		  loc[0],loc[1],loc[2]-rf.height,rf.angle,rf.vAngle,rf.height);
		if (rf.follow) {
			osg::Vec3f offset= rf.followOffset;
			for (TrainList::iterator k=trainList.begin();
			  k!=trainList.end(); ++k) {
				Train* train= *k;
				int n= 0;
				for (RailCarInst* car=train->firstCar;
				  car!=NULL; car=car->next) {
					for (int j=0; j<car->def->parts.size();
					  j++) {
						if (car->models[j] ==
						  rf.follow) {
							fprintf(out," %d %d %d"
							  " %.2f %.2f %.2f",
							  train->id,n,j,
							  offset[0],offset[1],
							  offset[2]);
						}
					}
					n++;
				}
			}
		}
		fprintf(out,"\n");
	}
	fprintf(out,"\tpersonindex %d\n",Person::stackIndex);
	fprintf(out,"end\n");
	fclose(out);
}

void startExplore()
{
//	if (trainList.size()>0 || trainMap.size()!=1)
//		return;
	TrainMap::iterator i= trainMap.find("explore");
	if (i == trainMap.end())
		return;
	Train* train= i->second;
	osg::Vec3d ploc= currentPerson.getLocation();
	osg::Vec3f aim= currentPerson.getAim();
	Track::Location tloc;
	findTrackLocation(ploc[0],ploc[1],ploc[2],&tloc);
	Track::Vertex* v= tloc.edge->v1;
	Track::Edge* e= tloc.edge;
	osg::Vec3d tdir= e->v2->location.coord - e->v1->location.coord;
	tdir.normalize();
	float dot= aim*tdir;
	fprintf(stderr,"dot %f\n",dot);
	if (dot > 0)
		tloc.rev= 1;
	train->location= tloc;
	float len= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next)
		len+= car->def->length;
	train->endLocation= train->location;
	train->endLocation.move(-len,1,0);
	float x= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next) {
		car->setLocation(x-car->def->length/2,&train->location);
		x-= car->def->length;
	}
	train->setModelsOn();
	train->setHeadLight(false);
	trainList.push_back(train);
	listener.addTrain(train);
	train->setOccupied();
}

//	multi character command parser
//	commands from network or typed by user
string handleCommand(const char* cmd)
{
	string result= "";
	Parser parser;
	parser.setDelimiters("|");
	parser.setCommand(cmd);
	try {
		if (parser.getString(0)=="print ts" && timeTable!=NULL) {
			timeTable->printTimeSheet2(stderr);
		} else if (parser.getString(0)=="print tsh" &&
		  timeTable!=NULL) {
			timeTable->printTimeSheetHorz2(stderr);
		} else if (parser.getString(0)=="print tt" && timeTable!=NULL) {
			timeTable->print(stderr);
		} else if (parser.getString(0)=="print tth" &&
		  timeTable!=NULL) {
			timeTable->printHorz(stderr);
		} else if (parser.getString(0)=="ts" && timeTable!=NULL) {
			timeTable->printTimeSheet2(result);
		} else if (parser.getString(0)=="meet" && timeTable!=NULL) {
			tt::Train* t1= timeTable->findTrain(
			  parser.getString(1).c_str());
			tt::Train* t2= timeTable->findTrain(
			  parser.getString(2).c_str());
			tt::Station* s= timeTable->findStation(
			  parser.getString(3).c_str());
			if (t1==NULL || t2==NULL)
				result= "cannot find train";
			else if (s==NULL && parser.getString(3)!="cancel")
				result= "cannot find station";
			else if (timeTable->addMeet(t1,t2,s))
				result= "meet order complete";
			else
				result= "invalid meet";
		} else if (parser.getString(0)=="block for" &&
		  timeTable!=NULL) {
			tt::Train* train= timeTable->findTrain(
			  parser.getString(1).c_str());
			if (train == NULL)
				result= "cannot find train";
			else if (timeTable->getBlockFor(train,simTime) == NULL)
				result= string("cannot block for ")+
				  parser.getString(1).c_str();
			else
				result= string("okay, blocking for ")+
				  parser.getString(1).c_str();
			timeTable->printBlocks(stderr);
		} else if (parser.getString(0)=="forces" && myTrain!=NULL) {
			for (RailCarInst* car=myTrain->firstCar; car!=NULL;
			  car=car->next)
				fprintf(stderr,"%s %f %f %f %f %f\n",
				  car->def->name.c_str(),car->speed,
				  car->force,car->drag,car->cU,car->mass);
		} else if (parser.getString(0)=="os") {
			AITrain* t= (AITrain*) timeTable->findTrain(
			  parser.getString(1).c_str());
			if (t == NULL)
				return "cannot find train";
			int r= t->getCurrentRow();
			if (timeTable->getRow(r)->getCallSign()!=userOSCallSign
			  && t->getActualLv(r)>=0)
				r= t->getNextRow(0);
			if (timeTable->getRow(r)->getCallSign()!=userOSCallSign)
				return "wrong station";
			t->osDist= 0;
			if (t->getActualAr(r) < 0)
				t->setArrival(r,simTime);
			t->recordOnSheet(r,simTime,false);
		} else if (parser.getString(0)=="save") {
			saveState(parser.getString(1));
		} else if (parser.getString(0)=="hudmouse") {
			hudMouseOn= true;
		} else if (parser.getString(0)=="exit") {
			exit(0);
		} else if (parser.getString(0)=="auto" && selectedTrain!=NULL) {
			selectedTrain->stop();
			autoSwitcher= new Switcher(selectedTrain);
		} else if (parser.getString(0)=="print loc") {
			printTrackLocations();
		} else if (parser.getString(0)=="edit track") {
			trackEditor->startEditing();
		} else if (parser.getString(0)=="start explore") {
			startExplore();
		} else if (parser.getString(0)=="shadows on") {
			if (shadowScene)
				shadowScene->setShadowTechnique(shadowMap);
		} else if (parser.getString(0)=="shadows off") {
			if (shadowScene)
				shadowScene->setShadowTechnique(NULL);
		} else if (parser.getString(0)=="calcao" && mstsRoute) {
			float ao= mstsRoute->calcAO(currentPerson.location[0],
			  currentPerson.location[1]);
			fprintf(stderr,"AO %.3f\n",ao);
		} else if (parser.getString(0)=="set maxeq" && myTrain) {
			myTrain->setMaxEqResPressure(
			  parser.getDouble(1,70,110,70));
		} else if (parser.getNumTokens()==1 && interlocking!=NULL) {
			int i= parser.getInt(0,1,interlocking->getNumLevers());
			interlocking->toggleState(i-1,simTime);
		} else {
			result= "?";
		}
	} catch (const char* msg) {
		result= msg;
	} catch (const std::exception& error) {
		result= error.what();
	}
	return result;
}

int waybillsOn= 0;

//	turns waybill display on and off
void toggleWaybills()
{
	waybillsOn= 1-waybillsOn;
	fprintf(stderr,"waybillsOn=%d\n",waybillsOn);
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* train= *i;
		for (RailCarInst* car=train->firstCar; car!=NULL;
		  car=car->next) {
			if (car->waybill==NULL || car->waybill->label==NULL)
				continue;
			if (waybillsOn)
				car->waybill->label->setAllChildrenOn();
			else
				car->waybill->label->setAllChildrenOff();
		}
	}
}

int trackLabelsOn= 0;
osg::Switch* trackLabels= NULL;

//	turns waybill display on and off
void toggleTrackLabels()
{
	if (!trackLabels)
		return;
	trackLabelsOn= 1-trackLabelsOn;
	fprintf(stderr,"trackLabelsOn=%d\n",trackLabelsOn);
	if (trackLabelsOn)
		trackLabels->setAllChildrenOn();
	else
		trackLabels->setAllChildrenOff();
}

void adjustAllRopes(int adj)
{
	for (RopeList::iterator j=ropeList.begin(); j!=ropeList.end(); ++j) {
		Rope* r= *j;
		if (r->cleat1->ship==selectedShip ||
		  (r->cleat2!=NULL && r->cleat2->ship==selectedShip)) {
			if (adj<-1 && r->adjust<-1)
				r->adjust--;
			else
				r->adjust= adj;
		}
	}
	selectedShip= NULL;
}


#if 0
//	code to draw a line down the center of track
//	used when viewing from a large distance among other things
//	also draws platform marker if there is a timetable
//	and interlocking signal state information
struct TrackPathDrawable : public osg::Drawable {
	int draw;
	int drawAll;
	TrackPathDrawable() {
		draw= 0;
		drawAll= 1;
	}
	~TrackPathDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new TrackPathDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new TrackPathDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const TrackPathDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "TrackPathDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
};

osg::BoundingSphere TrackPathDrawable::computeBound() const
{
	osg::BoundingSphere bsphere;
	if (myTrain != NULL) {
		Track::Location& loc= myTrain->location;
		WLocation wl;
		loc.getWLocation(&wl);
		bsphere.set(osg::Vec3(wl.coord[0],wl.coord[1],wl.coord[2]),
		  1000);
	}
	return bsphere;
}

void TrackPathDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (draw && timeTable!=NULL) {
		for (int i=0; i<timeTable->getNumRows(); i++) {
			Station* s= (Station*) timeTable->getRow(i);
			glColor3f(1,0,0);
			glBegin(GL_LINES);
			for (int j=0; j<s->locations.size(); j++) {
				WLocation wl;
				s->locations[j].getWLocation(&wl);
				glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]+10);
			}
			glEnd();
		}
	}
#if 0
	glBegin(GL_LINES);
	for (TrainList::iterator j=trainList.begin();
	  j!=trainList.end(); ++j) {
		Train* t= *j;
		WLocation wl;
		t->location.getWLocation(&wl);
		glColor3f(0,1,0);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+10);
		t->endLocation.getWLocation(&wl);
		glColor3f(1,0,0);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+10);
	}
	glEnd();
#endif
	if (drawAll) {
		for (TrackMap::iterator j=trackMap.begin(); j!=trackMap.end();
		  ++j) {
			WLocation wl;
			glColor3f(0,0,0);
			glBegin(GL_LINES);
			for (Track::EdgeList::iterator
			  i=j->second->edgeList.begin();
			  i!=j->second->edgeList.end(); ++i) {
				Track::Edge* e= *i;
				wl= e->v1->location;
				if (e->track->matrix != NULL)
					wl.coord=
					  e->track->matrix->preMult(wl.coord);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]-.1);
				wl= e->v2->location;
				if (e->track->matrix != NULL)
					wl.coord=
					  e->track->matrix->preMult(wl.coord);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]-.1);
			}
			glEnd();
		}
		glPointSize(5.);
		glColor3f(0,0,1);
		glBegin(GL_POINTS);
		for (TrainList::iterator j=trainList.begin();
		  j!=trainList.end(); ++j) {
			Train* t= *j;
			WLocation wl;
			t->location.getWLocation(&wl);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+2);
			t->endLocation.getWLocation(&wl);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+2);
		}
		glEnd();
	}
	if (draw && myTrain!=NULL) {
		Track::Location loc= myTrain->location;
		WLocation wl;
		loc.getWLocation(&wl);
		loc.getWLocation(&wl);
		Track::Vertex* v= loc.rev ? loc.edge->v2 : loc.edge->v1;
		float x= loc.rev ? loc.offset-loc.edge->length : loc.offset;
		glColor3f(0,1,0);
		glBegin(GL_LINE_STRIP);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		for (Track::Edge* e=loc.edge; e!=NULL && x<1000;
		  e=v->nextEdge(e)) {
			x+= e->length;
			v= v==e->v1 ? e->v2 : e->v1;
			if (e->length < .01)
				continue;
			wl= v->location;
			if (e->track->matrix != NULL)
				wl.coord= e->track->matrix->preMult(wl.coord);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		}
		glEnd();
		loc= myTrain->endLocation;
		loc.getWLocation(&wl);
		v= loc.rev ? loc.edge->v1 : loc.edge->v2;
		x= loc.rev ? loc.offset-loc.edge->length : loc.offset;
		glColor3f(1,0,0);
		glBegin(GL_LINE_STRIP);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		for (Track::Edge* e=loc.edge; e!=NULL && x<1000;
		  e=v->nextEdge(e)) {
			x+= e->length;
			v= v==e->v1 ? e->v2 : e->v1;
			if (e->length < .01)
				continue;
			wl= v->location;
			if (e->track->matrix != NULL)
				wl.coord= e->track->matrix->preMult(wl.coord);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		}
		glEnd();
	}
	if (draw)
	for (SignalMap::iterator i=signalMap.begin(); i!=signalMap.end(); ++i) {
	//	fprintf(stderr,"%s %d\n",i->first.c_str(),
	//	  i->second->getNumTracks());
		if (i->second->getState() == Signal::STOP)
			glColor3f(1,0,0);
		else
			glColor3f(0,1,0);
		for (int j=0; j<i->second->getNumTracks(); j++) {
			Track::Location& loc= i->second->getTrack(j);
			WLocation wl;
			loc.getWLocation(&wl);
			glBegin(GL_LINE_STRIP);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+1);
			Track::Vertex* v= loc.rev ? loc.edge->v2 : loc.edge->v1;
			float x= loc.rev ? loc.offset-loc.edge->length :
			  -loc.offset;
			for (Track::Edge* e=loc.edge; e!=NULL && x<10;
			  e=v->nextEdge(e)) {
				x+= e->length;
				v= v==e->v1 ? e->v2 : e->v1;
				if (e->length < .01)
					continue;
				wl= v->location;
				if (e->track->matrix != NULL)
					wl.coord= e->track->matrix->preMult(
					  wl.coord);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]+1);
			}
			glEnd();
		}
	}
}

TrackPathDrawable* trackPathDrawable= NULL;
#endif

int getAIMsgCount()
{
	int n= 0;
	if (timeTable != NULL) {
		for (int i=timeTable->getNumTrains()-1; i>=0; i--) {
			AITrain* train= (AITrain*) timeTable->getTrain(i);
			int r= train->getRow();
			int a= train->getActualAr(r);
			int d= train->getActualLv(r);
			if (train->consist==NULL || r<0 || a<0)
				continue;
			if (train->message.size()>0 &&
			  train->message[0]=='*')
				n++;
		}
	}
	return n;
}

//	code to draw interlocking lever state in the hud
//	also handles mouse hits to change lever state
struct InterlockingDrawable : public osg::Drawable {
	float x0;
	float centerY;
	float wid;
	float ht;
	float dx;
	float dy;
	float dh;
	InterlockingDrawable() {
		x0= 800;
		centerY= 710;
		wid= 4;
		dx= 10;
		ht= 30;
		dy= 10;
		dh= 5;
	}
	~InterlockingDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new InterlockingDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new InterlockingDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const InterlockingDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "InterlockingDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
	bool handleMouseHit(float x, float y);
};

osg::BoundingSphere InterlockingDrawable::computeBound() const
{
	osg::BoundingSphere bbox;
	if (interlocking != NULL) {
		bbox.set(osg::Vec3(x0,centerY-ht-dy/2,-100),
		  dx*interlocking->getNumLevers());
	}
	return bbox;
}

void InterlockingDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (hudState != 1)
		return;
	interlocking->updateRouteLocks(simTime);
	for (int i=0; i<interlocking->getNumLevers(); i++) {
		osg::Vec3f c= interlocking->getColor(i);
		glColor3f(c[0],c[1],c[2]);
		float x= x0 + dx*i;
		float y= centerY;
		float h= ht;
		if (interlocking->getState(i) == Interlocking::REVERSE) {
			y-= dy;
			h-= dh;
		} else if (interlocking->getState(i) ==
		  (Interlocking::REVERSE|Interlocking::NORMAL)) {
			y-= dy/2;
			h-= dh/2;
		}
		glRectd(x,y-h/2,x+wid,y+h/2);
		Signal* s= interlocking->getSignal(i);
		if (s!=NULL && s->trainDistance>0) {
			if (interlocking->getLockDurationS(i,simTime) > 0)
				glColor3f(1,.5,0);
			else
				glColor3f(1,1,1);
			glRectd(x,centerY+ht/2+wid,x+wid,centerY+ht/2+2*wid);
		}
		Interlocking::SwitchState occ=
		  interlocking->getSwitchOccupied(i);
		if (occ == Interlocking::OCCUPIED) {
			glColor3f(1,0,0);
			glRectd(x,centerY+ht/2+wid,x+wid,centerY+ht/2+2*wid);
		} else if (occ == Interlocking::ROUTELOCK) {
			glColor3f(1,1,0);
			glRectd(x,centerY+ht/2+wid,x+wid,centerY+ht/2+2*wid);
		}
	}
}

bool InterlockingDrawable::handleMouseHit(float x, float y)
{
	if (myTrain==NULL && timeTable!=NULL) {
		if (100<x && x<200 && 740<y && y<760) {
			if (timeWarp) {
				timeMult= 1;
				timeWarp= 0;
			} else {
				timeWarp= 2;
				if (timeMult < 32)
					timeMult= 32;
				timeWarpSigCount=
				  interlocking->countUnclearedSignals();
				timeWarpMsgCount= getAIMsgCount();
				timeWarpDist= 0;
			}
			return true;
		}
		int j= 0;
		for (int i=timeTable->getNumTrains()-1; i>=0; i--) {
			AITrain* t= (AITrain*) timeTable->getTrain(i);
			int r= t->getRow();
			int a= t->getActualAr(r);
			int d= t->getActualLv(r);
			if (t->consist==NULL || r<0 || a<0)
				continue;
//			fprintf(stderr,"i j %d %d\n",i,j);
			if (20<x && x<150 && 720-20*j<y && y<740-20*j) {
				int r= t->getCurrentRow();
				if (timeTable->getRow(r)->getCallSign()!=
				  userOSCallSign
				  && t->getActualLv(r)>=0)
					r= t->getNextRow(0);
				if (timeTable->getRow(r)->getCallSign()==
				  userOSCallSign) {
					t->osDist= 0;
					if (t->getActualAr(r) < 0)
						t->setArrival(r,simTime);
					t->recordOnSheet(r,simTime,false);
					return true;
				}
			}
			j++;
			if (t->message.size() > 0)
				j++;
		}
	}
	if (interlocking==NULL ||
	  x<x0 || x>x0+interlocking->getNumLevers()*dx ||
	  y<centerY-dy-ht/2 || y>centerY+ht/2)
		return false;
	int i= (x-x0)/dx;
//	fprintf(stderr,"toggle %d\n",i);
	if (i>=0 && i<interlocking->getNumLevers()) {
		interlocking->toggleState(i,simTime);
		if (!commandMode) {
			char buf[20];
			sprintf(buf,"%d",i+1);
			command= buf;
		}
	}
	return true;
}

InterlockingDrawable* interlockingDrawable= NULL;

//	code to draw a bar graph of couple slack in hud
//	bars are shown in green if there is a pulling force,
//	red for a pushing force and blue if no force
//	bar length depends on coupler force
struct SlackDrawable : public osg::Drawable {
	float x0;
	float y0;
	float dy;
	float ht;
	SlackDrawable() {
		x0= 8;
		y0= 750;
		ht= -4;
		dy= -6;
	}
	~SlackDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new SlackDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new SlackDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const SlackDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "SlackDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
};

osg::BoundingSphere SlackDrawable::computeBound() const
{
	osg::BoundingSphere bbox;
	if (myTrain != NULL) {
		int n= 0;
		for (RailCarInst* c=myTrain->firstCar; c!=myTrain->lastCar;
		  c=c->next)
			n++;
		bbox.set(osg::Vec3(x0,y0+dy*n/2+ht/2,0),500);
	}
	return bbox;
}

void SlackDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (myTrain == NULL)
		return;
	static float xScale= 0;
	if (xScale < myTrain->maxCForce)
		xScale= myTrain->maxCForce;
	else
		xScale= .999*xScale + .001*myTrain->maxCForce;
	float y= y0;
	for (RailCarInst* c=myTrain->firstCar; c!=myTrain->lastCar; c=c->next) {
		float x1= x0;
		float x2= x1+4;
		if (c->cU < 0) {
			glColor3f(0.,1.,0.);
			x1-= (int)(-6*c->cU/xScale+.5);
		} else if (c->cU > 0) {
			glColor3f(1.,0.,0.);
			x2+= (int)(6*c->cU/xScale+.5);
		} else {
			glColor3f(0.,0.,1.);
		}
		glRectd(x1,y+ht,x2,y);
		y+= dy;
	}
}

SlackDrawable* slackDrawable= NULL;

//	code to draw a graph of air brake pressures
struct AirBrakeDrawable : public osg::Drawable {
	float x0;
	float y0;
	float dx;
	float ht;
	AirBrakeDrawable() {
		x0= 10;
		y0= 10;
		ht= 70;
		dx= 6;
	}
	~AirBrakeDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new AirBrakeDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new AirBrakeDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const AirBrakeDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "AirBrakeDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
};

osg::BoundingSphere AirBrakeDrawable::computeBound() const
{
	osg::BoundingSphere bbox;
	if (myTrain != NULL) {
		int n= 0;
		for (RailCarInst* c=myTrain->firstCar; c!=myTrain->lastCar;
		  c=c->next)
			n++;
		bbox.set(osg::Vec3(x0,y0,0),dx*n+10);
	}
	return bbox;
}

void AirBrakeDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (myTrain == NULL)
		return;
	static float xScale= 0;
	if (xScale < myTrain->maxCForce)
		xScale= myTrain->maxCForce;
	else
		xScale= .999*xScale + .001*myTrain->maxCForce;
	float x= x0;
	for (RailCarInst* c=myTrain->firstCar; c!=NULL; c=c->next) {
		switch (c->airBrake->getTripleValveState()) {
		  case 0: glColor3f(0.,1.,0.); break;
		  case 1: glColor3f(0.,0.,1.); break;
		  case 2: glColor3f(1.,0.,0.); break;
		  case 3: glColor3f(0.,1.,1.); break;
		  case 4: glColor3f(1.,1.,0.); break;
		  default: glColor3f(1.,0.,1.); break;
		}
		glRectd(x-dx/2,y0-4,x+dx/2,y0-1);
		x+= dx;
	}
	for (int i=0; i<3; i++) {
		if (i == 0)
			glColor3f(0.,1.,0.);
		else if (i == 1)
			glColor3f(0.,0.,1.);
		else
			glColor3f(1.,0.,0.);
		float x= x0;
		glBegin(GL_LINE_STRIP);
		for (RailCarInst* c=myTrain->firstCar; c!=NULL; c=c->next) {
			float p= i==0 ? c->airBrake->getPipePressure() :
			  i==1 ? c->airBrake->getAuxResPressure() :
			  c->airBrake->getCylPressure();
			if (c==myTrain->firstCar)
				glVertex3d(x-.5*dx,y0+p,1);
			glVertex3d(x,y0+p,1);
			x+= dx;
		}
		glEnd();
	}
}

AirBrakeDrawable* airBrakeDrawable= NULL;

//	code to draw a track profile in the hud
//	draw green in front of the player train,
//	red behind it and blue in the middle
struct ProfileDrawable : public osg::Drawable {
	bool draw;
	float x0;
	float y0;
	float xScale;
	float yScale;
	ProfileDrawable() {
		draw= false;
		x0= 500;
		y0= 400;
		xScale= .4;
		yScale= 20*xScale;
	}
	~ProfileDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new ProfileDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new ProfileDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const ProfileDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "ProfileDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
};

osg::BoundingSphere ProfileDrawable::computeBound() const
{
	osg::BoundingSphere bbox;
	bbox.set(osg::Vec3(500,400,0),500);
	return bbox;
}

void ProfileDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (!draw || myTrain==NULL)
		return;
	for (int iter=0; iter<2; iter++) {
	Track::Location loc= myTrain->location;
	WLocation wl;
	loc.getWLocation(&wl);
	Track::Vertex* v= loc.rev ? loc.edge->v2 : loc.edge->v1;
	float x= loc.rev ? loc.edge->length-loc.offset : loc.offset;
	x= -x;
	float z= wl.coord[2];
	if (iter == 1)
		loc.getWLocation(&wl,0,true);
	if (iter == 1)
		glColor3f(0,1,0);
	else
		glColor3f(0,.6,0);
	glBegin(GL_LINE_STRIP);
	glVertex3d(x0,y0+yScale*(wl.coord[2]-z),1);
	for (Track::Edge* e=loc.edge; e!=NULL && x<1500;
	  e=v->nextEdge(e)) {
		x+= e->length;
		v= v==e->v1 ? e->v2 : e->v1;
		wl= v->location;
		if (e->track->matrix != NULL)
			wl.coord= e->track->matrix->preMult(wl.coord);
		if (iter == 1)
			wl.coord[2]= v->elevation;
		glVertex3d(x0+xScale*x,y0+yScale*(wl.coord[2]-z),1);
		if (v->type == Track::VT_SWITCH) {
			glVertex3d(x0+xScale*x,y0+yScale*(wl.coord[2]-z)-5,1);
			glVertex3d(x0+xScale*x,y0+yScale*(wl.coord[2]-z),1);
		}
	}
	glEnd();
	loc= myTrain->location;
	loc.getWLocation(&wl);
	v= !loc.rev ? loc.edge->v2 : loc.edge->v1;
	x= !loc.rev ? loc.edge->length-loc.offset : loc.offset;
	x= -x;
	if (iter == 1)
		glColor3f(0,0,1);
	else
		glColor3f(0,0,.6);
	if (iter == 1)
		loc.getWLocation(&wl,0,true);
	glBegin(GL_LINE_STRIP);
	glVertex3d(x0,y0+yScale*(wl.coord[2]-z),1);
	for (Track::Edge* e=loc.edge; e!=NULL; e=v->nextEdge(e)) {
 		if (e == myTrain->endLocation.edge)
			break;
		v= v==e->v1 ? e->v2 : e->v1;
		wl= v->location;
		if (e->track->matrix != NULL)
			wl.coord= e->track->matrix->preMult(wl.coord);
		if (iter == 1)
			wl.coord[2]= v->elevation;
		x+= e->length;
		glVertex3d(x0-xScale*x,y0+yScale*(wl.coord[2]-z),1);
		if (v->type == Track::VT_SWITCH) {
			glVertex3d(x0-xScale*x,y0+yScale*(wl.coord[2]-z)-5,1);
			glVertex3d(x0-xScale*x,y0+yScale*(wl.coord[2]-z),1);
		}
	}
	loc= myTrain->endLocation;
	loc.getWLocation(&wl);
	v= !loc.rev ? loc.edge->v2 : loc.edge->v1;
	x= !loc.rev ? loc.edge->length-loc.offset : loc.offset;
	x= -x;
	if (iter == 1)
		loc.getWLocation(&wl,0,true);
	glVertex3d(x0-xScale*myTrain->length,y0+yScale*(wl.coord[2]-z),1);
	glEnd();
	if (iter == 1)
		glColor3f(1,0,0);
	else
		glColor3f(.6,0,0);
	glBegin(GL_LINE_STRIP);
	glVertex3d(x0-xScale*myTrain->length,y0+yScale*(wl.coord[2]-z),1);
	x+= myTrain->length;
	for (Track::Edge* e=loc.edge; e!=NULL && x<1500;
	  e=v->nextEdge(e)) {
		x+= e->length;
		v= v==e->v1 ? e->v2 : e->v1;
		wl= v->location;
		if (e->track->matrix != NULL)
			wl.coord= e->track->matrix->preMult(wl.coord);
		if (iter == 1)
			wl.coord[2]= v->elevation;
		glVertex3d(x0-xScale*x,y0+yScale*(wl.coord[2]-z),1);
		if (v->type == Track::VT_SWITCH) {
			glVertex3d(x0-xScale*x,y0+yScale*(wl.coord[2]-z)-5,1);
			glVertex3d(x0-xScale*x,y0+yScale*(wl.coord[2]-z),1);
		}
	}
	glEnd();
	}
}

ProfileDrawable* profileDrawable= NULL;

void printMT(osg::MatrixTransform* mt)
{
	if (mt) {
		osg::Matrixd m= mt->getMatrix();
		float angle= atan2(m(2,0),m(2,1))*180/M_PI;
		fprintf(stderr,"%s %.3f %.3f %.3f %f %f %f %f\n",
		  mt->getName().c_str(),m(3,0),m(3,1),m(3,2),
		  m(2,0),m(2,1),m(2,2),angle);
	}
}

void moveMT(osg::MatrixTransform* mt, float dx, float dy, float dz)
{
	if (mt) {
		osg::Matrixd m= mt->getMatrix();
		m(3,0)+= dx;
		m(3,1)+= dy;
		m(3,2)+= dz;
		mt->setMatrix(m);
		printMT(mt);
	}
}

void rotateMT(osg::MatrixTransform* mt, float da)
{
	if (mt) {
		osg::Matrixd m= mt->getMatrix();
//		for (int i=0; i<4; i++)
//			fprintf(stderr," %d %f %f %f %f\n",
//			  i,m(i,0),m(i,1),m(i,2),m(i,3));
		float angle= atan2(-m(2,0),m(2,1)) + da*M_PI/180;
		float cs= cos(angle);
		float sn= sin(angle);
		fprintf(stderr,"%f %f %f\n",cs,sn,angle*180/M_PI);
		mt->setMatrix(osg::Matrixd(
		   cs,sn,0,0,
		   0,0,1,0,
		   -sn,cs,0,0,
		   m(3,0),m(3,1),m(3,2),1));
		printMT(mt);
	}
}

//	code to handle user input from mouse and keyboard
struct Controller : public osgGA::GUIEventHandler {
	float downX;
	float downY;
	float upX;
	float upY;
	Cleat* downCleat;
	Cleat* upCleat;
	Controller() {
		downX= 0;
		downY= 0;
		upX= 0;
		upY= 0;
	};
	~Controller() {
	};
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	void findSelection(osgViewer::Viewer* viewer, float x, float y,
	  bool moveTo);
	void controlClick(osgViewer::Viewer* viewer, float x, float y);
};

void Controller::controlClick(osgViewer::Viewer* viewer, float x, float y)
{
	if (viewer == NULL)
		return;
	Cleat* cleat1= currentPerson.findCleat();
	if (cleat1 == NULL)
		return;
	fprintf(stderr,"addrope %f %f\n",x,y);
	typedef osgUtil::LineSegmentIntersector::Intersections Hits;
	Hits hits;
	if (!viewer->computeIntersections(x,y,hits))
		return;
	double fovy,ar,zn,zf;
	camera->getProjectionMatrixAsPerspective(fovy,ar,zn,zf);
	double bestr= 1;
	Hits::iterator bestHit= hits.end();
	for (Hits::iterator hit=hits.begin(); hit!=hits.end(); ++hit) {
		osg::StateSet* ss= hit->drawable->getStateSet();
		if (ss && (ss->getBinNumber()>=10 ||
		  ss->getRenderingHint() == osg::StateSet::TRANSPARENT_BIN)) {
			unsigned int mask= 0xffffffff;
			for (int i=0; i<hit->nodePath.size(); i++)
				mask&= hit->nodePath[i]->getNodeMask();
			if (mask == 1)
				continue;
		}
		if (zn+(zf-zn)*hit->ratio>.5 && bestr>hit->ratio) {
			bestr= hit->ratio;
			bestHit= hit;
		}
	}
	if (bestHit == hits.end())
		return;
	clickLocation= bestHit->getWorldIntersectPoint();
	double bestD= 10;
	Cleat* bestC= NULL;
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i) {
		Ship* ship= *i;
		if (ship == cleat1->ship)
			continue;
		for (Cleat* c=ship->cleats; c!=NULL; c=c->next) {
			double xy[3];
			c->getPosition(xy);
			double dx= xy[0]-clickLocation[0];
			double dy= xy[1]-clickLocation[1];
			double dz= xy[2]-clickLocation[2];
			double d= dx*dx+dy*dy+dz*dz;
			if (bestD > d) {
				bestD= d;
				bestC= c;
			}
		}
	}
	if (bestC)
		addRope(cleat1,bestC);
	else
		addRope(cleat1,clickLocation[0],clickLocation[1],
		  clickLocation[2]);
}

//	finds the selected object from a mouse hit
void Controller::findSelection(osgViewer::Viewer* viewer, float x, float y,
  bool moveTo)
{
	if (viewer == NULL)
		return;
	fprintf(stderr,"clickxy %f %f\n",x,y);
	typedef osgUtil::LineSegmentIntersector::Intersections Hits;
	Hits hits;
	if (!viewer->computeIntersections(x,y,hits))
		return;
	double fovy,ar,zn,zf;
	camera->getProjectionMatrixAsPerspective(fovy,ar,zn,zf);
	double bestr= 1;
	Hits::iterator bestHit= hits.end();
	for (Hits::iterator hit=hits.begin(); hit!=hits.end(); ++hit) {
		osg::StateSet* ss= hit->drawable->getStateSet();
//		osg::Vec3d p= hit->getWorldIntersectPoint();
//		fprintf(stderr,"hit %lf %lf %lf %lf %d %d %f\n",
//		  p[0],p[1],p[2],hit->ratio,ss?ss->getRenderingHint():-1,
//		  hit->nodePath.size()>0?
//		  hit->nodePath[hit->nodePath.size()-1]->getNodeMask():0,
//		  zn+(zf-zn)*hit->ratio);
		if (ss && (ss->getBinNumber()>=10 ||
		  ss->getRenderingHint() == osg::StateSet::TRANSPARENT_BIN)) {
			unsigned int mask= 0xffffffff;
			for (int i=0; i<hit->nodePath.size(); i++)
				mask&= hit->nodePath[i]->getNodeMask();
//			fprintf(stderr," %x\n",mask);
			if (mask == 1)
				continue;
		}
		if (zn+(zf-zn)*hit->ratio>.5 && bestr>hit->ratio) {
			bestr= hit->ratio;
			bestHit= hit;
		}
	}
	if (bestHit == hits.end())
		return;
	clickLocation= bestHit->getWorldIntersectPoint();
	clickMT= NULL;
	selectedShip= NULL;
	selectedTrain= NULL;
	selectedRailCar= NULL;
	fprintf(stderr,"person %f %f %f %f %f %f\n",
	  currentPerson.location[0],currentPerson.location[1],currentPerson.location[2],
	  currentPerson.angle,currentPerson.cosAngle,currentPerson.sinAngle);
	fprintf(stderr,"click %lf %lf %lf\n",
	  clickLocation[0],clickLocation[1],clickLocation[2]);
	if (interlockingModel != NULL) {
		for (int k=0; k<bestHit->nodePath.size(); k++) {
			osg::MatrixTransform* t=
			  dynamic_cast<osg::MatrixTransform*>
			  (bestHit->nodePath[k]);
			if (t == NULL)
				continue;
			InterlockingAnimPathCB* apcb=
			  dynamic_cast<InterlockingAnimPathCB*>
			  (t->getUpdateCallback());
			if (apcb != NULL) {
				interlocking->toggleState(apcb->lever,simTime);
				if (!commandMode) {
					char buf[20];
					sprintf(buf,"%d",apcb->lever+1);
					command= buf;
				}
				return;
			}
		}
	}
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* train= *i;
		for (RailCarInst* car=train->firstCar; car!=NULL;
		  car=car->next) {
			for (int j=0; j<car->def->parts.size(); j++) {
				if (car->models[j] == NULL)
					continue;
				for (int k=0; k<bestHit->nodePath.size(); k++) {
					osg::MatrixTransform* t=
					  dynamic_cast<osg::MatrixTransform*>
					  (bestHit->nodePath[k]);
					if (car->models[j] == t) {
						clickMT= t;
						clickOffset= clickLocation-
						  t->getMatrix().getTrans();
						osg::Quat q= t->getMatrix().
						  getRotate();
						osg::Vec3f axis;
						double angle;
						q.getRotate(angle,axis);
						q.makeRotate(-angle,axis);
						clickOffset= q*clickOffset;
						selectedTrain= train;
						selectedRailCar= car;
					}
				}
			}
		}
	}
	for (ShipList::iterator i=shipList.begin(); i!=shipList.end(); ++i) {
		Ship* ship= *i;
		if (ship->model == NULL)
			continue;
		for (int j=0; j<bestHit->nodePath.size(); j++) {
			osg::MatrixTransform* t=
			  dynamic_cast<osg::MatrixTransform*>
			   (bestHit->nodePath[j]);
			if (t == ship->model) {
				selectedRailCar= NULL;
				selectedTrain= NULL;
				selectedShip= ship;
				clickMT= t;
				clickOffset= clickLocation-
				  t->getMatrix().getTrans();
				osg::Quat q= t->getMatrix().getRotate();
				osg::Vec3f axis;
				double angle;
				q.getRotate(angle,axis);
				q.makeRotate(-angle,axis);
				clickOffset= q*clickOffset;
				fprintf(stderr,"selectedShip %p %f %f %f %f\n",
				  selectedShip,clickOffset[0],
				  clickOffset[1]);
			}
		}
	}
	if (moveTo)
		currentPerson.setMoveTo(clickLocation,clickMT,clickOffset,
		  selectedRailCar,selectedTrain,selectedShip);
	if (myTrain == selectedTrain)
		myRailCar= selectedRailCar;
	else
		myRailCar= NULL;
	if (clickMT == NULL) {
		for (int k=0; k<bestHit->nodePath.size(); k++) {
			osg::MatrixTransform* t=
			  dynamic_cast<osg::MatrixTransform*>
			  (bestHit->nodePath[k]);
			//fprintf(stderr,"%d %p %d\n",k,t,t?t->getNodeMask():0);
			if (t && t->getNodeMask()==0x10)
				clickMT= t;
		}
		printMT(clickMT);
	}
}

bool handleHudMouseHit(float x, float y)
{
	osg::Viewport* vp= camera->getViewport();
	y-= vp->height()-768;
	if (myTrain) {
		if (20<x && x<95 && 680<y && y<690) {
			myTrain->decReverser();
			return true;
		}
		if (100<x && x<150 && 680<y && y<690) {
			myTrain->incReverser();
			return true;
		}
		if (20<x && x<95 && 660<y && y<670) {
			myTrain->decThrottle();
			return true;
		}
		if (100<x && x<150 && 660<y && y<670) {
			myTrain->incThrottle();
			return true;
		}
		if (20<x && x<76 && 640<y && y<650) {
			myTrain->decBrakes();
			return true;
		}
		if (80<x && x<150 && 640<y && y<650) {
			myTrain->incBrakes();
			return true;
		}
		if (20<x && x<112 && 620<y && y<630) {
			if (myTrain->engBControl == 0)
				myTrain->bailOff();
			else
				myTrain->decEngBrakes();
			return true;
		}
		if (114<x && x<150 && 620<y && y<630) {
			myTrain->incEngBrakes();
			return true;
		}
		if (myRailCar != NULL) {
			if (20<x && x<120 && 600<y && y<610) {
				myRailCar->decHandBrakes();
				return true;
			}
			if (124<x && x<153 && 600<y && y<610) {
				myRailCar->incHandBrakes();
				return true;
			}
			if (myRailCar->airBrake!=NULL) {
				if (155<x && x<192 && 600<y && y<610) {
					myRailCar->airBrake->decRetainer();
					return true;
				}
				if (196<x && x<250 && 600<y && y<610) {
					myRailCar->airBrake->incRetainer();
					return true;
				}
			}
		}
	}
	return false;
}

//	handles a single mouse press or key stroke
//	multi-character commands begin with ! and end with return
//	other user actions are single characters
bool Controller::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled() || (trackEditor && trackEditor->editing))
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::PUSH:
		downX= ea.getX();
		downY= ea.getY();
//		fprintf(stderr,"down %f %f %d\n",downX,downY,ea.getButton());
		if (ea.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON &&
		 (ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_SHIFT)!=0) {
			findSelection(dynamic_cast<osgViewer::Viewer*>(&aa),
			  downX,downY,true);
			return true;
		} else if (ea.getButton()==
		  osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON &&
		  (ea.getModKeyMask()&osgGA::GUIEventAdapter::MODKEY_CTRL)!=0) {
			controlClick(dynamic_cast<osgViewer::Viewer*>(&aa),
			  downX,downY);
			return true;
		} else if (ea.getButton()==
		  osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
//			if ((ea.getModKeyMask()&
//			  osg::GA::GUIEventAdapter::MODKEY_SHIFT) != 0)
//				move
			if (interlockingDrawable &&
			  interlockingDrawable->handleMouseHit(downX,downY))
				return true;
			if (hudMouseOn && handleHudMouseHit(downX,downY))
				return true;
			findSelection(dynamic_cast<osgViewer::Viewer*>(&aa),
			  downX,downY,
			  interlockingDrawable==NULL&&hudMouseOn==false);
		} else { 
			findSelection(
			  dynamic_cast<osgViewer::Viewer*>(&aa),downX,downY,
			  ea.getButton()==
			  osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON);
			return true;
		}
		break;
	 case osgGA::GUIEventAdapter::RELEASE:
		upX= ea.getX();
		upY= ea.getY();
		if (ea.getButton()==osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON &&
		  myTrain && myTrain->bControl>0)
			myTrain->decBrakes();
#if 0
		if (upX==downX && upY==downY) {
			if (interlockingDrawable &&
			  interlockingDrawable->handleMouseHit(upX,upY))
				return true;
			findSelection(
			  dynamic_cast<osgViewer::Viewer*>(&aa),upX,upY);
		}
#endif
//		fprintf(stderr,"up %f %f\n",upX,upY);
//		fprintf(stderr,"%d %d %d %d\n",
//		  ea.getWindowX(),ea.getWindowY(),
//		  ea.getWindowWidth(),ea.getWindowHeight());
//		fprintf(stderr,"%f %f %f %f %d\n",
//		  ea.getXmin(),ea.getXmax(),
//		  ea.getYmin(),ea.getYmax(),ea.getMouseYOrientation());
		break;
#if 0
	 case osgGA::GUIEventAdapter::SCROLL:
		if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_UP) {
//			fprintf(stderr,"scroll up %f %f\n",
//			  ea.getScrollingDeltaX(),ea.getScrollingDeltaY());
			currentPerson.incAngle(-5);
		} else if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_DOWN) {
//			fprintf(stderr,"scroll down %f %f\n",
//			  ea.getScrollingDeltaX(),ea.getScrollingDeltaY());
			currentPerson.incAngle(5);
		}
		break;
#endif
	 case osgGA::GUIEventAdapter::KEYDOWN:
		if (commandMode) {
			switch (ea.getKey()) {
			 case osgGA::GUIEventAdapter::KEY_Return:
				{
				fprintf(stderr,"handle %s\n",command.c_str());
				string r= handleCommand(command.c_str());
				fprintf(stderr,"returned %s\n",r.c_str());
				commandMode= false;
				command= r;
				}
				return true;
			 case osgGA::GUIEventAdapter::KEY_Delete:
			 case osgGA::GUIEventAdapter::KEY_BackSpace:
				if (command.size() > 0)
					command.resize(command.size()-1);
				return true;
			 default:
				if (ea.getKey()>=' ' && ea.getKey()<0x7f) {
					command+= (char)ea.getKey();
					return true;
				}
			}
		} else if (commandClearTime < simTime) {
			command.clear();
		}
		switch (ea.getKey()) {
		 case osgGA::GUIEventAdapter::KEY_F7:
			toggleWaybills();
			return true;
		 case osgGA::GUIEventAdapter::KEY_F6:
			toggleTrackLabels();
			return true;
		 case osgGA::GUIEventAdapter::KEY_F5:
			hudState++;
			if (hudState > 2)
				hudState= 0;
			return true;
		 case '!':
			commandMode= true;
			command.erase();
			return true;
		 case '>':
			if (myTrain) {
				myTrain->nextStopDist=
				  //myTrain->location.maxDistance(false);
				  myTrain->coupleDistance(false);
				myTrain->targetSpeed= myTrain->maxTargetSpeed;
				if (myTrain->targetSpeed < myTrain->speed)
					myTrain->targetSpeed= myTrain->speed;
				fprintf(stderr,"fwd %f\n",
				  myTrain->nextStopDist);
			}
			break;
		 case '<':
			if (myTrain) {
				myTrain->nextStopDist=
				  //-myTrain->endLocation.maxDistance(true);
				  myTrain->coupleDistance(true);
				myTrain->targetSpeed= myTrain->maxTargetSpeed;
				if (myTrain->targetSpeed < -myTrain->speed)
					myTrain->targetSpeed= -myTrain->speed;
				fprintf(stderr,"back %f\n",
				  myTrain->nextStopDist);
			}
			break;
		 case '^':
			if (myTrain) {
				myTrain->targetSpeed*= 1.4142;
			}
			break;
		 case ',':
			if (myTrain) {
				myTrain->targetSpeed/= 1.4142;
			} else {
				currentPerson.moveAlongPath(false);
			}
			break;
		 case '.':
			if (myTrain) {
				float d= .5*myTrain->speed*myTrain->speed/
				 (myTrain->decelMult*myTrain->maxDecel);
				float mind= .5*myTrain->speed*myTrain->speed/
				 myTrain->maxDecel;
				if (deferredThrow!=NULL &&
				  deferredThrow->occupied) {
					Track::Location loc(deferredThrow);
					float d1= myTrain->speed<0 ?
					  -myTrain->location.dDistance(&loc) :
					  myTrain->endLocation.dDistance(&loc);
					d1+= 1;
					fprintf(stderr,"%f\n",d1);
					if (d1 < mind)
						d1= mind;
					if (d1 < 1000)
						d= d1;
				}
				if (myTrain->speed < 0) {
				  	myTrain->nextStopDist=
				  	 (myTrain->nextStopDist-mind)/2;
				  	if (myTrain->nextStopDist < -d)
						myTrain->nextStopDist= -d;
				}
				if (myTrain->speed > 0) {
			  		myTrain->nextStopDist=
			  		 (myTrain->nextStopDist+mind)/2;
					if (myTrain->nextStopDist > d)
						myTrain->nextStopDist= d;
				}
				//myTrain->targetSpeed/= 2;
				fprintf(stderr,"stop %f %f %f %f %f\n",
				  myTrain->nextStopDist,myTrain->speed,
				 myTrain->decelMult,myTrain->maxDecel,mind);
			} else {
				currentPerson.moveAlongPath(true);
			}
			break;
		 case '(':
			if (myTrain)
				currentPerson.moveAlongTrain(myTrain,false);
			else
				currentPerson.moveAlongPath(false);
			return true;
		 case ')':
			if (myTrain)
				currentPerson.moveAlongTrain(myTrain,true);
			else
				currentPerson.moveAlongPath(true);
			return true;
		 case 'z':
			timeMult/= 2;
			return true;
		 case 'x':
			if (timeMult == 0)
				timeMult= 1;
			else if (timeMult < 1000)
				timeMult*= 2;
			return true;
		 case 'W':
			timeWarp= 2;
			if (timeMult < 32)
				timeMult= 32;
			timeWarpSigCount=
			  interlocking->countUnclearedSignals();
			timeWarpMsgCount= getAIMsgCount();
			timeWarpDist= 0;
			return true;
		 case 'g':
			if (currentPerson.follow==NULL) {
				osg::Vec3d loc= currentPerson.location;
				for (TrackMap::iterator i=trackMap.begin();
				  i!=trackMap.end(); ++i) {
					deferredThrow= i->second->findSwitch(
					  loc[0],loc[1],loc[2]);
					if (deferredThrow == NULL)
						continue;
			  		if (myTrain!=NULL &&
					  deferredThrow->occupied &&
					  myTrain->nextStopDist!=0) {
						myTrain->stopAtSwitch(
						  deferredThrow);
						return true;
					}
					deferredThrow->throwSwitch(NULL,false);
					if (deferredThrow->occupied == 0)
						deferredThrow= NULL;
					return true;
				}
			}
			break;
		 case 'a':
			if (myShip != NULL)
				myShip->engine.decThrottle();
			else if (myTrain != NULL)
				myTrain->decThrottle();
			else if (clickMT)
				moveMT(clickMT,-.1,0,0);
			return true;
		 case 'd':
			if (myShip != NULL)
				myShip->engine.incThrottle();
			else if (myTrain != NULL)
				myTrain->incThrottle();
			else if (clickMT)
				moveMT(clickMT,.1,0,0);
			return true;
		 case 's':
			if (myShip != NULL)
				myShip->engine.setThrottle(0);
			else if (myTrain != NULL)
				myTrain->decReverser();
			else if (clickMT)
				moveMT(clickMT,0,-.1,0);
			return true;
		 case 'w':
			if (myTrain != NULL)
				myTrain->incReverser();
			else if (clickMT)
				moveMT(clickMT,0,.1,0);
			return true;
		 case 'J':
			if (myShip != NULL) {
				myShip->rudder.move(1);
			}
			return true;
		 case osgGA::GUIEventAdapter::KEY_Return:
			currentPerson.stopMove();
			return true;
		 case 'j':
			if (myShip != NULL) {
				myShip->rudder.move(5);
			} else if (currentPerson.moveTo) {
				currentPerson.jump();
			}
			return true;
		 case 'f':
			if (selectedRailCar!=NULL &&
			  selectedRailCar->engine!=NULL &&
			  dynamic_cast<SteamEngine*>
			  (selectedRailCar->engine)!=NULL) {
				if (mySteamEngine != NULL)
					mySteamEngine->setAutoFire(true);
				mySteamEngine= dynamic_cast<SteamEngine*>
				  (selectedRailCar->engine);
				mySteamEngine->setAutoFire(false);
			} else if (clickMT) {
				moveMT(clickMT,0,0,-.1);
			}
			return true;
		 case 'F':
			if (mySteamEngine != NULL)
				mySteamEngine->setAutoFire(true);
			mySteamEngine= NULL;
			return true;
		 case 'k':
			if (myShip != NULL)
				myShip->rudder.set(0);
			else if (mySteamEngine != NULL)
				mySteamEngine->incInjector(0);
			else if (clickMT)
				rotateMT(clickMT,-.1);
			return true;
		 case 'l':
			if (myShip != NULL)
				myShip->rudder.move(-5);
			else if (mySteamEngine != NULL)
				mySteamEngine->incInjector(1);
			else if (clickMT)
				rotateMT(clickMT,.1);
			return true;
		 case 'L':
			if (myShip != NULL)
				myShip->rudder.move(-1);
			else if (mySteamEngine != NULL)
				mySteamEngine->decInjector(1);
			return true;
		 case 'i':
			if (mySteamEngine != NULL)
				mySteamEngine->toggleInjector(0);
			return true;
		 case 'K':
			if (mySteamEngine != NULL)
				mySteamEngine->decInjector(0);
			return true;
		 case 'o':
			if (mySteamEngine != NULL)
				mySteamEngine->toggleInjector(1);
			return true;
		 case 'n':
			if (mySteamEngine != NULL)
				mySteamEngine->incBlowerFraction();
			return true;
		 case 'N':
			if (mySteamEngine != NULL)
				mySteamEngine->decBlowerFraction();
			return true;
		 case 'm':
			if (mySteamEngine != NULL)
				mySteamEngine->incDamperFraction();
			return true;
		 case 'M':
			if (mySteamEngine != NULL)
				mySteamEngine->decDamperFraction();
			return true;
		 case ';':
			if (myTrain != NULL)
				myTrain->decBrakes();
			return true;
		 case '\'':
			if (myTrain != NULL)
				myTrain->incBrakes();
			return true;
		 case '/':
			if (myTrain != NULL)
				myTrain->bailOff();
			return true;
		 case '[':
			if (myTrain != NULL)
				myTrain->decEngBrakes();
			return true;
		 case ']':
			if (myTrain != NULL)
				myTrain->incEngBrakes();
			return true;
		 case '{':
			if (myRailCar != NULL)
				myRailCar->decHandBrakes();
			return true;
		 case '}':
			if (myRailCar != NULL)
				myRailCar->incHandBrakes();
			return true;
		 case ':':
			if (myRailCar!=NULL && myRailCar->airBrake!=NULL)
				myRailCar->airBrake->decRetainer();
			return true;
		 case '"':
			if (myRailCar!=NULL && myRailCar->airBrake!=NULL)
				myRailCar->airBrake->incRetainer();
			return true;
		 case '4':
			currentPerson.moveInside();
			return true;
		 case '$':
			currentPerson.setRemoteLocation();
			return true;
		 case 'c':
			if (selectedShip != NULL) {
				myShip= selectedShip;
				myTrain= NULL;
			} else if (selectedTrain != NULL) {
				myTrain= selectedTrain;
				myRailCar= selectedRailCar;
				if (myRailCar->airBrake!=NULL &&
				  dynamic_cast<EngAirBrake*>
				  (myRailCar->airBrake)!=NULL)
					myTrain->setEngAirBrake(
					  dynamic_cast<EngAirBrake*>
					  (myRailCar->airBrake));
				myShip= NULL;
				if (timeTable!=NULL &&
				  ttoSim.takeControlOfAI(myTrain))
					myTrain->convertToAirBrakes();
			}
			return true;
		 case 'r':
			if (mySteamEngine != NULL) {
				mySteamEngine->incFiringRate();
			} else if (clickMT) {
				moveMT(clickMT,0,0,.1);
			} else {
				myTrain= NULL;
				myShip= NULL;
			}
			return true;
		 case 'R':
			if (mySteamEngine != NULL) {
				mySteamEngine->decFiringRate();
			} else {
				if (myTrain!=NULL && timeTable!=NULL &&
				  !ttoSim.convertToAI(myTrain))
					fprintf(stderr,
					  "%s not converted to AI\n",
					  myTrain->name.c_str());
				myTrain= NULL;
				myShip= NULL;
			}
			return true;
		 case 'O':
			if (myTrain!=NULL && timeTable!=NULL &&
			  !ttoSim.osUserTrain(myTrain,simTime))
				fprintf(stderr,"%s not os'd\n",
				  myTrain->name.c_str());
			return true;
		 case 'C':
			if (myTrain != NULL)
				myTrain->connectAirHoses();
			else
				currentPerson.connectFloatBridge();
			return true;
		 case '9':
			Person::swap(4);
			return true;
		 case '7':
			Person::swap(2);
			return true;
		 case '8':
			Person::swap(3);
			return true;
		 case '5':
			Person::swap(0);
			//rootNode->setAllChildrenOff();
			return true;
		 case '6':
			Person::swap(1);
			//rootNode->setAllChildrenOn();
			return true;
		 case 'p':
			if (trackPathDrawable != NULL) {
				trackPathDrawable->draw=
				  1-trackPathDrawable->draw;
			}
			return true;
		 case 'P':
			if (profileDrawable != NULL)
				profileDrawable->draw= !profileDrawable->draw;
			return true;
		 case 'H':
			if (myTrain != NULL)
				myTrain->setHeadLight(false);
			return true;
		 case 'u':
			if (myTrain!=NULL && currentPerson.follow==NULL) {
				osg::Vec3d loc= currentPerson.location;
				myTrain->uncouple(loc);
			} else {
				currentPerson.removeRopes();
#if 0
			} else if (selectedShip != NULL) {
				RopeList del;
				for (RopeList::iterator j=ropeList.begin();
				  j!=ropeList.end(); ++j) {
					Rope* r= *j;
					if (r->cleat1->ship==selectedShip ||
					  (r->cleat2!=NULL &&
					  r->cleat2->ship==selectedShip))
						del.push_back(r);
				}
				for (RopeList::iterator j=del.begin();
				  j!=del.end(); ++j)
					ropeList.remove(*j);
				if (selectedShip->mass == 0) {
					for (FBList::iterator j=fbList.begin();
					  j!=fbList.end(); ++j) {
						FloatBridge* fb= *j;
						if (fb == selectedShip)
							fb->disconnect();
					}
				}
				selectedShip= NULL;
#endif
			}
			return true;
		 case 'e':
			currentPerson.adjustRopes(1);
			return true;
		 case 't':
			currentPerson.adjustRopes(-1);
			return true;
		 case 'h':
			if (myTrain != NULL) {
				myTrain->setHeadLight(true);
			} else {
				currentPerson.adjustRopes(0);
			}
			return true;
#if 0
		 case 'c':
			if (selectedShip!=NULL && selectedShip->mass==0) {
				for (FBList::iterator j=fbList.begin();
				  j!=fbList.end(); ++j) {
					FloatBridge* fb= *j;
					if (fb == selectedShip)
						fb->connect();
				}
				selectedShip= NULL;
			} else if (selectedShip != NULL) {
				myShip= selectedShip;
				myTrain= NULL;
				selectedShip= NULL;
			} else if (selectedTrain != NULL) {
				myTrain= selectedTrain;
				myShip= NULL;
				selectedTrain= NULL;
				selectedRailCar= NULL;
			}
			return true;
		 case 'p':
			if (currentPerson.ship) {
				currentPerson.adjustRopes(-2);
			} else if (trackPathDrawable != NULL) {
				trackPathDrawable->draw=
				  1-trackPathDrawable->draw;
			}
			return true;
#endif
		 default:
			break;
		}
		break;
	 default:
		break;
	}
	return false;
}

#if 0
//	camera manipulator for looking down at the person (map view)
struct MapManipulator : public osgGA::CameraManipulator {
	osg::Vec3d offset;
	float distance;
	MapManipulator() {
		distance= 20000;
		offset= osg::Vec3d(0,0,0);
	};
	~MapManipulator() {
	};
	void init(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix(const osg::Matrixd& m);
	virtual void setByInverseMatrix(const osg::Matrixd& m);
	void viewAllTrack();
};

osg::Matrixd MapManipulator::getMatrix() const
{
	osg::Matrixd m;
	osg::Vec3d c= currentPerson.getLocation()+offset;
	m.makeLookAt(c+osg::Vec3d(0,0,distance),c,osg::Vec3d(0,1,0));
	return m;
}

osg::Matrixd MapManipulator::getInverseMatrix() const
{
	return getMatrix();
}

void MapManipulator::setByMatrix(const osg::Matrixd& m)
{
}

void MapManipulator::setByInverseMatrix(const osg::Matrixd& m)
{
}

void MapManipulator::init(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
//	fprintf(stderr,"mminit\n");
	camera->setComputeNearFarMode(
	  osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
	trackPathDrawable->drawAll= 0;
	if (distance > 1000)
		trackPathDrawable->drawAll= 1;
	CabOverlay::setImage(NULL);
}

bool MapManipulator::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled())
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		 case ' ':
			offset[0]= 0;
			offset[1]= 0;
			offset[2]= 0;
			distance= 20000;
			trackPathDrawable->drawAll= 1;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Up:
			offset[1]+= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Down:
			offset[1]-= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Home:
			offset[0]-= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_End:
			offset[0]+= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Up:
			distance/= 2;
			if (distance < 1000)
				trackPathDrawable->drawAll= 0;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Down:
			distance*= 2;
			if (distance > 1000)
				trackPathDrawable->drawAll= 1;
			return true;
		 default:
			break;
		}
		break;
	 default:
		break;
	}
	return false;
}

void MapManipulator::viewAllTrack()
{
	double minx= 1e20;
	double maxx= -1e20;
	double miny= 1e20;
	double maxy= -1e20;
	double minz= 1e20;
	double maxz= -1e20;
	for (TrackMap::iterator j=trackMap.begin(); j!=trackMap.end(); ++j) {
		Track* t= j->second;
		t->calcMinMax();
		if (minx > t->minVertexX)
			minx= t->minVertexX;
		if (maxx < t->maxVertexX)
			maxx= t->maxVertexX;
		if (miny > t->minVertexY)
			miny= t->minVertexY;
		if (maxy < t->maxVertexY)
			maxy= t->maxVertexY;
		if (minz > t->minVertexZ)
			minz= t->minVertexZ;
		if (maxz < t->maxVertexZ)
			maxz= t->maxVertexZ;
	}
	float x= 2*(maxx-minx);
	float y= 2*(maxy-miny);
	if (x>y && distance>x)
		distance= x;
	if (y>x && distance>y)
		distance= y;
}

//	camera manipulator for looking at the person
struct LookAtManipulator : public osgGA::CameraManipulator {
	float distance;
	int angle;
	float cosAngle;
	float sinAngle;
	int vAngle;
	float cosv;
	float sinv;
	void setAngle(int a) {
		if (a < 0)
			a+= 360;
		if (a > 360)
			a-= 360;
		angle= a;
		cosAngle= cos(a*3.14159/180);
		sinAngle= sin(a*3.14159/180);
		//currentPerson.setAngle(cosAngle,sinAngle);
	};
	void incAngle(int d) { setAngle(angle+d); }
	void setVAngle(int a) {
		if (a < 0)
			a+= 360;
		if (a > 360)
			a-= 360;
		vAngle= a;
		cosv= cos(a*3.14159/180);
		sinv= sin(a*3.14159/180);
	};
	void incVAngle(int d) { setVAngle(vAngle+d); }
	LookAtManipulator() {
		distance= 100;
		setAngle(0);
		setVAngle(10);
	};
	~LookAtManipulator() {
	};
	void init(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix(const osg::Matrixd& m);
	virtual void setByInverseMatrix(const osg::Matrixd& m);
};

osg::Matrixd LookAtManipulator::getMatrix() const
{
	osg::Matrixd m;
	osg::Vec3d c= currentPerson.getLocation();
	if (currentPerson.useRemote) {
		if ((c-currentPerson.remoteLocation).length() > 1000)
			currentPerson.setRemoteLocation();
		m.makeLookAt(currentPerson.remoteLocation,c,osg::Vec3d(0,0,1));
		return m;
	}
#if 0
	osg::Vec3d aim= osg::Vec3d(cosv*distance*cosAngle,
	  cosv*distance*sinAngle,distance*sinv);
	if (currentPerson.follow) {
		osg::Quat q= currentPerson.follow->getMatrix().getRotate();
		aim= q*aim;
	}
	m.makeLookAt(c+aim,c,osg::Vec3d(0,0,1));
#else
	osg::Vec3d ra= currentPerson.getAim();
	osg::Vec3d aim= osg::Vec3d(
#if 0
	  -cosv*distance*(cosAngle*ra[0]-sinAngle*ra[1]),
	  -cosv*distance*(cosAngle*ra[1]+sinAngle*ra[0]),distance*sinv);
#else
	  -cosv*distance*ra[0],-cosv*distance*ra[1],distance*sinv);
#endif
	m.makeLookAt(c+aim,c,osg::Vec3d(0,0,1));
#endif
	return m;
}

osg::Matrixd LookAtManipulator::getInverseMatrix() const
{
	return getMatrix();
//	osg::Matrixd m;
//	m.invert(getMatrix());
//	return m;
}

void LookAtManipulator::setByMatrix(const osg::Matrixd& m)
{
}

void LookAtManipulator::setByInverseMatrix(const osg::Matrixd& m)
{
}

void LookAtManipulator::init(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
//	fprintf(stderr,"lookatinit\n");
//	if (distance < 50) {
		double fovy,ar,zn,zf;
		camera->getProjectionMatrixAsPerspective(
		  fovy,ar,zn,zf);
		camera->setComputeNearFarMode(
		  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
		camera->setProjectionMatrixAsPerspective(
		  fovy,ar,.2,4000);
//	} else {
//		camera->setComputeNearFarMode(
//		  osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//	}
	trackPathDrawable->drawAll= 0;
	if (distance > 1000)
		trackPathDrawable->drawAll= 1;
	CabOverlay::setImage(NULL);
}

bool LookAtManipulator::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled())
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		 case ' ':
			currentPerson.reset();
			distance= 100;
			setAngle(0);
			setVAngle(10);
			trackPathDrawable->drawAll= 0;
			//camera->setComputeNearFarMode(osg::CullSettings::
			//  COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
			return true;
		 case '0':
			if (currentPerson.modelSwitch->getValue(0))
				currentPerson.modelSwitch->setAllChildrenOff();
			else
				currentPerson.modelSwitch->setAllChildrenOn();
			return true;
		 case osgGA::GUIEventAdapter::KEY_Up:
			distance/= 1.5;
			if (distance < 1000)
				trackPathDrawable->drawAll= 0;
//			if (distance<50 ){//&& distance*1.5>=50) {
//				double fovy,ar,zn,zf;
//				camera->getProjectionMatrixAsPerspective(
//				  fovy,ar,zn,zf);
//				camera->setComputeNearFarMode(
//				  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
//				camera->setProjectionMatrixAsPerspective(
//				  fovy,ar,.2,4000);
//			}
//			fprintf(stderr,"dist %f\n",distance);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Down:
			distance*= 1.5;
			if (distance > 1000)
				trackPathDrawable->drawAll= 1;
//			if (distance>=50 )//&& distance/1.5<50)
//				camera->setComputeNearFarMode(
//				  osg::CullSettings::
//				  COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//			fprintf(stderr,"dist %f\n",distance);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Left:
			currentPerson.incAngle(5);
			//incAngle(-5);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Right:
			currentPerson.incAngle(-5);
			//incAngle(5);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Down:
			incVAngle(-5);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Up:
			incVAngle(5);
			return true;
		 case '$':
			//currentPerson.moveInside();
			currentPerson.setRemoteLocation();
			return true;
		 default:
			break;
		}
		break;
	 case osgGA::GUIEventAdapter::SCROLL:
		if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_UP) {
#if 0
			currentPerson.incAngle(-5);
#else
			distance/= 1.5;
			if (distance < 1000)
				trackPathDrawable->drawAll= 0;
			if (distance<50 ){//&& distance*1.5>=50) {
				double fovy,ar,zn,zf;
				camera->getProjectionMatrixAsPerspective(
				  fovy,ar,zn,zf);
				camera->setComputeNearFarMode(
				  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
				camera->setProjectionMatrixAsPerspective(
				  fovy,ar,.2,4000);
			}
//			fprintf(stderr,"up dist %f\n",distance);
#endif
			return true;
		} else if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_DOWN) {
#if 0
			currentPerson.incAngle(5);
#else
			distance*= 1.5;
			if (distance > 1000)
				trackPathDrawable->drawAll= 1;
			if (distance>=50 )//&& distance/1.5<50)
				camera->setComputeNearFarMode(
				  osg::CullSettings::
				  COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//			fprintf(stderr,"down dist %f\n",distance);
#endif
			return true;
		}
		break;
	 default:
		break;
	}
	return false;
}

//	camera manipulator for looking from the person
struct LookFromManipulator : public osgGA::CameraManipulator {
	LookFromManipulator() {
	};
	~LookFromManipulator() {
	};
	void init(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix(const osg::Matrixd& m);
	virtual void setByInverseMatrix(const osg::Matrixd& m);
};

osg::Matrixd LookFromManipulator::getMatrix() const
{
	osg::Matrixd m;
	m.makeLookAt(currentPerson.getLocation(),
	  currentPerson.getLocation()+currentPerson.getAim(),
	  osg::Vec3d(0,0,1));
	return m;
}

osg::Matrixd LookFromManipulator::getInverseMatrix() const
{
	return getMatrix();
}

void LookFromManipulator::setByMatrix(const osg::Matrixd& m)
{
}

void LookFromManipulator::setByInverseMatrix(const osg::Matrixd& m)
{
}

void LookFromManipulator::init(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	fprintf(stderr,"lookfrominit %f %f %f\n",currentPerson.getAim()[0],
	  currentPerson.getAim()[1],currentPerson.getAim()[2]);
	double fovy,ar,zn,zf;
	camera->getProjectionMatrixAsPerspective(fovy,ar,zn,zf);
	camera->setComputeNearFarMode(
	  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	camera->setProjectionMatrixAsPerspective(fovy,ar,.2,4000);
	trackPathDrawable->drawAll= 0;
//	if (currentPerson.modelSwitch != NULL)
//		currentPerson.modelSwitch->setAllChildrenOff();
	CabOverlay::setImage(currentPerson.insideImage);
	fprintf(stderr,"insideimage %p\n",currentPerson.insideImage);
}

bool LookFromManipulator::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled())
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		 case ' ':
			currentPerson.reset();
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Up:
			currentPerson.incVAngle(5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Down:
			currentPerson.incVAngle(-5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Left:
			currentPerson.incAngle(5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Right:
			currentPerson.incAngle(-5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Home:
			currentPerson.move(0,.5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_End:
			currentPerson.move(0,-.5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Up:
			currentPerson.incHeight(.5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Down:
			currentPerson.incHeight(-.5);
			CabOverlay::setImage(NULL);
			return true;
		 case '4':
			currentPerson.moveInside();
			CabOverlay::setImage(currentPerson.insideImage);
			fprintf(stderr,"lookfrom moveiside\n");
			return true;
		 default:
			break;
		}
		break;
	 case osgGA::GUIEventAdapter::SCROLL:
		if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_UP) {
			currentPerson.incAngle(-5);
			return true;
		} else if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_DOWN) {
			currentPerson.incAngle(5);
			return true;
		}
		break;
	 default:
		break;
	}
	return false;
}
#endif

//	head up display code

#define HUDLINES	20

osg::Geode* hudGeode;

struct HudUpdateCallback : public osg::NodeCallback {
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	osg::Camera* hudCamera;
	HudUpdateCallback(osg::Camera* cam) {
		hudCamera= cam;
	}
};

osg::Camera* createHUD()
{
	osg::Camera* camera= new osg::Camera;
	camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1024,0,768));
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setViewMatrix(osg::Matrix::identity());
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	camera->setRenderOrder(osg::Camera::POST_RENDER,1);
	camera->setAllowEventFocus(false);
	hudGeode= new osg::Geode;
	camera->addChild(hudGeode);
	hudGeode->setUpdateCallback(new HudUpdateCallback(camera));
	osg::StateSet* stateSet= hudGeode->getOrCreateStateSet();
	stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	for (int i=0; i<HUDLINES; i++) {
		osgText::Text* text= new osgText::Text;
		hudGeode->addDrawable(text);
		text->setDataVariance(osg::Object::DYNAMIC);
		text->setFont("fonts/arial.ttf");
		text->setCharacterSize(15);
		//text->setPosition(osg::Vec3(20,20+20*i,0));
		text->setPosition(osg::Vec3(20,740-20*i,0));
		text->setColor(osg::Vec4(1,0,0,1));
	}
	if (interlocking != NULL) {
		interlockingDrawable= new InterlockingDrawable();
		interlockingDrawable->x0= 1010-
		  interlockingDrawable->dx*interlocking->getNumLevers();
		interlockingDrawable->centerY= 750-interlockingDrawable->ht/2;
		interlockingDrawable->setDataVariance(osg::Object::DYNAMIC);
		interlockingDrawable->setUseDisplayList(false);
		hudGeode->addDrawable(interlockingDrawable);
		if (interlockingModel != NULL) {
			SetInterlockingVisitor visitor(interlocking);
			interlockingModel->accept(visitor);
		}
	}
	slackDrawable= new SlackDrawable();
	slackDrawable->setDataVariance(osg::Object::DYNAMIC);
	slackDrawable->setUseDisplayList(false);
	hudGeode->addDrawable(slackDrawable);
	airBrakeDrawable= new AirBrakeDrawable();
	airBrakeDrawable->setDataVariance(osg::Object::DYNAMIC);
	airBrakeDrawable->setUseDisplayList(false);
	hudGeode->addDrawable(airBrakeDrawable);
	profileDrawable= new ProfileDrawable();
	profileDrawable->setDataVariance(osg::Object::DYNAMIC);
	profileDrawable->setUseDisplayList(false);
	hudGeode->addDrawable(profileDrawable);
	return camera;
}

void setHUDText(int i, const char* s)
{
	if (i >= HUDLINES)
		return;
	osgText::Text* text= (osgText::Text*) hudGeode->getDrawable(i);
	if (text != NULL)
		text->setText(s);
}

void HudUpdateCallback::operator()(osg::Node* node,
  osg::NodeVisitor* nv)
{
	if (hudCamera) {
		osg::Viewport* vp= camera->getViewport();
		hudCamera->setProjectionMatrix(
		  osg::Matrix::ortho2D(0,vp->width(),
		   768-vp->height(),768));
	}
	char buf[100];
	int n= 0;
	if (hudState == 0) {
		for (; n<HUDLINES; n++)
			setHUDText(n,"");
		return;
	}
	int t= (int)simTime;
	if (rtt > 0)
		sprintf(buf,"%d:%2.2d:%2.2d Time mult: %d fps %.3lf rtt %.3f",
		  t/3600,t/60%60,t%60,timeMult,fps,rtt);
	else
		sprintf(buf,"%d:%2.2d:%2.2d Time mult: %d fps %.3lf",
		  t/3600,t/60%60,t%60,timeMult,fps);
	setHUDText(n++,buf);
	if (myTrain != NULL) {
		sprintf(buf,"Speed: %.1f mph %.2f",myTrain->speed*2.23693,
		  myTrain->firstCar->distance*3.281/5280);
		setHUDText(n++,buf);
		if (myTrain->nextStopDist!=0) {
			sprintf(buf,"Target: %.1f mph %.3f",
			  myTrain->targetSpeed*2.23693,
			  myTrain->bControl);
			setHUDText(n++,buf);
		}
		sprintf(buf,"Accel: %.3f g %f",myTrain->accel/9.8,
		  myTrain->location.grade());
		setHUDText(n++,buf);
		sprintf(buf,"Reverser: %.0f%%",100*myTrain->dControl);
		setHUDText(n++,buf);
		sprintf(buf,"Throttle: %.0f%%",100*myTrain->tControl);
		setHUDText(n++,buf);
		if (myTrain->engAirBrake != NULL)
			sprintf(buf,"Brakes: %s %.0f %.0f %.0f %.0f %.0f %.1f",
			  myTrain->bControl<0?"R":myTrain->bControl>0?"S":"L",
			  myTrain->engAirBrake->getEqResPressure(),
			  myTrain->engAirBrake->getPipePressure(),
			  myTrain->engAirBrake->getAuxResPressure(),
			  myTrain->engAirBrake->getCylPressure(),
			  myTrain->engAirBrake->getMainResPressure(),
			  myTrain->engAirBrake->getAirFlowCFM());
		else
			sprintf(buf,"Brakes: %.1f",myTrain->bControl);
		setHUDText(n++,buf);
		sprintf(buf,"Eng Brakes: %.0f%%",100*myTrain->engBControl);
		setHUDText(n++,buf);
		if (myRailCar!=NULL && myRailCar->airBrake!=NULL) {
			sprintf(buf,"Hand Brakes: %.0f%% Ret: %s %.0f %.0f",
			  100*myRailCar->handBControl,
			  myRailCar->airBrake->getRetainerName().c_str(),
			  myRailCar->airBrake->getCylPressure(),
			  myRailCar->airBrake->getPipePressure());
			setHUDText(n++,buf);
		} else if (myRailCar != NULL) {
			sprintf(buf,"Hand Brakes: %.0f%%",
			  100*myRailCar->handBControl);
			setHUDText(n++,buf);
		}
		float bp= -1;
		float usage= -1;
		float usagePct= 0;
		for (RailCarInst* c=myTrain->firstCar; c!=NULL; c=c->next) {
			if (c->engine == NULL)
				continue;
			SteamEngine* e= dynamic_cast<SteamEngine*>(c->engine);
			if (e) {
				float x= e->getBoilerPressure();
				if (x > bp)
					bp= x;
				x= e->getUsage();
				if (x > usage)
					usage= x;
				x= e->getUsagePercent();
				if (x > usagePct)
					usagePct= x;
			}
		}
		if (bp >= 0) {
			float upm= 0;
			if (myTrain->speed>0)
				upm= 3600*usage/(myTrain->speed*2.23693);
			sprintf(buf,"Boiler Pressure: %.0f %.1f %.1f",
			  bp,3600*usage,usagePct);//upm);
			setHUDText(n++,buf);
		}
		if (myTrain->firstCar != myTrain->lastCar) {
			sprintf(buf,"CForce: %.0f",myTrain->maxCForce);
			setHUDText(n++,buf);
		}
		if (timeTable!=NULL &&
		  ttoSim.dispatcher.isOnReservedBlock(myTrain))
			setHUDText(n++,"On Reserved Block");
	}
	if (myShip != NULL) {
		float sfwd= myShip->getSpeed();
		float thrust= myShip->engine.getThrust(sfwd);
		float drag= sfwd>0 ? myShip->fwdDrag(sfwd) :
		  -myShip->backDrag(-sfwd);
		sprintf(buf,"Speed: %.1f mph %f knots %f %f",
		  myShip->getSpeed()*2.237,
		  myShip->getSpeed()*1.944,
		  thrust,drag);
		setHUDText(n++,buf);
		sprintf(buf,"Heading: %.0f",myShip->getHeading());
		setHUDText(n++,buf);
		sprintf(buf,"Throttle: %.2f",myShip->engine.throttle);
		setHUDText(n++,buf);
		sprintf(buf,"Rudder: %d",myShip->rudder.angle);
		setHUDText(n++,buf);
		sprintf(buf,"Depth: %.1f",myShip->location.depth);
		setHUDText(n++,buf);
		sprintf(buf,"SOG: %.1f mph",myShip->getSOG()*2.23693);
		setHUDText(n++,buf);
		sprintf(buf,"COG: %.0f",myShip->getCOG());
		setHUDText(n++,buf);
	}
	if (timeTable != NULL) {
		for (int i=timeTable->getNumTrains()-1; i>=0; i--) {
			AITrain* train= (AITrain*) timeTable->getTrain(i);
			int r= train->getRow();
			int a= train->getActualAr(r);
			int d= train->getActualLv(r);
			if (train->consist==NULL || r<0 || a<0)
				continue;
			if (d < 0)
				d= a;
			d/= 60;
			WLocation wl;
			train->consist->location.getWLocation(&wl);
			sprintf(buf, "train %-5.5s %s %2d:%2.2d %s "
			  "%4.1f m %2.0f mph %2.0f %3.0f",
			  train->getName().c_str(),
			  timeTable->getRow(r)->getCallSign().c_str(),
			  d/60,d%60,
			  train->route.c_str(),
			  (currentPerson.getLocation()-wl.coord).length()*3.281/5280,
			  train->consist->speed*2.24,
			  8*train->consist->tControl,
			  100*train->consist->bControl);
			setHUDText(n++,buf);
			if (train->message.size()>0) {
				sprintf(buf," %s",train->message.c_str());
				setHUDText(n++,buf);
			}
		}
	}
	if (autoSwitcher && autoSwitcher->train) {
		sprintf(buf, "autoswitcher %2.0f mph %5.3f %5.3f %d %.1f",
		  autoSwitcher->train->speed*2.24,
		  autoSwitcher->train->tControl,
		  autoSwitcher->train->bControl,
		  autoSwitcher->moves,
		  autoSwitcher->train->nextStopDist);
		setHUDText(n++,buf);
	}
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t->remoteControl==0 || t->speed==0)
			continue;
		WLocation wl;
		t->location.getWLocation(&wl);
		sprintf(buf, "train %-5.5s "
		  "%4.1f m %2.0f mph %.3f",
		  t->name.c_str(),
		  (currentPerson.getLocation()-wl.coord).length()*3.281/5280,
		  t->speed*2.24,t->positionError);
		setHUDText(n++,buf);
	}
	if (mySteamEngine != NULL) {
		sprintf(buf,"Boiler Pressure: %.1f",
		  mySteamEngine->getBoilerPressure());
		setHUDText(n++,buf);
		sprintf(buf,"Steam Usage: %.0f",3600*mySteamEngine->getUsage());
		setHUDText(n++,buf);
		sprintf(buf,"Steam Gen.: %.0f",3600*mySteamEngine->getEvap());
		setHUDText(n++,buf);
		sprintf(buf,"Firing Rate: %.0f %.1f",
		  3600*mySteamEngine->getFiringRate(),
		  mySteamEngine->getFireMass());
		setHUDText(n++,buf);
		sprintf(buf,"Blower: %.0f%%",
		  100*mySteamEngine->getBlowerFraction());
		setHUDText(n++,buf);
		sprintf(buf,"Damper: %.0f%%",
		  100*mySteamEngine->getDamperFraction());
		setHUDText(n++,buf);
		sprintf(buf,"Water Level: %.3f",
		  mySteamEngine->getWaterFraction());
		setHUDText(n++,buf);
		sprintf(buf,"Injectors: %.0f%% %.0f%%",
		  mySteamEngine->getInjectorOn(0)?
		  100*mySteamEngine->getInjectorFraction(0):0,
		  mySteamEngine->getInjectorOn(1)?
		  100*mySteamEngine->getInjectorFraction(1):0);
		setHUDText(n++,buf);
	}
#if 0
	if (interlocking) {
		for (int i=0; i<interlocking->getNumLevers(); i++)
			if (interlocking->isLocked(i))
				buf[i]= interlocking->getState(i)==
				  Interlocking::NORMAL?'N':'R';
			else
				buf[i]= interlocking->getState(i)==
				  Interlocking::NORMAL?'n':'r';
		buf[interlocking->getNumLevers()]= '\0';
		setHUDText(n++,buf);
	}
#endif
	if (commandMode) {
		sprintf(buf,"!%s",command.c_str());
		setHUDText(n++,buf);
	} else if (command.size() > 0) {
		setHUDText(n++,command.c_str());
	}
	if (listener.playingMorse()) {
		setHUDText(n++,listener.getMorseMessage().c_str());
	}
	for (; n<HUDLINES; n++)
		setHUDText(n,"");
}

void makeWaterDrawables(osg::Group* root)
{
	osg::Geode* geode= new osg::Geode;
	osg::StateSet* stateSet= geode->getOrCreateStateSet();
	stateSet->setMode(GL_CULL_FACE,osg::StateAttribute::OFF);
	if (mstsRoute==NULL || mstsRoute->drawWater==0) {
		for (WaterList::iterator i=waterList.begin();
		  i!=waterList.end(); ++i) {
			WaterDrawable* wd= new WaterDrawable(*i);
//			wd->setUseDisplayList(false);
			geode->addDrawable(wd);
		}
	}
	ropeDrawable= new RopeDrawable();
	ropeDrawable->setDataVariance(osg::Object::DYNAMIC);
	ropeDrawable->setUseDisplayList(false);
	geode->addDrawable(ropeDrawable);
	root->addChild(geode);
}

void makeTrackGeometry(osg::Group* rootNode)
{
	for (TrackMap::iterator i=trackMap.begin(); i!=trackMap.end(); ++i) {
		Track* t= i->second;
		if (t->shape == NULL)
			continue;
		if (mstsRoute && i->first==mstsRoute->routeID)
			continue;
//		fprintf(stderr,"maketrackgeom %s\n",i->first.c_str());
		Track* copy= t->expand();
		copy->shape= t->shape;
//		fprintf(stderr," %d %d\n",
//		  t->vertexList.size(),copy->vertexList.size());
		osg::Geometry* g= copy->makeGeometry();
		delete copy;
		if (g == NULL)
			continue;
		t->geode= new osg::Geode();
		t->geode->addDrawable(g);
		if (t->matrix) {
			for (ShipList::iterator i=shipList.begin();
			  i!=shipList.end(); ++i) {
				Ship* ship= *i;
				if (ship->model == NULL)
					continue;
				if (ship->track == t)
					ship->model->addChild(t->geode);
			}
			for (FBList::iterator i=fbList.begin();
			  i!=fbList.end(); ++i) {
				FloatBridge* fb= *i;
				if (fb->model == NULL)
					continue;
				for (FloatBridge::FBTrackList::iterator j=
				  fb->tracks.begin(); j!=fb->tracks.end();
				  ++j) {
					FloatBridge::FBTrack* fbt= *j;
					if (fbt->track == t) {
						osg::MatrixTransform* transform=
						  new osg::MatrixTransform;
						transform->addChild(t->geode);
						rootNode->addChild(transform);
						fbt->model= transform;
					}
				}
			}
		} else {
			rootNode->addChild(t->geode);
		}
	}
}

void updateActivityEvents()
{
	if (!mstsRoute || commandMode)
		return;
	if (command.size()>0) {
		if (simTime < commandClearTime) {
			int dt= (int)(4*(commandClearTime-simTime));
			if (dt>15)
				hudState= dt%2;
			else
				hudState= 1;
			return;
		}
		hudState= 1;
		command.erase();
	}
	for (MSTSRoute::EventMap::iterator i=mstsRoute->eventMap.begin();
	  i!=mstsRoute->eventMap.end(); i++) {
		Event* event= i->second;
		if (event->time>0 && event->time<simTime) {
			fprintf(stderr,"\nevent %d time %d\n%s\n\n",
			  event->id,event->time,event->message.c_str());
			if (event->message.size() > 30)
				command= event->message.substr(0,30)+"...";
			else
				command= event->message;
			commandClearTime= simTime+10;
			mstsRoute->eventMap.erase(i);
			return;
		}
	}
	if (!myTrain)
		return;
	WLocation loc;
	myTrain->location.getWLocation(&loc);
	for (MSTSRoute::EventMap::iterator i=mstsRoute->eventMap.begin();
	  i!=mstsRoute->eventMap.end(); i++) {
		Event* event= i->second;
		if (event->time > 0)
			continue;
		double dx= mstsRoute->convX(event->tx,event->x) - loc.coord[0];
		double dy= mstsRoute->convZ(event->tz,event->z) - loc.coord[1];
		if (event->radius*event->radius < dx*dx+dy*dy)
			continue;
		if (event->onStop && myTrain->speed != 0)
			continue;
		fprintf(stderr,"\nevent %d dist %f %f\n%s\n\n",
		  event->id,sqrt(dx*dx+dy*dy),simTime,event->message.c_str());
		if (event->message.size() > 30)
			command= event->message.substr(0,30)+"...";
		else
			command= event->message;
		commandClearTime= simTime+10;
		mstsRoute->eventMap.erase(i);
		return;
	}
}

void updateLightDirection(osg::Light* light)
{
	if (!mstsRoute || !timeTable)
		return;
	static osg::Vec4f** sunDir= nullptr;
	if (sunDir == nullptr) {
		double lat,lng;
		mstsRoute->xy2ll(0,0,&lat,&lng);
		fprintf(stderr,"lat %f\n",lat);
		lat*= M_PI/180;
		float dec= 0; //day of year not known
		sunDir= (osg::Vec4f**) calloc(13,sizeof(osg::Vec4f*));
		for (int i=0; i<13; i++) {
			int hr= i+6;
			float z= sin(lat)*sin(dec) +
			  cos(lat)*cos(dec)*cos(hr*15*M_PI/180);
			float y= (sin(lat)*z - sin(dec)) / cos(lat);
			float x= 1 - z*z - y*y;
			if (x <= 0)
				x= 0;
			else
				x= sqrt(x) * (hr>12?1:-1);
			sunDir[i]= new osg::Vec4f(-x,y,-z,0);
			fprintf(stderr,"sundir %d %d %f %f %f\n",
			  i,hr,x,y,z);
		}
	}
	float h= simTime/3600;
	if (h<6 || h>18) {
//		fprintf(stderr,"lightdir %f\n",h);
		light->setAmbient(osg::Vec4f(.49,.49,.49,1));
		light->setDiffuse(osg::Vec4f(0,0,0,1));
		light->setSpecular(osg::Vec4f(0,0,0,1));
	} else {
		int i= (int)floor(h-6);
		float a= h-6-i;
		osg::Vec4f p= (*sunDir[i+1])*a + (*sunDir[i])*(1-a);
//		fprintf(stderr,"lightdir %f %d %f %f %f %f\n",
//		 h,i,a,p.x(),p.y(),p.z());
		light->setPosition(p);
		if (p.z() < .05) {
			float amb= .49 + 10*p.z();
			float dif= .99*20*p.z();
			light->setAmbient(osg::Vec4f(amb,amb,amb,1));
			light->setDiffuse(osg::Vec4f(dif,dif,dif,1));
			light->setSpecular(osg::Vec4f(dif,dif,dif,1));
		} else {
			light->setAmbient(osg::Vec4f(.99,.99,.99,1));
			light->setDiffuse(osg::Vec4f(.99,.99,.99,1));
			light->setSpecular(osg::Vec4f(.99,.99,.99,1));
		}
	}
}

osg::PositionAttitudeTransform* skyTransform= NULL;

void setupSky(osg::Group* staticModels)
{
	auto skyModel= osgDB::readNodeFile("skydome.osgt");
	if (skyModel) {
		osg::Geode* geode= dynamic_cast<osg::Geode*>(skyModel);
		if (geode) {
			osg::Geometry* geom=
			  dynamic_cast<osg::Geometry*>(geode->getDrawable(0));
			if (geom) {
				osg::StateSet* stateSet=
				  geom->getOrCreateStateSet();
				osg::TexEnv* te= new osg::TexEnv();
				te->setMode(osg::TexEnv::MODULATE);
				stateSet->setTextureAttributeAndModes(0,te,
				  osg::StateAttribute::ON);
				osg::Material* m= new osg::Material;
				m->setAmbient(osg::Material::FRONT_AND_BACK,
				  osg::Vec4(.99,.99,.99,1));
				m->setDiffuse(osg::Material::FRONT_AND_BACK,
				  osg::Vec4(0,0,0,1));
				stateSet->setAttribute(m,
				  osg::StateAttribute::ON);
			}
		}
		skyTransform= new osg::PositionAttitudeTransform;
		skyTransform->addChild(skyModel);
		skyTransform->setScale(osg::Vec3(.147,.105,.105));
		staticModels->addChild(skyTransform);
		osg::ComputeBoundsVisitor bbv;
		skyModel->accept(bbv);
		osg::BoundingBox bb= bbv.getBoundingBox();
		skyTransform->setPivotPoint(bb.center());
	}
}

void updateSky()
{
	if (skyTransform) {
		if (mapViewOn)
			skyTransform->setPosition(currentPerson.getLocation()
			  -osg::Vec3(0,0,4000));
		else
			skyTransform->setPosition(currentPerson.getLocation());
		skyTransform->setAttitude(
		  osg::Quat(simTime/5000,osg::Vec3(0,0,1)));
//		fprintf(stderr,"pos %f %f %f\n",
//		  skyTransform->getPosition().x(),
//		  skyTransform->getPosition().y(),
//		  skyTransform->getPosition().z());
	}
}

int main(int argc, char* argv[])
{
	const char* server= NULL;
	int loopDelay= 0;
	while (argc>2 && argv[1][0]=='-') {
		switch (argv[1][1]) {
		 case 's':
			server= argv[1]+2;
			break;
		 case 'd':
			loopDelay= atoi(argv[1]+2);
			if (loopDelay == 0)
				loopDelay= 1000000/30;
			break;
		 default:
			fprintf(stderr,"unknown option %s\n",argv[1]);
			break;
		}
		argv++;
		argc--;
	}
	MHD_Daemon* webServer= MHD_start_daemon(MHD_USE_THREAD_PER_CONNECTION,
	  8888,NULL,NULL,handleWebRequest,NULL,MHD_OPTION_END);
	RMSSocket mySocket(server);
	listener.init();
	osg::Group* staticModels= new osg::Group;
	if (argc < 2) {
		system("firefox http://127.0.0.1:8888/routes&");
		while (mstsRoute == NULL)
			sleep(1);
		mstsRoute->readTiles();
		mstsRoute->adjustWater(0);
		mstsRoute->createSignals= true;
		mstsRoute->makeTrack(0,0);
		timeTable= new TimeTable();
		timeTable->addRow(timeTable->addStation("start"));
		mstsRoute->loadActivity(staticModels,-1);
	} else {
		const char* fname= argv[1];
		argc--;
		argv++;
		try {
//			fprintf(stderr,"personstack %d\n",Person::stack.size());
			parseFile(fname,staticModels,server!=NULL,argc,argv);
			fprintf(stderr,"person %f %f %f\n",
			  currentPerson.location[0],
			  currentPerson.location[1],currentPerson.location[2]);
		} catch (const char* msg) {
			fprintf(stderr,"caught %s\n",msg);
			exit(1);
		} catch (const std::exception& error) {
			fprintf(stderr,"caught %s\n",error.what());
			exit(1);
		}
	}
	osg::Geode* geode= new osg::Geode;
	osg::StateSet* stateSet= geode->getOrCreateStateSet();
	trackPathDrawable= new TrackPathDrawable();
	trackPathDrawable->setDataVariance(osg::Object::DYNAMIC);
	trackPathDrawable->setUseDisplayList(false);
	geode->addDrawable(trackPathDrawable);
	staticModels->addChild(geode);
	staticModels->addChild(createHUD());
	staticModels->addChild(CabOverlay::create());
	if (mstsRoute != NULL)
		mstsRoute->makeTileMap(staticModels);
	makeTrackGeometry(staticModels);
	makeWaterDrawables(staticModels);
	trackLabels= addTrackLabels();
	staticModels->addChild(trackLabels);
	currentPerson.createModel(staticModels);
	setupSky(staticModels);
	rootNode= new osg::Switch;
	osg::LightSource* lightSource= new osg::LightSource;
	osg::Light* light= lightSource->getLight();
#if 0
	light->setAmbient(osg::Vec4f(.7,.7,.7,1));
	light->setDiffuse(osg::Vec4f(.5,.5,.5,1));
	light->setSpecular(osg::Vec4f(1,1,1,1));
	light->setPosition(osg::Vec4f(.25,-.5,.8,0));
#else
//	light->setAmbient(osg::Vec4f(.5,.5,.5,1));
//	light->setDiffuse(osg::Vec4f(.8,.8,.8,1));
	light->setAmbient(osg::Vec4f(.99,.99,.99,1));
	light->setDiffuse(osg::Vec4f(.99,.99,.99,1));
	light->setSpecular(osg::Vec4f(.99,.99,.99,1));
	//light->setPosition(osg::Vec4f(0,0,1,0));
	//light->setPosition(osg::Vec4f(.5,-1,1,0));
	auto dir= osg::Vec3f(0,0,-1);
	dir.normalize();
	light->setPosition(osg::Vec4f(0,-.6,.8,0));
	light->setDirection(dir);//spot direction
#endif
	staticModels->addChild(lightSource);
	stateSet= rootNode->getOrCreateStateSet();
//	osg::BlendFunc* bf= new osg::BlendFunc();
//	bf->setFunction(osg::BlendFunc::SRC_ALPHA,
//	  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
//	stateSet->setAttributeAndModes(bf,osg::StateAttribute::ON);
	stateSet->setMode(GL_CULL_FACE,osg::StateAttribute::ON);
	rootNode->addChild(staticModels);
	osg::Group* newModels= NULL;
//	if (mstsRoute != NULL)
//		newModels= mstsRoute->loadTiles(currentPerson.getLocation()[0],
//		  currentPerson.getLocation()[1]);
	if (mstsRoute && mstsRoute->createSignals) {
		for (SignalMap::iterator i=signalMap.begin();
		  i!=signalMap.end(); i++)
			i->second->update();
		for (SignalMap::iterator i=signalMap.begin();
		  i!=signalMap.end(); i++)
			i->second->update();
	}
	if (timeTable) {
		double start= ttoSim.init(server!=NULL);
		if (simTime == 0)
			simTime= start;
		timeTable->printHorz(stderr);
	}
	if (newModels != NULL)
		rootNode->addChild(newModels);
	for (TrainList::iterator j=trainList.begin();
	  j!=trainList.end(); ++j)
		listener.addTrain(*j);
	osg::Timer timer;
	osg::Timer_t prevTime= timer.tick();
	osg::ArgumentParser args(&argc,argv);
	osgViewer::Viewer viewer(args);
	rootNode->setAllChildrenOn();
#if 1
	shadowMap= new osgShadow::ShadowMap();
	shadowMap->setLight(lightSource);
	shadowMap->setTextureSize(osg::Vec2s(1024,1024));
	shadowMap->setTextureUnit(2);
	shadowScene= new osgShadow::ShadowedScene();
//	shadowScene->setShadowTechnique(shadowMap);
	shadowScene->setReceivesShadowTraversalMask(0x1);
	shadowScene->setCastsShadowTraversalMask(0x2);
	shadowScene->addChild(rootNode);
	viewer.setSceneData(shadowScene);
#else
	viewer.setSceneData(rootNode);
#endif
	trackEditor= new TrackEditor(rootNode);
	viewer.addEventHandler(trackEditor);
	viewer.addEventHandler(new Controller());
	osgGA::KeySwitchMatrixManipulator* ksm=
	  new osgGA::KeySwitchMatrixManipulator;
	ksm->addNumberedMatrixManipulator(new LookFromManipulator());
	ksm->addNumberedMatrixManipulator(new LookAtManipulator());
	MapManipulator* mm= new MapManipulator();
	mm->viewAllTrack();
	ksm->addNumberedMatrixManipulator(mm);
	ksm->selectMatrixManipulator(2);
	if (currentPerson.location[0]==0 && currentPerson.location[1]==0)
		currentPerson.centerOverTrack();
	viewer.setCameraManipulator(ksm);
	osgViewer::StatsHandler* stats= new osgViewer::StatsHandler();
	stats->setKeyEventPrintsOutStats('Q');
	stats->setKeyEventTogglesOnScreenStats('S');
	fprintf(stderr,"stats %c %c\n",
	  stats->getKeyEventPrintsOutStats(),
	  stats->getKeyEventTogglesOnScreenStats());
	viewer.addEventHandler(stats);
	viewer.realize();
	camera= viewer.getCamera();
	camera->setClearColor(osg::Vec4(.45,.54,.73,1));
	if (mstsRoute) {
		osgDB::DatabasePager* pager= viewer.getDatabasePager();
		pager->setTargetMaximumNumberOfPageLOD(15);
	}
//	fprintf(stderr,"nfratio %lf\n",camera->getNearFarRatio());
//	camera->setNearFarRatio(.00001);
//	double fovy,ar,zn,zf;
//	camera->getProjectionMatrixAsPerspective(fovy,ar,zn,zf);
//	camera->setComputeNearFarMode(
//	  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
//	camera->setProjectionMatrixAsPerspective(fovy,ar,.2,10000);
//	printTree(rootNode,0);
#if 0
	osg::Light* light= viewer.getLight();
	if (light == NULL) {
		light= new osg::Light;
		light->setLightNum(0);
		viewer.setLight(light);
	}
	light->setAmbient(osg::Vec4f(.3,.3,.3,1));
	light->setDiffuse(osg::Vec4f(.7,.7,.7,1));
	light->setSpecular(osg::Vec4f(1,1,1,1));
	light->setPosition(osg::Vec4f(0,0,1,0));
	//light->setPosition(osg::Vec4f(0,0,1e10,1));
	//light->setDirection(osg::Vec3f(0,0,-1));
#endif
	while (!viewer.done()) {
		osg::Timer_t now= timer.tick();
		double dt= timer.delta_s(prevTime,now)*timeMult;
		prevTime= now;
		simTime+= dt;
		if (simTime > endTime)
			 timeMult= 0;
		if (dt>0) {
			listener.setGain(1);
			fps= .9*fps + .1*timeMult/dt;
			if (autoSwitcher != NULL)
				autoSwitcher->update();
			adjustShipMass();
			moveShips(dt);
			moveFloatBridges();
			updateTrains(dt);
			if (deferredThrow!=NULL && deferredThrow->occupied==0 &&
			  (myTrain==NULL || myTrain->nextStopDist==0)) {
				deferredThrow->throwSwitch(NULL,false);
				deferredThrow= NULL;
				if (myTrain && myTrain->targetSpeed>0) {
					float d=
					  .5*myTrain->speed*myTrain->speed/
					 (myTrain->decelMult*myTrain->maxDecel);
					myTrain->nextStopDist=
					  myTrain->speed<0 ? -d : d;
					myTrain->targetSpeed= 0;
				}
			}
			updateActivityEvents();
			updateLightDirection(light);
			ttoSim.processEvents(simTime);
			Person::updateLocations(dt);
			updateSky();
			if (timeWarp && trainList.size()==0) {
				simTime= ttoSim.getNextEventTime();
			} else if (timeWarp) {
				float d= ttoSim.movingTrainDist2(
				  currentPerson.getLocation());
				if (timeWarp==2 && d>500*500)
					timeWarp= 1;
				if (interlocking->countUnclearedSignals()>
				   timeWarpSigCount || commandMode ||
				  getAIMsgCount()>timeWarpMsgCount ||
				  (d<500*500 &&
				   (timeWarp==1 || d<timeWarpDist))) {
					timeMult= 1;
					timeWarp= 0;
				}
				timeWarpDist= d;
			}
			if (myTrain!=NULL && trackPathDrawable->draw)
				trackPathDrawable->dirtyBound();
			osg::Group* newModels= NULL;
			if (newModels != NULL) {
				rootNode= new osg::Switch;
				osg::StateSet* stateSet=
				  rootNode->getOrCreateStateSet();
				//stateSet->setAttributeAndModes(bf,
				//  osg::StateAttribute::ON);
				stateSet->setMode(GL_CULL_FACE,
				  osg::StateAttribute::ON);
				rootNode->addChild(staticModels);
				rootNode->addChild(newModels);
				rootNode->setAllChildrenOn();
				viewer.setSceneData(rootNode);
				viewer.assignSceneDataToCameras();
				prevTime= timer.tick();
			}
		} else if (timeMult == 0) {
			// turn off sound when paused
			listener.setGain(0);
			sleep(1);
		}
		// this delay added as a workaround for an overheating problem
		// needs to be removed when fan works
		if (loopDelay > 0)
			usleep(loopDelay);
		listener.update(currentPerson.getLocation(),currentPerson.cosAngle,
		  currentPerson.sinAngle);
		mySocket.update(timer.time_s());
		rtt= mySocket.getRTT();
		viewer.frame(simTime);
	}
	if (timeTable!=NULL && server==NULL)
		timeTable->printTimeSheetHorz2(stderr);
	if (server == NULL)
		ChangeLog::instance()->print();
	if (webServer)
		MHD_stop_daemon(webServer);
	return 0;
}
