//	code to maintain person location and orientation
//	used to control camera
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
#include <osgText/Text>
#include <osg/Drawable>
#include <osg/BlendFunc>
#include <osgSim/LightPointNode>

Person currentPerson;

void Person::setMoveTo(osg::Vec3d loc, osg::MatrixTransform* mt,
  osg::Vec3f offset, RailCarInst* railcar, Train* train, Ship* ship)
{
	if (moveTo != NULL) {
		delete moveTo;
		moveTo= NULL;
	}
	this->railCar= railcar;
	if (train || follow==NULL)
		this->train= train;
	this->ship= ship;
	if ((location-loc).length2()<1e6 &&
	  (true || follow==NULL || mt==NULL)) {
		moveTo= new Person;
		moveTo->location= loc;
		moveTo->height= 1.7;
		moveTo->location[2]+= moveTo->height;
		moveTo->follow= mt;
		moveTo->followOffset= offset;
		if (moveTo->follow != NULL) {
//			moveTo->followOffset[2]= .5;
//			if (moveTo->followOffset[1] > 0)
//				moveTo->followOffset[1]= 1.6;
//			else
//				moveTo->followOffset[1]= -1.6;
			if (railcar) {
				moveTo->followOffset[2]= .5;
				fprintf(stderr,"railcar %f %f\n",
				  railcar->def->length,railcar->def->width);
				if (moveTo->followOffset[1] > 0)
					moveTo->followOffset[1]=
					  .5*railcar->def->width + .3;
				else
					moveTo->followOffset[1]=
					  -.5*railcar->def->width - .3;
				if (moveTo->followOffset[0] > 0)
					moveTo->followOffset[0]=
					  .5*railcar->def->length - .7;
				else
					moveTo->followOffset[0]=
					  -.5*railcar->def->length + .7;
			}
		}
		fprintf(stderr,"moveto %f %f %f %f %f %f %p\n",
		  moveTo->location[0],moveTo->location[1],moveTo->location[2],
		  moveTo->followOffset[0],moveTo->followOffset[1],
		  moveTo->followOffset[2],moveTo->follow);
		//stopFollowing(false);
		follow= NULL;
		setAimOnMove= true;
		prevDist= 1e10;
	} else {
		setLocation(loc,mt,offset);
	}
}

void Person::stopMove()
{
	if (moveTo)
		delete moveTo;
	moveTo= NULL;
}

void Person::jump()
{
	if (moveTo) {
//		moveTo->location[2]-= moveTo->height;
//		setLocation(moveTo->location,moveTo->follow,
//		  moveTo->followOffset);
		if (follow) {
			osg::Quat q= follow->getMatrix().getRotate();
			osg::Vec3f axis;
			double fAngle;
			q.getRotate(fAngle,axis);
			double dot= axis*osg::Vec3(0,0,1);
			if (dot < 0)
				fAngle= -fAngle;
			angle+= fAngle/3.14159*180;
			fprintf(stderr,"preadj %f %f %f %f %f\n",
			  fAngle/3.14159*180,
			  axis[0],axis[1],axis[2],dot);
			setAngle(angle);
		}
		location= moveTo->location;
		height= moveTo->height;
		follow= moveTo->follow;
		followOffset= moveTo->followOffset;
		insideIndex= -1;
		delete moveTo;
		moveTo= NULL;
		if (follow) {
			osg::Quat q= follow->getMatrix().getRotate();
			osg::Vec3f axis;
			double fAngle;
			q.getRotate(fAngle,axis);
			double dot= axis*osg::Vec3(0,0,1);
			if (dot < 0)
				fAngle= -fAngle;
			angle-= fAngle/3.14159*180;
			fprintf(stderr,"postadj %f %f %f %f %f\n",
			  fAngle/3.14159*180,
			  axis[0],axis[1],axis[2],dot);
//			fprintf(stderr,"postangle=%f\n",angle);
			setAngle(angle);
		}
	}
}

void Person::stopFollowing(bool moveDown)
{
	if (follow == NULL)
		return;
	if (train && abs(train->speed)>.5*speed)
		return;
	location[2]-= 1;
	osg::Quat q= follow->getMatrix().getRotate();
	osg::Vec3f axis;
	double fAngle;
	q.getRotate(fAngle,axis);
	double dot= axis*osg::Vec3(0,0,1);
	if (dot < 0)
		fAngle= -fAngle;
	angle+= fAngle/3.14159*180;
	fprintf(stderr,"preadj %f %f %f %f %f\n",fAngle/3.14159*180,
	  axis[0],axis[1],axis[2],dot);
	setAngle(angle);
	follow= NULL;
	train= NULL;
	ship= NULL;
	if (moveDown) {
		if (moveTo != NULL)
			delete moveTo;
		moveTo= new Person;
		moveTo->location= location;
		moveTo->height= height;
		location[2]+= 1;
	}
}

//	sets the person location
//	person tracks a moving object if mt is defined
//	orientation is preserved when jumping between static and moving objects
//	(although there might be a bug in this)
void Person::setLocation(osg::Vec3d loc, osg::MatrixTransform* mt,
  osg::Vec3f offset)
{
//	fprintf(stderr,"preangle=%f\n",angle);
	if (moveTo != NULL) {
		delete moveTo;
		moveTo= NULL;
	}
	if (follow) {
		osg::Quat q= follow->getMatrix().getRotate();
		osg::Vec3f axis;
		double fAngle;
		q.getRotate(fAngle,axis);
		double dot= axis*osg::Vec3(0,0,1);
		if (dot < 0)
			fAngle= -fAngle;
		angle+= fAngle/3.14159*180;
		fprintf(stderr,"preadj %f %f %f %f %f\n",fAngle/3.14159*180,
		  axis[0],axis[1],axis[2],dot);
		setAngle(angle);
	}
	if ((location-loc).length2() < 1e6) {
		location= loc;
		height= 1.7;
		location[2]+= height;
		follow= mt;
		followOffset= offset;
		insideIndex= -1;
		if (follow) {
			osg::Quat q= follow->getMatrix().getRotate();
			osg::Vec3f axis;
			double fAngle;
			q.getRotate(fAngle,axis);
			double dot= axis*osg::Vec3(0,0,1);
			if (dot < 0)
				fAngle= -fAngle;
			angle-= fAngle/3.14159*180;
			fprintf(stderr,"postadj %f %f %f %f %f\n",
			  -fAngle/3.14159*180,axis[0],axis[1],axis[2],dot);
			setAngle(angle);
		}
//		fprintf(stderr,"postangle=%f\n",angle);
		setModelMatrix();
	} else {
		setLocation(loc);
	}
	sumDT= 0;
}

//	sets the person location
//	moves above track if the jump is long
void Person::setLocation(osg::Vec3d loc)
{
	follow= NULL;
	sumDT= 0;
	if ((location-loc).length2() < 1e6) {
		location= loc;
		height= 1.7;
		location[2]+= height;
	} else {
		double bestd= 1e40;
		for (TrackMap::iterator j=trackMap.begin(); j!=trackMap.end();
		  ++j) {
			for (Track::VertexList::iterator
			  i=j->second->vertexList.begin();
			  i!=j->second->vertexList.end(); ++i) {
				Track::Vertex* v= *i;
				double d= (v->location.coord-loc).length2();
				if (bestd > d) {
					bestd= d;
					location= v->location.coord;
				}
			}
		}
		location[2]+= 3;
		height= 3;
	}
	setModelMatrix();
}

//	moves the rail fan inside the car being ridden
void Person::moveInside()
{
	if (follow == NULL)
		return;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* train= *i;
		for (RailCarInst* car=train->firstCar; car!=NULL;
		  car=car->next) {
			for (int j=0; j<car->def->parts.size(); j++) {
				if (car->models[j] == follow) {
					if (car->def->inside.size()==0)
						return;
					if (insideIndex<0)
						insideIndex= 0;
					else
						insideIndex++;
					if (insideIndex >=
					  car->def->inside.size())
						insideIndex= 0;
					RailCarInside& inside=
					  car->def->inside[insideIndex];
					followOffset= inside.position -
					  osg::Vec3f(0,0,
					   car->def->parts[j].zoffset);
					setAngle(inside.angle);
					setVAngle(inside.vAngle);
					insideImage= inside.image;
					setHeight(0);
					sumDT= 0;
				}
			}
		}
	}
}

//	updates the person location if following a moving object
void Person::updateLocation(double dt)
{
	aim= osg::Vec3d(cosVAngle*cosAngle,cosVAngle*sinAngle,sinVAngle);
	if (moveTo && follow)
		stopFollowing(false);
	if (moveTo && follow==NULL) {
		moveTo->updateLocation(dt);
		sumDT+= dt;
		osg::Vec3f dloc= moveTo->location-location;
		float dist= dloc.length();
		if (dist < sumDT*speed) {
			moveTo->location[2]-= moveTo->height;
			setLocation(moveTo->location,moveTo->follow,
			  moveTo->followOffset);
			delete moveTo;
			moveTo= NULL;
		} else if (follow == NULL) {
			dloc*= sumDT*speed/dist;
			location += dloc;
		} else {
//			fprintf(stderr,"dloc %f %f %f\n",
//			  dloc[0],dloc[1],dloc[2]);
			dloc*= sumDT*speed/dist;
			dloc[2]= -dloc[2];
			osg::Quat q= follow->getMatrix().getRotate();
			followOffset-= q*dloc;
//			fprintf(stderr,"dist %f\n",dist);
//			fprintf(stderr,"offset %f %f %f\n",
//			  followOffset[0],followOffset[1],followOffset[2]);
		}
		if (setAimOnMove) {
			dloc.normalize();
			aim= dloc;
			setAimOnMove= false;
			setAngle(atan2(aim[1],aim[0])*180/3.14159);
			setVAngle(0);
		}
		sumDT= 0;
	}
	if (follow != NULL) {
		location= follow->getMatrix().getTrans();
		location[2]+= height;
		osg::Quat q= follow->getMatrix().getRotate();
		location+= q*followOffset;
		aim= q*aim;
	}
	setModelMatrix();
//	fprintf(stderr,"aim %f %f %f\n",aim[0],aim[1],aim[2]);
}

void Person::move(float df, float ds)
{
	osg::Vec3d d(df*cosAngle-ds*sinAngle,ds*cosAngle+df*sinAngle,0);
	location+= d;
	followOffset+= d;
	sumDT= 0;
	insideImage= NULL;
}

//	move the 3D model that represents the person in the world
void Person::setModelMatrix()
{
	if (!model)
		return;
	osg::Vec3f u= osg::Vec3f(0,0,1);
	osg::Vec3f s= aim^u;
	model->setMatrix(osg::Matrixd(
	   aim[0],aim[1],aim[2],0,
	   s[0],s[1],s[2],0,
	   u[0],u[1],u[2],0,
	   location[0],location[1],location[2],1));
}

void Person::createModel(osg::Group* root)
{
	model= new osg::MatrixTransform;
	modelSwitch= new osg::Switch;
#if 0
#if 0
	osg::MatrixTransform* m= new osg::MatrixTransform;
	m->setMatrix(osg::Matrixd(1,0,0,0, 0,0,1,0, 0,1,0,0, 0,0,0,1));
	m->addChild(osgDB::readNodeFile("canbitt.ac"));
	model->addChild(m);
#else
	model->addChild(osgDB::readNodeFile("camera.ac"));
#endif
#else
	osgSim::LightPointNode* lpn= new osgSim::LightPointNode;
	osgSim::LightPoint lp;
	lp._on= true;
	lp._position= osg::Vec3d(0,0,-.7);
	lp._color= osg::Vec4d(1,1,1,1);
	lp._radius= .1;
	lpn->addLightPoint(lp);
	model->addChild(lpn);
#endif
	modelSwitch->addChild(model);
	root->addChild(modelSwitch);
	modelSwitch->setAllChildrenOff();
}

inline double triArea(osg::Vec3d& p1, osg::Vec3d& p2, osg::Vec3d& p3)
{
	return triArea(p1[0],p1[1],p2[0],p2[1],p3[0],p3[1]);
}

void Person::moveAlongTrain(Train* train, bool toRight)
{
	if (train==NULL)
		return;
	if (follow != NULL) {
		stopFollowing(true);
		return;
	}
	osg::Vec3d loc= location;
	if (moveTo)
		loc= moveTo->location;
	osg::Vec3d pc,pf,pr;
	train->findCouplers(loc,pf,pc,pr);
	float dc= (loc-pc).length();
	osg::Vec3d moveto;
	if (pf == pc) {
		osg::Vec3d n= osg::Vec3d(0,0,1)^(pr-pc);
		n.normalize();
		osg::Vec3d fwd= pc-pr;
		fwd.normalize();
		double a= triArea(pc,pr,loc);
		fprintf(stderr,"pf==pc %f %f %d\n",dc,a,toRight);
		if (dc>4 && a>0) 
			moveto= pc+n*2.5;
		else if (dc>4)
			moveto= pc-n*2.5;
		else if (toRight && a>0)
			moveto= pc-n*2.5+fwd;
		else if (toRight)
			moveto= pr-n*2.5;
		else if (a>0)
			moveto= pr+n*2.5;
		else
			moveto= pc+n*2.5+fwd;
	} else if (pr == pc) {
		osg::Vec3d n= osg::Vec3d(0,0,1)^(pf-pc);
		n.normalize();
		osg::Vec3d fwd= pf-pc;
		fwd.normalize();
		double a= triArea(pc,pf,loc);
		fprintf(stderr,"pr==pc %f %f %d\n",dc,a,toRight);
		if (dc>4 && a>0) 
			moveto= pc+n*2.5;
		else if (dc>4)
			moveto= pc-n*2.5;
		else if (toRight && a>0)
			moveto= pc-n*2.5-fwd;
		else if (toRight)
			moveto= pf-n*2.5;
		else if (a>0)
			moveto= pf+n*2.5;
		else
			moveto= pc+n*2.5-fwd;
	} else {
		osg::Vec3d n= (pf-pr)^osg::Vec3d(0,0,1);
		n.normalize();
		double a= triArea(pf,pc,pr);
		fprintf(stderr,"3p %f %f\n",dc,a);
		if (a >= 0) {
			double a1= triArea(pc,loc,pf);
			double a2= triArea(loc,pc,pr);
			a= a1>0 && a2>0 ? 1 : -1;
		} else {
			double a1= triArea(pc,loc,pr);
			double a2= triArea(loc,pc,pf);
			a= (!(a1>=0 && a2>=0)) ? 1 : -1;
		}
		fprintf(stderr,"3p %f %f %d\n",dc,a,toRight);
		if (dc>4 && a>0) 
			moveto= pc+n*2.5;
		else if (dc>4)
			moveto= pc-n*2.5;
		else if (toRight && a>0)
			moveto= pf+n*2.5;
		else if (toRight)
			moveto= pr-n*2.5;
		else if (a>0)
			moveto= pr+n*2.5;
		else
			moveto= pf-n*2.5;
	}
	setMoveTo(moveto,NULL,osg::Vec3f(0,0,0),NULL,NULL,NULL);
}

void Person::moveAlongPath(bool toRight)
{
	if (ship) {
		moveToNextCleat(toRight);
		return;
	}
	if (follow!=NULL || path.size()<=0)
		return;
	int besti= -1;
	double bestd= 1e10;
	for (int i=0; i<path.size(); i++) {
		double d= (path[i]-location).length2();
		if (d < bestd) {
			besti= i;
			bestd= d;
		}
	}
	fprintf(stderr,"%d %f\n",besti,bestd);
	if (toRight && besti<path.size()-1)
		setMoveTo(path[besti+1],NULL,osg::Vec3f(0,0,0),NULL,NULL,NULL);
	else if (!toRight && besti>0)
		setMoveTo(path[besti-1],NULL,osg::Vec3f(0,0,0),NULL,NULL,NULL);
	else if (besti>=0 && bestd>10)
		setMoveTo(path[besti],NULL,osg::Vec3f(0,0,0),NULL,NULL,NULL);
}

Cleat* Person::findCleat()
{
	if (follow==NULL || ship==NULL)
		return NULL;
	return ship->findCleat(followOffset[0],followOffset[1],100);
}

void Person::moveToNextCleat(bool toRight)
{
	if (moveTo) {
		jump();
		return;
	}
	fprintf(stderr,"movetonextcleat %d %p %p\n",toRight,follow,ship);
	Cleat* cleat= findCleat();
	if (cleat == NULL)
		return;
	if (toRight) {
		if (cleat->next)
			cleat= cleat->next;
		else
			cleat= ship->cleats;
	} else {
		if (cleat == ship->cleats) {
			while (cleat->next)
				cleat= cleat->next;
		} else {
			Cleat* c=ship->cleats;
			for (; c!=NULL && c->next!=cleat; c=c->next)
				;
			cleat= c;
		}
	}
	fprintf(stderr," %p %f %f %f\n",cleat,cleat->x,cleat->y,cleat->z);
	setMoveTo(osg::Vec3d(location[0],location[1],location[2]-1.7),
	  follow,osg::Vec3f(cleat->x,cleat->y,cleat->standingZ),NULL,NULL,ship);
}

void Person::adjustRopes(int adj)
{
	fprintf(stderr,"adjustropes %d %p %p\n",adj,follow,ship);
	Cleat* cleat= findCleat();
	if (cleat == NULL)
		return;
	for (RopeList::iterator j=ropeList.begin(); j!=ropeList.end(); ++j) {
		Rope* r= *j;
		if (r->cleat1==cleat || r->cleat2==cleat) {
			if (adj<-1 && r->adjust<-1)
				r->adjust--;
			else
				r->adjust= adj;
		}
	}
}

void Person::removeRopes()
{
	fprintf(stderr,"removeropes %p %p\n",follow,ship);
	Cleat* cleat= findCleat();
	if (cleat == NULL)
		return;
	RopeList del;
	for (RopeList::iterator j=ropeList.begin(); j!=ropeList.end(); ++j) {
		Rope* r= *j;
		if (r->cleat1==cleat || r->cleat2==cleat) {
			del.push_back(r);
		}
	}
	for (RopeList::iterator j=del.begin(); j!=del.end(); ++j)
		ropeList.remove(*j);
	if (ship->mass == 0) {
		for (FBList::iterator j=fbList.begin(); j!=fbList.end(); ++j) {
			FloatBridge* fb= *j;
			if (fb == ship)
				fb->disconnect();
		}
	}
}

void Person::connectFloatBridge()
{
	if (ship==NULL || ship->mass>0)
		return;
	for (FBList::iterator j=fbList.begin(); j!=fbList.end(); ++j) {
		FloatBridge* fb= *j;
		if (fb == ship)
			fb->connect();
	}
}

void Person::centerOverTrack()
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
	fprintf(stderr,"minmax %f %f %f %f %f %f\n",
	  minx,maxx,miny,maxy,minz,maxz);
	location[0]= .5*(minx+maxx);
	location[1]= .5*(miny+maxy);
	location[2]= maxz+1.7;
	sumDT= 0;
}

bool Person::getOnOrOff()
{
	return moveTo!=NULL && moveTo->follow!=follow;
}

float Person::getOnOrOffDist()
{
	osg::Vec3f dloc= moveTo->location-location;
	float dist= dloc.length();
	if (dist > prevDist) {
		prevDist= dist;
		return -dist;
	}
	prevDist= dist;
	return dist;
}

void Person::setFollow(int trainID, int carNum, int partNum,
  float offsetX, float offsetY, float offsetZ)
{
	Train* t= Train::findTrain(trainID);
	if (!t)
		return;
	for (RailCarInst* car=t->firstCar; car!=NULL; car=car->next) {
		if (carNum == 0) {
			train= t;
			railCar= car;
			follow= car->models[partNum];
			followOffset= osg::Vec3f(offsetX,offsetY,offsetZ);
			return;
		}
		carNum--;
	}
}

void Person::setRemoteLocation()
{
	fprintf(stderr,"setremote %p\n",train);
	if (train==NULL)
		return;
	useRemote= false;
	Track::Location loc;
	if (train->speed>=0) {
		loc= train->location;
		loc.move(100,false,0);
	} else {
		loc= train->endLocation;
		loc.move(-100,false,0);
	}
	WLocation wloc1,wloc2;
	loc.getWLocation(&wloc1);
	fprintf(stderr,"wloc1 %f %f %f\n",
	  wloc1.coord[0],wloc1.coord[1],wloc1.coord[2]);
	loc.move(1,false,0);
	loc.getWLocation(&wloc2);
	fprintf(stderr,"wloc2 %f %f %f\n",
	  wloc2.coord[0],wloc2.coord[1],wloc2.coord[2]);
	float offset= drand48()>.5 ? 5 : -5;
	osg::Vec3d side= osg::Vec3d(0,0,1)^(wloc2.coord-wloc1.coord)*offset;
	fprintf(stderr,"side %f %f %f\n",side[0],side[1],side[2]);
	remoteLocation= wloc1.coord + side;
	remoteLocation[2]+= 1.7;
	useRemote= true;
	fprintf(stderr,"remote %f %f %f\n",
	  remoteLocation[0],remoteLocation[1],remoteLocation[2]);
	fprintf(stderr,"loc    %f %f %f\n",
	  location[0],location[1],location[2]);
}

vector<Person> Person::stack;
int Person::stackIndex= 0;

extern Train* selectedTrain;
extern Train* myTrain;
extern RailCarInst* myRailCar;
extern RailCarInst* selectedRailCar;
extern Ship* selectedShip;

void Person::swap(int index)
{
	fprintf(stderr,"swap from %d to %d\n",stackIndex,index);
	Person* t= currentPerson.moveTo;
	currentPerson.moveTo= NULL;
	while (stack.size() <= index) {
		int i= stack.size();
		stack.push_back(currentPerson);
		osg::Group* parent= currentPerson.modelSwitch->getParent(0);
		stack[i].createModel(parent);
	}
	currentPerson.moveTo= t;
	stack[stackIndex]= currentPerson;
	currentPerson= stack[index];
	stackIndex= index;
	selectedTrain= currentPerson.train;
	selectedRailCar= currentPerson.railCar;
	selectedShip= currentPerson.ship;
	if (myTrain == selectedTrain)
		myRailCar= selectedRailCar;
	else
		myRailCar= NULL;
}

void Person::updateLocations(float dt)
{
	currentPerson.updateLocation(dt);
	for (int i=0; i<stack.size(); i++)
		if (i != stackIndex)
			stack[i].updateLocation(dt);
}
