//	OSG animation callbacks
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
#include <osg/AnimationPath>
#include <osg/MatrixTransform>

TwoStateAnimPathCB::TwoStateAnimPathCB(Track::SwVertex* sw,
  osg::AnimationPath* ap, int i) : osg::AnimationPathCallback(ap,0,1)
{
	swVertex= sw;
	prevState= -1;
	newState= 0;
	car= NULL;
	index= i;
	osg::AnimationPath::TimeControlPointMap& tcpm=
	  ap->getTimeControlPointMap();
	if (tcpm.size() > 2) {
		double t0= ap->getFirstTime();
		double t1= ap->getLastTime();
		osg::AnimationPath::ControlPoint& cp0= tcpm.begin()->second;
		osg::AnimationPath::ControlPoint& cp1= tcpm.rbegin()->second;
		if (cp0.getPosition() == cp1.getPosition() &&
		  cp0.getRotation() == cp1.getRotation()) {
			tcpm.erase(t1);
		}
	}
}

//	callback for implementing two state animations
void TwoStateAnimPathCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR &&
	  getAnimationPath() && nv->getFrameStamp()) {
		if (swVertex)
			newState= swVertex->edge2!=
			  swVertex->swEdges[swVertex->mainEdge];
		else if (car)
			newState= car->getAnimState(index);
		else if (index >= 0)
			newState= index;
		else
			newState= 1;
		double t= nv->getFrameStamp()->getSimulationTime();
		_latestTime= t;
		if (newState != prevState) {
			prevState= newState;
			_firstTime= t;
			if (newState) {
				_timeOffset= 0;
				_timeMultiplier= 1;
			} else {
				_timeOffset= 1;
				_timeMultiplier= -1;
			}
			//fprintf(stderr,"newState %d %f %p %p %f\n",
			//  newState,t,swVertex,this,getAnimationTime());
		}
		update(*node);
	}
	NodeCallback::traverse(node,nv);
}

//	callback for implementing switch point animations
void SetSwVertexVisitor::apply(osg::MatrixTransform& mt)
{
	TwoStateAnimPathCB* tsapc= dynamic_cast<TwoStateAnimPathCB*>
	  (mt.getUpdateCallback());
	if (tsapc!=NULL && tsapc->swVertex) {
		TwoStateAnimPathCB* apc1=
		  new TwoStateAnimPathCB(swVertex,tsapc->getAnimationPath());
		mt.setUpdateCallback(apc1);
//		fprintf(stderr,"newSwV %p %p %p %p %p\n",swVertex,apc1,
//		  swVertex->edge2,swVertex->swEdges[0],swVertex->swEdges[1]);
	} else if (tsapc != NULL) {
		tsapc->swVertex= swVertex;
//		fprintf(stderr,"setSwV %p %p %p %p %p\n",swVertex,tsapc,
//		  swVertex->edge2,swVertex->swEdges[0],swVertex->swEdges[1]);
		osg::AnimationPath* ap= tsapc->getAnimationPath();
		ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
	} else {
		osg::AnimationPathCallback* apc=
		  dynamic_cast<osg::AnimationPathCallback*>
		  (mt.getUpdateCallback());
		if (apc != NULL) {
			osg::AnimationPath* ap= apc->getAnimationPath();
			tsapc= new TwoStateAnimPathCB(swVertex,ap);
			ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
			tsapc->setPivotPoint(apc->getPivotPoint());
			mt.setUpdateCallback(tsapc);
		}
	}
	traverse(mt);
}

//	Callback for rod animations
RodAnimPathCB::RodAnimPathCB(RailCarInst* c, osg::AnimationPath* ap, int n) :
  osg::AnimationPathCallback(ap,0,1)
{
	car= c;
	nFrames= n;
	if (ap->getLastTime() < n) {
		osg::AnimationPath::ControlPoint cp;
		ap->getInterpolatedControlPoint(0,cp);
		ap->insert(n,cp);
	}
}

//	Callback for rod animations
void RodAnimPathCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR &&
	  getAnimationPath() && nv->getFrameStamp()) {
		_firstTime= 0;
		_timeOffset= 0;
		_timeMultiplier= 1;
		if (car)
			_latestTime= nFrames*(1-car->getMainWheelState());
//		fprintf(stderr,"%f %f %f %f\n",_firstTime,_latestTime,
//		  _timeOffset,_timeMultiplier);
		update(*node);
	}
	NodeCallback::traverse(node,nv);
}

//	Callback for coupler animations
CouplerAnimPathCB::CouplerAnimPathCB(RailCarInst* c, osg::AnimationPath* ap,
  bool f) : osg::AnimationPathCallback(ap,0,1)
{
	car= c;
	front= f;
}

//	Callback for rod animations
void CouplerAnimPathCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR &&
	  getAnimationPath() && nv->getFrameStamp()) {
		_firstTime= 0;
		_timeOffset= 0;
		_timeMultiplier= 1;
		if (car)
			_latestTime= car->getCouplerState(front);
		update(*node);
	}
	NodeCallback::traverse(node,nv);
}

//	node visitor for associating rod animation with rail cars
void SetCarVisitor::apply(osg::MatrixTransform& mt)
{
	RodAnimPathCB* apc= dynamic_cast<RodAnimPathCB*>
	  (mt.getUpdateCallback());
	if (apc!=NULL && apc->car) {
		RodAnimPathCB* apc1=
		  new RodAnimPathCB(car,apc->getAnimationPath(),apc->nFrames);
		mt.setUpdateCallback(apc1);
		nRods++;
	} else if (apc != NULL) {
		apc->car= car;
		//osg::AnimationPath* ap= apc->getAnimationPath();
		//ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
		nRods++;
	} else {
		CouplerAnimPathCB* capc= dynamic_cast<CouplerAnimPathCB*>
		  (mt.getUpdateCallback());
		if (capc) {
			capc->car= car;
			osg::AnimationPath* ap= capc->getAnimationPath();
			ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
		} else {
			TwoStateAnimPathCB* tsapc=
			  dynamic_cast<TwoStateAnimPathCB*>
			  (mt.getUpdateCallback());
			if (tsapc && tsapc->car==NULL) {
				tsapc->car= car;
				osg::AnimationPath* ap=
				  tsapc->getAnimationPath();
				ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
			}
		}
	}
	traverse(mt);
}

//	callback for implementing interlocking lever animations
void InterlockingAnimPathCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR &&
	  getAnimationPath() && nv->getFrameStamp()) {
		newState= interlocking->getState(lever);
		double t= nv->getFrameStamp()->getReferenceTime();
		_latestTime= t;
		if (newState != prevState) {
			prevState= newState;
			_firstTime= t;
			if (newState ==
			  (Interlocking::NORMAL|Interlocking::REVERSE)) {
				_timeOffset= .5;
				_timeMultiplier= -1;
			} else if (newState == Interlocking::NORMAL) {
				_timeOffset= 1;
				_timeMultiplier= -1;
			} else {
				_timeOffset= 0;
				_timeMultiplier= 1;
			}
			//fprintf(stderr,"newState %d %f %p %p %f\n",
			//  newState,t,swVertex,this,getAnimationTime());
		}
		update(*node);
	}
	NodeCallback::traverse(node,nv);
}

//	callback for implementing switch point animations
void SetInterlockingVisitor::apply(osg::MatrixTransform& mt)
{
	osg::AnimationPathCallback* apc=
	  dynamic_cast<osg::AnimationPathCallback*> (mt.getUpdateCallback());
	int lever= -1;
	if (mt.getName().substr(0,5) == "lever")
		lever= atoi(mt.getName().c_str()+5) - 1;
	if (apc!=NULL && lever>=0 & lever<interlocking->getNumLevers()) {
		osg::AnimationPath* ap= apc->getAnimationPath();
		ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
		InterlockingAnimPathCB* apc1=
		  new InterlockingAnimPathCB(interlocking,lever,ap);
		apc1->interlocking= interlocking;
		apc1->lever= lever;
		apc1->setPivotPoint(apc->getPivotPoint());
		mt.setUpdateCallback(apc1);
	}
	traverse(mt);
}

//	Callback for signal animations
SignalAnimPathCB::SignalAnimPathCB(MSTSSignal* sig, osg::AnimationPath* ap,
  int u) : osg::AnimationPathCallback(ap,0,1)
{
	if (sig)
		signal= sig->units[u];
	else
		signal= NULL;
	unit= u;
	prevState= -1;
	newState= 0;
}

//	Callback for signal animations
void SignalAnimPathCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR &&
	  getAnimationPath() && nv->getFrameStamp() && signal) {
		_firstTime= 0;
		_timeOffset= 0;
		_timeMultiplier= 1;
		if (signal->getColor(unit) == Signal::GREEN)
			_latestTime= 1;
		else if (signal->getColor(unit) == Signal::YELLOW)
			_latestTime= 2;
		else
			_latestTime= 0;
		update(*node);
	}
	NodeCallback::traverse(node,nv);
}

//	Callback for signal animations
SignalLightUCB::SignalLightUCB(MSTSSignal* sig)
{
	if (sig) {
		for (int i=0; i<sig->units.size(); i++)
			signals.push_back(sig->units[i]);
	}
	prevState= -1;
	newState= 0;
}

void SignalLightUCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	osgSim::LightPointNode* lpn= 
	  dynamic_cast<osgSim::LightPointNode*>(node);
	if (nv->getVisitorType()==osg::NodeVisitor::UPDATE_VISITOR && lpn &&
	  signals.size()>0) {
//		fprintf(stderr,"siglightucb %d %p\n",signals.size(),signals[0]);
		newState= signals[0]->getIndication();
//		if (newState != prevState)
//			fprintf(stderr,"lightstate %d %d\n",newState,prevState);
		for (int i=0; newState!=prevState && i<signals.size() &&
		  i<lpn->getNumLightPoints(); i++) {
			Signal* sig= signals[i];
			osgSim::LightPoint& lp= lpn->getLightPoint(i);
//			fprintf(stderr," %d %p %d\n",
//			  i,sig,sig?sig->getColor(i):-1);
			if (sig && sig->getColor(i)==Signal::GREEN)
				lp._color= osg::Vec4d(0,1,0,1);
			else if (sig && sig->getColor(i)==Signal::YELLOW)
				lp._color= osg::Vec4d(1,1,0,1);
			else
				lp._color= osg::Vec4d(1,0,0,1);
		}
		prevState= newState;
	}
	NodeCallback::traverse(node,nv);
}

void SetSignalVisitor::apply(osg::MatrixTransform& mt)
{
	SignalAnimPathCB* sapc= dynamic_cast<SignalAnimPathCB*>
	  (mt.getUpdateCallback());
	if (sapc!=NULL && sapc->signal) {
		SignalAnimPathCB* apc1=
		  new SignalAnimPathCB(signal,sapc->getAnimationPath(),
		   sapc->unit);
		mt.setUpdateCallback(apc1);
//		fprintf(stderr,"setsignal1 anim %p %d %d\n",signal,sapc->unit,
//		  signal->units[sapc->unit]->getState());
	} else if (sapc != NULL) {
		sapc->signal= signal->units[sapc->unit];
		osg::AnimationPath* ap= sapc->getAnimationPath();
		ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
//		fprintf(stderr,"setsignal0 anim %p %d %d\n",signal,sapc->unit,
//		  signal->units[sapc->unit]->getState());
	}
	traverse(mt);
}

void SetSignalVisitor::apply(osg::Node& node)
{
	osgSim::LightPointNode* lpn=
	  dynamic_cast<osgSim::LightPointNode*>(&node);
	if (lpn != NULL) {
		SignalLightUCB* slucb= dynamic_cast<SignalLightUCB*>
		  (node.getUpdateCallback());
		if (slucb!=NULL && slucb->signals.size()>0) {
			SignalLightUCB* slucb1= new SignalLightUCB(signal);
			node.setUpdateCallback(slucb1);
			fprintf(stderr,"setsignal1 lpn %p %p %d %s\n",
			  slucb,signal,signal->units.size(),node.className());
		} else if (slucb != NULL) {
			for (int i=0; i<signal->units.size(); i++)
				slucb->signals.push_back(signal->units[i]);
			slucb->prevState= -1;
			fprintf(stderr,"setsignal0 lpn %p %p %d %s\n",
			  slucb,signal,signal->units.size(),node.className());
//		} else {
//			SignalLightUCB* slucb1= new SignalLightUCB(signal);
//			node.setUpdateCallback(slucb1);
//			fprintf(stderr,"setsignal2 lpn %p\n",signal);
		}
	}
	traverse(node);
}

void SwitchStandZUpdateCB::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (swVertex->edge1->track->matrix) {
		osg::Vec3d coord= swVertex->edge1->track->matrix->
		  preMult(swVertex->location.coord);
		osg::MatrixTransform* mt=
		  dynamic_cast<osg::MatrixTransform*> (node);
		osg::Matrixd m= mt->getMatrix();
		m(3,2)= coord[2]+zOffset;
		mt->setMatrix(m);
	}
	NodeCallback::traverse(node,nv);
}
