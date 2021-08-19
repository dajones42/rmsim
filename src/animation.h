//	OSG animation callbacks for various animations
//
//Copyright 2009 Doug Jones
#ifndef ANIMATION_H
#define ANIMATION_H

#include <osg/AnimationPath>
#include <osg/NodeVisitor>
#include <osgSim/LightPointNode>

struct TwoStateAnimPathCB : public osg::AnimationPathCallback {
	Track::SwVertex* swVertex;
	RailCarInst* car;
	int index;
	int prevState;
	int newState;
	TwoStateAnimPathCB(Track::SwVertex* sw, osg::AnimationPath* ap,
	  int i=-1);
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

struct SetSwVertexVisitor : public osg::NodeVisitor {
	Track::SwVertex* swVertex;
	SetSwVertexVisitor(Track::SwVertex* sw) :
	  osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
		swVertex= sw;
	};
	virtual void apply(osg::MatrixTransform& mt);
};

struct RodAnimPathCB : public osg::AnimationPathCallback {
	RailCarInst* car;
	int nFrames;
	RodAnimPathCB(RailCarInst* c, osg::AnimationPath* ap, int n);
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

struct CouplerAnimPathCB : public osg::AnimationPathCallback {
	RailCarInst* car;
	bool front;
	CouplerAnimPathCB(RailCarInst* c, osg::AnimationPath* ap, bool f);
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

struct SetCarVisitor : public osg::NodeVisitor {
	RailCarInst* car;
	int nRods;
	SetCarVisitor(RailCarInst* c) :
	  osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
		car= c;
		nRods= 0;
	};
	virtual void apply(osg::MatrixTransform& mt);
};

#include "interlocking.h"

struct InterlockingAnimPathCB : public osg::AnimationPathCallback {
	Interlocking* interlocking;
	int lever;
	int prevState;
	int newState;
	InterlockingAnimPathCB(Interlocking* i, int l, osg::AnimationPath* ap) :
	  osg::AnimationPathCallback(ap,0,1) {
		interlocking= i;
		lever= l;
		prevState= -1;
		newState= 0;
	};
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

struct SetInterlockingVisitor : public osg::NodeVisitor {
	Interlocking* interlocking;
	SetInterlockingVisitor(Interlocking* i) :
	  osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
		interlocking= i;
	};
	virtual void apply(osg::MatrixTransform& mt);
};

struct SignalAnimPathCB : public osg::AnimationPathCallback {
	Signal* signal;
	int unit;
	int prevState;
	int newState;
	SignalAnimPathCB(MSTSSignal* sig, osg::AnimationPath* ap, int u);
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

struct SignalLightUCB : public osg::NodeCallback {
	std::vector<Signal*> signals;
	int prevState;
	int newState;
	SignalLightUCB(MSTSSignal* sig);
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

struct SetSignalVisitor : public osg::NodeVisitor {
	MSTSSignal* signal;
	SetSignalVisitor(MSTSSignal* sig) :
	  osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {
		signal= sig;
	};
	virtual void apply(osg::MatrixTransform& mt);
	virtual void apply(osg::Node& node);
};

struct SwitchStandZUpdateCB : public osg::NodeCallback {
	Track::SwVertex* swVertex;
	float zOffset;
	SwitchStandZUpdateCB(Track::SwVertex* sw, float zo) {
		swVertex= sw;
		zOffset= zo;
	};
	void operator()(osg::Node* node, osg::NodeVisitor* nv);
};

#endif
