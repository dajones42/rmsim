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
#ifndef TRACKEDITOR_H
#define TRACKEDITOR_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <string>
#include <list>
#include <vector>

#include <osg/Notify>
#include <osg/Switch>
#include <osg/PositionAttitudeTransform>
#include <osg/AutoTransform>
#include <osg/ShapeDrawable>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

struct EdgeDrawable;
typedef	list<osg::ref_ptr<EdgeDrawable> > EdgeList;
struct EdgeGeode;
struct VertexDragger;
typedef	vector<osg::ref_ptr<VertexDragger> > VertexList;

struct VertexDragger : public osg::PositionAttitudeTransform 
{
	osg::ref_ptr<osg::ShapeDrawable> shapeDrawable;
	EdgeList edges;
	osg::Vec3f dir;	// track direction
	float heading;
	float elevation;
	float grade;
	int id;
	string name;
	float edgeDistance;
	float sign;
	VertexDragger(osg::Vec3d pos);
	void setColor(osg::Vec4f color) {
		shapeDrawable->setColor(color);
	};
	void addEdge(EdgeDrawable* e) {
		edges.push_back(e);
	};
	void removeEdge(EdgeDrawable* e) {
		edges.remove(e);
	};
	void moveTo(osg::Vec3d pos);
	void moveTo(osg::Vec3d p, EdgeDrawable* e);
	void moveTo(osg::Vec3d p, osg::Vec3d p1, osg::Vec3d p2, double& mind,
	  float dist, bool onSegment=true);
	void setDirection(osg::Vec3f d) { dir= d; };
	osg::Vec3f getDirection(bool toward) { return toward ? dir : -dir; };
	std::string toString();
};

struct EdgeDrawable : public osg::Drawable {
	osg::ref_ptr<VertexDragger> vertex1;
	osg::ref_ptr<VertexDragger> vertex2;
	enum { STRAIGHT, CURVE } type;
	bool selected;
	list<osg::Vec3d> points;
	float radius;
	float angle;
	float length;
	float grade;
	int id;
	EdgeDrawable(VertexDragger* v1, VertexDragger* v2);
	~EdgeDrawable() {
		vertex1->removeEdge(this);
		vertex2->removeEdge(this);
	}
	virtual osg::Object* cloneType() const {
		return new EdgeDrawable(vertex1,vertex2);
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new EdgeDrawable(vertex1,vertex2);
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const EdgeDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rredit"; }
	virtual const char* className() const { return "EdgeDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
	VertexDragger* otherEnd(VertexDragger* v) {
		return v==vertex1 ? vertex2 : vertex1;
	}
	void update();
	void addCurvePoints(osg::Vec3d p1, osg::Vec3d p2, osg::Vec3f d1,
	  osg::Vec3f d2);
	void addStraightPoints(float spacing);
	std::string toString();
	bool isStraight() {
		return type==EdgeDrawable::STRAIGHT ||
		  vertex1->edges.size()==1 || vertex2->edges.size()==1;
	}
};

struct EdgeGeode : public osg::Geode {
	EdgeGeode() {
		getOrCreateStateSet()->setMode(GL_DEPTH_TEST,
		  osg::StateAttribute::OFF);
		getOrCreateStateSet()->setMode(GL_LIGHTING,
		  osg::StateAttribute::OFF);
	}
	EdgeDrawable* addEdge(VertexDragger* v1, VertexDragger* v2) {
		EdgeDrawable* e= new EdgeDrawable(v1,v2);
		addDrawable(e);
		return e;
	}
	void removeEdge(EdgeDrawable* e) {
		removeDrawable(e);
	}
};

struct TrackEditor : public osgGA::GUIEventHandler 
{
	bool editing;
	osg::ref_ptr<VertexDragger> selectedV;
	osg::ref_ptr<EdgeDrawable> selectedE;
	osg::ref_ptr<VertexDragger> dragging;
	osg::ref_ptr<osg::Switch> rootNode;
	osg::ref_ptr<EdgeGeode> edgeGeode;
	osg::Geode* profileGeode;
	bool commandMode;
	TrackEditor(osg::Group* root);
	bool handle( const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa );
	std::string handleCommand(const char*);
	VertexDragger* findSelection(osgViewer::Viewer* viewer,
	  float x, float y, bool add);
	osg::Vec3d mouseHitCoord(osgViewer::Viewer* viewer,
	  float x, float y, osg::Vec3d dflt);
	void clearSelected();
	void setSelected(VertexDragger* v);
	EdgeDrawable* closestEdge(osg::Vec3d point, VertexDragger* avoid);
	void moveAway(VertexDragger* vd, float dist);
	void alignStraight(VertexDragger* vd);
	void updateEdges();
	void startEditing();
	void stopEditing();
	void makeProfile();
};

#endif
