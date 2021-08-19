//	classes for track system
//
//Copyright 2009 Doug Jones
#ifndef TRACK_H
#define TRACK_H

#include <string>
#include <list>
#include <map>
#include <osg/Group>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Vec3>

#include "trackshape.h"

//	world location information
struct WLocation {
	osg::Vec3d coord;
	osg::Vec3f up;
};

struct Signal;

struct Track {
	enum { ET_STRAIGHT, ET_SPLINE } EdgeType;
	enum { VT_SIMPLE, VT_SWITCH } VertexType;
	struct Vertex;
	struct SSEdge;
	struct Edge {	// straight section of track between two points
		short type;
		short occupied;
		Vertex* v1;
		Vertex* v2;
		float length;
		float ssOffset;
		SSEdge* ssEdge;
		Track* track;
		float curvature;	// degrees
		std::list<Signal*> signals;
		Vertex* otherV(Vertex* v) { return v==v1 ? v2 : v1; };
		float grade() {
			return length<=0 ? 0 :
			  (v2->elevation-v1->elevation)/length;
		}
		float grade(Vertex* v) {
			return v==v1 ? grade() : -grade();
		}
		void updateSignals();
	};
	struct SplineEdge : public Edge { //spline track section
		float dd1[3];
		float dd2[3];
		float splineMult;
		float angle;
		void setCircle(float radius, float angle);
	};
	struct SSEdge : public Edge { // direct switch to switch edge
		short block;
	};
	struct Vertex {	// track section end point
		short type;
		short occupied;
		WLocation location;
		Edge* edge1;
		Edge* edge2;
		Edge* inEdge;
		float dist;
		float grade;
		float elevation;
		inline Edge* nextEdge(Edge* e) {
			if (e == edge1)
				return edge2;
			if (e == edge2)
				return edge1;
			return NULL;
		}
		void saveEdge(int n, Edge* e);
	};
	struct SwVertex : public Vertex { // switch point
		Edge* swEdges[2];
		SSEdge* ssEdges[3];
		short outEdge;
		short mainEdge;
		short hasInterlocking;
		bool locked;
		int id;
		void throwSwitch(Edge* edge, bool force);
		SwVertex() {
			id= -1;
			hasInterlocking= 0;
			locked= false;
			mainEdge= 0;
			for (int i=0; i<3; i++)
				ssEdges[i]= NULL;
		}
		SSEdge* findSSEdge(SwVertex* sw) {
			for (int i=0; i<3; i++)
				if (ssEdges[i]->v1==sw || ssEdges[i]->v2==sw)
					return ssEdges[i];
			return NULL;
		}
	};
	struct Location { // movable track location
		Edge* edge;
		float offset;	// distance from edge->v1
		int rev;	// !=0 if positive move is toward edge->v1
		int move(float distance, int throwSwitches, int dOccupied);
		void getWLocation(WLocation* location, int local=0,
		  bool useElevation=false);
		float grade();
		float distance(Location* other);
		float dDistance(Location* other);
		float maxDistance(bool behind, float alignTol=-1);
		float vDistance(Vertex* targetV, bool behind, bool* facing);
		float getDist();
		float curvature() { return edge->curvature; };
		void set(SSEdge* sse, float ssOffset, int r);
		Location() { };
		Location(Vertex* v) {
			edge= v->edge1;
			offset= v==edge->v1 ? 0 : edge->length;
			rev= 0;
		};
	};
	struct Path {
		typedef enum {
			OTHER, STOP, SIDINGSTART, SIDINGEND, COUPLE, UNCOUPLE,
			REVERSE
		} NodeType;
		struct Node {
			NodeType type;
			int value;	// wait time or cars to uncouple
			Location loc;
			SwVertex* sw;
			Node* next;
			Node* nextSiding;
			SSEdge* nextSSEdge;
			SSEdge* nextSidingSSEdge;
			Node() {
				type= OTHER;
				sw= NULL;
				next= NULL;
				nextSiding= NULL;
				nextSSEdge= NULL;
				nextSidingSSEdge= NULL;
			};
		};
		Node* firstNode;
	};
	typedef std::list<Edge*> EdgeList;
	typedef std::list<Vertex*> VertexList;
	typedef std::multimap<std::string,Track::Location> LocationMap;
	typedef std::map<int,SwVertex*> SwitchMap;
	typedef std::map<int,SSEdge*> SSEdgeMap;
	LocationMap locations;
	VertexList vertexList;
	EdgeList edgeList;
	SwitchMap switchMap;
	SSEdgeMap ssEdgeMap;
	TrackShape* shape;
	osg::Geode* geode;
	osg::Matrixd* matrix;
	double sumMass;
	double sumXMass;
	double sumYMass;
	double sumZMass;
	Edge* addEdge(int type, Vertex* v1, int n1, Vertex* v2, int n2);
	Vertex* addVertex(int type, double x, double y, float z);
	Edge* addCurve(Vertex* v1, int n1, Vertex* v2, int n2);
	float findLocation(double x, double y, double z, Location* loc);
	float findLocation(double x, double y, Location* loc);
	void saveLocation(double x, double y, double z, std::string& name,
	  int rev=0);
	int findLocation(std::string& name, int index, Location* loc);
	int findLocation(std::string& name, Location* loc);
	int throwSwitch(double x, double y, double z);
	int lockSwitch(double x, double y, double z);
	SwVertex* findSwitch(double x, double y, double z);
	void calcMinMax();
	double minVertexX;
	double maxVertexX;
	double minVertexY;
	double maxVertexY;
	float minVertexZ;
	float maxVertexZ;
	Vertex** vQueue;
	Track();
	~Track();
	void translate(double dx, double dy, double dz);
	void rotate(double angle);
	osg::Geometry* makeGeometry();
	void makeMovable() {
		matrix= new osg::Matrixd();
	};
	void setMatrix(osg::Matrixd& m) {
		matrix->set(m);
	};
	void calcSplines(EdgeList& splines, Edge* e1, Edge* e2);
	int findSPT(Location& startLocation, bool bothDirections=true,
	  Path* path=NULL);
	int findSPT(Track::Location& startLocation, float chgPenalty,
	  float occupiedPenalty, Track::Vertex* avoid=NULL);
	float checkOccupied(Track::Vertex* farv);
	Vertex* findSiding(float distance, float tol);
	Vertex* findSidingParent(Vertex* v);
	Vertex* findCommonParent(Vertex* v1, Vertex* v2);
	Vertex* findSiding(osg::Vec3d& coord, float len);
	void orient(Path* path);
	void makeSSEdges();
	void calcGrades();
	void calcSmoothGrades(int nInterations, float distance);
	void alignSwitches(Path::Node* from, Path::Node* to, bool takeSiding);
	void makeSwitchCurves();
	void addSwitchStand(int swid, double offset, double zoffset,
	  osg::Node* model, osg::Group* rootNode, double fOffset=0);
	void alignSwitches(std::string from, std::string to);
	void alignSwitches(Location& from, Location& to);
	float averageElevation(Edge* edge, Vertex* vertex, float dist);
	bool updateSignals;
	void split(std::string& newName, double x1, double y1,
	  double x2, double y2);
	Track* expand();
};
typedef std::map<std::string,Track*> TrackMap;
extern TrackMap trackMap;

extern void findTrackLocation(double x, double y, double z,
  Track::Location* loc);
extern Track::SwVertex* findTrackSwitch(int id);
extern Track::SSEdge* findTrackSSEdge(int id);
extern void printTrackLocations();
extern osg::Switch* addTrackLabels();

struct TrackConn {
	Track track;
	Track* track1;
	Track* track2;
	TrackConn(Track* t1, Track* t2);
	~TrackConn();
	int occupied();
};

#endif
