//	float bridge control functions
//
//Copyright 2009 Doug Jones
#include "rmsim.h"

FBList fbList;

FloatBridge::FloatBridge()
{
	Ship();
	carFloat= NULL;
}

FloatBridge::~FloatBridge()
{
	for (FBTrackList::iterator i=tracks.begin(); i!=tracks.end(); ++i) {
		FBTrack* fbt= *i;
		delete fbt->track;
		if (fbt->conn != NULL)
			delete fbt->conn;
	}
}

//	updates the float bridge position based on car float movement
//	also moves track on the bridge
void FloatBridge::move()
{
	if (carFloat != NULL) {
		for (FBTrackList::iterator i=tracks.begin(); i!=tracks.end();
		  ++i) {
			FBTrack* fbt= *i;
			if (fbt->conn != NULL)
				continue;
			double x1= pivot.coord[0] + heading[0]*length;
			double y1= pivot.coord[1] + heading[1]*length;
			float z1= pivot.coord[2] - trim*length;
			double x2= carFloat->position[0] +
			  carFloat->heading[0]*cfOffset;
			double y2= carFloat->position[1] +
			  carFloat->heading[1]*cfOffset;
			float z2= carFloat->position[2] -
			  cfOffset*carFloat->trim + cfZOffset;
			float x= x1-x2;
			float y= y1-y2;
			float z= z1-z2;
			fprintf(stderr,"tdist %f %f %f %f %f %f\n",
			  x,y,z,x*x+y*y+z*z,z1,z2);
			if (x*x+y*y+z*z < .1) {
				fbt->conn= new TrackConn(fbt->track,
				  carFloat->track);
				if (fbt->conn->track.edgeList.size() == 0) {
					delete fbt->conn;
					fbt->conn= NULL;
				}
			}
		}
		float dz= carFloat->position[2] - cfOffset*carFloat->trim +
		  cfZOffset - pivot.coord[2];
		position[2]= dz/2;
		float dx= length;//sqrt(length*length-dz*dz);
		float dy= -carFloat->list* (-cfZOffset +
		  carFloat->position[2] + cfOffset*carFloat->trim);
		trim= -dz/length;
		list= -carFloat->list;
//		fprintf(stderr,"%f %f %f\n",dx,dz,trim);
		double x= pivot.coord[0] + heading[0]*(cfOffset+dx) -
		  heading[1]*dy;
		double y= pivot.coord[1] + heading[1]*(cfOffset+dx) +
		  heading[0]*dy;
		carFloat->position[0]= .99*carFloat->position[0] + .01*x;
		carFloat->position[1]= .99*carFloat->position[1] + .01*y;
		carFloat->heading[0]= .99*carFloat->heading[0] - .01*heading[0];
		carFloat->heading[1]= .99*carFloat->heading[1] - .01*heading[1];
	}
	position[0]= pivot.coord[0] + heading[0]*length/2;
	position[1]= pivot.coord[1] + heading[1]*length/2;
//	fprintf(stderr,"%s %lf %lf\n",name.c_str(),position[0],position[1]);
	for (FBTrackList::iterator i=tracks.begin(); i!=tracks.end(); ++i) {
		FBTrack* fbt= *i;
		float dz= trim*length+list*fbt->offset;
		float ttrim= dz/length;
		osg::Vec3f fwd= osg::Vec3f(heading[0],heading[1],-ttrim);
//		osg::Vec3f side= osg::Vec3f(-heading[1],heading[0],-list/2);
		osg::Vec3f side= osg::Vec3f(-heading[1],heading[0],0);
		fwd.normalize();
		side.normalize();
		osg::Vec3f up= fwd^side;
		osg::Matrixd m(
		  fwd[0],fwd[1],fwd[2],0,
		  side[0],side[1],side[2],0,
		  up[0],up[1],up[2],0,
		  position[0]-heading[1]*fbt->offset,
		  position[1]+heading[0]*fbt->offset,
		  pivot.coord[2]-dz/2,1);
		fbt->track->setMatrix(m);
		if (fbt->model != NULL)
			fbt->model->setMatrix(m);
//		fprintf(stderr,"fbt setMat %s %p %f %f\n",
//		  name.c_str(),fbt->model,pivot.coord[2],-dz/2);
	}
	list*= .5;
	list= 0;
}

//	connects the float bridge track to a car float's track
void FloatBridge::connect()
{
	if (carFloat != NULL)
		return;
	for (RopeList::iterator i=ropeList.begin(); i!=ropeList.end(); ++i) {
		Rope* r= *i;
		if (r->cleat2==NULL ||
		  (r->cleat1->ship!=this && r->cleat2->ship!=this))
			continue;
		Ship* ship= r->cleat1->ship==this ?  r->cleat2->ship :
		  r->cleat1->ship;
		if (ship->mass==0 || ship->track==NULL)
			continue;
		carFloat= ship;
	}
	if (carFloat == NULL)
		return;
	cfOffset= 0;
	cfZOffset= 0;
	for (Track::VertexList::iterator i=carFloat->track->vertexList.begin();
	  i!=carFloat->track->vertexList.end(); ++i) {
		Track::Vertex* v= *i;
		if (v->edge1!=NULL && v->edge2!=NULL)
			continue;
		if (cfOffset < v->location.coord[0]) {
			cfOffset= v->location.coord[0];
			cfZOffset= v->location.coord[2];
		}
	}
	float dx= carFloat->position[0] -
	  (pivot.coord[0] + heading[0]*(cfOffset+length));
	float dy= carFloat->position[1] -
	  (pivot.coord[1] + heading[1]*(cfOffset+length));
	fprintf(stderr,"connect %s %f %lf %lf %lf %lf %f\n",
	  carFloat->name.c_str(),dx*dx+dy*dy,
	  carFloat->position[0],carFloat->position[1],
	  pivot.coord[0] + heading[0]*cfOffset,
	  pivot.coord[1] + heading[1]*cfOffset,
	  cfOffset);
	if (dx*dx + dy*dy > 2) {
		carFloat= NULL;
	} else {
		for (RopeList::iterator i=ropeList.begin(); i!=ropeList.end();
		  ++i) {
			Rope* r= *i;
			if (r->cleat1->ship==this ||
			  (r->cleat2!=NULL && r->cleat2->ship==this))
				r->adjust= 1;
		}
	}
}

//	disconnects car float and float bridge tracks
void FloatBridge::disconnect()
{
	if (carFloat == NULL)
		return;
	carFloat= NULL;
	for (FBTrackList::iterator i=tracks.begin(); i!=tracks.end(); ++i) {
		FBTrack* fbt= *i;
		if (fbt->conn != NULL)
			delete fbt->conn;
		fbt->conn= NULL;
	}
}

void moveFloatBridges()
{
	for (FBList::iterator i=fbList.begin(); i!=fbList.end(); ++i)
		(*i)->move();
}

void FloatBridge::addTrack(float o, Track* t)
{
	FBTrack* fbt= new FBTrack;
	fbt->offset= o;
	fbt->track= t;
	t->makeMovable();
	fbt->conn= NULL;
	fbt->model= NULL;
	tracks.push_back(fbt);
}

const char* twistVShader=
	"uniform float floatBridgeLength;"
	"uniform float floatBridgeTwist;"
	"void main()"
	"{"
	" vec4 vert= gl_Vertex;"
	" vert.z+= vert.y*floatBridgeTwist*(vert.x/floatBridgeLength+.5);"
	" gl_Position= gl_ModelViewProjectionMatrix * vert;"
	" gl_FrontColor= gl_Color;"
	"}";

const char* twistFShader=
	"uniform sampler2D sampler0;"
	"void main()"
	"{"
	" gl_FragColor= texture2D(sampler0,gl_TexCoord[0].st);"
	"}";

#include <osg/Uniform>
#include <osg/Program>
#include <osg/Shader>

struct FloatBridgeTwistCB : public osg::Uniform::Callback {
	FloatBridge* floatBridge;
	FloatBridgeTwistCB(FloatBridge* fb) {
		floatBridge= fb;
	};
	virtual void operator() (osg::Uniform* uniform, osg::NodeVisitor* nv) {
		if (floatBridge->carFloat) {
			uniform->set((float)(floatBridge->carFloat->list));
//			fprintf(stderr,"fbtwist %f\n",
//			  floatBridge->carFloat->list);
		} else {
			uniform->set((float)0.);
		}
	}
};

void FloatBridge::setupTwist()
{
	if (model == NULL)
		return;
	osg::Program* program= new osg::Program;
	osg::Shader* vShader= new osg::Shader(osg::Shader::VERTEX,twistVShader);
	program->addShader(vShader);
//	osg::Shader* fShader= new osg::Shader(osg::Shader::FRAGMENT,
//	  twistFShader);
//	program->addShader(fShader);
	osg::StateSet* stateSet= model->getOrCreateStateSet();
	stateSet->setAttributeAndModes(program);
	stateSet->addUniform(new osg::Uniform("sampler0",(int)0));
	stateSet->addUniform(new osg::Uniform("floatBridgeLength",
	  (float)length));
	osg::Uniform* twistUniform= new osg::Uniform("floatBridgeTwist",
	  (float)0.);
	twistUniform->setUpdateCallback(new FloatBridgeTwistCB(this));
	stateSet->addUniform(twistUniform);
}
