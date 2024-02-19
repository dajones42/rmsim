//	file parser for rmsim
//	should be rewritten to use command reader for each class
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
#include "parser.h"
#include "mstswag.h"
#include <time.h>
#include "changelog.h"

#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Matrix>
#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/NodeVisitor>
//#include <osgEarth/MapNode>

struct RMParser : public Parser {
//	void parseWindow();
	void parseTriFiles(Water* water, const char* path);
	void parseWater();
	void parseForceTable(ForceTable& table);
	void parseShip();
	void parseFloatBridge();
	void parseTrack();
	void parseTrackShape();
	void parseRailCarDef(RailCarDef* def);
	void parseTrain();
	void parse2DVertices(Model2D* model);
	void parseModel2D();
	void parseTexture();
	void parseModel3D();
	void parsePersonPath();
	void parseMSTSRoute(string& dir, string& route);
	void parseSave();
	void parseFile(const char* path);
	osg::Node* find3DModel(string& s);
	Ship* findShip(string& s);
	Track* findTrack(string& s);
	typedef map<string,osg::Node*> ModelMap;
	ModelMap modelMap;
	osg::Group* rootNode;
	bool isClient;
	bool readingSave;
};

struct PrintVisitor : public osg::NodeVisitor
{
	int level;
	PrintVisitor() {
		level= 0;
		setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	}
	std::string spaces() {
		return std::string(level*2,' ');
	}
	virtual void apply(osg::Node& node) {
		level++;
		traverse(node);
		level--;
	}
	virtual void apply(osg::Geode& geode) {
		for (int i=0; i<geode.getNumDrawables(); i++) {
			osg::Geometry* geom=
			  dynamic_cast<osg::Geometry*>(geode.getDrawable(i));
			if (!geom)
				continue;
			printf("%s%s %d %f\n",spaces().c_str(),
			  geode.getName().c_str(),i,geom->getBound().radius());
		}
	}
};

struct RemoveVisitor : public osg::NodeVisitor
{
	std::string name;
	int drawable;
	RemoveVisitor(std::string nm, int i) {
		name= nm;
		drawable= i;
		setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	}
	virtual void apply(osg::Geode& geode) {
		if (geode.getName()==name && drawable<geode.getNumDrawables()) {
			geode.removeDrawables(drawable,1);
//			fprintf(stderr,"removed %s %d\n",name.c_str(),drawable);
		}
	}
};

struct CopyVisitor : public osg::NodeVisitor
{
	std::string name;
	osg::PositionAttitudeTransform* pat;
	CopyVisitor(std::string nm, osg::PositionAttitudeTransform* pat) {
		name= nm;
		this->pat= pat;
		setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
	}
	virtual void apply(osg::Geode& geode) {
		if (geode.getName()==name) {
			pat->addChild(&geode);
			fprintf(stderr,"copied %s\n",name.c_str());
		}
	}
};

#if 0
void RMParser::parseWindow()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	Window* wind= new Window(tokens[1].c_str());
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"type") == 0) {
				wind->setType(getInt(1,0,1));
			} else if (strcasecmp(cmd,"size") == 0) {
				wind->setSize(getInt(1,10,2000),
				  getInt(2,10,2000));
			} else if (strcasecmp(cmd,"position") == 0) {
				wind->setPosition(getInt(1,10,2000),
				  getInt(2,10,2000));
			} else if (strcasecmp(cmd,"bgcolor") == 0) {
				wind->setBGColor(getDouble(1,0,1),
				  getDouble(2,0,1),getDouble(3,0,1));
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
}
#endif

void RMParser::parseTriFiles(Water* water, const char* path)
{
	string nodeFile(string(path)+".node");
	pushFile(nodeFile.c_str(),0);
	getCommand();
	int nNodes= getInt(0,0,0x7fffffff);
	for (int i=0; i<nNodes; i++) {
		getCommand();
		water->addVertex(getInt(0,0,0x7fffffff),getDouble(1,-1e20,1e20),
		  getDouble(2,-1e20,1e20),getDouble(3,0,1000,0),-1);
	}
	fileStack.pop_back();
	string eleFile(string(path)+".ele");
	pushFile(eleFile.c_str(),0);
	getCommand();
	int nTris= getInt(0,0,0x7fffffff);
	for (int i=0; i<nTris; i++) {
		getCommand();
		water->addTriangle(getInt(1,0,0x7fffffff),
		  getInt(2,0,0x7fffffff),getInt(3,0,0x7fffffff));
	}
	fileStack.pop_back();
}

void RMParser::parseWater()
{
	float c0[3],c1[3];
	float depth= -1;
	string shorefile;
	Water* water= new Water();
	waterList.push_back(water);
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"vertex") == 0) {
				water->addVertex(getInt(1,0,0x7fffffff),
				  getDouble(2,-1e20,1e20),
				  getDouble(3,-1e20,1e20),
				  getDouble(4,0,1000,0),
				  getInt(5,-1,0x7fffffff,-1));
			} else if (strcasecmp(cmd,"vertices") == 0) {
				int n= getInt(1,0,0x7fffffff);
				for (int i=0; i<n; i++) {
					getCommand();
					try {
						water->addVertex(i,
						  getDouble(0,-1e20,1e20),
						  getDouble(1,-1e20,1e20),
						  getDouble(2,0,1000,0),
					  	  getInt(3,-1,0x7fffffff,-1));
					} catch (const char* message) {
						printError(message);
					} catch (const std::exception& error) {
						printError(error.what());
					}
				}
				getCommand();
			} else if (strcasecmp(cmd,"triangle") == 0) {
				water->addTriangle(getInt(1,0,0x7fffffff),
				  getInt(2,0,0x7fffffff),
				  getInt(3,0,0x7fffffff));
			} else if (strcasecmp(cmd,"triangles") == 0) {
				int n= getInt(1,0,0x7fffffff);
				for (int i=0; i<n; i++) {
					getCommand();
					try {
						water->addTriangle(
						  getInt(0,0,0x7fffffff),
						  getInt(1,0,0x7fffffff),
						  getInt(2,0,0x7fffffff));
					} catch (const char* message) {
						printError(message);
					} catch (const std::exception& error) {
						printError(error.what());
					}
				}
				getCommand();
			} else if (strcasecmp(cmd,"trifiles") == 0) {
				parseTriFiles(water,makePath());
			} else if (strcasecmp(cmd,"color") == 0) {
				c0[0]= getDouble(1,0,1);
				c0[1]= getDouble(2,0,1);
				c0[2]= getDouble(3,0,1);
				c1[0]= getDouble(4,0,1,c0[0]);
				c1[1]= getDouble(5,0,1,c0[1]);
				c1[2]= getDouble(6,0,1,c0[2]);
				depth= getDouble(7,0,1e10,5);
			} else if (strcasecmp(cmd,"shorefile") == 0) {
				shorefile= tokens[1];
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
			exit(1);
		} catch (const std::exception& error) {
			printError(error.what());
			exit(1);
		}
	}
	water->matchTriangles();
	if (depth >= 0)
		water->setVColor(c0,c1,depth);
	if (shorefile.size() > 0)
		water->saveShoreLine(shorefile.c_str());
}

void RMParser::parseForceTable(ForceTable& table)
{
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			float x= getDouble(0,0,1e20);
			float y= getDouble(1,0,1e20);
			table.add(x,y);
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	table.compute();
#if 0
	// test least squares fit
	double sy= 0;
	double syx= 0;
	double syx2= 0;
	double syx3= 0;
	double sx= 0;
	double sx2= 0;
	double sx3= 0;
	double sx4= 0;
	double sx6= 0;
	int n= 0;
	for (int i=0; i<table.size(); i++) {
		float x= table.getX(i);
		float y= table.getY(i);
		n++;
		sy+= y;
		syx+= y*x;
		syx2+= y*x*x;
		syx3+= y*x*x*x;
		sx+= x;
		sx2+= x*x;
		sx3+= x*x*x;
		sx4+= x*x*x*x;
		sx6+= x*x*x*x*x*x;
	}
	fprintf(stderr,"tablefit %d %f %f %f %f %f %f %f\n",
	  n,sy,syx,syx2,sx,sx2,sx3,sx4);
	double s1= syx - sy*sx/n;
	double s2= sx*sx2/n - sx3;
	double s3= sx2 - sx*sx/n;
	double s4= syx2 - sy*sx2/n;
	double s5= sx2*sx2/n - sx4;
	double s6= sx3 - sx*sx2/n;
	float c= (s1*s6-s3*s4)/(s3*s5-s2*s6);
	float b= (s1+c*s2)/s3;
	float a= (sy-b*sx-c*sx2)/n;
	float c2= syx2/sx4;
	float c3= syx3/sx6;
	fprintf(stderr," %f %f %f %f\n",a,b,c,c2);
	for (int i=0; i<table.size(); i++) {
		float x= table.getX(i);
		float y= table.getY(i);
		float z= a + b*x + c*x*x;
		float z1= c*x*x;
		float z2= c2*x*x;
		float z3= c3*x*x*x;
		fprintf(stderr," %f %f %f %f %f %f\n",x,y,z,y-z,z2,z3);
	}
#endif
}

void RMParser::parseShip()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	Ship* ship= new Ship();
	ship->name= tokens[1].c_str();
	shipMap[tokens[1]]= ship;
	shipList.push_back(ship);
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"mass") == 0) {
				ship->setMass(getDouble(1,0,1e10),
				getDouble(2,0,1e10,1),getDouble(3,0,1e10,1),
				getDouble(4,0,1e10,1));
			} else if (strcasecmp(cmd,"draft") == 0) {
				ship->draft= getDouble(1,0,10);
			} else if (strcasecmp(cmd,"lwl") == 0) {
				ship->lwl= getDouble(1,0,1000);
			} else if (strcasecmp(cmd,"bwl") == 0) {
				ship->bwl= getDouble(1,0,1000);
			} else if (strcasecmp(cmd,"fwddrag") == 0) {
				parseForceTable(ship->fwdDrag);
			} else if (strcasecmp(cmd,"backdrag")==0 &&
			  tokens.size()>=3) {
				ship->backDrag.copy(ship->fwdDrag,
				  getDouble(1,0,1e10),getDouble(2,0,1e10));
			} else if (strcasecmp(cmd,"backdrag") == 0) {
				parseForceTable(ship->backDrag);
			} else if (strcasecmp(cmd,"sidedrag")==0 &&
			  tokens.size()>=3) {
				ship->sideDrag.copy(ship->fwdDrag,
				  getDouble(1,0,1e10),getDouble(2,0,1e10));
			} else if (strcasecmp(cmd,"sidedrag") == 0) {
				parseForceTable(ship->sideDrag);
			} else if (strcasecmp(cmd,"rotdrag")==0 &&
			  tokens.size()>=3) {
				ship->rotDrag.copy(ship->fwdDrag,
				  getDouble(1,0,1e10),getDouble(2,0,1e10));
			} else if (strcasecmp(cmd,"rotdrag") == 0) {
				parseForceTable(ship->rotDrag);
			} else if (strcasecmp(cmd,"siderotdrag")==0 &&
			  tokens.size()>=3) {
				ship->sideRotDrag.copy(ship->fwdDrag,
				  getDouble(1,0,1e10),getDouble(2,0,1e10));
			} else if (strcasecmp(cmd,"siderotdrag") == 0) {
				parseForceTable(ship->sideRotDrag);
			} else if (strcasecmp(cmd,"calcdrag") == 0) {
				ship->calcDrag(getDouble(1,0,1),
				  getDouble(2,0,20),getDouble(3,0,1e10),
				  getDouble(4,0,10));
			} else if (strcasecmp(cmd,"position") == 0) {
				ship->setPosition(getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10));
			} else if (strcasecmp(cmd,"heading") == 0) {
				ship->setHeading(getDouble(1,0,360));
			} else if (strcasecmp(cmd,"power") == 0) {
				ship->engine.setPower(getDouble(1,0,1e10));
			} else if (strcasecmp(cmd,"propeller") == 0) {
				ship->engine.setProp(getDouble(1,0,10),
				  getDouble(2,0,10,0));
			} else if (strcasecmp(cmd,"rudder") == 0) {
				ship->rudder.cFwdSpeed= getDouble(1,0,1e10);
				ship->rudder.cBackSpeed= getDouble(2,0,1e10);
				ship->rudder.cFwdThrust= getDouble(3,0,1e10);
				ship->rudder.cBackThrust= getDouble(4,0,1e10);
			} else if (strcasecmp(cmd,"model") == 0) {
				osg::Node* model= find3DModel(tokens[1]);
				osg::MatrixTransform* transform=
				  new osg::MatrixTransform;
				transform->addChild(model);
				rootNode->addChild(transform);
				ship->model= transform;
			} else if (strcasecmp(cmd,"track") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				TrackMap::iterator i= trackMap.find(tokens[1]);
				if (i == trackMap.end())
					throw "cannot find track";
				ship->track= i->second;
				ship->track->makeMovable();
			} else if (strcasecmp(cmd,"model3d") == 0) {
				ship->model3D= string(tokens[1]);
			} else if (strcasecmp(cmd,"modeloffset") == 0) {
				ship->modelOffset= getDouble(1,-1000,1000);
			} else if (strcasecmp(cmd,"model2d") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				Model2DMap::iterator i=
				  model2DMap.find(tokens[1]);
				if (i == model2DMap.end())
					throw "cannot find model";
				ship->model2D= i->second;
				if (ship->boundary == NULL)
					ship->boundary= i->second;
			} else if (strcasecmp(cmd,"boundary") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				Model2DMap::iterator i=
				  model2DMap.find(tokens[1]);
				if (i == model2DMap.end())
					throw "cannot find model";
				ship->boundary= i->second;
			} else if (strcasecmp(cmd,"cleat") == 0) {
				Cleat* cleat= new Cleat;
				cleat->ship= ship;
				cleat->x= getDouble(1,-1000,1000);
				cleat->y= getDouble(2,-1000,1000);
				cleat->z= getDouble(3,-1000,1000,0);
				cleat->standingZ= getDouble(4,-1000,1000,
				  cleat->z);
				cleat->next= ship->cleats;
				ship->cleats= cleat;
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	if (ship->track != NULL)
		ship->move(0);
	fprintf(stderr,"ship %s %f %f %f\n",ship->name.c_str(),
	  ship->position[0],ship->position[1],ship->position[2]);
//	float d= ship->fwdDrag1*ship->fwdDrag1 +
//	  4*ship->fwdDrag2*ship->engine.maxThrust;
//	fprintf(stderr,"%s max fwd speed=%f\n",ship->name.c_str(),
//	  .5*(-ship->fwdDrag1+sqrt(d))/ship->fwdDrag2);
//	d= ship->backDrag1*ship->backDrag1 +
//	  4*ship->backDrag2*ship->engine.maxThrust;
//	fprintf(stderr,"%s max back speed=%f\n",ship->name.c_str(),
//	  .5*(-ship->backDrag1+sqrt(d))/ship->backDrag2);
}

void RMParser::parseFloatBridge()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	FloatBridge* fb= new FloatBridge;
	fb->name= tokens[1].c_str();
	shipMap[tokens[1]]= (Ship*)fb;
	shipList.push_back((Ship*)fb);
	fbList.push_back(fb);
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"pivot") == 0) {
				fb->pivot.coord[0]= getDouble(1,-1e10,1e10);
				fb->pivot.coord[1]= getDouble(2,-1e10,1e10);
				fb->pivot.coord[2]= getDouble(3,-1e10,1e10);
			} else if (strcasecmp(cmd,"length") == 0) {
				fb->length= getDouble(1,1,100);
			} else if (strcasecmp(cmd,"heading") == 0) {
				fb->setHeading(getDouble(1,0,360));
			} else if (strcasecmp(cmd,"model") == 0) {
				osg::Node* model= find3DModel(tokens[1]);
				osg::MatrixTransform* transform=
				  new osg::MatrixTransform;
				transform->addChild(model);
				rootNode->addChild(transform);
				fb->model= transform;
			} else if (strcasecmp(cmd,"track") == 0) {
				if (tokens.size() < 3)
					throw "name missing";
				TrackMap::iterator i= trackMap.find(tokens[2]);
				if (i == trackMap.end())
					throw "cannot find track";
				fb->addTrack(getDouble(1,-1000,1000),i->second);
			} else if (strcasecmp(cmd,"boundary") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				Model2DMap::iterator i=
				  model2DMap.find(tokens[1]);
				if (i == model2DMap.end())
					throw "cannot find model";
				fb->boundary= i->second;
			} else if (strcasecmp(cmd,"cleat") == 0) {
				Cleat* cleat= new Cleat;
				cleat->ship= (Ship*)fb;
				cleat->x= getDouble(1,-1000,1000);
				cleat->y= getDouble(2,-1000,1000);
				cleat->z= getDouble(3,-1000,1000,0);
				cleat->standingZ= getDouble(4,-1000,1000,
				  cleat->z);
				cleat->next= fb->cleats;
				fb->cleats= cleat;
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	fb->move();
	((Ship*)fb)->move(0);
	fprintf(stderr,"fb %s %f %f %f\n",fb->name.c_str(),fb->position[0],
	  fb->position[1],fb->position[2]);
	fb->setupTwist();
}

void RMParser::parse2DVertices(Model2D* model)
{
	if (tokens.size() < 2)
		throw "nVertices missing";
	model->nVertices= getInt(1,1,0x65535);
	model->vertices=
	  (Model2D::Vertex*) calloc(model->nVertices,sizeof(Model2D::Vertex));
	int i= 0;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (i >= model->nVertices)
				throw "too many vertices";
			float x= getDouble(0,-1e20,1e20);
			float y= getDouble(1,-1e20,1e20);
			model->vertices[i].x= x;
			model->vertices[i].y= y;
			float r= sqrt(x*x + y*y);
			if (model->radius < r)
				model->radius= r;
			if (model->texture != NULL) {
				model->vertices[i].u= getDouble(2,0,1);
				model->vertices[i].v= getDouble(3,0,1);
			}
			i++;
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	if (i < model->nVertices)
		throw "not enough vertices";
}

void RMParser::parseModel2D()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	Model2D* model= new Model2D;
	model2DMap[tokens[1]]= model;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"vertices") == 0) {
				parse2DVertices(model);
			} else if (strcasecmp(cmd,"color") == 0) {
				model->color[0]= getDouble(1,0,1);
				model->color[1]= getDouble(2,0,1);
				model->color[2]= getDouble(3,0,1);
			} else if (strcasecmp(cmd,"polygon") == 0) {
				model->primitive= GL_POLYGON;
			} else if (strcasecmp(cmd,"quads") == 0) {
				model->primitive= GL_QUADS;
			} else if (strcasecmp(cmd,"texture") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				TextureMap::iterator i=
				  textureMap.find(tokens[1]);
				if (i == textureMap.end())
					throw "cannot find texture";
				model->texture= i->second;
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
}

void RMParser::parseTexture()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	Texture* texture= new Texture();
	textureMap[tokens[1]]= texture;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"file") == 0) {
				texture->filename= string(makePath());
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
}

void RMParser::parseMSTSRoute(string& dir, string& route)
{
	mstsRoute= new MSTSRoute(dir.c_str(),route.c_str());
	string shorefile;
	int saveTerrain= 0;
	int activityFlags= -1;
	float ssOffset= 0;
	float ssZOffset= 0;
	float ssPOffset= 0;
	float createWaterLevel= -1e10;
	osg::Node* ssModel= NULL;
	int smoothGradesIterations= 0;
	float smoothGradesDistance= 100;
	string pathFile;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"filename") == 0) {
				mstsRoute->fileName= tokens[1];
			} else if (strcasecmp(cmd,"center") == 0) {
				mstsRoute->centerTX= getInt(1,-16384,16384);
				mstsRoute->centerTZ= getInt(2,-16384,16384);
				mstsRoute->centerLat= getDouble(3,-90,90);
				mstsRoute->centerLong= getDouble(4,-180,180);
			} else if (strcasecmp(cmd,"saveTerrain") == 0) {
				saveTerrain= 1;
			} else if (strcasecmp(cmd,"smoothgrades") == 0) {
				smoothGradesIterations= getInt(1,0,1000,100);
				smoothGradesDistance= getDouble(2,1,1000,100);
			} else if (strcasecmp(cmd,"ignoreHiddenTerrain") == 0) {
				mstsRoute->ignoreHiddenTerrain= true;
			} else if (strcasecmp(cmd,"signalswitchstands") == 0) {
				mstsRoute->signalSwitchStands= true;
			} else if (strcasecmp(cmd,"createsignals") == 0) {
				mstsRoute->createSignals= true;
			} else if (strcasecmp(cmd,"shorefile") == 0) {
				shorefile= tokens[1];
			} else if (strcasecmp(cmd,"trackoverride") == 0) {
				mstsRoute->overrideTrackModel(
				  tokens[1],tokens[2]);
			} else if (strcasecmp(cmd,"drawwater") == 0) {
				mstsRoute->drawWater= getInt(1,0,1);
			} else if (strcasecmp(cmd,"waterleveldelta") == 0) {
				mstsRoute->waterLevelDelta=
				  getDouble(1,-100,100);
			} else if (strcasecmp(cmd,"createwater") == 0) {
				createWaterLevel= getDouble(1,-100,1e10);
			} else if (strcasecmp(cmd,"activity") == 0) {
				mstsRoute->activityName= tokens[1];
				activityFlags= getInt(2,-1,8,-1);
			} else if (strcasecmp(cmd,"switchstand") == 0) {
				ssOffset= getDouble(1,-10,10);
				ssZOffset= getDouble(2,-10,10);
				ssPOffset= getDouble(4,-10,10,0);
				ssModel= find3DModel(tokens[3]);
			} else if (strcasecmp(cmd,"berm") == 0) {
				mstsRoute->bermHeight= getDouble(1,0,100);
			} else if (strcasecmp(cmd,"bridge") == 0) {
				mstsRoute->bridgeBase= getInt(1,0,1);
			} else if (strcasecmp(cmd,"scalerail") == 0) {
				mstsRoute->srDynTrack= getInt(1,0,1);
			} else if (strcasecmp(cmd,"ustracks") == 0) {
				mstsRoute->ustDynTrack= getInt(1,0,1);
			} else if (strcasecmp(cmd,"wire") == 0) {
				mstsRoute->wireHeight= getDouble(1,0,10);
				mstsRoute->wireModelsDir= tokens[2];
			} else if (strcasecmp(cmd,"path") == 0) {
				pathFile= tokens[1];
			} else if (strcasecmp(cmd,"ignorepolygon") == 0) {
				for (int i=1; i<tokens.size(); i++)
					mstsRoute->ignorePolygon.push_back(
					  getDouble(i,-1e10,1e10));
				mstsRoute->ignorePolygon.push_back(
				  getDouble(1,-1e10,1e10));
				mstsRoute->ignorePolygon.push_back(
				  getDouble(2,-1e10,1e10));
			} else if (strcasecmp(cmd,"ignoreshape") == 0) {
				mstsRoute->ignoreShapeMap.insert(
				  make_pair(tokens[1],osg::Vec3d(
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  getDouble(4,-1e10,1e10))));
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	mstsRoute->readTiles();
	mstsRoute->adjustWater(saveTerrain);
	if (shorefile.size() > 0)
		mstsRoute->saveShoreMarkers(shorefile.c_str());
	mstsRoute->makeTrack(smoothGradesIterations,smoothGradesDistance);
	mstsRoute->loadActivity(rootNode,activityFlags);
	if (ssModel)
		mstsRoute->addSwitchStands(ssOffset,ssZOffset,ssModel,rootNode,
		  ssPOffset);
	if (pathFile.size() > 0)
		mstsRoute->loadPath(pathFile,true);
	if (createWaterLevel > -1e5)
		mstsRoute->createWater(createWaterLevel);
}

void RMParser::parseFile(const char* path)
{
	try {
		pushFile(path,0);
	} catch (const char* message) {
		fprintf(stderr,"%s %s\n",message,path);
		return;
	} catch (const std::exception& error) {
		fprintf(stderr,"%s %s\n",error.what(),path);
		return;
	}
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"water") == 0) {
				parseWater();
			} else if (strcasecmp(cmd,"ship") == 0) {
				parseShip();
			} else if (strcasecmp(cmd,"save") == 0) {
				parseSave();
			} else if (strcasecmp(cmd,"mstsroute") == 0) {
				parseMSTSRoute(tokens[1],tokens[2]);
			} else if (strcasecmp(cmd,"timetable") == 0) {
				timeTable= new TimeTable();
				timeTable->parse((CommandReader&)*this);
			} else if (strcasecmp(cmd,"signal") == 0) {
				SignalParser sparser;
				parseBlock((CommandBlockHandler*)&sparser);
			} else if (strcasecmp(cmd,"interlocking") == 0) {
				InterlockingParser iparser;
				parseBlock((CommandBlockHandler*)&iparser);
				interlocking= iparser.interlocking;
			} else if (strcasecmp(cmd,"useroscallsign") == 0) {
				userOSCallSign= tokens[1];
			} else if (strcasecmp(cmd,"floatbridge") == 0) {
				parseFloatBridge();
			} else if (strcasecmp(cmd,"rope") == 0) {
				Cleat* c1= findShip(tokens[1])->findCleat(
				  getDouble(2,-1e3,1e3),getDouble(3,-1e3,1e3));
				Cleat* c2= findShip(tokens[4])->findCleat(
				  getDouble(5,-1e3,1e3),getDouble(6,-1e3,1e3));
				if (c1==NULL || c2==NULL)
					throw "cannot find cleat";
				addRope(c1,c2);
			} else if (strcasecmp(cmd,"track") == 0) {
				parseTrack();
			} else if (strcasecmp(cmd,"trackshape") == 0) {
				parseTrackShape();
			} else if (strcasecmp(cmd,"splittrack") == 0) {
				Track* track= findTrack(tokens[1]);
				if (track == NULL)
					throw std::invalid_argument(
					  "cannot find track");
				track->split(tokens[2],
				  getDouble(3,-1e10,1e10),
				  getDouble(4,-1e10,1e10),
				  getDouble(5,-1e10,1e10),
				  getDouble(6,-1e10,1e10));
			} else if (strcasecmp(cmd,"railcar") == 0) {
				if (tokens.size() < 2)
					throw std::invalid_argument(
					  "name missing");
				RailCarDef* def= new RailCarDef();
				railCarDefMap[tokens[1]]= def;
				def->name= string(tokens[1]);
				parseRailCarDef(def);
			} else if (strcasecmp(cmd,"wag") == 0) {
				if (tokens.size() < 3)
					throw "dir and file expected";
				RailCarDef* def= readMSTSWag(
				  tokens[1].c_str(),tokens[2].c_str(),true);
				if (def == NULL)
					throw "cannot find car definition";
				railCarDefMap[tokens[2]]= def;
				parseRailCarDef(def);
			} else if (strcasecmp(cmd,"train") == 0) {
				parseTrain();
			} else if (strcasecmp(cmd,"connect") == 0) {
				Track* t1= findTrack(tokens[1]);
				Track* t2= findTrack(tokens[2]);
				new TrackConn(t1,t2);
			} else if (strcasecmp(cmd,"model2d") == 0) {
				parseModel2D();
			} else if (strcasecmp(cmd,"model3d") == 0) {
				parseModel3D();
			} else if (strcasecmp(cmd,"texture") == 0) {
				parseTexture();
			} else if (strcasecmp(cmd,"scenery") == 0) {
				osg::Node* model= find3DModel(tokens[1]);
				osg::PositionAttitudeTransform* pat=
				  new osg::PositionAttitudeTransform;
				pat->setPosition(
				  osg::Vec3(getDouble(2,-1e10,1e10),
				    getDouble(3,-1e10,1e10),
				    getDouble(4,-1e10,1e10)));
				pat->setAttitude(
				  osg::Quat(getDouble(5,0,360,0)*M_PI/180,
				  osg::Vec3(0,0,1)));
				pat->addChild(model);
				rootNode->addChild(pat);
#if 0
			} else if (strcasecmp(cmd,"earth") == 0) {
				osg::Node* model= find3DModel(tokens[1]);
				osgEarth::MapNode* mapNode=
				  osgEarth::MapNode::findMapNode(model);
				if (mapNode == NULL)
					throw "cannot find map node";
				double lat0= getDouble(2,-90,90)*M_PI/180;
				double long0= getDouble(3,-180,180)*M_PI/180;
				double hgt= -getDouble(4,-1e10,1e10);
#if 0
				double x,y,z;
				mapNode->getMap()->getProfile()->getSRS()->
				  getEllipsoid()->convertLatLongHeightToXYZ(
				  lat0,long0,0,x,y,z);
				double r= sqrt(x*x+y*y+z*z);
				fprintf(stderr,"earth %f %f %f %f %f %f\n",
				  x,y,z,r,asin(z/r),atan2(y,x));
				osg::Matrixd m1;
				m1.makeRotate(-(M_PI/2-asin(z/r)),1,0,0);
				osg::Matrixd m2;
				m2.makeRotate(-(atan2(y,x)+M_PI/2),0,0,1);
				osg::Matrixd m3;
				m3.makeTranslate(0,0,
				  getDouble(4,-1e10,1e10)-r);
				osg::MatrixTransform* mt=
				  new osg::MatrixTransform;
				mt->setMatrix(m2*m1*m3);
#else
				osg::Matrixd l2w;
				mapNode->getMap()->getProfile()->getSRS()->
				  getEllipsoid()->
				  computeLocalToWorldTransformFromLatLongHeight(
				  lat0,long0,hgt,l2w);
				osg::Matrixd w2l;
				w2l.invert(l2w);
				osg::MatrixTransform* mt=
				  new osg::MatrixTransform;
				mt->setMatrix(w2l);
#endif
				mt->addChild(model);
				rootNode->addChild(mt);
#endif
			} else if (strcasecmp(cmd,"interlockingmodel") == 0) {
				interlockingModel= find3DModel(tokens[1]);
			} else if (strcasecmp(cmd,"switchstand") == 0) {
				TrackMap::iterator i= trackMap.find(tokens[1]);
				if (i == trackMap.end())
					throw "cannot find track";
				i->second->addSwitchStand(getInt(2,-1,1000000),
				  getDouble(3,-10,10),getDouble(4,-10,10),
				  find3DModel(tokens[5]),rootNode);
			} else if (strcasecmp(cmd,"throwswitch") == 0) {
				TrackMap::iterator i= trackMap.find(tokens[1]);
				if (i == trackMap.end())
					throw "cannot find track";
				i->second->throwSwitch(
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  getDouble(4,-1e10,1e10));
			} else if (strcasecmp(cmd,"lockswitch") == 0) {
				TrackMap::iterator i= trackMap.find(tokens[1]);
				if (i == trackMap.end())
					throw "cannot find track";
				i->second->lockSwitch(
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  getDouble(4,-1e10,1e10));
			} else if (strcasecmp(cmd,"starttime") == 0) {
				simTime=
				  3600*getInt(1,0,23)+60*getInt(2,0,59);
			} else if (strcasecmp(cmd,"endtime") == 0) {
				endTime=
				  3600*getInt(1,0,1000)+60*getInt(2,0,59);
			} else if (strcasecmp(cmd,"randomize") == 0) {
				long t= time(NULL);
				if (tokens.size() > 1)
					t= atol(tokens[1].c_str());
				fprintf(stderr,"seed %ld\n",t);
				srand48(t);
			} else if (strcasecmp(cmd,"morse") == 0) {
				listener.getMorseConverter()->parse(
				  (CommandReader&)*this);
			} else if (strcasecmp(cmd,"railfan") == 0 ||
			  strcasecmp(cmd,"person") == 0) {
//				fprintf(stderr,"newperson %d %d\n",
//				  Person::stack.size(),Person::stackIndex);
				currentPerson.location= osg::Vec3d(
				  getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10)-1.7);
				currentPerson.setLocation(currentPerson.location);
				currentPerson.setAngle(getDouble(4,0,360,0));
				Person::stack.push_back(currentPerson);
				Person::stackIndex= Person::stack.size()-1;
//				fprintf(stderr,"newperson2 %d %d\n",
//				  Person::stack.size(),Person::stackIndex);
			} else if (strcasecmp(cmd,"railfanpath") == 0 ||
			  strcasecmp(cmd,"personpath") == 0) {
				parsePersonPath();
			} else if (strcasecmp(cmd,"pickwaybills") == 0) {
				selectWaybills(getInt(1,0,100),
				  getInt(2,0,100,0),tokens[3]);
			} else if (strcasecmp(cmd,"location") == 0) {
				TrackMap::iterator j= trackMap.find(tokens[5]);
				if (j == trackMap.end())
					throw "cannot find track";
				j->second->saveLocation(
				  getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  tokens[4]);
			} else if (strcasecmp(cmd,"align") == 0) {
				Track* t= findTrack(tokens[1]);
				if (t == NULL)
					throw "cannot find track";
				t->alignSwitches(tokens[2],tokens[3]);
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
}

void parseFile(const char* path, osg::Group* root, bool isClient, int argc,
  char** argv)
{
	saveString= path;
	RMParser parser;
	parser.isClient= isClient;
	parser.rootNode= root;
	parser.readingSave= false;
	for (int i=1; i<argc; i++) {
		saveString+= " ";
		saveString+= argv[i];
		parser.symbols.insert(argv[i]);
		fprintf(stderr,"symbol %s\n",argv[i]);
	}
	parser.parseFile(path);
}

int swEnd(string& s)
{
	if (s=="p" || s=="P")
		return 0;
	if (s=="n" || s=="N")
		return 1;
	if (s=="r" || s=="R")
		return 2;
	throw "invalid switch endpoint";
}

void RMParser::parseTrack()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	TrackMap::iterator i= trackMap.find(tokens[1]);
	Track* track;
	if (i != trackMap.end()) {
		track= i->second;
	} else {
		track= new Track();
		trackMap[tokens[1]]= track;
	}
	typedef map<string,Track::Vertex*> VMap;
	VMap vMap;
	Track::Vertex* pv= NULL;
	Track::Edge* pe= NULL;
	Track::EdgeList splines;
	int pend;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"translate") == 0) {
				track->translate(getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10));
			} else if (strcasecmp(cmd,"rotate") == 0) {
				track->rotate(getDouble(1,-360,360));
			} else if (strcasecmp(cmd,"size") == 0) {
				;
			} else if (strcasecmp(cmd,"shape") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				TrackShapeMap::iterator i=
				  trackShapeMap.find(tokens[1]);
				if (i == trackShapeMap.end())
					throw "cannot find shape";
				track->shape= i->second;
			} else if (strcasecmp(cmd,"v") == 0) {
				double x,y,z;
				x= getDouble(2,-1e10,1e10);
				y= getDouble(3,-1e10,1e10);
				z= getDouble(4,-1e10,1e10);
				Track::Vertex* v= NULL;
				v= track->addVertex(Track::VT_SIMPLE,x,y,z);
				vMap[tokens[1]]= v;
			} else if (strcasecmp(cmd,"e") == 0) {
				VMap::iterator j= vMap.find(tokens[1]);
				if (j == vMap.end())
					throw "cannot find vertex 1";
				int n1= getInt(2,0,2);
				VMap::iterator k= vMap.find(tokens[3]);
				if (k == vMap.end())
					throw "cannot find vertex 2";
				int n2= getInt(4,0,2);
				float radius= getDouble(5,0,1e5,0);
				float angle= getDouble(6,-M_PI,M_PI,0);
				Track::Edge* e= track->addEdge(
				  radius<=0?Track::ET_STRAIGHT:Track::ET_SPLINE,
				  j->second,n1,k->second,n2);
				if (radius > 0)
					((Track::SplineEdge*)e)->setCircle(
					  radius,angle);
			} else if (strcasecmp(cmd,"sw") == 0) {
				double x,y,z;
				x= getDouble(2,-1e10,1e10);
				y= getDouble(3,-1e10,1e10);
				z= getDouble(4,-1e10,1e10);
				Track::Vertex* v= NULL;
				v= track->addVertex(Track::VT_SWITCH,x,y,z);
				vMap[tokens[1]]= v;
			} else {
				int n= tokens.size()-1;
				int i= 1;
				string label;
				if (cmd[strlen(cmd)-1] == ':') {
					label= string(cmd,strlen(cmd)-1);
					i++;
					n--;
					cmd= tokens[1].c_str();
				}
				Track::Vertex* v= NULL;
				double x,y,z;
				int end= -1;
				if (n < 1)
					throw "not enough fields";
				if (n <= 2) {
					VMap::iterator j=
					  vMap.find(tokens[i]);
					if (j == vMap.end())
						throw
						  "cannot find named vertex";
					v= j->second;
					if (n == 2)
						end= swEnd(tokens[i+1]);
				} else if (n>=5 && tokens[i+1]=="+") {
					VMap::iterator j=
					  vMap.find(tokens[i]);
					if (j == vMap.end())
						throw
						  "cannot find named vertex";
					x= j->second->location.coord[0] +
					  getDouble(i+2,-1e10,1e10);
					y= j->second->location.coord[1] +
					  getDouble(i+3,-1e10,1e10);
					z= j->second->location.coord[2] +
					  getDouble(i+4,-1e10,1e10);
					if (n > 5)
						end= swEnd(tokens[i+5]);
				} else if (n>=4 && tokens[i]=="+") {
					if (pv == NULL)
						throw "no previous location";
					x= pv->location.coord[0] +
					  getDouble(i+1,-1e10,1e10);
					y= pv->location.coord[1] +
					  getDouble(i+2,-1e10,1e10);
					z= pv->location.coord[2] +
					  getDouble(i+3,-1e10,1e10);
					if (n > 4)
						end= swEnd(tokens[i+4]);
				} else {
					x= getDouble(i,-1e10,1e10);
					y= getDouble(i+1,-1e10,1e10);
					z= getDouble(i+2,-1e10,1e10);
					if (n > 3)
						end= swEnd(tokens[i+3]);
				}
				if (v == NULL)
					v= track->addVertex(end<0?
					  Track::VT_SIMPLE:Track::VT_SWITCH,
					  x,y,z);
				if (end < 0)
					end= 0;
				if (strcasecmp(cmd,"move") == 0) {
					pend= end;
					if (splines.size() > 0) {
						track->calcSplines(splines,
						  pe,NULL);
						pe= NULL;
					}
				} else if (strcasecmp(cmd,"strt") == 0) {
					Track::Edge* e= track->addEdge(
					  Track::ET_STRAIGHT,pv,pend,v,end);
					pend= end==1 ? 0 : 1;
					if (splines.size() > 0)
						track->calcSplines(splines,
						  pe,e);
					pe= e;
				} else if (strcasecmp(cmd,"curv") == 0) {
					Track::Edge* e=
					  track->addCurve(pv,pend,v,end);
					pend= end==1 ? 0 : 1;
					if (splines.size() > 0)
						track->calcSplines(splines,
						  pe,e);
					pe= e;
				} else if (strcasecmp(cmd,"spln") == 0) {
					Track::Edge* e= track->addEdge(
					  Track::ET_SPLINE,pv,pend,v,end);
					pend= end==1 ? 0 : 1;
					splines.push_back(e);
				} else {
					throw std::invalid_argument(
					  "unknown command");
				}
				if (label.size() > 0)
					vMap[label]= v;
				pv= v;
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	if (splines.size() > 0)
		track->calcSplines(splines,pe,NULL);
	if (vMap.size() > 0) {
		track->makeSwitchCurves();
		track->makeSSEdges();
		track->calcGrades();
	}
}

void RMParser::parseRailCarDef(RailCarDef* def)
{
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"axles") == 0) {
				def->axles= getInt(1,2,100);
			} else if (strcasecmp(cmd,"mass")==0 ||
			  strcasecmp(cmd,"mass1")==0) {
				def->mass0= 1e3*getDouble(1,.1,10000);
				def->mass1= 1e3*getDouble(2,.1,10000,
				  def->mass0);
			} else if (strcasecmp(cmd,"rmass")==0 ||
			  strcasecmp(cmd,"mass2")==0) {
				def->mass2= 1e3*getDouble(1,.1,10000);
			} else if (strcasecmp(cmd,"drag0a") == 0) {
				def->drag0a= getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"drag0b") == 0) {
				def->drag0b= getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"drag1") == 0) {
				def->drag1= getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"drag2") == 0) {
				def->drag2= getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"drag2a") == 0) {
				def->drag2a= getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"area") == 0) {
				def->area= getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"length") == 0) {
				def->length= getDouble(1,.1,1000);
			} else if (strcasecmp(cmd,"offset") == 0) {
				def->offset= getDouble(1,-def->length,
				  def->length);
			} else if (strcasecmp(cmd,"width") == 0) {
				def->width= getDouble(1,.1,1000);
			} else if (strcasecmp(cmd,"maxbforce") == 0) {
				def->maxBForce= 1e3*getDouble(1,0,10000);
			} else if (strcasecmp(cmd,"steamengine") == 0) {
				if (def->engine == NULL) {
					SteamEngine* e= new SteamEngine;
					def->engine= e;
				}
				parseBlock((CommandBlockHandler*)
				  (SteamEngine*)def->engine);
			} else if (strcasecmp(cmd,"dieselengine") == 0) {
				if (def->engine == NULL) {
					DieselEngine* e= new DieselEngine;
					def->engine= e;
				}
				parseBlock((CommandBlockHandler*)
				  (DieselEngine*)def->engine);
			} else if (strcasecmp(cmd,"electricengine") == 0) {
				if (def->engine == NULL) {
					ElectricEngine* e= new ElectricEngine;
					def->engine= e;
				}
				parseBlock((CommandBlockHandler*)
				  (ElectricEngine*)def->engine);
			} else if (strcasecmp(cmd,"part") == 0) {
				def->parts.push_back(
				  RailCarPart(getInt(1,-1,100),
				    getDouble(2,-10000,10000),
				    getDouble(3,-100,100)));
			} else if (strcasecmp(cmd,"insertpart") == 0) {
				int i= getInt(1,0,100);
				int sz= def->parts.size();
				def->parts.push_back(RailCarPart(-1,0,0));
				for (int j=sz-1; j>=0; j--) {
					RailCarPart& p= def->parts[j];
					if (p.parent >= i)
						p.parent++;
					if (j >= i)
						def->parts[j+1]= p;
				}
				def->parts[i]= RailCarPart(getInt(2,-1,100),
				    getDouble(3,-10000,10000),
				    getDouble(4,-100,100));
				if (i < def->axles)
					def->axles++;
			} else if (strcasecmp(cmd,"smoke") == 0) {
				RailCarSmoke smoke;
				smoke.position=
				  osg::Vec3f(getDouble(1,-100,100),
				  getDouble(2,-100,100),getDouble(3,-100,100));
				smoke.normal=
				  osg::Vec3f(getDouble(4,-100,100,0),
				  getDouble(5,-100,100,0),
				  getDouble(6,-100,100,1));
				smoke.size= getDouble(7,.001,1,.1);
				smoke.minRate= getDouble(8,0,100,20);
				smoke.maxRate= getDouble(9,0,1000,200);
				smoke.minSpeed= getDouble(10,0,100,1);
				smoke.maxSpeed= getDouble(11,0,100,5);
				def->smoke.push_back(smoke);
			} else if (strcasecmp(cmd,"model") == 0) {
				int i= getInt(1,0,def->parts.size()-1);
				if (def->parts[i].model) {
					osg::Node* model=
					  find3DModel(tokens[2]);
					osg::MatrixTransform* mt=
					  (osg::MatrixTransform*)
					  def->parts[i].model;
					mt->addChild(model);
					fprintf(stderr,"add to part %d %p %p\n",
					  i,model,mt);
				} else {
					def->parts[i].model=
					  find3DModel(tokens[2]);
				}
			} else if (strcasecmp(cmd,"copy") == 0) {
				RailCarDefMap::iterator i=
				  railCarDefMap.find(tokens[1]);
				if (i == railCarDefMap.end())
					throw "cannot find car definition";
				def->copy(i->second);
			} else if (strcasecmp(cmd,"copywheels") == 0) {
				RailCarDefMap::iterator i=
				  railCarDefMap.find(tokens[1]);
				if (i == railCarDefMap.end())
					throw "cannot find car definition";
				def->copyWheels(i->second);
			} else if (strcasecmp(cmd,"copypart") == 0) {
				int i= getInt(1,0,def->parts.size()-1);
				printf("copypart %s part %d\n",
				  def->name.c_str(),i);
				osg::PositionAttitudeTransform* pat=
				  new osg::PositionAttitudeTransform;
				pat->setPosition(
				  osg::Vec3(getDouble(3,-1e10,1e10),
				    getDouble(4,-1e10,1e10),
				    getDouble(5,-1e10,1e10)));
				pat->setAttitude(
				  osg::Quat(getDouble(6,0,360,0)*M_PI/180,
				  osg::Vec3(0,1,0)));
				CopyVisitor cv(tokens[2],pat);
				def->parts[i].model->accept(cv);
				osg::MatrixTransform* mt=
				  (osg::MatrixTransform*) def->parts[i].model;
				mt->addChild(pat);
			} else if (strcasecmp(cmd,"mstsshape") == 0) {
				MSTSShape shape;
				shape.readFile(makePath());
				if (tokens.size() > 2)
					shape.printSubobjects();
				shape.createRailCar(def,tokens.size()>2);
			} else if (strcasecmp(cmd,"print") == 0) {
				int i= getInt(1,0,def->parts.size()-1);
				printf("%s part %d\n",def->name.c_str(),i);
				PrintVisitor pv;
				def->parts[i].model->accept(pv);
			} else if (strcasecmp(cmd,"printparts") == 0) {
				for (int i=0; i<def->parts.size(); i++) {
					fprintf(stderr,
					  "part %d %d %f %f %p\n",i,
					  def->parts[i].parent,
					  def->parts[i].xoffset,
					  def->parts[i].zoffset,
					  def->parts[i].model);
				}
			} else if (strcasecmp(cmd,"remove") == 0) {
				int i= getInt(1,0,def->parts.size()-1);
//				printf("remove %s part %d\n",
//				  def->name.c_str(),i);
				RemoveVisitor rv(tokens[2],getInt(3,0,1000));
				def->parts[i].model->accept(rv);
			} else if (strcasecmp(cmd,"inside") == 0) {
				osg::Image* img= NULL;
				if (tokens.size() > 6)
					img= osgDB::readImageFile(tokens[6]);
				def->inside.push_back(RailCarInside(
				  getDouble(1,-100,100),getDouble(2,-100,100),
				  getDouble(3,-100,100),
				  getDouble(4,-180,180,0),
				  getDouble(5,-180,180,0),img));
				fprintf(stderr,"inside img %p\n",img);
			} else if (strcasecmp(cmd,"sound") == 0) {
				def->soundFile= tokens[1];
				def->soundGain= getDouble(2,0,1,1);
			} else if (strcasecmp(cmd,"brakevalve") == 0) {
				def->brakeValve= tokens[1];
			} else if (strcasecmp(cmd,"headlight") == 0) {
				def->headlights.push_back(HeadLight(
				  getDouble(1,-100,100),getDouble(2,-100,100),
				  getDouble(3,-100,100),
				  getDouble(4,0,1,0),
				  getInt(5,0,3,2),
				  getInt(6,0x80000000,0x7fffffff,0xffffffff)));
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
}

void printTree(osg::Node* node, int depth);

void RMParser::parseModel3D()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	string name= tokens[1];
	osg::Node* model= NULL;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"file") == 0) {
				model= osgDB::readNodeFile(makePath());
				fprintf(stderr,"model=%s %p\n",
				  tokens[1].c_str(),model);
				//printTree(model,5);
			} else if (strcasecmp(cmd,"box") == 0) {
				osg::Geode* geode= new osg::Geode;
				osg::Box* box= new osg::Box(osg::Vec3d(0,0,0),
				  getDouble(1,0,1000),getDouble(2,0,1000),
				  getDouble(3,0,1000));
				osg::ShapeDrawable* sd=
				  new osg::ShapeDrawable(box);
				sd->setColor(osg::Vec4d(getDouble(4,0,1),
				  getDouble(5,0,1),getDouble(6,0,1),
				  getDouble(7,0,1,1)));
				geode->addDrawable(sd);
				model= geode;
			} else if (strcasecmp(cmd,"mstsshape") == 0) {
				MSTSShape shape;
				shape.readFile(makePath());
				model= shape.createModel(1,10,tokens.size()>2);
			} else if (strcasecmp(cmd,"save") == 0) {
				osgDB::writeNodeFile(*model,makePath());
			} else if (strcasecmp(cmd,"matrix") == 0) {
				if (model == NULL)
					throw "no model to transform";
				osg::MatrixTransform* t1=
				  new osg::MatrixTransform;
				t1->setMatrix(osg::Matrixd(
				  getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  getDouble(4,-1e10,1e10),
				  getDouble(5,-1e10,1e10),
				  getDouble(6,-1e10,1e10),
				  getDouble(7,-1e10,1e10),
				  getDouble(8,-1e10,1e10),
				  getDouble(9,-1e10,1e10),
				  getDouble(10,-1e10,1e10),
				  getDouble(11,-1e10,1e10),
				  getDouble(12,-1e10,1e10),
				  getDouble(13,-1e10,1e10),
				  getDouble(14,-1e10,1e10),
				  getDouble(15,-1e10,1e10),
				  getDouble(16,-1e10,1e10)));
				t1->addChild(model);
				model= t1;
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	if (model != NULL)
		modelMap[name]= model;
}

void RMParser::parsePersonPath()
{
	currentPerson.path.clear();
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			currentPerson.path.push_back(osg::Vec3d(
			  getDouble(0,-1e10,1e10),
			  getDouble(1,-1e10,1e10),
			  getDouble(2,-1e10,1e10)-1.7));
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	currentPerson.setLocation(currentPerson.path[0]);
	Person::stack.push_back(currentPerson);
	Person::stackIndex= Person::stack.size()-1;
	if (currentPerson.path.size() > 1) {
		osg::Vec3f dir= currentPerson.path[1]-currentPerson.path[0];
		dir.normalize();
		currentPerson.setAngle(atan2(dir[1],dir[0])*180/3.14159);
	}
}

osg::Node* RMParser::find3DModel(string& s)
{
	ModelMap::iterator i= modelMap.find(s);
	if (i == modelMap.end())
		throw "cannot find model";
	return i->second;
}

Ship* RMParser::findShip(string& s)
{
	ShipMap::iterator i= shipMap.find(s);
	if (i == shipMap.end())
		throw "cannot find ship";
	return i->second;
}

Track* RMParser::findTrack(string& s)
{
	TrackMap::iterator i= trackMap.find(s);
	if (i == trackMap.end())
		throw "cannot find ship";
	return i->second;
}

void RMParser::parseTrain()
{
	Train* train= new Train;
	if (tokens.size() >= 2) {
		train->name= tokens[1];
		trainMap[tokens[1]]= train;
	}
	trainList.push_back(train);
	float initAux= 50;
	float initCyl= 50;
	float initEqRes= 0;
	float maxEqRes= 70;
	string brakeValve;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"car") == 0) {
				RailCarDefMap::iterator i=
				  railCarDefMap.find(tokens[1]);
				if (i == railCarDefMap.end())
					throw "cannot find car definition";
				RailCarInst* car= new
				  RailCarInst(i->second,rootNode,maxEqRes,
				  brakeValve);
				car->setLoad(getDouble(2,0,1,0));
				car->prev= train->lastCar;
				if (train->lastCar == NULL)
					train->firstCar= car;
				else
					train->lastCar->next= car;
				train->lastCar= car;
				car->rev= getInt(3,0,1,0);
			} else if (strcasecmp(cmd,"wag") == 0) {
				if (tokens.size() < 3)
					throw "dir and file expected";
				RailCarDef* def= readMSTSWag(
				  tokens[1].c_str(),tokens[2].c_str());
				if (def == NULL)
					throw "cannot find car definition";
				RailCarInst* car=
				  new RailCarInst(def,rootNode,maxEqRes,
				  brakeValve);
				car->setLoad(0);
				car->prev= train->lastCar;
				if (train->lastCar == NULL)
					train->firstCar= car;
				else
					train->lastCar->next= car;
				train->lastCar= car;
				car->rev= getInt(3,0,1,0);
			} else if (strcasecmp(cmd,"waybill") == 0) {
				if (train->lastCar == NULL)
					throw "no car for waybill";
				if (1-getDouble(1,0,1) > drand48())
					continue;
				train->lastCar->addWaybill(tokens[5],
				  getDouble(2,0,1),getDouble(3,0,1),
				  getDouble(4,0,1),
				  getInt(6,0,100000,100));
			} else if (strcasecmp(cmd,"xyz") == 0) {
				findTrackLocation(getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  &train->location);
				train->location.rev= 0;
			} else if (strcasecmp(cmd,"txyz") == 0) {
				if (tokens.size() < 2)
					throw "track name missing";
				TrackMap::iterator i= trackMap.find(tokens[1]);
				if (i == trackMap.end())
					throw "cannot find track";
				i->second->findLocation(getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10),
				  getDouble(4,-1e10,1e10),
				  &train->location);
				train->location.rev= 0;
			} else if (strcasecmp(cmd,"loc") == 0) {
				if (tokens.size() < 2)
					throw "location name missing";
				bool found= false;
				for (TrackMap::iterator i=trackMap.begin();
				  i!=trackMap.end(); ++i) {
					if (i->second->findLocation(tokens[1],
					  &train->location)) {
						found= true;
						break;
					}
				}
				if (!found)
					throw "cannot find location";
			} else if (strcasecmp(cmd,"reverse") == 0) {
				train->location.rev= 1;
			} else if (strcasecmp(cmd,"solid") == 0) {
				train->modelCouplerSlack= 0;
				train->bControl= 1;
			} else if (strcasecmp(cmd,"speed") == 0) {
				train->modelCouplerSlack= 0;
				train->targetSpeed= getDouble(1,-100,100);
			} else if (strcasecmp(cmd,"decelmult") == 0) {
				train->decelMult= getDouble(1,.01,1);
			} else if (strcasecmp(cmd,"accelmult") == 0) {
				train->accelMult= getDouble(1,.01,1);
			} else if (strcasecmp(cmd,"move") == 0) {
				train->location.move(
				  getDouble(1,-1e10,1e10),0,0);
			} else if (strcasecmp(cmd,"pick") == 0) {
				if (!isClient && !readingSave)
				train->selectRandomCars(getInt(1,1,100),
				  getInt(4,0,100,0),getInt(2,0,100,0),
				  getInt(3,0,100,0));
			} else if (strcasecmp(cmd,"brakes") == 0) {
				maxEqRes= getDouble(1,0,110);
				initEqRes= getDouble(2,0,110);
				initAux= getDouble(3,0,110);
				initCyl= getDouble(4,0,110);
				if (tokens.size() > 5)
					brakeValve= tokens[5];
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	float len= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next)
		len+= car->def->length;
	train->location.move(len/2,1,0);
	train->location.edge->occupied++;
	train->endLocation= train->location;
	train->endLocation.move(-len,1,-1);
	train->connectAirHoses();
	if (train->engAirBrake != NULL)
		train->engAirBrake->setEqResPressure(initEqRes);
	if (initCyl > initAux)
		initCyl= initAux;
	float x= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next) {
		car->setLocation(x-car->def->length/2,&train->location);
		x-= car->def->length;
		if (car->airBrake != NULL) {
			car->airBrake->setCylPressure(initCyl);
			car->airBrake->setAuxResPressure(initAux);
			car->airBrake->setPipePressure(initEqRes);
			if (initEqRes > 0)
				car->airBrake->setEmergResPressure(maxEqRes);
			else
				car->airBrake->setEmergResPressure(initEqRes);
		}
		if (initCyl == 0)
			car->handBControl= 1;
	}
	train->calcPerf();
	WLocation loc;
	train->location.getWLocation(&loc);
	fprintf(stderr,"train %s at %lf %lf %f\n",
	  train->name.c_str(),loc.coord[0],loc.coord[1],loc.coord[2]);
}

void RMParser::parseTrackShape()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	TrackShape* shape= new TrackShape();
	trackShapeMap[tokens[1]]= shape;
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"offset") == 0) {
				shape->offsets.push_back(
				  TrackShape::Offset(getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10)));
			} else if (strcasecmp(cmd,"surface") == 0) {
				shape->surfaces.push_back(
				  TrackShape::Surface(
				  getInt(1,0,shape->offsets.size()-1),
				  getInt(2,0,shape->offsets.size()-1),
				  getDouble(3,0,1,0),
				  getDouble(4,0,1,0),
				  getDouble(5,0,100,1),
				  getInt(6,0,100,0)));
			} else if (strcasecmp(cmd,"texture") == 0) {
				if (tokens.size() < 2)
					throw "name missing";
				TextureMap::iterator i=
				  textureMap.find(tokens[1]);
				if (i == textureMap.end())
					throw "cannot find texture";
				shape->texture= i->second;
			} else {
				throw std::invalid_argument("unknown command");
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	shape->matchOffsets();
}

extern Train* selectedTrain;
extern RailCarInst* selectedRailCar;

void RMParser::parseSave()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("name missing");
	saveString= tokens[1];
	for (int i=2; i<tokens.size(); i++) {
		symbols.insert(tokens[i]);
		saveString+= " ";
		saveString+= tokens[i];
	}
	FileInfo* fi= fileStack.back();
	fileStack.pop_back();
	readingSave= true;
	parseFile(tokens[1].c_str());
	fileStack.push_back(fi);
	while (getCommand()) {
		try {
			const char* cmd= tokens[0].c_str();
			if (strcasecmp(cmd,"end") == 0)
				break;
			if (strcasecmp(cmd,"time") == 0) {
				simTime= getDouble(1,-1e10,1e10);
			} else if (strcasecmp(cmd,"randomselection") == 0) {
				Train* t=
				  Train::findTrain(getInt(1,0,0x7fffffff));
				if (t == NULL)
					throw std::invalid_argument(
					  "cannot find train");
				int n= getInt(2,0,1000);
				int nEng= getInt(3,0,1000);
				int nCab= getInt(4,0,1000);
				std::list<int> carList;
				for (int i=5; i<getNumTokens(); i++)
					carList.push_back(getInt(i,0,1000));
				ChangeLog::instance()->addRandomSelection(
				  t,nEng,nCab,carList);
				t->clearOccupied();
				t->selectCars(nEng,nCab,carList);
				t->positionCars();
				t->setOccupied();
				t->connectAirHoses();
			} else if (strcasecmp(cmd,"railfan") == 0 ||
			  strcasecmp(cmd,"person") == 0) {
				osg::Vec3d loc= osg::Vec3d(
				  getDouble(1,-1e10,1e10),
				  getDouble(2,-1e10,1e10),
				  getDouble(3,-1e10,1e10));
				currentPerson.location= loc;//prevent move to track
				currentPerson.setLocation(loc);
				currentPerson.setAngle(getDouble(4,-1e10,1e10));
				currentPerson.setVAngle(getDouble(5,-1e10,1e10));
				currentPerson.setHeight(getDouble(6,-1e10,1e10));
				if (getNumTokens() >= 13) {
					currentPerson.setFollow(
					  getInt(7,0,0x7fffffff),
					  getInt(8,0,0x7fffffff),
					  getInt(9,0,0x7fffffff),
					  getDouble(10,-1e10,1e10),
					  getDouble(11,-1e10,1e10),
					  getDouble(12,-1e10,1e10));
				}
				Person::stack.push_back(currentPerson);
			} else if (strcasecmp(cmd,"railfanindex") == 0 ||
			  strcasecmp(cmd,"personindex") == 0) {
				Person::stackIndex= getInt(1,0,5);
				currentPerson= Person::stack[Person::stackIndex];
				selectedTrain= currentPerson.train;
				selectedRailCar= currentPerson.railCar;
			} else if (strcasecmp(cmd,"train") == 0) {
				Train* t=
				  Train::findTrain(getInt(1,0,0x7fffffff));
				if (t) {
					t->clearOccupied();
					findTrackLocation(
					  getDouble(2,-1e10,1e10),
					  getDouble(3,-1e10,1e10),
					  getDouble(4,-1e10,1e10),
					  &t->location);
					t->location.rev= getInt(5,0,1);
					t->positionCars();
					t->setOccupied();
					t->speed= getDouble(6,-1e10,1e10);
					t->dControl= getDouble(7,-1,1);
					t->tControl= getDouble(8,0,1);
					t->bControl= getDouble(9,-1,1);
					t->engBControl= getDouble(10,0,1);
					if (t->bControl<0 &&
					  t->modelCouplerSlack) {
						t->convertToAirBrakes();
						t->bControl= getDouble(9,-1,1);
					}
					for (RailCarInst* car=t->firstCar;
					  car!=NULL; car=car->next)
						car->speed= t->speed;
				}
			} else if (strcasecmp(cmd,"throw") == 0) {
				Track* track= trackMap.begin()->second;
				Track::SwitchMap::iterator i=
				  track->switchMap.find(getInt(1,0,0x7fffffff));
				if (i == track->switchMap.end())
					throw std::invalid_argument(
					  "cannot find switch");
				Track::SwVertex* sw= i->second;
				int j= getInt(2,0,1);
				if (sw->edge2 != sw->swEdges[j])
					sw->throwSwitch(sw->swEdges[j],true);
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	readingSave= false;
}
