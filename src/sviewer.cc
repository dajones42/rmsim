//	simple program to view osg readable 3D models
//
/*
Copyright © 2024 Doug Jones

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
#include <stdio.h>
#include <stdlib.h>

using namespace std;
#include <map>
#include <osg/Group>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osg/Texture2D>
#include <osgGA/TrackballManipulator>
#include <osg/NodeVisitor>
#include <osgShadow/ShadowedScene>
#include <osgShadow/ShadowMap>
#include <osg/PolygonMode>
#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>

#include "rmsim.h"
#include "mstsshape.h"
#include "shaders.h"

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
		printf("node %d\n",level);
		printf("%s%s\n",spaces().c_str(),node.getName().c_str());
		traverse(node);
		level--;
	}
	virtual void apply(osg::Geode& geode) {
		printf("geode %d %d\n",level,geode.getNumDrawables());
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
			fprintf(stderr,"removed %s %d\n",name.c_str(),drawable);
		}
	}
};

int main(int argc, char** argv)
{
	bool top= false;
	try {
		if (argc < 1) {
			fprintf(stderr,"usage: %s file\n",argv[0]);
			exit(1);
		}
		bool wire= false;
		bool shader= false;
		float roughness= 0;
		while (argc>2 && argv[1][0]=='-') {
			switch (argv[1][1]) {
			 case 'w':
				wire= true;
				break;
			 case 's':
				shader= true;
				break;
			 case 'r':
				shader= true;
				roughness= atof(argv[1]+2);
				break;
			 case 'a':
				MSTSShape::useAO= true;
				break;
			 default:
				fprintf(stderr,"unknown option %s\n",argv[1]);
			}
			argc--;
			argv++;
		}
		osg::Group* rootNode= new osg::Group;
		osg::Node* model= osgDB::readNodeFile(argv[1]);
//		PrintVisitor pv;
//		model->accept(pv);
		rootNode->addChild(model);
		osg::StateSet* stateSet= rootNode->getOrCreateStateSet();
		if (wire) {
			osg::PolygonMode* pm= new osg::PolygonMode;
			pm->setMode(osg::PolygonMode::FRONT_AND_BACK,
			  osg::PolygonMode::LINE);
			stateSet->setAttribute(pm);
		}
		if (shader) {
			addShaders(stateSet,roughness);
		}
		osg::LightSource* lightSource= new osg::LightSource;
		osg::Light* light= lightSource->getLight();
		fprintf(stderr,"light %d\n",light->getLightNum());
//		light->setAmbient(osg::Vec4f(.4,.4,.4,1));
//		light->setDiffuse(osg::Vec4f(.9,.9,.9,1));
		light->setAmbient(osg::Vec4f(1,1,1,1));
		light->setDiffuse(osg::Vec4f(1,1,1,1));
		light->setSpecular(osg::Vec4f(1,1,1,1));
		light->setPosition(osg::Vec4f(-5,-5,5,0));
//		lightSource->getOrCreateStateSet()->setMode(GL_LIGHTING,
//		  osg::StateAttribute::ON);
		rootNode->addChild(lightSource);
#if 0
		osgShadow::ShadowedScene* ss= new osgShadow::ShadowedScene;
		osgShadow::ShadowMap* sm= new osgShadow::ShadowMap;
		sm->setLight(lightSource);
		sm->setTextureSize(osg::Vec2s(1024,1024));
		sm->setTextureUnit(1);
		ss->setShadowTechnique(sm);
		ss->setReceivesShadowTraversalMask(0x1);
		ss->setCastsShadowTraversalMask(0x2);
		ss->addChild(rootNode);
		model->setNodeMask(0x3);
		rootNode= ss;
#endif
		argc--;
		argv++;
		osgViewer::Viewer viewer;
		viewer.setSceneData(rootNode);
		if (top) {
			osg::Vec3 up= osg::Vec3(0,1,0);
			osg::Vec3 dir= osg::Vec3(0,0,-1);
			osg::Vec3 center= model->getBound().center();
			double radius= model->getBound().radius();
			fprintf(stderr,"%f %f %f %f\n",
			  center[0],center[1],center[2],radius);
			osg::Camera* camera= viewer.getCamera();
			osg::Viewport* viewport= camera->getViewport();
			camera->setViewMatrixAsLookAt(
			  center-dir*3*8*radius,center,up);
			double left,right,bottom,top,znear,zfar;
			camera->getProjectionMatrixAsFrustum(
			  left,right,bottom,top,znear,zfar);
			fprintf(stderr,"%f %f %f %f %f %f\n",
			  left,right,bottom,top,znear,zfar);
			camera->setProjectionMatrixAsFrustum(
			  left/8,right/8,bottom/8,top/8,znear,zfar);
			osg::Matrixd m= camera->getProjectionMatrix();
			const double* mp= m.ptr();
			fprintf(stderr,"%f %f %f %f\n",
			  mp[0],mp[1],mp[2],mp[3]);
			fprintf(stderr,"%f %f %f %f\n",
			  mp[4],mp[5],mp[6],mp[7]);
			fprintf(stderr,"%f %f %f %f\n",
			  mp[8],mp[9],mp[10],mp[11]);
			fprintf(stderr,"%f %f %f %f\n",
			  mp[12],mp[13],mp[14],mp[15]);
			camera->setCullingMode(osg::CullSettings::NO_CULLING);
#if 0
			camera->setProjectionMatrixAsOrtho(
			  -radius,radius,-4*radius/3,4*radius/3,
			  -1000,1000);
			  //left,right,bottom,top);//,znear,zfar);
			m= camera->getProjectionMatrix();
			mp= m.ptr();
			fprintf(stderr,"%f %f %f %f\n",
			  mp[0],mp[1],mp[2],mp[3]);
			fprintf(stderr,"%f %f %f %f\n",
			  mp[4],mp[5],mp[6],mp[7]);
			fprintf(stderr,"%f %f %f %f\n",
			  mp[8],mp[9],mp[10],mp[11]);
			fprintf(stderr,"%f %f %f %f\n",
			  mp[12],mp[13],mp[14],mp[15]);
#endif
			while (!viewer.done())
				viewer.frame();
		} else {
			viewer.setCameraManipulator(
			  new osgGA::TrackballManipulator);
			viewer.run();
		}
	} catch (const exception& error) {
		fprintf(stderr,"error: %s\n",error.what());
		exit(1);
	}
	exit(0);
}
