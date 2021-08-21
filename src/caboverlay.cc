//	cab overlay code
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

#include <stdio.h>

#include "caboverlay.h"
#include <osg/BlendFunc>

CabOverlay* CabOverlay::updateCallback= NULL;

osg::Camera* CabOverlay::create()
{
	osg::Camera* camera= new osg::Camera;
	camera->setProjectionMatrix(osg::Matrix::ortho2D(0,1024,0,768));
	camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
	camera->setViewMatrix(osg::Matrix::identity());
	camera->setClearMask(GL_DEPTH_BUFFER_BIT);
	camera->setRenderOrder(osg::Camera::POST_RENDER);
	camera->setAllowEventFocus(false);
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::DrawElementsUShort* drawElements=
	  new osg::DrawElementsUShort(GL_QUADS);;
	verts->push_back(osg::Vec3(0,0,0));
	verts->push_back(osg::Vec3(1024,0,0));
	verts->push_back(osg::Vec3(1024,768,0));
	verts->push_back(osg::Vec3(0,768,0));
	texCoords->push_back(osg::Vec2(0,1));
	texCoords->push_back(osg::Vec2(1,1));
	texCoords->push_back(osg::Vec2(1,0));
	texCoords->push_back(osg::Vec2(0,0));
	drawElements->push_back(3);
	drawElements->push_back(2);
	drawElements->push_back(1);
	drawElements->push_back(0);
	drawElements->push_back(0);
	drawElements->push_back(1);
	drawElements->push_back(2);
	drawElements->push_back(3);
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(1,1,1,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->addPrimitiveSet(drawElements);
	geometry->setTexCoordArray(0,texCoords);
	osg::StateSet* stateSet= geometry->getOrCreateStateSet();
	stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	osg::Texture2D* texture= new osg::Texture2D;
	texture->setWrap(osg::Texture2D::WRAP_S,
	  osg::Texture2D::CLAMP_TO_BORDER);
	texture->setWrap(osg::Texture2D::WRAP_T,
	  osg::Texture2D::CLAMP_TO_BORDER);
	stateSet->setTextureAttributeAndModes(0,texture,
	  osg::StateAttribute::ON);
	osg::BlendFunc* bf= new osg::BlendFunc();
	bf->setFunction(osg::BlendFunc::SRC_ALPHA,
	  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
	stateSet->setAttributeAndModes(bf,osg::StateAttribute::ON);
	osg::Geode* geode= new osg::Geode;
	geode->addDrawable(geometry);
	osg::Switch* swNode= new osg::Switch;
	swNode->addChild(geode);
	camera->addChild(swNode);
	updateCallback= new CabOverlay(swNode,texture);
	swNode->setUpdateCallback(updateCallback);
	swNode->setAllChildrenOff();
	fprintf(stderr,"cab camera %p\n",camera);
	return camera;
}

void CabOverlay::setImage(osg::Image* img)
{
	if (!updateCallback)
		return;
	updateCallback->newImage= img;
}

void CabOverlay::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
	if (image == newImage)
		return;
	image= newImage;
	if (image) {
		texture->setImage(image);
		swNode->setAllChildrenOn();
		fprintf(stderr,"overlay on\n");
	} else {
		swNode->setAllChildrenOff();
		fprintf(stderr,"overlay off\n");
	}
}
