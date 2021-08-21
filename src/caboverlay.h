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
#ifndef CABOVERLAY_H
#define CABOVERLAY_H

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/Switch>
#include <osg/Image>
#include <osg/Camera>
#include <osg/Texture2D>

class CabOverlay : public osg::NodeCallback {
	osg::Image* image;
	osg::Image* newImage;
	osg::Switch* swNode;
	osg::Texture2D* texture;
	CabOverlay(osg::Switch* sw, osg::Texture2D* t) {
		swNode= sw;
		texture= t;
		image= NULL;
		newImage= NULL;
	}
	static CabOverlay* updateCallback;
 public:
	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv);
	static osg::Camera* create();
	static void setImage(osg::Image* image);
};

#endif
