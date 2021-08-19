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
