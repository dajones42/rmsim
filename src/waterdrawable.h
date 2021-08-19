#ifndef WATERDRAWABLE_H
#define WATERDRAWABLE_H

struct WaterDrawable : public osg::Drawable {
	Water* water;
	int drawTriangles;
	WaterDrawable(Water* w) {
		water= w;
		drawTriangles= 0;
	}
	~WaterDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new WaterDrawable(water);
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new WaterDrawable(water);
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const WaterDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "WaterDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
};

#endif
