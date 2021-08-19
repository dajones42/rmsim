#ifndef ROPEDRAWABLE_H
#define ROPEDRAWABLE_H

struct RopeDrawable : public osg::Drawable {
	RopeDrawable() {
	}
	~RopeDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new RopeDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new RopeDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const RopeDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "RopeDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
	virtual bool supports(const osg::PrimitiveFunctor& pf) const {
		return true;
	}
	virtual void accept(osg::PrimitiveFunctor& pf) const;
};

extern RopeDrawable* ropeDrawable;

#endif
