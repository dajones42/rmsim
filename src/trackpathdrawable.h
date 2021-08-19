#ifndef TRACKPATHDRAWABLE_H
#define TRACKPATHDRAWABLE_H

//	code to draw a line down the center of track
//	used when viewing from a large distance among other things
//	also draws platform marker if there is a timetable
//	and interlocking signal state information
struct TrackPathDrawable : public osg::Drawable {
	int draw;
	int drawAll;
	TrackPathDrawable() {
		draw= 0;
		drawAll= 1;
	}
	~TrackPathDrawable() {
	}
	virtual osg::Object* cloneType() const {
		return new TrackPathDrawable();
	}
	virtual osg::Object* clone(const osg::CopyOp& op) const {
		return new TrackPathDrawable();
	}
	virtual bool isSameKindAs(const osg::Object* obj) const {
		return dynamic_cast<const TrackPathDrawable*>(obj)!=NULL;
	}
	virtual const char* libraryName() const { return "rmsim"; }
	virtual const char* className() const { return "TrackPathDrawable"; }
	virtual void drawImplementation(osg::RenderInfo& renderInfo) const;
	virtual osg::BoundingSphere computeBound() const;
};

extern TrackPathDrawable* trackPathDrawable;

#endif
