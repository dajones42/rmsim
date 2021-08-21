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
