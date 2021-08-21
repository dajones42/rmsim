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
