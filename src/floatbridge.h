//	data for rail-marine bridge between land and carfloat
//	derived from ship for collision detection purposes
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
#ifndef FLOATBRIDGE_H
#define FLOATBRIDGE_H

struct FloatBridge : public Ship {
	struct FBTrack {
		float offset;
		Track* track;
		TrackConn* conn;
		osg::MatrixTransform* model;
	};
	typedef std::list<FBTrack*> FBTrackList;
	WLocation pivot;
	float length;
	Ship* carFloat;
	float cfOffset;
	float cfZOffset;
	FBTrackList tracks;
	FloatBridge();
	~FloatBridge();
	void move();
	void connect();
	void disconnect();
	void addTrack(float offset, Track* track);
	void setupTwist();
};
typedef list<FloatBridge*> FBList;
extern FBList fbList;

void moveFloatBridges();

#endif
