//	data for rail-marine bridge between land and carfloat
//	derived from ship for collision detection purposes
//
//Copyright 2009 Doug Jones
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
