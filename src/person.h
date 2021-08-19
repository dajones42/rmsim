//	data for user location within world
//	used for camera control
//
//Copyright 2009 Doug Jones
#ifndef PERSON_H
#define PERSON_H

struct Person {
	osg::Vec3d location;
	osg::Vec3d remoteLocation;
	osg::Vec3f aim;
	osg::Vec3f followOffset;
	float angle;
	float cosAngle;
	float sinAngle;
	float vAngle;
	float cosVAngle;
	float sinVAngle;
	float height;
	float sumDT;
	float speed;
	float prevDist;
	osg::MatrixTransform* follow;
	osg::MatrixTransform* model;
	osg::Switch* modelSwitch;
	Person* moveTo;
	bool setAimOnMove;
	bool useRemote;
	int insideIndex;
	osg::Image* insideImage;
	RailCarInst* railCar;
	Train* train;
	Ship* ship;
	std::vector<osg::Vec3d> path;
	void setLocation(osg::Vec3d loc);
	void setLocation(osg::Vec3d loc, osg::MatrixTransform* mt,
	  osg::Vec3f offset);
	void setMoveTo(osg::Vec3d loc, osg::MatrixTransform* mt,
	  osg::Vec3f offset, RailCarInst* railCar, Train* train, Ship* ship);
	void setFollow(int trainID, int carNum, int partNum,
	  float offsetX, float offsetY, float offsetZ);
	void stopMove();
	void jump();
	void stopFollowing(bool moveDown);
	void moveAlongTrain(Train* train, bool toRight);
	void moveAlongPath(bool toRight);
	Cleat* findCleat();
	void moveToNextCleat(bool toRight);
	void adjustRopes(int adj);
	void removeRopes();
	void connectFloatBridge();
	void setRemoteLocation();
	void setAngle(float a) {
		if (a < 0)
			a+= 360;
		if (a > 360)
			a-= 360;
		angle= a;
		cosAngle= cos(a*3.14159/180);
		sinAngle= sin(a*3.14159/180);
		insideImage= NULL;
		useRemote= false;
	};
	void incAngle(float d) { setAngle(angle+d); }
	void setVAngle(float a) {
		if (a < 0)
			a+= 360;
		if (a > 360)
			a-= 360;
		vAngle= a;
		cosVAngle= cos(a*3.14159/180);
		sinVAngle= sin(a*3.14159/180);
		insideImage= NULL;
		useRemote= false;
	};
	void incVAngle(float d) { setVAngle(vAngle+d); }
	void setHeight(float h) {
		location[2]-= height;
		height= h;
		location[2]+= height;
	};
	void incHeight(float dh) { setHeight(height+dh); }
	void move(float df, float ds);
	void reset() {
		setAngle(0);
		setVAngle(0);
		setHeight(1.7);
		insideImage= NULL;
		useRemote= false;
	};
	void setModelMatrix();
	void createModel(osg::Group* root);
	osg::Vec3d getLocation() { return location; };
	osg::Vec3f getAim() { return aim; };
	void updateLocation(double dt);
	void moveInside();
	void centerOverTrack();
	void zeroSumDT() { sumDT= 0; };
	bool getOnOrOff();
	float getOnOrOffDist();
	Person() {
		model= NULL;
		follow= NULL;
		angle= 0;
		cosAngle= 1;
		sinAngle= 0;
		vAngle= 0;
		cosVAngle= 1;
		sinVAngle= 0;
		height= 0;
		location= osg::Vec3d(0,0,0);
		moveTo= NULL;
		sumDT= 0;
		speed= 1;
		prevDist= 1e10;
		insideIndex= -1;
		insideImage= NULL;
		reset();
		useRemote= false;
	};
	static vector<Person> stack;
	static int stackIndex;
	static void swap(int newIndex);
	static void updateLocations(float dt);
};
extern Person currentPerson;

#endif
