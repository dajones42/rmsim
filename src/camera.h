//	Camera manipulators

#ifndef CAMERA_H
#define CAMERA_H

//	camera manipulator for looking down at the person (map view)
struct MapManipulator : public osgGA::CameraManipulator {
	osg::Vec3d offset;
	float distance;
	MapManipulator() {
		distance= 20000;
		offset= osg::Vec3d(0,0,0);
	};
	~MapManipulator() {
	};
	void init(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix(const osg::Matrixd& m);
	virtual void setByInverseMatrix(const osg::Matrixd& m);
	void viewAllTrack();
};

//	camera manipulator for looking at the person
struct LookAtManipulator : public osgGA::CameraManipulator {
	float distance;
	int angle;
	float cosAngle;
	float sinAngle;
	int vAngle;
	float cosv;
	float sinv;
	void setAngle(int a) {
		if (a < 0)
			a+= 360;
		if (a > 360)
			a-= 360;
		angle= a;
		cosAngle= cos(a*3.14159/180);
		sinAngle= sin(a*3.14159/180);
		//currentPerson.setAngle(cosAngle,sinAngle);
	};
	void incAngle(int d) { setAngle(angle+d); }
	void setVAngle(int a) {
		if (a < 0)
			a+= 360;
		if (a > 360)
			a-= 360;
		vAngle= a;
		cosv= cos(a*3.14159/180);
		sinv= sin(a*3.14159/180);
	};
	void incVAngle(int d) { setVAngle(vAngle+d); }
	LookAtManipulator() {
		distance= 100;
		setAngle(0);
		setVAngle(10);
	};
	~LookAtManipulator() {
	};
	void init(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix(const osg::Matrixd& m);
	virtual void setByInverseMatrix(const osg::Matrixd& m);
};

//	camera manipulator for looking from the person
struct LookFromManipulator : public osgGA::CameraManipulator {
	LookFromManipulator() {
	};
	~LookFromManipulator() {
	};
	void init(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	bool handle(const osgGA::GUIEventAdapter& ea,
	  osgGA::GUIActionAdapter& aa);
	virtual osg::Matrixd getMatrix() const;
	virtual osg::Matrixd getInverseMatrix() const;
	virtual void setByMatrix(const osg::Matrixd& m);
	virtual void setByInverseMatrix(const osg::Matrixd& m);
};

#endif
