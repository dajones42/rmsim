//	camera maipulators for rmsim

#include "rmsim.h"
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/CameraManipulator>
#include "camera.h"
#include "caboverlay.h"
#include "trackpathdrawable.h"

osg::Matrixd MapManipulator::getMatrix() const
{
	osg::Matrixd m;
	osg::Vec3d c= currentPerson.getLocation()+offset;
	m.makeLookAt(c+osg::Vec3d(0,0,distance),c,osg::Vec3d(0,1,0));
	return m;
}

osg::Matrixd MapManipulator::getInverseMatrix() const
{
	return getMatrix();
}

void MapManipulator::setByMatrix(const osg::Matrixd& m)
{
}

void MapManipulator::setByInverseMatrix(const osg::Matrixd& m)
{
}

void MapManipulator::init(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
//	fprintf(stderr,"mminit\n");
	camera->setComputeNearFarMode(
	  osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
	trackPathDrawable->drawAll= 0;
	if (distance > 1000)
		trackPathDrawable->drawAll= 1;
	CabOverlay::setImage(NULL);
}

bool MapManipulator::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled())
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		 case ' ':
			offset[0]= 0;
			offset[1]= 0;
			offset[2]= 0;
			distance= 20000;
			trackPathDrawable->drawAll= 1;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Up:
			offset[1]+= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Down:
			offset[1]-= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Home:
			offset[0]-= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_End:
			offset[0]+= distance/100;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Up:
			distance/= 2;
			if (distance < 1000)
				trackPathDrawable->drawAll= 0;
			return true;
		 case osgGA::GUIEventAdapter::KEY_Down:
			distance*= 2;
			if (distance > 1000)
				trackPathDrawable->drawAll= 1;
			return true;
		 default:
			break;
		}
		break;
	 default:
		break;
	}
	return false;
}

void MapManipulator::viewAllTrack()
{
	double minx= 1e20;
	double maxx= -1e20;
	double miny= 1e20;
	double maxy= -1e20;
	double minz= 1e20;
	double maxz= -1e20;
	for (TrackMap::iterator j=trackMap.begin(); j!=trackMap.end(); ++j) {
		Track* t= j->second;
		t->calcMinMax();
		if (minx > t->minVertexX)
			minx= t->minVertexX;
		if (maxx < t->maxVertexX)
			maxx= t->maxVertexX;
		if (miny > t->minVertexY)
			miny= t->minVertexY;
		if (maxy < t->maxVertexY)
			maxy= t->maxVertexY;
		if (minz > t->minVertexZ)
			minz= t->minVertexZ;
		if (maxz < t->maxVertexZ)
			maxz= t->maxVertexZ;
	}
	float x= 2*(maxx-minx);
	float y= 2*(maxy-miny);
	if (x>y && distance>x)
		distance= x;
	if (y>x && distance>y)
		distance= y;
}

osg::Matrixd LookAtManipulator::getMatrix() const
{
	osg::Matrixd m;
	osg::Vec3d c= currentPerson.getLocation();
	if (currentPerson.useRemote) {
		if ((c-currentPerson.remoteLocation).length() > 1000)
			currentPerson.setRemoteLocation();
		m.makeLookAt(currentPerson.remoteLocation,c,osg::Vec3d(0,0,1));
		return m;
	}
#if 0
	osg::Vec3d aim= osg::Vec3d(cosv*distance*cosAngle,
	  cosv*distance*sinAngle,distance*sinv);
	if (currentPerson.follow) {
		osg::Quat q= currentPerson.follow->getMatrix().getRotate();
		aim= q*aim;
	}
	m.makeLookAt(c+aim,c,osg::Vec3d(0,0,1));
#else
	osg::Vec3d ra= currentPerson.getAim();
	osg::Vec3d aim= osg::Vec3d(
#if 0
	  -cosv*distance*(cosAngle*ra[0]-sinAngle*ra[1]),
	  -cosv*distance*(cosAngle*ra[1]+sinAngle*ra[0]),distance*sinv);
#else
	  -cosv*distance*ra[0],-cosv*distance*ra[1],distance*sinv);
#endif
	m.makeLookAt(c+aim,c,osg::Vec3d(0,0,1));
#endif
	return m;
}

osg::Matrixd LookAtManipulator::getInverseMatrix() const
{
	return getMatrix();
//	osg::Matrixd m;
//	m.invert(getMatrix());
//	return m;
}

void LookAtManipulator::setByMatrix(const osg::Matrixd& m)
{
}

void LookAtManipulator::setByInverseMatrix(const osg::Matrixd& m)
{
}

void LookAtManipulator::init(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
//	fprintf(stderr,"lookatinit\n");
//	if (distance < 50) {
		double fovy,ar,zn,zf;
		camera->getProjectionMatrixAsPerspective(
		  fovy,ar,zn,zf);
		camera->setComputeNearFarMode(
		  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
		camera->setProjectionMatrixAsPerspective(
		  fovy,ar,.2,4000);
//	} else {
//		camera->setComputeNearFarMode(
//		  osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//	}
	trackPathDrawable->drawAll= 0;
	if (distance > 1000)
		trackPathDrawable->drawAll= 1;
	CabOverlay::setImage(NULL);
}

bool LookAtManipulator::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled())
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		 case ' ':
			currentPerson.reset();
			distance= 100;
			setAngle(0);
			setVAngle(10);
			trackPathDrawable->drawAll= 0;
			//camera->setComputeNearFarMode(osg::CullSettings::
			//  COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
			return true;
		 case '0':
			if (currentPerson.modelSwitch->getValue(0))
				currentPerson.modelSwitch->setAllChildrenOff();
			else
				currentPerson.modelSwitch->setAllChildrenOn();
			return true;
		 case osgGA::GUIEventAdapter::KEY_Up:
			distance/= 1.5;
			if (distance < 1000)
				trackPathDrawable->drawAll= 0;
//			if (distance<50 ){//&& distance*1.5>=50) {
//				double fovy,ar,zn,zf;
//				camera->getProjectionMatrixAsPerspective(
//				  fovy,ar,zn,zf);
//				camera->setComputeNearFarMode(
//				  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
//				camera->setProjectionMatrixAsPerspective(
//				  fovy,ar,.2,4000);
//			}
//			fprintf(stderr,"dist %f\n",distance);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Down:
			distance*= 1.5;
			if (distance > 1000)
				trackPathDrawable->drawAll= 1;
//			if (distance>=50 )//&& distance/1.5<50)
//				camera->setComputeNearFarMode(
//				  osg::CullSettings::
//				  COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//			fprintf(stderr,"dist %f\n",distance);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Left:
			currentPerson.incAngle(5);
			//incAngle(-5);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Right:
			currentPerson.incAngle(-5);
			//incAngle(5);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Down:
			incVAngle(-5);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Up:
			incVAngle(5);
			return true;
		 case '4':
			//currentPerson.moveInside();
			currentPerson.setRemoteLocation();
			return true;
		 default:
			break;
		}
		break;
	 case osgGA::GUIEventAdapter::SCROLL:
		if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_UP) {
#if 0
			currentPerson.incAngle(-5);
#else
			distance/= 1.5;
			if (distance < 1000)
				trackPathDrawable->drawAll= 0;
			if (distance<50 ){//&& distance*1.5>=50) {
				double fovy,ar,zn,zf;
				camera->getProjectionMatrixAsPerspective(
				  fovy,ar,zn,zf);
				camera->setComputeNearFarMode(
				  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
				camera->setProjectionMatrixAsPerspective(
				  fovy,ar,.2,4000);
			}
//			fprintf(stderr,"up dist %f\n",distance);
#endif
			return true;
		} else if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_DOWN) {
#if 0
			currentPerson.incAngle(5);
#else
			distance*= 1.5;
			if (distance > 1000)
				trackPathDrawable->drawAll= 1;
			if (distance>=50 )//&& distance/1.5<50)
				camera->setComputeNearFarMode(
				  osg::CullSettings::
				  COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
//			fprintf(stderr,"down dist %f\n",distance);
#endif
			return true;
		}
		break;
	 default:
		break;
	}
	return false;
}

osg::Matrixd LookFromManipulator::getMatrix() const
{
	osg::Matrixd m;
	m.makeLookAt(currentPerson.getLocation(),
	  currentPerson.getLocation()+currentPerson.getAim(),
	  osg::Vec3d(0,0,1));
	return m;
}

osg::Matrixd LookFromManipulator::getInverseMatrix() const
{
	return getMatrix();
}

void LookFromManipulator::setByMatrix(const osg::Matrixd& m)
{
}

void LookFromManipulator::setByInverseMatrix(const osg::Matrixd& m)
{
}

void LookFromManipulator::init(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	fprintf(stderr,"lookfrominit %f %f %f\n",currentPerson.getAim()[0],
	  currentPerson.getAim()[1],currentPerson.getAim()[2]);
	double fovy,ar,zn,zf;
	camera->getProjectionMatrixAsPerspective(fovy,ar,zn,zf);
	camera->setComputeNearFarMode(
	  osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	camera->setProjectionMatrixAsPerspective(fovy,ar,.2,4000);
	trackPathDrawable->drawAll= 0;
//	if (currentPerson.modelSwitch != NULL)
//		currentPerson.modelSwitch->setAllChildrenOff();
	CabOverlay::setImage(currentPerson.insideImage);
	fprintf(stderr,"insideimage %p\n",currentPerson.insideImage);
}

bool LookFromManipulator::handle(const osgGA::GUIEventAdapter& ea,
  osgGA::GUIActionAdapter& aa)
{
	if (ea.getHandled())
		return false;
	switch (ea.getEventType()) {
	 case osgGA::GUIEventAdapter::KEYDOWN:
		switch (ea.getKey()) {
		 case ' ':
			currentPerson.reset();
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Up:
			currentPerson.incVAngle(5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Down:
			currentPerson.incVAngle(-5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Left:
			currentPerson.incAngle(5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Right:
			currentPerson.incAngle(-5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Home:
			currentPerson.move(0,.5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_End:
			currentPerson.move(0,-.5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Up:
			currentPerson.incHeight(.5);
			CabOverlay::setImage(NULL);
			return true;
		 case osgGA::GUIEventAdapter::KEY_Page_Down:
			currentPerson.incHeight(-.5);
			CabOverlay::setImage(NULL);
			return true;
		 case '4':
			currentPerson.moveInside();
			CabOverlay::setImage(currentPerson.insideImage);
			fprintf(stderr,"lookfrom moveiside\n");
			return true;
		 default:
			break;
		}
		break;
	 case osgGA::GUIEventAdapter::SCROLL:
		if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_UP) {
			currentPerson.incAngle(-5);
			return true;
		} else if (ea.getScrollingMotion() ==
		  osgGA::GUIEventAdapter::SCROLL_DOWN) {
			currentPerson.incAngle(5);
			return true;
		}
		break;
	 default:
		break;
	}
	return false;
}
