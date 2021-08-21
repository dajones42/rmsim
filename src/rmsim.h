//	main include file for rmsim (rail-marine)
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
#ifndef RMSIM_H
#define RMSIM_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#ifdef FREEGLUT_IS_PRESENT
#include <GL/freeglut.h>
#else
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#endif

#include "plib/pu.h"
#include "plib/sg.h"
#include "plib/ssg.h"
#include "plib/ssgAux.h"

using namespace std;
#include <string>
#include <list>
#include <map>
#include <set>
#include <vector>

#include <osg/Group>
#include <osg/Switch>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/Vec3>
#include <osg/Vec2>

#include "water.h"
#include "texture.h"
#include "model2d.h"
#include "trackshape.h"
#include "track.h"
#include "ship.h"
#include "floatbridge.h"
#include "geometry.h"
#include "railcar.h"
#include "locoeng.h"
#include "train.h"
#include "mstsace.h"
#include "mstsshape.h"
#include "mstsroute.h"
#include "timetable.h"
#include "dispatcher.h"
#include "ttosim.h"
#include "listener.h"
#include "animation.h"
#include "signal.h"
#include "interlocking.h"
#include "morse.h"
#include "person.h"

void parseFile(const char* name, osg::Group* root, bool isClient=false,
  int argc=0, char** argv=NULL);
extern double simTime;
extern int startTime;
extern int endTime;
extern int timeMult;
extern MSTSRoute* mstsRoute;
extern TimeTable* timeTable;
extern Interlocking* interlocking;
extern osg::Node* interlockingModel;
extern std::string userOSCallSign;
extern std::string saveString;
extern double fps;
extern float rtt;
extern int timeWarp;
extern int timeWarpSigCount;
extern int timeWarpMsgCount;
extern float timeWarpDist;
extern osg::Vec3d clickLocation;
extern osg::MatrixTransform* clickMT;
extern osg::Vec3f clickOffset;
extern Ship* selectedShip;
extern Train* selectedTrain;
extern RailCarInst* selectedRailCar;
extern RailCarInst* myRailCar;
extern osg::Camera* camera;
extern TTOSim ttoSim;
extern std::string command;
extern bool commandMode;
extern osg::Switch* rootNode;
extern Track::SwVertex* deferredThrow;
extern Switcher* autoSwitcher;
extern int hudState;

extern bool hudMouseOn;

#endif
