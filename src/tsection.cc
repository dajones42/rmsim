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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mstsfile.h"
#include "tsection.h"

MSTSTrackShape::~MSTSTrackShape()
{
	for (int i=0; i<paths.size(); ++i)
		delete paths[i];
}

TSection::TSection()
{
}

TSection::~TSection()
{
	for (CurveMap::iterator i=curveMap.begin(); i!=curveMap.end(); ++i)
		delete i->second;
	for (MSTSTrackShapeMap::iterator i=shapeMap.begin();
	  i!=shapeMap.end(); ++i)
		delete i->second;
}

void TSection::readGlobalFile(const char* path, bool saveShapes)
{
	MSTSFile file;
	file.readFile(path);
	MSTSFileNode* sections= file.find("TrackSections");
	if (sections != NULL) {
		for (MSTSFileNode* s= sections->children->find("TrackSection");
		  s!=NULL; s=s->find("TrackSection")) {
			MSTSFileNode* curve= s->children->find("SectionCurve");
			int index= atoi(s->getChild(0)->value->c_str());
			if (curve==NULL && saveShapes) {
				MSTSFileNode* size=
				  s->children->find("SectionSize");
				lengthMap[index]=
				  atof(size->getChild(1)->value->c_str());
			}
			if (curve == NULL)
				continue;
			Curve* c= new Curve();
			c->radius= atof(curve->getChild(0)->value->c_str());
			c->angle= atof(curve->getChild(1)->value->c_str());
			curveMap[index]= c;
		}
	}
	MSTSFileNode* shapes= file.find("TrackShapes");
	if (shapes != NULL) {
		for (MSTSFileNode* s= shapes->children->find("TrackShape");
		  s!=NULL; s=s->find("TrackShape")) {
			int index= atoi(s->getChild(0)->value->c_str());
			MSTSFileNode* mainRoute= s->children->find("MainRoute");
			if (mainRoute != NULL)
				mainRouteMap[index]=
				  atoi(mainRoute->getChild(0)->value->c_str());
			if (!saveShapes)
				continue;
			MSTSTrackShape* shape= new MSTSTrackShape;
			shapeMap[index]= shape;
			MSTSFileNode* filename= s->children->find("FileName");
			if (filename != NULL)
				shape->name= filename->getChild(0)->value[0];
			for (MSTSFileNode* s1= s->children->find("SectionIdx");
			  s1!=NULL; s1=s1->find("SectionIdx")) {
				MSTSTrackShape::Path* path= 
				  new MSTSTrackShape::Path;
				shape->paths.push_back(path);
				int n= atoi(s1->getChild(0)->value->c_str());
				for (int i=0; i<3; i++)
					path->start[i]= atof(s1->
					  getChild(i+1)->value->c_str());
				path->angle=
				  atof(s1->getChild(4)->value->c_str());
				for (int i=0; i<n; i++)
					path->sections.push_back(atoi(
					  s1->getChild(i+5)->value->c_str()));
			}
		}
	}
}

void TSection::readRouteFile(const char* path)
{
	MSTSFile file;
	try {
		file.readFile(path);
	} catch (const char* msg) {
		return;
	}
	MSTSFileNode* sections= file.find("TrackSections");
	if (sections != NULL) {
		for (MSTSFileNode* s= sections->children->find("TrackSection");
		  s!=NULL; s=s->find("TrackSection")) {
			float radius= atof(s->getChild(4)->value->c_str());
			if (radius == 0)
				continue;
			Curve* c= new Curve();
			c->radius= radius;
			c->angle= 180/3.14159*
			  atof(s->getChild(3)->value->c_str());
			curveMap[atoi(s->getChild(2)->value->c_str())]= c;
		}
	}
}

Curve* TSection::findCurve(int id)
{
	CurveMap::iterator i=curveMap.find(id);
	return i!=curveMap.end() ? i->second : NULL;
}

int TSection::findMainRoute(int id)
{
	MainRouteMap::iterator i=mainRouteMap.find(id);
	return i!=mainRouteMap.end() ? i->second : 0;
}
