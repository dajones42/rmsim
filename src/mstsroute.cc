//	code for reading various MSTS route data
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
#include "rmsim.h"
#include "mstsroute.h"
#include "trackdb.h"
#include "mstsfile.h"
#include "activity.h"
#include "service.h"
#include "consist.h"
#include "trackpath.h"
#include "mstsbfile.h"
#include "ghproj.h"
#include "trigrid.h"
#include "mstswag.h"
#include "train.h"

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Texture>
#include <osg/Material>
#include <osg/Depth>
#include <osg/PolygonOffset>
#include <osg/PolygonMode>
#include <osg/FrontFace>
#include <osg/TexEnvCombine>
#include <osg/TexEnvFilter>
#include <osg/Billboard>
#include <osg/PagedLOD>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/LOD>
#include <osgUtil/Simplifier>
#include <osgDB/ReadFile>
#include <osgUtil/DelaunayTriangulator>
#include <osgSim/LineOfSight>
#include "shaders.h"

extern double simTime;

int checkTree(osg::Node* node);

string fixFilenameCase(string path)
{
	return fixFilenameCase(path.c_str());
}

MSTSRoute::MSTSRoute(const char* mDir, const char* rID)
{
	mstsDir= string(mDir);
	dirSep= "/";
	routeID= string(rID);
	fileName= routeID;
	string routesDir= fixFilenameCase(mstsDir+dirSep+"ROUTES");
	string globalDir= fixFilenameCase(mstsDir+dirSep+"GLOBAL");
	routeDir= fixFilenameCase(routesDir+dirSep+routeID);
	gShapesDir= fixFilenameCase(globalDir+dirSep+"SHAPES");
	gTexturesDir= fixFilenameCase(globalDir+dirSep+"TEXTURES")+dirSep;
	tilesDir= fixFilenameCase(routeDir+dirSep+"TILES");
	terrtexDir= fixFilenameCase(routeDir+dirSep+"TERRTEX");
	worldDir= fixFilenameCase(routeDir+dirSep+"WORLD");
	rShapesDir= fixFilenameCase(routeDir+dirSep+"SHAPES");
	rTexturesDir= fixFilenameCase(routeDir+dirSep+"TEXTURES")+dirSep;
	dynTrackBase= NULL;
	dynTrackRails= NULL;
	dynTrackWire= NULL;
	dynTrackTies= NULL;
	centerLat= centerLong= 0;
	drawWater= 1;
	waterLevelDelta= 0;
	bermHeight= 0;
	wireHeight= 0;
	bridgeBase= false;
	srDynTrack= false;
	ustDynTrack= false;
	ignoreHiddenTerrain= false;
	signalSwitchStands= false;
	createSignals= false;
#if 0
	for (int i=0; i<256; i++) {
		float az= 360*(i&037)/(float)32;
		float ah= 90*(1-.125*(i&0340)/(float)256);
		float radz= (90-az)*3.14159/180;
		float radh= ah*3.14159/180;
		float ch= cos(radh);
		float sh= sin(radh);
		osg::Vec3f normal(cos(radz)*sh,sin(radz)*sh,ch);
		normal.normalize();
		terrainNormals.push_back(normal);
	}
#else
#if 0
	terrainNormals.push_back(osg::Vec3f(0,0,1));
	for (int ah= 86; ah>20; ah-=8) {
		for (int i=0; i<30; i++) {
			float radz= (90-12*i)*3.14159/180;
			float radh= ah*3.14159/180;
			float ch= cos(radh);
			float sh= sin(radh);
			osg::Vec3f normal(cos(radz)*sh,sin(radz)*sh,ch);
			normal.normalize();
			terrainNormals.push_back(normal);
		}
	}
#endif
	while (terrainNormals.size() < 256)
		terrainNormals.push_back(osg::Vec3f(0,0,1));
#endif
}

MSTSRoute::~MSTSRoute()
{
}

//	find the center of the route
//	used for coordinate conversion
void MSTSRoute::findCenter(TrackDB* trackDB)
{
	int minTX= 0x7fffffff;
	int maxTX= -0x7fffffff;
	int minTZ= 0x7fffffff;
	int maxTZ= -0x7fffffff;
	for (int i=0; i<trackDB->nNodes; i++) {
		TrackNode* node= &trackDB->nodes[i];
		if (node->nSections > 0)
			continue;
		if (minTX > node->tx)
			minTX= node->tx;
		if (maxTX < node->tx)
			maxTX= node->tx;
		if (minTZ > node->tz)
			minTZ= node->tz;
		if (maxTZ < node->tz)
			maxTZ= node->tz;
	}
	fprintf(stderr,"%d %d %d %d\n",minTX,maxTX,minTZ,maxTZ);
	if (centerLat!=0 || centerLong!=0) {
		cosCenterLat= cos(centerLat/(180*3.14159));
		fprintf(stderr,"%f %f %lf\n",
		  centerLat,centerLong,cosCenterLat);
	} else {
		centerTX= (minTX+maxTX)/2;
		centerTZ= (minTZ+maxTZ)/2;
	}
	fprintf(stderr,"center %d %d\n",centerTX,centerTZ);
	double lat,lng,xx,yy;
	xy2ll(0,0,&lat,&lng);
	ll2xy(lat,lng,&xx,&yy);
	fprintf(stderr,"centerll %f %f %f %f\n",lat,lng,xx,yy);
#if 0
	char buf[256];
	sprintf(buf,"+proj=tmerc +ellps=WGS84 +lat_0=%f +lon_0=%f "
	  "+y_0=0 +x_0=0 +k_0=.9999",lat,lng);
	projPJ proj= pj_init_plus(buf);
	fprintf(stderr,"%p %s\n",proj,buf);
	projUV p;
	p.u= lng*M_PI/180;
	p.v= lat*M_PI/180;
	p= pj_fwd(p,proj);
	fprintf(stderr," %f %f\n",p.u,p.v);
	double x= 2048*(double)(minTX-centerTX);
	double y= 2048*(double)(minTZ-centerTZ);
	xy2ll(x,y,&lat,&lng);
	p.u= lng*M_PI/180;
	p.v= lat*M_PI/180;
	p= pj_fwd(p,proj);
	fprintf(stderr,"xy %f %f ll %f %f\n",x,y,lat,lng);
	fprintf(stderr,"uv %f %f e %f %f\n",p.u,p.v,x-p.u,y-p.v);
	x= 2048*(double)(maxTX-centerTX);
	y= 2048*(double)(minTZ-centerTZ);
	xy2ll(x,y,&lat,&lng);
	p.u= lng*M_PI/180;
	p.v= lat*M_PI/180;
	p= pj_fwd(p,proj);
	fprintf(stderr,"xy %f %f ll %f %f\n",x,y,lat,lng);
	fprintf(stderr,"uv %f %f e %f %f\n",p.u,p.v,x-p.u,y-p.v);
	x= 2048*(double)(maxTX-centerTX);
	y= 2048*(double)(maxTZ-centerTZ);
	xy2ll(x,y,&lat,&lng);
	p.u= lng*M_PI/180;
	p.v= lat*M_PI/180;
	p= pj_fwd(p,proj);
	fprintf(stderr,"xy %f %f ll %f %f\n",x,y,lat,lng);
	fprintf(stderr,"uv %f %f e %f %f\n",p.u,p.v,x-p.u,y-p.v);
	x= 2048*(double)(minTX-centerTX);
	y= 2048*(double)(maxTZ-centerTZ);
	xy2ll(x,y,&lat,&lng);
	p.u= lng*M_PI/180;
	p.v= lat*M_PI/180;
	p= pj_fwd(p,proj);
	fprintf(stderr,"xy %f %f ll %f %f\n",x,y,lat,lng);
	fprintf(stderr,"uv %f %f e %f %f\n",p.u,p.v,x-p.u,y-p.v);
	pj_free(proj);
#endif
}

static int findPin(TrackNode* n1, TrackNode* n2, int end)
{
	for (int i=0; i<n1->nPins; i++)
		if (n1->pin[i]==n2 && n1->pinEnd[i]==end)
			return i;
	fprintf(stderr,"cannot find pin %d %d %d\n",n1->id,n2->id,end);
	return -1;
}

void MSTSRoute::ll2xy(double lat, double lng, double* xp, double* yp)
{
	if (centerLat!=0 || centerLong!=0) {
		*xp= (lng-centerLong)*cosCenterLat*60*1852.216;
		*yp= (lat-centerLat)*60*1852.216;
	} else {
		double x,y;
		ghProj.ll2xy(lat,lng,&x,&y);
		*xp= x - centerTX*2048 + 20015000 - 16385*2048 + 1024;
		*yp= y - centerTZ*2048 - 8673000 + 16385*2048 - 3072;
	}
}

void MSTSRoute::xy2ll(double x, double y, double* lat, double* lng)
{
	if (centerLat!=0 || centerLong!=0) {
		*lng= x/(cosCenterLat*60*1852.216) + centerLong;
		*lat= y/(60*1852.216) + centerLat;
	} else {
		ghProj.xy2ll(x + centerTX*2048 - 20015000 + 16385*2048 - 1024,
		  y + centerTZ*2048 + 8673000 - 16385*2048 + 3072, lat,lng);
	}
}

//	Makes Track class data from tsection and tdb data
void MSTSRoute::makeTrack(int smoothGradesIterations,
  float smoothGradesDistance)
{
	TSection tSection;
	string globalDir= fixFilenameCase(mstsDir+dirSep+"GLOBAL");
	string path= fixFilenameCase(globalDir+dirSep+"tsection.dat");
	tSection.readGlobalFile(path.c_str());
	path= fixFilenameCase(routeDir+dirSep+"tsection.dat");
	tSection.readRouteFile(path.c_str());
	path= fixFilenameCase(routeDir+dirSep+fileName+".tdb");
	if (path.size() == 0) {
		ulDir* dir= ulOpenDir(routeDir.c_str());
		if (dir == NULL) {
			fprintf(stderr,"cannot read route directory %s\n",
			  routeDir.c_str());
		}
		for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL;
		  ent=ulReadDir(dir)) {
			int n= strlen(ent->d_name);
			if (strcasecmp(ent->d_name+n-4,".trk")==0)
				path= routeDir+dirSep+ent->d_name;
		}
		if (path.size() == 0)
			fprintf(stderr,"cannot find tdb file\n");
		fprintf(stderr,"tdbfile %s\n",path.c_str());
	}
	TrackDB trackDB;
	trackDB.readFile(path.c_str(),&tSection);
	fprintf(stderr,"nNodes %d nTrItems %d\n",
	  trackDB.nNodes,trackDB.nTrItems);
	findCenter(&trackDB);
	Track* track= new Track;
	trackMap[routeID]= track;
	track->updateSignals= createSignals;
	typedef map<int,Track::Vertex*> VMap;
	VMap vMap;
	for (int i=0; i<trackDB.nNodes; i++) {
		TrackNode* node= &trackDB.nodes[i];
		if (node->nSections > 0)
			continue;
		Track::Vertex* v= track->addVertex(
		  node->shape==0?Track::VT_SIMPLE:Track::VT_SWITCH,
		  convX(node->tx,node->x),
		  convZ(node->tz,node->z),node->y+.275);
		if (v->type == Track::VT_SWITCH)
			((Track::SwVertex*)v)->id= i;
		Tile* tile= findTile(node->tx,node->tz);
		saveTrackLocation(tile,node->x,node->z);
		if (node->shape != 0) {
			((Track::SwVertex*)v)->mainEdge=
			  tSection.findMainRoute(node->shape);
			if (tile != NULL)
				tile->swVertexMap[node->id]=
				  (Track::SwVertex*)v;
			//fprintf(stderr,"swid %p %d %p\n",tile,node->id,
			//  (Track::SwVertex*)v);
		}
		vMap[i]= v;
		node->tx= i;
	}
	for (int i=0; i<trackDB.nNodes; i++) {
		TrackNode* node= &trackDB.nodes[i];
		if (node->nSections == 0)
			continue;
		VMap::iterator i1= vMap.find(node->pin[0]->tx);
		if (i1 == vMap.end()) {
			fprintf(stderr,"? cannot find start node %d\n",
			  node->pin[0]->tx);
			continue;
		}
		Track::Vertex* v1= i1->second;
		int n1= findPin(node->pin[0],node,1);
		Track::SSEdge* sse= new Track::SSEdge;
		sse->track= track;
		sse->block= i;
		sse->length= 0;
		sse->v1= v1;
		if (v1->type == Track::VT_SWITCH)
			((Track::SwVertex*)v1)->ssEdges[n1]= sse;
		track->ssEdgeMap[sse->block]= sse;
		double len= node->sections[0].length;
		TrSection* s1= &node->sections[0];
		Tile* tile= findTile(s1->tx,s1->tz);
		saveTrackLocation(tile,s1->x,s1->z);
		for (int j=1; j<node->nSections; j++) {
			TrSection* s2= &node->sections[j];
			if (tile==NULL || tile->x!=s2->x || tile->z!=s2->z)
				tile= findTile(s2->tx,s2->tz);
			saveTrackLocation(tile,s2->x,s2->z);
			Track::Vertex* v2= NULL;
			int n2= 0;
			if (j == node->nSections-1) {
				VMap::iterator i2= vMap.find(node->pin[1]->tx);
				if (i2 == vMap.end())
					fprintf(stderr,
					  "? cannot find end node %d\n",
					  node->pin[1]->tx);
				else
					v2= i2->second;
				n2= findPin(node->pin[1],node,0);
				sse->v2= v2;
				if (v2->type == Track::VT_SWITCH)
					((Track::SwVertex*)v2)->ssEdges[n2]=
					  sse;
			}
			if (v2 == NULL)
				v2= track->addVertex(Track::VT_SIMPLE,
				  convX(s2->tx,s2->x),convZ(s2->tz,s2->z),
				  s2->y+.275);
			if (s1->radius > 0) {
				Track::SplineEdge* e= (Track::SplineEdge*)
				  track->addEdge(Track::ET_SPLINE,v1,n1,v2,n2);
				e->length= s1->length;
				e->setCircle(s1->radius,s1->angle*3.14159/180);
				e->ssEdge= sse;
				e->ssOffset= sse->length;
				sse->length+= e->length;
			} else {
				Track::Edge* e=
				 track->addEdge(Track::ET_STRAIGHT,v1,n1,v2,n2);
				e->ssEdge= sse;
				e->ssOffset= sse->length;
				sse->length+= e->length;
			}
			v1= v2;
			n1= n2+1;
			len= s1->length;
			s1= s2;
		}
	}
	for (int i=0; i<trackDB.nTrItems; i++) {
		TrItem* item= &trackDB.trItems[i];
		if (item->name==NULL)
			continue;
//		if (item->otherID!=0 && item->otherID<item->id)
//			continue;
		double x= convX(item->tx,item->x);
		double z= convZ(item->tz,item->z);
		double y= item->y;
		if (item->type==0 || item->type==1) {
			track->saveLocation(x,z,y,*item->name,item->type);
		} else if (createSignals && item->type==2) {
			Track::Location loc;
			track->findLocation(x,z,y,&loc);
			if (loc.edge == NULL)
				continue;
			if (item->dir) {
				loc.rev= 1;
				if (loc.edge->v1->type != Track::VT_SWITCH) {
					loc.move(loc.offset+.01,0,0);
				}
			} else if (loc.edge->v2->type != Track::VT_SWITCH) {
				loc.move(loc.edge->length-loc.offset+.01,0,0);
			}
			Signal* sig= new Signal(Signal::CLEAR);
			//sig->setState(Signal::CLEAR);
			sig->addTrack(&loc);
			loc.edge->signals.push_back(sig);
			char buf[10];
			sprintf(buf,"%d",item->id);
			signalMap[buf]= sig;
		}
//		else
//			fprintf(stderr,"%s %d %d %d %f %f %f %d %d %f\n",
//			  item->name->c_str(),
//			  item->id,item->otherID,item->type,x,z,y,item->flags,
//			  item->dir,item->offset);
	}
	setPatchDetail();
	track->makeSSEdges();
	if (smoothGradesIterations > 0)
		track->calcSmoothGrades(smoothGradesIterations,
		  smoothGradesDistance);
	else
		track->calcGrades();
}

static char hexdigit [] = {
	'0','1','4','5',
	'3','2','7','6',
	'c','d','8','9',
	'f','e','b','a'
};

//	returns file name for tile x,z
const char* MSTSRoute::tFile(int x, int z)
{
	static char filename[10];
	x+= 16384;
	z= 16384-z-1;
	x<<= 1;
	z<<= 1;
	filename[0]= '-';
	filename[9]= '\0';
	for (int i=1; i<9; i++) {
		int s= 16-2*i;
		int m= 3<<s;
		int j= ((x&m)>>s) + 4*((z&m)>>s);
//		printf("%d %d %d %d %d %d\n",i,s,m,j,
//		  (x&m)>>s,(z&m)>>s);
		filename[i]= hexdigit[j];
	}
	return filename;
}

//	converts tile filename to x and z
void MSTSRoute::tFileToXZ(char* filename, int *xp, int * zp)
{
	int x= 0;
	int z= 0;
	for (int i=1; i<9; i++) {
		int j= 0;
		for (; j<16; j++)
			if (filename[i] == hexdigit[j])
				break;
		int s= 16-2*i;
		int m= 3<<s;
		x|= (j&3)<<s;
		z|= ((j&017)>>2)<<s;
	}
	x>>= 1;
	z>>= 1;
	*xp= x-16384;
	*zp= 16384-z-1;
}

//	Reads all files in tiles directory to get tiles in route
void MSTSRoute::readTiles()
{
	ulDir* dir= ulOpenDir(tilesDir.c_str());
	if (dir == NULL) {
		fprintf(stderr,"cannot read tiles directory %s\n",
		  tilesDir.c_str());
		return;
	}
	for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL; ent=ulReadDir(dir)) {
		int n= strlen(ent->d_name);
		if (ent->d_name[n-1] != 't')
			continue;
		int x,z;
		tFileToXZ(ent->d_name,&x,&z);
//		fprintf(stderr,"%s %d %d\n",ent->d_name,x,z);
		Tile* t= new Tile(x,z);
		t->tFilename= tFile(x,z);
		string path= tilesDir+dirSep+ent->d_name;
		tileMap[tileID(x,z)]= t;
		terrainTileMap[t->tFilename]= t;
		readTFile(path.c_str(),t);
//		fprintf(stderr,"%s tf=%f sc=%f\n",
//		  ent->d_name,t->floor,t->scale);
//		fprintf(stderr," %d %d %s\n",x,z,tFile(x,z));
//		if (t->swWaterLevel != 0)
//			fprintf(stderr,"%d %d sw=%f se=%f ne=%f nw=%f\n",
//			  x,z,t->swWaterLevel,t->seWaterLevel,
//			  t->neWaterLevel,t->nwWaterLevel);
	}
	ulCloseDir(dir);
}

//	reads a single tile file and saves some of the data
int MSTSRoute::readTFile(const char* filename, Tile* tile)
{
	MSTSBFile reader;
	if (reader.open(filename))
		return 0;
//	fprintf(stderr,"%s is binary %d\n",filename,reader.compressed);
	Byte buf[16];
	reader.getBytes(buf,16);
	Patch* patch= tile->patches;
	for (;;) {
		unsigned short code= (unsigned short) reader.getShort();
		unsigned short flags= (unsigned short) reader.getShort();
		int len= reader.getInt();
		if (code == 0)
			break;
//		fprintf(stderr,"%d %d\n",code,len);
		switch (code) {
		 case 145: // terrain sample fbuffer
		 case 146: // terrain sample ybuffer
		 case 147: // terrain sample ebuffer
		 case 148: // terrain sample nbuffer
		 case 149: // terrain sample cbuffer
		 case 150: // terrain sample dbuffer
		 case 281: // terrain sample asbuffer
		 case 282: // terrain sample usbuffer
		 case 138: // terrain always select maxdist
		 case 160: // terrain patchset dist
		 case 161: // terrain patchset npatches
			reader.getBytes(NULL,len);
			break;
		 case 140: // terrain nsamples
		 case 158: // terrain patchsets
			reader.getString();
			reader.getInt();
			break;
		 case 137: // terrain errthreshold_scale
		 case 141: // terrain sample rotation
		 case 144: // terrain sample size
			reader.getString();
			reader.getFloat();
			break;
		 case 136: // terrain
		 case 139: // terrain_samples
		 case 157: // terrain patches
		 case 159: // terrain patchset
		 case 163: // terrain patchset patches
			reader.getString();
			break;
		 case 142: // terrain sample floor
			reader.getString();
//			fprintf(stderr,"floor offset %d\n",ftell(reader.in));
			tile->floor= reader.getFloat();
			break;
		 case 143: // terrain sample scale
			reader.getString();
//			fprintf(stderr,"scale offset %d\n",ftell(reader.in));
			tile->scale= reader.getFloat();
			break;
		 case 164: // terrain patchset patch
			reader.getString();
			{
			patch->flags= reader.getInt();
			float f1= reader.getFloat();//coord
			float f2= reader.getFloat();
			float f3= reader.getFloat();//coord
			float f4= reader.getFloat();
			float f5= reader.getFloat();
			float f6= reader.getFloat();
			patch->centerX= f1;
			patch->centerZ= f3;
//			fprintf(stderr,"patch %f %f %f %f %f %f\n",
//			  f1,f2,f3,f4,f5,f6);
			patch->texIndex= (short) reader.getInt();
			patch->u0= reader.getFloat();
			patch->v0= reader.getFloat();
			patch->dudx= reader.getFloat();
			patch->dudz= reader.getFloat();
			patch->dvdx= reader.getFloat();
			patch->dvdz= reader.getFloat();
//			fprintf(stderr,"patch %d %f %f %f %f %f %f\n",
//			  patch->texIndex,patch->u0,patch->v0,
//			  patch->dudx,patch->dudz,patch->dvdx,patch->dvdz);
			reader.getFloat();//error threshold
			patch->detail= 0;
			patch->edgeDetail= 0;
			if ((patch->flags&1)!=0)
				patch->detail= 5;
			patch++;
			}
			break;
		 case 251: // water level?
			reader.getString();
			if (len == 17) {
				tile->swWaterLevel= reader.getFloat();
				tile->seWaterLevel= reader.getFloat();
				tile->neWaterLevel= reader.getFloat();
				tile->nwWaterLevel= reader.getFloat();
			} else if (len == 5) {
				tile->swWaterLevel= 
				tile->seWaterLevel=
				tile->neWaterLevel=
				tile->nwWaterLevel= reader.getFloat();
			} else {
				reader.getBytes(NULL,len);
			}
			break;
		 case 151: // terrain shaders
			reader.getString();
			{
				int n= reader.getInt();
				//fprintf(stderr,"%d terrain shaders\n",n);
			}
			break;
		 case 152: // terrain shader
			reader.getString();
			{
				int n= reader.getShort();
				string s= reader.getString(n);
//				fprintf(stderr," terrain shader %s\n",s.c_str());
			}
			break;
		 case 153: // terrain texslots
		 case 155: // terrain uvcalcs
			reader.getString();
			reader.getInt();
			break;
		 case 154: // terrain texslot
			reader.getString();
			{
				int n= reader.getShort();
				string s= reader.getString(n);
				int n1= reader.getInt();
				int n2= reader.getInt();
//				fprintf(stderr,"  texslot %s %d %d\n",
//				  s.c_str(),n1,n2);
				if (n2 == 0)
					tile->textures.push_back(s);
				else if (n2 == 1)
					tile->microTextures.push_back(s);
			}
			break;
		 case 156: // terrain uvcalc
			reader.getString();
			{
				int n1= reader.getInt();
				int n2= reader.getInt();
				int n3= reader.getInt();
				float f1= reader.getFloat();
				if (n3 == 1)
					tile->microTexUVMult= f1;
//				fprintf(stderr,"  uvcalc %d %d %d %f\n",
//				  n1,n2,n3,f1);
			}
			break;
		 case 1: // comment
//			fprintf(stderr,"%d %d\n",code,len);
			for (int i=0; i<len; i++) {
				Byte b;
				reader.getBytes(&b,1);
//				fprintf(stderr," %d",0xff&b);
//				if (i%16==15)
//					fprintf(stderr,"\n");
			}
//			fprintf(stderr,"\n");
			break;
		 default:
			fprintf(stderr,
			  "unsupported tfile content %u %d %s %d\n",
			  code,len,filename,reader.read);
			reader.getBytes(NULL,len);
			break;
		}
	}
	return 1;
}

//	reads and saves a tile's terrain data
void MSTSRoute::readTerrain(Tile* tile)
{
	if (tile->terrain != NULL)
		return;
	tile->terrain= new Terrain;
	string path= tilesDir+dirSep+tile->tFilename+"_y.raw";
	FILE* in= fopen(path.c_str(),"r");
	if (in == NULL) {
		fprintf(stderr,"cannot read %s\n",path.c_str());
		memset(tile->terrain->y,0,sizeof(tile->terrain->y));
	} else {
		fread(tile->terrain->y,1,sizeof(tile->terrain->y),in);
		fclose(in);
	}
	path= tilesDir+dirSep+tile->tFilename+"_n.raw";
	in= fopen(path.c_str(),"r");
	if (in == NULL) {
//		fprintf(stderr,"cannot read %s\n",path.c_str());
		memset(tile->terrain->n,0,sizeof(tile->terrain->n));
	} else {
		fread(tile->terrain->n,1,sizeof(tile->terrain->n),in);
		fclose(in);
	}
	path= tilesDir+dirSep+tile->tFilename+"_f.raw";
	in= fopen(path.c_str(),"r");
	if (in == NULL) {
//		fprintf(stderr,"cannot read %s\n",path.c_str());
		memset(tile->terrain->f,0,sizeof(tile->terrain->f));
	} else {
		fread(tile->terrain->f,1,sizeof(tile->terrain->f),in);
		fclose(in);
	}
}

//	writes terrain data
//	this was used once to build under water terrain based on an
//	electronic chart
void MSTSRoute::writeTerrain(Tile* tile)
{
	if (tile->terrain == NULL)
		return;
	string path= tilesDir+dirSep+tile->tFilename+"_y.raw";
	FILE* out= fopen(path.c_str(),"w");
	if (out == NULL) {
		fprintf(stderr,"cannot read %s\n",path.c_str());
		return;
	}
	fwrite(tile->terrain->y,1,sizeof(tile->terrain->y),out);
	fclose(out);
}

//	loads models for tiles given camera coordinates
osg::Group* MSTSRoute::loadTiles(double x, double y)
{
	return NULL;
	int tx= centerTX + (int)rint(x/2048);
	int tz= centerTZ + (int)rint(y/2048);
	float dx= x - 2048*(tx-centerTX);
	float dz= y - 2048*(tz-centerTZ);
	int tx1= dx>128 ? tx+1 : dx<-128 ? tx-1 : tx;
	int tz1= dz>128 ? tz+1 : dz<-128 ? tz-1 : tz;
	Tile* tile= findTile(tx,tz);
	if (tile == NULL)
		return NULL;
	int n= 0;
	static Tile* prevTile= NULL;
	if (prevTile != tile) {
		for (int i=-1; i<2; i++) {
			for (int j=-1; j<2; j++) {
				Tile* t= findTile(tx+i,tz+j);
				if (t!=NULL && t->terrModel==NULL) {
					//makeTerrainModel(t);
					makeTerrainPatches(t);
					n++;
				}
			}
		}
	}
	prevTile= tile;
	if (tile->models == NULL) {
		loadModels(tile);
		n++;
	}
	if (tx1 != tx) {
		Tile* t= findTile(tx1,tz);
		if (t!=NULL && t->models==NULL) {
			loadModels(t);
			n++;
		}
	}
	if (tz1 != tz) {
		Tile* t= findTile(tx,tz1);
		if (t!=NULL && t->models==NULL) {
			loadModels(t);
			n++;
		}
	}
	if (tx1!=tx && tz1!=tz) {
		Tile* t= findTile(tx1,tz1);
		if (t!=NULL && t->models==NULL) {
			loadModels(t);
			n++;
		}
	}
	if (n == 0)
		return NULL;
	osg::Group* root= new osg::Group;
	for (TileMap::iterator i=tileMap.begin(); i!=tileMap.end(); ++i) {
		Tile* t= i->second;
		if (t->terrain!=NULL &&
		  (t->x<tile->x-1 || t->x>tile->x+1 ||
		   t->z<tile->z-1 || t->z>tile->z+1))
			t->freeTerrain();
		if (t->terrModel!=NULL &&
		  (t->x<tile->x-1 || t->x>tile->x+1 ||
		   t->z<tile->z-1 || t->z>tile->z+1)) {
			t->terrModel->unref();
			t->terrModel= NULL;
		}
//		if (t->terrModel != NULL)
//			root->addChild(t->terrModel);
		if (t->models == NULL)
			continue;
		if ((t->x!=tx && t->x!=tx1) || (t->z!=tz && t->z!=tz1)) {
			t->models->unref();
			t->models= NULL;
			continue;
		}
//		root->addChild(t->models);
	}
	return root;
}

void MSTSRoute::loadTerrainData(Tile* tile)
{
	int nt= 0;
	int ntm= 0;
	int nm= 0;
	for (TileMap::iterator i=tileMap.begin(); i!=tileMap.end(); ++i) {
		Tile* t= i->second;
		if (t->terrain)
			nt++;
		if (t->terrModel)
			ntm++;
		if (t->models)
			nm++;
		if (t->terrain==NULL &&
		  t->x>=tile->x-1 && t->x<=tile->x+1 &&
		  t->z>=tile->z-1 && t->z<=tile->z+1)
			readTerrain(t);
		else if (t->terrain!=NULL &&
		  (t->x<tile->x-1 || t->x>tile->x+1 ||
		   t->z<tile->z-1 || t->z>tile->z+1))
			t->freeTerrain();
		if (t->plod && t->plod->getNumChildren() > 1) {
#if 0
			fprintf(stderr,"loaded %d %d %d %d %d",
			  t->x,t->z,t->plod->getNumChildren(),
			  t->plod->getNumChildrenThatCannotBeExpired(),
			  t->plod->getDisableExternalChildrenPaging());
			for (int j=1; j<t->plod->getNumChildren(); j++)
				fprintf(stderr," %d.%d",
				  t->plod->getFrameNumber(j)==
				  t->plod->getFrameNumberOfLastTraversal(),
				  t->plod->getTimeStamp(j));
			fprintf(stderr,"\n");
#endif
		} else {
			if (t->terrModel!=NULL &&
			  (t->x<tile->x-1 || t->x>tile->x+1 ||
			   t->z<tile->z-1 || t->z>tile->z+1)) {
//				fprintf(stderr,"unref tmodel %d %d %d\n",
//				  t->x,t->z,t->terrModel->referenceCount());
				t->terrModel->unref();
				t->terrModel= NULL;
			}
			if (t->models!=NULL &&
			  (t->x<tile->x-1 || t->x>tile->x+1 ||
			   t->z<tile->z-1 || t->z>tile->z+1)) {
//				fprintf(stderr,"unref models %d %d %d\n",
//				  t->x,t->z,t->models->referenceCount());
				t->models->unref();
				t->models= NULL;
			}
		}
	}
//	fprintf(stderr,"loadtd %d %d %d\n",nt,ntm,nm);
}

//	makes a 3D model for a tiles terrain data
//	not used anymore
void MSTSRoute::makeTerrainModel(Tile* tile)
{
	if (tile->terrModel != NULL)
		return;
	readTerrain(tile);
//	fprintf(stderr,"makeTerrain %d %d %f %f\n",
//	  tile->x,tile->z,tile->floor,tile->scale);
	Tile* t12= findTile(tile->x,tile->z-1);
	if (t12 != NULL)
		readTerrain(t12);
	Tile* t21= findTile(tile->x+1,tile->z);
	if (t21 != NULL)
		readTerrain(t21);
	Tile* t22= findTile(tile->x+1,tile->z-1);
	if (t22 != NULL)
		readTerrain(t22);
	float maxa= tile->floor;
	float x0= 2048*(tile->x-centerTX);
	float z0= 2048*(tile->z-centerTZ);
	float dt= 1/64.;
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::Vec3Array* norms= new osg::Vec3Array;
	osg::DrawElementsUInt* drawElements=
	  new osg::DrawElementsUInt(GL_TRIANGLES);;
	for (int i=0; i<=256; i++) {
		int k= i*257;
		for (int j=0; j<=256; j++) {
			float a;
			int ni;
			if (i<256 && j<256) {
				a= tile->floor+
				  tile->scale*tile->terrain->y[i][j];
				ni= tile->terrain->n[i][j];
			} else if (i<256 && j>=256) {
			  if (t21 != NULL) {
				a= t21->floor+
				  t21->scale*t21->terrain->y[i][j-256];
				ni= t21->terrain->n[i][j-256];
			  } else {
				a= tile->floor+
				  tile->scale*tile->terrain->y[i][255];
				ni= tile->terrain->n[i][255];
			  }
			} else if (i>=256 && j<256) {
			  if (t12 != NULL) {
				a= t12->floor+
				  t12->scale*t12->terrain->y[i-256][j];
				ni= t12->terrain->n[i-256][j];
			  } else {
				a= tile->floor+
				  tile->scale*tile->terrain->y[255][j];
				ni= tile->terrain->n[255][j];
			  }
			} else if (i>=256 && j>=256) {
			  if (t22 != NULL) {
				a= t22->floor+
				  t22->scale*t22->terrain->y[i-256][j-256];
				ni= t22->terrain->n[i-256][j-256];
			  } else {
				a= tile->floor+
				  tile->scale*tile->terrain->y[255][255];
				ni= tile->terrain->n[255][255];
			  }
			}
			if (maxa < a)
				maxa= a;
			verts->push_back(
			  osg::Vec3(x0+8*(j-128),z0+8*(128-i),a));
			texCoords->push_back(osg::Vec2(dt*i,dt*j));
			norms->push_back(terrainNormals[ni]);
			if (i<256 && j<256) {
				drawElements->push_back(k+j);
				drawElements->push_back(k+j+257);
				drawElements->push_back(k+j+1);
				drawElements->push_back(k+j+1);
				drawElements->push_back(k+j+257);
				drawElements->push_back(k+j+257+1);
			}
		}
	}
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(1,1,1,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->setNormalArray(norms);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->setTexCoordArray(0,texCoords);
	geometry->addPrimitiveSet(drawElements);
	static osg::Texture2D* t= NULL;
	if (t == NULL) {
		t= new osg::Texture2D;
		t->ref();
		t->setDataVariance(osg::Object::DYNAMIC);
		string path= terrtexDir+dirSep+"terrain.ace";
		osg::Image* image= readMSTSACE(path.c_str());
		if (image != NULL)
			t->setImage(image);
		t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
		t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
	}
	osg::StateSet* stateSet= geometry->getOrCreateStateSet();
	stateSet->setTextureAttributeAndModes(0,t,osg::StateAttribute::ON);
	stateSet->setRenderingHint(osg::StateSet::OPAQUE_BIN);
	osg::Material* m= new osg::Material;
	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	stateSet->setAttribute(m,osg::StateAttribute::ON);
	osg::Geode* geode= new osg::Geode;
	geode->addDrawable(geometry);
	tile->terrModel= geode;
	geode->ref();
}

void MSTSRoute::Tile::freeTerrain()
{
	if (terrain != NULL)
		delete terrain;
	terrain= NULL;
}

MSTSRoute::Tile* MSTSRoute::findTile(int tx, int tz)
{
	TileMap::iterator i= tileMap.find(tileID(tx,tz));
	return i==tileMap.end() ? NULL : i->second;
}

//	makes a 3D model to show a map of unloaded tiles when viewing
//	the route from a long distance
//	the model is a green and white checker board with one square per tile
void MSTSRoute::makeTileMap(osg::Group* root)
{
//	fprintf(stderr,"%d nodes before makeTileMap\n",checkTree(root));
	for (TileMap::iterator i=tileMap.begin(); i!=tileMap.end(); ++i) {
		Tile* tile= i->second;
		float x0= 2048*(float)(tile->x-centerTX);
		float z0= 2048*(float)(tile->z-centerTZ);
		osg::Vec3Array* verts= new osg::Vec3Array;
		osg::Vec4Array* colors= new osg::Vec4Array;
		osg::DrawElementsUShort* drawElements=
		  new osg::DrawElementsUShort(GL_QUADS);;
		verts->push_back(osg::Vec3(x0-1024,z0-1024,tile->floor));
		verts->push_back(osg::Vec3(x0+1024,z0-1024,tile->floor));
		verts->push_back(osg::Vec3(x0+1024,z0+1024,tile->floor));
		verts->push_back(osg::Vec3(x0-1024,z0+1024,tile->floor));
		if ((tile->x&01) == (tile->z&01)) {
			colors->push_back(osg::Vec4(1,1,1,1));
			colors->push_back(osg::Vec4(1,1,1,1));
			colors->push_back(osg::Vec4(1,1,1,1));
			colors->push_back(osg::Vec4(1,1,1,1));
		} else {
			colors->push_back(osg::Vec4(.7,1,.7,1));
			colors->push_back(osg::Vec4(.7,1,.7,1));
			colors->push_back(osg::Vec4(.7,1,.7,1));
			colors->push_back(osg::Vec4(.7,1,.7,1));
		}
		drawElements->push_back(0);
		drawElements->push_back(1);
		drawElements->push_back(2);
		drawElements->push_back(3);
		osg::Geometry* geometry= new osg::Geometry;
		geometry->setVertexArray(verts);
		geometry->setColorArray(colors);
		geometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
		osg::Vec3Array* norms= new osg::Vec3Array;
		norms->push_back(osg::Vec3(0,0,1));
		geometry->setNormalArray(norms);
		geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
		geometry->addPrimitiveSet(drawElements);
		osg::StateSet* stateSet= geometry->getOrCreateStateSet();
		stateSet->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
		osg::Geode* geode= new osg::Geode;
		geode->addDrawable(geometry);
		osg::PagedLOD* plod= new osg::PagedLOD;
		plod->addChild(geode,6000,FLT_MAX);
		plod->setCenter(geode->getBound().center());
//		plod->setRadius(1000);
		plod->setFileName(1,tile->tFilename+"_y.raw");
		plod->setRange(1,0,6000);
		plod->setFileName(2,tile->tFilename+".world");
		plod->setRange(2,0,3000);
		plod->setMinimumExpiryTime(2,50);
		root->addChild(plod);
		tile->plod= plod;
		plod->ref();
	}
//	fprintf(stderr,"%d nodes before makeTileMap\n",checkTree(root));
	for (TileMap::iterator i=tileMap.begin(); i!=tileMap.end(); ++i) {
		Tile* tile= i->second;
		int n= 0;
		for (int i=0; i<256; i++)
			if ((tile->patches[i].flags&0xc0) != 0)
				n++;
		if (n == 0)
			tile->swWaterLevel= -1e10;
	}
}

//	loads the models for a tile
void MSTSRoute::loadModels(Tile* tile)
{
	if (tile->models != NULL)
		return;
	loadTerrainData(tile);
	if (tile->terrModel==NULL)
		makeTerrainPatches(tile);
	tile->models= new osg::Group;
	tile->models->ref();
//	makeTerrainModel(tile);
	float x0= 2048*(float)(tile->x-centerTX);
	float z0= 2048*(float)(tile->z-centerTZ);
	char buf[100];
	sprintf(buf,"w%+6.6d%+6.6d.w",tile->x,tile->z);
	string path= worldDir+dirSep+buf;
//	fprintf(stderr,"loadModels from %s %f %f\n",path.c_str(),x0,z0);
	if (readBinWFile(path.c_str(),tile,x0,z0) == 0) {
	try {
		MSTSFile file;
		file.readFile(path.c_str());
//		fprintf(stderr,"file read\n");
		MSTSFileNode* wf= file.find("Tr_Worldfile");
		if (wf == NULL) {
			fprintf(stderr,"%s not a MSTS world file?",
			  path.c_str());
			return;
		}
		for (MSTSFileNode* node=wf->children; node!=NULL;
		  node=node->next) {
//			fprintf(stderr," %p %p %p\n",
//			  node,node->value,node->next);
			if (node->value==NULL ||
			  (*(node->value)!="TrackObj" &&
			   *(node->value)!="Static" &&
			   *(node->value)!="Transfer" &&
			   *(node->value)!="Forest" &&
			   *(node->value)!="Dyntrack" &&
			   *(node->value)!="Signal" &&
			   *(node->value)!="LevelCr" &&
			   *(node->value)!="Speedpost" &&
			   *(node->value)!="Hazard" &&
			   *(node->value)!="CollideObject" &&
			   *(node->value)!="Gantry")) {
#if 0
				MSTSFileNode* file= node->next ?
				  node->next->children->find("FileName") : NULL;
				if (node->value!=NULL)
					fprintf(stderr,"unhandled %s %s\n",
					  node->value->c_str(),
					  file&&file->getChild(0)->value?
					  file->getChild(0)->value->c_str():"");
#endif
				continue;
			}
			MSTSFileNode* next= node->next;
			MSTSFileNode* file= next->children->find("FileName");
			MSTSFileNode* pos= next->children->find("Position");
			MSTSFileNode* qdir= next->children->find("QDirection");
			if (pos==NULL || qdir==NULL)
				continue;
			osg::Node* model= NULL;
			if (*(node->value)=="TrackObj" && file!=NULL) {
				Track::SwVertex* swVertex= NULL;
				if (next->children->find("JNodePosn") != NULL) {
					MSTSFileNode* uid=
					  next->children->find("UiD");
					if (uid != NULL) {
						int swid=
						  atoi(uid->getChild(0)->
						  value->c_str());
						SwVertexMap::iterator i=
						  tile->swVertexMap.find(swid);
						if (i !=
						  tile->swVertexMap.end()) {
							swVertex= i->second;
							//fprintf(stderr,
							//  "swid %p %d %p\n",
							//  tile,swid,swVertex);
						}
					}
				}
				model= loadTrackModel(file->getChild(0)->value,
				  swVertex);
			} else if (*(node->value)=="Dyntrack") {
				model= makeDynTrack(next);
			} else if (*(node->value)=="Transfer") {
				model= makeTransfer(next,
				  file->getChild(0)->value,tile,pos,qdir);
			} else if (*(node->value)=="Forest") {
				model= makeForest(next,tile,pos,qdir);
			} else if (*(node->value)=="Hazard" && file!=NULL) {
				model=
				  loadHazardModel(file->getChild(0)->value);
#if 0
			  	fprintf(stderr,"hazard pos %s %s %s %f %f\n",
				  pos->getChild(0)->value->c_str(),
			  	  pos->getChild(2)->value->c_str(),
			  	  pos->getChild(1)->value->c_str(),x0,z0);
			  	fprintf(stderr,"hazard qdir %s %s %s %s\n",
				  qdir->getChild(0)->value->c_str(),
				  qdir->getChild(1)->value->c_str(),
				  qdir->getChild(2)->value->c_str(),
				  qdir->getChild(3)->value->c_str());
#endif
			} else if (*(node->value)=="Signal") {
#if 0
				fprintf(stderr,"signal %s\n",
				  file->get(0)->c_str());
#endif
				MSTSSignal* signal= findSignalInfo(next);
				model=
				  loadStaticModel(file->getChild(0)->value,
				  signal);
				if (model && signalSwitchStands)
				  model= attachSwitchStand(tile,model,
				    x0+atof(pos->getChild(0)->value->c_str()),
				    z0+atof(pos->getChild(2)->value->c_str()),
				    atof(pos->getChild(1)->value->c_str()));
			} else if (file != NULL) {
				model=
				  loadStaticModel(file->getChild(0)->value);
				if (*(node->value)=="Static" && model)
					model->setNodeMask(1);
#if 0
				if (model) {
					osg::LOD* lod= new osg::LOD();
					lod->addChild(model,0,2000);
					model= lod;
				}
#endif
			}
			if (model == NULL)
				continue;
			if (file && file->getChild(0)->value &&
			  (ignorePolygon.size()>0 || ignoreShapeMap.size()>0) &&
			  ignoreShape(file->getChild(0)->value,
			  x0+atof(pos->getChild(0)->value->c_str()),
			  z0+atof(pos->getChild(2)->value->c_str()),
			  atof(pos->getChild(1)->value->c_str())))
				continue;
			double a,x,y,z;
			osg::Quat(
			  atof(qdir->getChild(0)->value->c_str()),
			  atof(qdir->getChild(1)->value->c_str()),
			  atof(qdir->getChild(2)->value->c_str()),
			  atof(qdir->getChild(3)->value->c_str())).getRotate(
			  a,x,y,z);
#if 0
			if (file && file->getChild(0)->value)
				fprintf(stderr,"%s %lf %lf %lf\n",
				  file->getChild(0)->value->c_str(),
				  x0+atof(pos->getChild(0)->value->c_str()),
				  z0+atof(pos->getChild(2)->value->c_str()),
				  atof(pos->getChild(1)->value->c_str()));
#endif
#if 0
			if (file && file->getChild(0)->value)
				fprintf(stderr,"%s %lf %lf %lf %lf\n",
				  file->getChild(0)->value->c_str(),
				  180*a/3.14159,x,y,z);
			else
				fprintf(stderr,"? %lf %lf %lf %lf\n",
				  180*a/3.14159,x,y,z);
			fprintf(stderr," %lf %lf %lf %lf\n",
			  atof(qdir->getChild(0)->value->c_str()),
			  atof(qdir->getChild(1)->value->c_str()),
			  atof(qdir->getChild(2)->value->c_str()),
			  atof(qdir->getChild(3)->value->c_str()));
#endif
			osg::Quat q;
			q.makeRotate(-a,x,y,z);
//			fprintf(stderr," %lf %lf %lf %lf\n",
//			  q[0],q[1],q[2],q[3]);
			osg::MatrixTransform* mt= new osg::MatrixTransform;
			mt->setMatrix(osg::Matrix(q)*
			 osg::Matrix(1,0,0,0, 0,0,1,0, 0,1,0,0, 0,0,0,1) *
			 osg::Matrix(1,0,0,0, 0,1,0,0, 0,0,1,0,
			  x0+atof(pos->getChild(0)->value->c_str()),
			  z0+atof(pos->getChild(2)->value->c_str()),
			  atof(pos->getChild(1)->value->c_str()),1) );
			mt->addChild(model);
			if (*(node->value)=="Static" &&
			  file && file->getChild(0)->value) {
				mt->setName(*file->getChild(0)->value);
				mt->setNodeMask(0x10);
			}
			tile->models->addChild(mt);
#if 0
			if (atof(pos->getChild(1)->value->c_str()) < 2100) {
				MSTSFileNode* uid= next->children->find("UiD");
				fprintf(stderr,"uidelev %d %f\n",
				  atoi(uid->getChild(0)->value->c_str()),
				  atof(pos->getChild(1)->value->c_str()));
				osg::Vec3d trans= mt->getMatrix().getTrans();
				fprintf(stderr," mt %lf %lf %lf\n",
				  trans[0],trans[1],trans[2]);
				printTree(mt,0);
			}
#endif
		}
	} catch (const char* msg) {
		fprintf(stderr,"loadModels caught %s %s\n",msg,path.c_str());
	} catch (const std::exception& error) {
		fprintf(stderr,"loadModels caught %s %s\n",
		  error.what(),path.c_str());
	}
	}
	makeWater(tile,waterLevelDelta-1,"waterbot.ace",0);
	makeWater(tile,waterLevelDelta-.5,"watermid.ace",1);
	makeWater(tile,waterLevelDelta,"watertop.ace",2);
//	fprintf(stderr,"tile models %d\n",tile->models->getNumChildren());
	cleanStaticModelMap();
//	fprintf(stderr,"cleanStatic\n");
	cleanACECache();
//	fprintf(stderr,"cleanACE\n");
}

//	reads a binary world file
int MSTSRoute::readBinWFile(const char* wfilename, Tile* tile,
  float x0, float z0)
{
	MSTSBFile reader;
	if (reader.open(wfilename))
		return 0;
//	fprintf(stderr,"%s is binary %d\n",wfilename,reader.compressed);
	Byte buf[16];
	reader.getBytes(buf,16);
	int prevCode= 0;
	int remainingBytes= 0;
	float posX,posY,posZ,qDirX,qDirY,qDirZ,qDirW;
	string filename;
	int uid= 0;
	bool print= false;
	bool visible= false;
	TrackSections trackSections;
	float width= 0;
	float height= 0;
	for (;;) {
		int code= reader.getShort();
		int flags= reader.getShort();
		int len= reader.getInt();
		if (code == 0)
			break;
		//fprintf(stderr,"%d %d\n",code,len);
		switch (code) {
		 case 75: // world_file
			reader.getString();
			break;
		 case 6: // dyntrack
		 case 3: // static
		 case 56: // gantry?
		 case 63: // transfer
			prevCode= code;
			remainingBytes= 2*len+8 -1 - reader.getString().size();
			filename= "";
			//fprintf(stderr,"static %d %d %x %d\n",code,len,flags,
			//  remainingBytes);
			//reader.getBytes(NULL,len);
			break;
		 case 5: // trackobj
			prevCode= code;
			remainingBytes= 2*len+8 -1 - reader.getString().size();
			filename= "";
			//fprintf(stderr,"trackobj %d %d %x %d\n",
			//code,len,flags,remainingBytes);
			//reader.getBytes(NULL,len);
			break;
		 case 8: // forest
//			fprintf(stderr,"forest %d %d %x\n",code,len,flags);
			reader.getBytes(NULL,len);
			remainingBytes= len+8;
			break;
		 case 11: // collide
			//fprintf(stderr,"collide %d %d %x\n",code,len,flags);
			reader.getBytes(NULL,len);
			remainingBytes= len+8;
			break;
		 case 17: // signal
			prevCode= code;
			remainingBytes= 2*len+8 -1 - reader.getString().size();
			//fprintf(stderr,"signal %d %d %x\n",code,len,flags);
			//reader.getBytes(NULL,len);
			break;
		 case 60: // platform
			//fprintf(stderr,"platform %d %d %x\n",code,len,flags);
			reader.getBytes(NULL,len);
			remainingBytes= len+8;
			break;
		 case 61: // siding?
			//fprintf(stderr,"platform %d %d %x\n",code,len,flags);
			reader.getBytes(NULL,len);
			remainingBytes= len+8;
			break;
		 case 62: // levelcr
			prevCode= code;
			remainingBytes= 2*len+8 -1 - reader.getString().size();
			//print= true;
			//fprintf(stderr,"levelcr %d %d %x\n",code,len,flags);
			//reader.getBytes(NULL,len);
			break;
		 case 64: // speedpost
			prevCode= code;
			remainingBytes= 2*len+8 -1 - reader.getString().size();
			//fprintf(stderr,"speedpost %d %d %x\n",code,len,flags);
			//reader.getBytes(NULL,len);
			break;
		 case 65: // hazard
			//fprintf(stderr,"hazard %d %d %x\n",code,len,flags);
			reader.getBytes(NULL,len);
			remainingBytes= len+8;
			break;
		 case 76: // watermark
			//fprintf(stderr,"watermark %d %d %x\n",code,len,flags);
			remainingBytes= len+8;
			reader.getBytes(NULL,len);
			break;
		 case 95: // filename
			reader.getString();
			{
				int n= reader.getShort();
				filename= reader.getString(n);
				if (print)
					fprintf(stderr,"  filename %d %ld %s\n",
					  n,filename.size(),
					  filename.c_str());
			}
			break;
		 case 97: // position
			reader.getString();
			posX= reader.getFloat();
			posY= reader.getFloat();
			posZ= reader.getFloat();
			if (print)
				fprintf(stderr,"  position %f %f %f\n",
				  posX,posY,posZ);
			break;
		 case 645: // qdirection
			reader.getString();
			qDirX= reader.getFloat();
			qDirY= reader.getFloat();
			qDirZ= reader.getFloat();
			qDirW= reader.getFloat();
			if (print)
				fprintf(stderr,"  qdir %f %f %f %f\n",
				  qDirX,qDirY,qDirZ,qDirW);
			break;
		 case 279: // viewdbsphere
		 case 284: // vdbidcount
		 case 280: // radius
		 case 283: // vdbid
			reader.getBytes(NULL,len);
			break;
		 case 101: // static detaillevel
		 case 104: // static flags
		 case 105: // collide flags
		 case 119: // sectionidx
			//reader.getString();
			//reader.getInt();
			//fprintf(stderr," %d %x\n",code,reader.getInt());
			reader.getBytes(NULL,len);
			break;
		 case 193: // elevation
			//reader.getString();
			//reader.getFloat();
			//fprintf(stderr," %d %f\n",code,reader.getFloat());
			reader.getBytes(NULL,len);
			break;
		 case 108: // uid
			reader.getString();
			uid= reader.getInt();
			break;
		 case 109: // tracksections
			reader.getString();
			trackSections.clear();
			remainingBytes+= len-1;
			break;
		 case 110: // tracksection
			if (print)
				fprintf(stderr,"tracksection %d %d %x\n",
				  code,len,flags);
			if (len == 26) {
				reader.getString();
				reader.getShort();//section curve code
				reader.getShort();//section curve flags
				reader.getInt();//section curve length
				reader.getString();
				int flag= reader.getInt();//section curve flag
				int id= reader.getInt();//section id
				float d= reader.getFloat();
				float r= reader.getFloat();
				if (d!=0 || r!=0)
					trackSections.push_back(
					  TrackSection(d,r));
				if (print)
				fprintf(stderr," %f %f %d %d %x\n",d,r,
				  (int)trackSections.size(),id,flag);
			} else {
				reader.getBytes(NULL,len);
			}
			break;
		 case 852: // width
			if (len == 5) {
				reader.getString();
				width= reader.getFloat();
			} else {
				reader.getBytes(NULL,len);
			}
			break;
		 case 853: // height
			if (len == 5) {
				reader.getString();
				height= reader.getFloat();
			} else {
				reader.getBytes(NULL,len);
			}
			break;
		 case 823:  // levelCrData
			reader.getString();
			visible= ((reader.getInt()&0x1) == 0);
			reader.getInt();
			//fprintf(stderr," %d %x %x\n",code,reader.getInt(),
			//  reader.getInt());
			break;
		 default:
//			if (print || remainingBytes<0)
//				fprintf(stderr,"unknown %d %d %x\n",
//				  code,len,flags);
			reader.getBytes(NULL,len);
			break;
		}
		remainingBytes-= len+8;
		if (print)
			fprintf(stderr," %d remaining %d %d %x\n",
			  remainingBytes,code,len,flags);
		if (remainingBytes == 0) {
			if (print)
				fprintf(stderr,"prev %d\n",prevCode);
			print= false;
			osg::Node* model= NULL;
			switch (prevCode) {
			  case 62: // levelcr
				//fprintf(stderr,"levelcr %d\n",visible);
				if (visible)
					model= loadStaticModel(&filename);
				break;
			  case 3: // static
			  case 56: // gantry
			  case 17: // signal
			  case 64: // speedpost
				model= loadStaticModel(&filename);
				if (prevCode==3 && model)
					model->setNodeMask(1);
				if (prevCode==17 && model)
					model= attachSwitchStand(tile,model,
					  x0+posX,z0+posZ,posY);
				break;
			  case 5: // trackobj
				{
					Track::SwVertex* swVertex= NULL;
					SwVertexMap::iterator i=
					  tile->swVertexMap.find(uid);
					if (i != tile->swVertexMap.end())
						swVertex= i->second;
					model= loadTrackModel(&filename,
					  swVertex);
				}
				break;
			  case 6: // trackobj
				model= makeDynTrack(trackSections,false);
				break;
			  case 63: // transfer
				model= makeTransfer(&filename,tile,
				  osg::Vec3(posX,posY,posZ),
				  osg::Quat(qDirX,qDirY,qDirZ,qDirW),
				  width,height);
			  default:
				break;
			}
			if (model != NULL) {
				double a,x,y,z;
				osg::Quat(qDirX,qDirY,qDirZ,qDirW).getRotate(
				  a,x,y,z);
				osg::Quat q;
				q.makeRotate(-a,x,y,z);
				osg::MatrixTransform* mt=
				  new osg::MatrixTransform;
				mt->setMatrix(osg::Matrix(q)*
				 osg::Matrix(1,0,0,0, 0,0,1,0, 0,1,0,0,
				  0,0,0,1) *
				 osg::Matrix(1,0,0,0, 0,1,0,0, 0,0,1,0,
				  x0+posX,z0+posZ,posY,1) );
				mt->addChild(model);
				tile->models->addChild(mt);
			}
			remainingBytes= -1;
			visible= false;
			prevCode= 0;
		}
	}
	return 1;
}

//	overrides an MSTS track shape model with the specified OSG shape
void MSTSRoute::overrideTrackModel(string& shapeName, string& modelName)
{
	osg::Node* model= osgDB::readNodeFile(modelName);
	trackModelMap[shapeName]= model;
	model->ref();
}

//	loads a track model and attaches it to the Track data so it
//	can be animated
osg::Node* MSTSRoute::loadTrackModel(string* filename,
  Track::SwVertex* swVertex)
{
//	return NULL;
	if (filename == NULL)
		return NULL;
	int idx= filename->rfind("\\");
	if (idx != string::npos) {
		filename->erase(0,idx+1);
//		fprintf(stderr,"remove tm path %s\n",filename->c_str());
	}
	ModelMap::iterator i= trackModelMap.find(*filename);
	if (i != trackModelMap.end()) {
		osg::Node* model= i->second;
		if (swVertex) {
			model= (osg::Node*)
			  model->clone(osg::CopyOp::DEEP_COPY_NODES);
			SetSwVertexVisitor visitor(swVertex);
			model->accept(visitor);
		}
		return model;
	}
	string path= idx != string::npos ?
	  rShapesDir+dirSep+*filename : gShapesDir+dirSep+*filename;
//	fprintf(stderr,"loading track model %s %p\n",path.c_str(),swVertex);
	MSTSShape shape;
	try {
		shape.readFile(path.c_str(),rTexturesDir.c_str(),
		  gTexturesDir.c_str());
		shape.fixTop();
		osg::Node* model= shape.createModel(0,10,false,true);
		if (wireHeight > 0) {
			string path= wireModelsDir+dirSep+*filename+".osg";
			osg::Node* wire= osgDB::readNodeFile(path);
			osg::Group* g= new osg::Group;
			g->addChild(wire);
			g->addChild(model);
			model= g;
		}
		trackModelMap[*filename]= model;
		model->ref();
		if (swVertex) {
			SetSwVertexVisitor visitor(swVertex);
			model->accept(visitor);
		}
		return model;
	} catch (const char* msg) {
		fprintf(stderr,"loadTrackModel caught %s\n",msg);
		return NULL;
	} catch (const std::exception& error) {
		fprintf(stderr,"loadTrackModel caught %s\n",error.what());
		return NULL;
	}
}

void MSTSRoute::cleanStaticModelMap()
{
	for (ModelMap::iterator i=staticModelMap.begin();
	  i!=staticModelMap.end(); i++) {
		if (i->second == NULL)
			continue;
		if (i->second->referenceCount() <= 1) {
//			fprintf(stderr,"unused %s %d\n",
//			  i->first.c_str(),i->second->referenceCount());
			i->second->unref();
			i->second= NULL;
//		} else {
//			fprintf(stderr,"ref count %s %d\n",
//			  i->first.c_str(),i->second->referenceCount());
		}
	}
}

osg::Node* MSTSRoute::loadHazardModel(string* filename)
{
	if (filename == NULL)
		return NULL;
//	fprintf(stderr,"hazard %s\n",filename->c_str());
	string path= routeDir+dirSep+*filename;
	MSTSFile file;
	file.readFile(path.c_str());
	MSTSFileNode* wf= file.find("Tr_Worldfile");
	if (wf == NULL) {
		fprintf(stderr,"%s not a MSTS hazard file?",
		  path.c_str());
		return NULL;
	}
	MSTSFileNode* fileNm= wf->children->find("FileName");
	if (fileNm == NULL)
		return NULL;
//	fprintf(stderr,"load hazard %s\n",fileNm->getChild(0)->value->c_str());
	osg::Node* model= loadStaticModel(fileNm->getChild(0)->value);
//	fprintf(stderr,"load hazard %p\n",model);
	return model;
}

//	loads a static model
//	just returns a pointer if already loaded
osg::Node* MSTSRoute::loadStaticModel(string* filename, MSTSSignal* signal)
{
//	return NULL;
	if (filename == NULL)
		return NULL;
	int idx= filename->rfind("\\");
	if (idx != string::npos) {
		filename->erase(0,idx+1);
//		fprintf(stderr,"remove path %s\n",filename->c_str());
	}
	ModelMap::iterator i= staticModelMap.find(*filename);
	if (i != staticModelMap.end() && i->second) {
		osg::Node* model= i->second;
		if (signal) {
			model= (osg::Node*)
			  model->clone(osg::CopyOp::DEEP_COPY_NODES);
			SetSignalVisitor visitor(signal);
			model->accept(visitor);
		}
		return model;
	}
	string path= rShapesDir+dirSep+*filename;
//	fprintf(stderr,"loading static model %s\n",path.c_str());
	MSTSShape shape;
	if (signal && strncasecmp(filename->c_str(),"hsuq",4)==0)
		shape.signalLightOffset= new osg::Vec3d(.23,-.23,-.1);
	else if (signal)
		shape.signalLightOffset= new osg::Vec3d(0,0,-.15);
	try {
		shape.readFile(path.c_str(),rTexturesDir.c_str());
	} catch (const char* msg) {
		try {
			path= gShapesDir+dirSep+*filename;
			shape.readFile(path.c_str(),rTexturesDir.c_str(),
			  gTexturesDir.c_str());
		} catch (const char* msg) {
			fprintf(stderr,"loadStaticModel caught %s for %s\n",
			  msg,filename->c_str());
			return NULL;
		}
	}
	try {
		shape.fixTop();
		osg::Node* model= shape.createModel(0);
		if (model == NULL) {
			fprintf(stderr,"failed to load static model %s\n",
			  path.c_str());
			return NULL;
		}
		if (signal) {
			SetSignalVisitor visitor(signal);
			model->accept(visitor);
		}
		staticModelMap[*filename]= model;
		model->ref();
		return model;
	} catch (const char* msg) {
		fprintf(stderr,"loadStaticModel caught %s for %s\n",
		  msg,filename->c_str());
		return NULL;
	} catch (const std::exception& error) {
		fprintf(stderr,"loadStaticModel caught %s for %s\n",
		  error.what(),filename->c_str());
		return NULL;
	}
}

int checkTree(osg::Node* node)
{
	if (node == NULL) {
		fprintf(stderr,"null node\n");
		return 0;
	}
	osg::Geode* geode= dynamic_cast<osg::Geode*>(node);
	if (geode) {
		return 1;
	} else {
		osg::MatrixTransform* mt=
		  dynamic_cast<osg::MatrixTransform*>(node);
		osg::Group* group=
		  dynamic_cast<osg::Group*>(node);
		if (mt) {
			int s= 0;
			for (int i=0; i<mt->getNumChildren(); i++)
				s+= checkTree(mt->getChild(i));
			return s;
		} else if (group) {
			int s= 0;
			for (int i=0; i<group->getNumChildren(); i++)
				s+= checkTree(group->getChild(i));
			return s;
		} else {
			//fprintf(stderr,"unknown %s\n",node->className());
			return 1;
		}
	}
}

float MSTSRoute::Tile::getWaterLevel(int i, int j)
{
	float x= 1-.0625*j;
	float z= .0625*i;
//	fprintf(stderr,"wl %d %d %f %f %f\n",i,j,x,z,
//	  x*(1-z)*nwWaterLevel + x*z*swWaterLevel +
//	  (1-x)*z*seWaterLevel + (1-x)*(1-z)*neWaterLevel);
	return x*(1-z)*nwWaterLevel + x*z*swWaterLevel +
	  (1-x)*z*seWaterLevel + (1-x)*(1-z)*neWaterLevel;
}

//	makes a 3D model for water in a tile
void MSTSRoute::makeWater(Tile* tile, float dl, const char* texture,
  int renderBin)
{
//	fprintf(stderr,"makeWater %d %f\n",drawWater,tile->swWaterLevel);
//	for (int i=0; i<256; i++)
//		fprintf(stderr," %x\n",tile->patches[i].flags);
	if (!drawWater || tile->swWaterLevel==-1e10)
		return;
//	fprintf(stderr,"makeWater %p %f sw=%f se=%f ne=%f nw=%f\n",
//	  tile,dl,tile->swWaterLevel,tile->seWaterLevel,
//	  tile->neWaterLevel,tile->nwWaterLevel);
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::DrawElementsUShort* drawElements=
	  new osg::DrawElementsUShort(GL_QUADS);;
	float x0= 2048*(float)(tile->x-centerTX);
	float z0= 2048*(float)(tile->z-centerTZ);
	int n= 0;
	for (int i=0; i<16; i++) {
		for (int j=0; j<16; j++) {
			Patch* patch= &tile->patches[i*16+j];
			if ((patch->flags&0xc0) == 0)
				continue;
			float x= x0 + 128*(j-8);
			float z= z0 + 128*(8-i);
//			fprintf(stderr,"patch flags %d %d %x %f %f %f %f %f\n",
//			  i,j,patch->flags,x-x0,z-z0,
//			  patch->centerX,patch->centerZ,
//			  tile->getWaterLevel(i,j));
			verts->push_back(osg::Vec3(x,z,
			  tile->getWaterLevel(i,j)+dl));
			verts->push_back(osg::Vec3(x,z-128,
			  tile->getWaterLevel(i+1,j)+dl));
			verts->push_back(osg::Vec3(x+128,z-128,
			  tile->getWaterLevel(i+1,j+1)+dl));
			verts->push_back(osg::Vec3(x+128,z,
			  tile->getWaterLevel(i,j+1)+dl));
			texCoords->push_back(osg::Vec2(.05,.05));
			texCoords->push_back(osg::Vec2(.05,.95));
			texCoords->push_back(osg::Vec2(.95,.95));
			texCoords->push_back(osg::Vec2(.95,.05));
			drawElements->push_back(n);
			drawElements->push_back(n+1);
			drawElements->push_back(n+2);
			drawElements->push_back(n+3);
			n+= 4;
		}
	}
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(1,1,1,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	osg::Vec3Array* norms= new osg::Vec3Array;
	norms->push_back(osg::Vec3(0,0,1));
	geometry->setNormalArray(norms);
	geometry->setNormalBinding(osg::Geometry::BIND_OVERALL);
	geometry->setTexCoordArray(0,texCoords);
	geometry->addPrimitiveSet(drawElements);
	osg::Texture2D* t= new osg::Texture2D;
	string envDir= fixFilenameCase(routeDir+dirSep+"ENVFILES");
	string envTexDir= fixFilenameCase(envDir+dirSep+"TEXTURES");
	string path= fixFilenameCase(envTexDir+dirSep+texture);
	osg::Image* image= readMSTSACE(path.c_str());
	if (image != NULL)
		t->setImage(image);
	t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
	t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
	osg::StateSet* stateSet= geometry->getOrCreateStateSet();
	stateSet->setTextureAttributeAndModes(0,t,osg::StateAttribute::ON);
	osg::Material* m= new osg::Material;
	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	m->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(1,1,1,1));
//	m->setShininess(osg::Material::FRONT_AND_BACK,4.);
	m->setShininess(osg::Material::FRONT_AND_BACK,128.);
	stateSet->setAttribute(m,osg::StateAttribute::ON);
	addShaders(stateSet,.3);
//	stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	if (renderBin > 0)
		stateSet->setRenderBinDetails(renderBin,"DepthSortedBin");
	osg::BlendFunc* bf= new osg::BlendFunc();
	bf->setFunction(osg::BlendFunc::SRC_ALPHA,
	  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
	stateSet->setAttributeAndModes(bf,osg::StateAttribute::ON);
	osg::Geode* geode= new osg::Geode;
	geode->addDrawable(geometry);
	tile->models->addChild(geode);
}

void addClipTriangle(int j1, int j2, int j3, osg::Vec3Array* verts,
  osg::DrawElementsUInt* drawElements, int axis, float w, float h,
  osg::Vec3Array* normals)
{
	float* v= (float*) verts->getDataPointer();
	float* v1= v+3*j1;
	float* v2= v+3*j2;
	float* v3= v+3*j3;
	float half= axis==0 ? w/2 : h/2;
	if (v1[axis]<=-half && v2[axis]<=-half && v3[axis]<=-half)
		return;
	if (v1[axis]>=half && v2[axis]>=half && v3[axis]>=half)
		return;
	if (v2[axis]<v1[axis] && v2[axis]<v3[axis]) {
		addClipTriangle(j2,j3,j1,verts,drawElements,axis,w,h,normals);
		return;
	}
	if (v3[axis]<v1[axis] && v3[axis]<v2[axis]) {
		addClipTriangle(j3,j1,j2,verts,drawElements,axis,w,h,normals);
		return;
	}
	if (v1[axis]>=-half && v2[axis]<=half && v3[axis]<=half) {
		if (axis == 0) {
			addClipTriangle(j1,j2,j3,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j3);
//			fprintf(stderr,"transfer add %7.3f %7.3f %7.3f  "
//			  "%7.3f %7.3f %7.3f  %7.3f %7.3f %7.3f\n",
//			  v1[0],v1[1],v1[2],v2[0],v2[1],v2[2],
//			  v3[0],v3[1],v3[2]);
		}
		return;
	}
	int axis2= 2-axis;
	if (v1[axis]<-half && v2[axis]<-half) {
		float x23= axis==0 ? -half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],-half);
		float y23= segAxisInt(v2[axis],v2[1],v3[axis],v3[1],-half);
		float z23= axis==2 ? -half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],-half);
		float x13= axis==0 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],-half);
		float y13= segAxisInt(v1[axis],v1[1],v3[axis],v3[1],-half);
		float z13= axis==2 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],-half);
//		fprintf(stderr,"transfer cut12-%d %f %f %f %f %f %f\n",axis,
//		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
		j1= verts->getNumElements();
		verts->push_back(osg::Vec3(x13,y13,z13));
		normals->push_back(osg::Vec3(0,0,1));
		j2= verts->getNumElements();
		verts->push_back(osg::Vec3(x23,y23,z23));
		normals->push_back(osg::Vec3(0,0,1));
		if (axis == 0) {
			addClipTriangle(j1,j2,j3,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j3);
		}
	} else if (v1[axis]<-half && v3[axis]<-half) {
		float x12= axis==0 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],-half);
		float y12= segAxisInt(v1[axis],v1[1],v2[axis],v2[1],-half);
		float z12= axis==2 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],-half);
		float x23= axis==0 ? -half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],-half);
		float y23= segAxisInt(v2[axis],v2[1],v3[axis],v3[1],-half);
		float z23= axis==2 ? -half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],-half);
//		fprintf(stderr,"transfer cut13-%d %f %f %f %f %f %f\n",axis,
//		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
		j1= verts->getNumElements();
		verts->push_back(osg::Vec3(x12,y12,z12));
		normals->push_back(osg::Vec3(0,0,1));
		j3= verts->getNumElements();
		verts->push_back(osg::Vec3(x23,y23,z23));
		normals->push_back(osg::Vec3(0,0,1));
		if (axis == 0) {
			addClipTriangle(j1,j2,j3,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j3);
		}
	} else if (v1[axis]<-half) {
		float x12= axis==0 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],-half);
		float z12= axis==2 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],-half);
		float y12= segAxisInt(v1[axis],v1[1],v2[axis],v2[1],-half);
		float x13= axis==0 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],-half);
		float z13= axis==2 ? -half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],-half);
		float y13= segAxisInt(v1[axis],v1[1],v3[axis],v3[1],-half);
//		fprintf(stderr,"transfer cut1-%d %f %f %f %f %f %f\n",axis,
//		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
		j1= verts->getNumElements();
		verts->push_back(osg::Vec3(x12,y12,z12));
		normals->push_back(osg::Vec3(0,0,1));
		int j4= verts->getNumElements();
		verts->push_back(osg::Vec3(x13,y13,z13));
		normals->push_back(osg::Vec3(0,0,1));
		if (axis == 0) {
			addClipTriangle(j1,j2,j3,verts,drawElements,2-axis,w,h,
			  normals);
			addClipTriangle(j1,j3,j4,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j3);
			drawElements->push_back(j1);
			drawElements->push_back(j3);
			drawElements->push_back(j4);
		}
	} else if (v2[axis]>half && v3[axis]>half) {
		float x12= axis==0 ? half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],half);
		float z12= axis==2 ? half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],half);
		float y12= segAxisInt(v1[axis],v1[1],v2[axis],v2[1],half);
		float x13= axis==0 ? half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],half);
		float z13= axis==2 ? half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],half);
		float y13= segAxisInt(v1[axis],v1[1],v3[axis],v3[1],half);
//		fprintf(stderr,"transfer cut23+%d %f %f %f %f %f %f\n",axis,
//		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
		j2= verts->getNumElements();
		verts->push_back(osg::Vec3(x12,y12,z12));
		normals->push_back(osg::Vec3(0,0,1));
		j3= verts->getNumElements();
		verts->push_back(osg::Vec3(x13,y13,z13));
		normals->push_back(osg::Vec3(0,0,1));
		if (axis == 0) {
			addClipTriangle(j1,j2,j3,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j3);
		}
	} else if (v2[axis]>half) {
		float x12= axis==0 ? half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],half);
		float z12= axis==2 ? half :
		  segAxisInt(v1[axis],v1[axis2],v2[axis],v2[axis2],half);
		float y12= segAxisInt(v1[axis],v1[1],v2[axis],v2[1],half);
		float x23= axis==0 ? half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],half);
		float z23= axis==2 ? half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],half);
		float y23= segAxisInt(v2[axis],v2[1],v3[axis],v3[1],half);
//		fprintf(stderr,"transfer cut2+%d %f %f %f %f %f %f\n",axis,
//		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
		j2= verts->getNumElements();
		verts->push_back(osg::Vec3(x12,y12,z12));
		normals->push_back(osg::Vec3(0,0,1));
		int j4= verts->getNumElements();
		verts->push_back(osg::Vec3(x23,y23,z23));
		normals->push_back(osg::Vec3(0,0,1));
		if (axis == 0) {
			addClipTriangle(j1,j2,j4,verts,drawElements,2-axis,w,h,
			  normals);
			addClipTriangle(j1,j4,j3,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j4);
			drawElements->push_back(j1);
			drawElements->push_back(j4);
			drawElements->push_back(j3);
		}
	} else if (v3[axis]>half) {
		float x23= axis==0 ? half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],half);
		float z23= axis==2 ? half :
		  segAxisInt(v2[axis],v2[axis2],v3[axis],v3[axis2],half);
		float y23= segAxisInt(v2[axis],v2[1],v3[axis],v3[1],half);
		float x13= axis==0 ? half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],half);
		float z13= axis==2 ? half :
		  segAxisInt(v1[axis],v1[axis2],v3[axis],v3[axis2],half);
		float y13= segAxisInt(v1[axis],v1[1],v3[axis],v3[1],half);
//		fprintf(stderr,"transfer cut3+%d %f %f %f %f %f %f\n",axis,
//		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
		j3= verts->getNumElements();
		verts->push_back(osg::Vec3(x23,y23,z23));
		normals->push_back(osg::Vec3(0,0,1));
		int j4= verts->getNumElements();
		verts->push_back(osg::Vec3(x13,y13,z13));
		normals->push_back(osg::Vec3(0,0,1));
		if (axis == 0) {
			addClipTriangle(j1,j2,j3,verts,drawElements,2-axis,w,h,
			  normals);
			addClipTriangle(j1,j3,j4,verts,drawElements,2-axis,w,h,
			  normals);
		} else {
			drawElements->push_back(j1);
			drawElements->push_back(j2);
			drawElements->push_back(j3);
			drawElements->push_back(j1);
			drawElements->push_back(j3);
			drawElements->push_back(j4);
		}
	} else {
		fprintf(stderr,"transfer dont add %f %f %f %f %f %f\n",
		  v1[0],v1[2],v2[0],v2[2],v3[0],v3[2]);
	}
}

osg::Node* MSTSRoute::makeTransfer(MSTSFileNode* transfer, string* filename,
  Tile* tile, MSTSFileNode* pos, MSTSFileNode* qdir)
{
	osg::Quat quat(
	  atof(qdir->getChild(0)->value->c_str()),
	  atof(qdir->getChild(1)->value->c_str()),
	  atof(qdir->getChild(2)->value->c_str()),
	  atof(qdir->getChild(3)->value->c_str()));
	osg::Vec3 center= osg::Vec3(atof(pos->getChild(0)->value->c_str()),
	  atof(pos->getChild(1)->value->c_str()),
	  atof(pos->getChild(2)->value->c_str()));
	MSTSFileNode* wid= transfer->children->find("Width");
	MSTSFileNode* hgt= transfer->children->find("Height");
	float w= atof(wid->getChild(0)->value->c_str());
	float h= atof(hgt->getChild(0)->value->c_str());
	return makeTransfer(filename,tile,center,quat,w,h);
}

osg::Node* MSTSRoute::makeTransfer(string* filename, Tile* tile,
  osg::Vec3 center, osg::Quat quat, float w, float h)
{
	float x0= 2048*(tile->x-centerTX);
	float z0= 2048*(tile->z-centerTZ);
	double a,x,y,z;
	quat.getRotate(a,x,y,z);
	osg::Quat q;
	q.makeRotate(a,x,y,z);
	osg::Matrix rot(q);
	q.makeRotate(-a,x,y,z);
	osg::Matrix rot1(q);
//	if (w<8 || h<8)
//		fprintf(stderr,"small transfer %s %f %f\n",
//		  filename->c_str(),w,h);
//	fprintf(stderr,"transfer %s %f %f\n",filename->c_str(),w,h);
//	fprintf(stderr,"transfer rot %lf %lf %lf %lf\n",180*a/3.14159,x,y,z);
//	fprintf(stderr,"transfer pos %lf %lf %lf\n",
//	  center[0],center[1],center[2]);
	osg::Texture2D* t= new osg::Texture2D;
	t->setDataVariance(osg::Object::DYNAMIC);
	string path= rTexturesDir+dirSep+(*filename);
	osg::Image* image= readMSTSACE(path.c_str());
	if (image != NULL)
		t->setImage(image);
	t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
	t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
	t->setBorderColor(osg::Vec4(.3,.3,.3,0));
	osg::Material* mat= new osg::Material;
//	mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
//	mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(1,1,1,1));
	mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(0,0,0,1));
	osg::Geode* geode= new osg::Geode;
	for (int i=0; i<tile->terrModel->getNumDrawables(); i++) {
		osg::Geometry* patchGeom=
		  dynamic_cast<osg::Geometry*>(tile->terrModel->getDrawable(i));
		if (!patchGeom)
			continue;
		osg::Vec3Array* pVerts=
		  (osg::Vec3Array*) patchGeom->getVertexArray();
		float* v= (float*) pVerts->getDataPointer();
		float minx= 1e10;
		float maxx= -1e10;
		float minz= 1e10;
		float maxz= -1e10;
		for (int j=0; j<pVerts->getNumElements(); j++) {
			int j3= j*3;
			osg::Vec3 p=
			  rot1*(osg::Vec3(v[j3]-x0,v[j3+2],v[j3+1]-z0)-center);
			if (minx > p[0])
				minx= p[0];
			if (maxx < p[0])
				maxx= p[0];
			if (minz > p[2])
				minz= p[2];
			if (maxz < p[2])
				maxz= p[2];
		}
		if (maxx<-w/2 || minx>w/2 || maxz<-h/2 || minz>h/2)
			continue;
//		fprintf(stderr,"transfer patch %d %p\n",i,tile);
		osg::Vec3Array* pNormals=
		  (osg::Vec3Array*) patchGeom->getVertexArray();
		float* n= (float*) pNormals->getDataPointer();
		osg::Vec3Array* verts= new osg::Vec3Array;
		osg::Vec3Array* normals= new osg::Vec3Array;
		for (int j=0; j<pVerts->getNumElements(); j++) {
			int j3= j*3;
			osg::Vec3 p= rot1*
			  (osg::Vec3(v[j3]-x0,v[j3+2],v[j3+1]-z0)-center);
//			if ((-w/2<=p[0] && p[0]<=w/2) &&
//			  (-h/2<=p[2] && p[2]<=h/2)) {
//				p[1]+= .05;
//			}
			verts->push_back(p);
			p= rot1*(osg::Vec3(n[j3],n[j3+2],n[j3+1]));
			normals->push_back(p);
		}
		osg::Vec3 sum(0,0,0);
		osg::Geometry* geometry= new osg::Geometry;
		for (int j=0; j<patchGeom->getNumPrimitiveSets(); j++) {
			osg::DrawElementsUInt* patchDE= (osg::DrawElementsUInt*)
			  patchGeom->getPrimitiveSet(j)->getDrawElements();
			osg::DrawElementsUInt* drawElements=
			  new osg::DrawElementsUInt(GL_TRIANGLES);;
			for (int k=0; k<patchDE->getNumIndices(); k+=3) {
				int j1= patchDE->getElement(k);
				int j2= patchDE->getElement(k+1);
				int j3= patchDE->getElement(k+2);
				addClipTriangle(j1,j2,j3,verts,drawElements,
				  0,w,h,normals);
			}
//			fprintf(stderr,"transfer triangles %d\n",
//			  drawElements->getNumIndices());
			if (drawElements->getNumIndices() == 0) {
				drawElements->unref();
				continue;
			}
			geometry->addPrimitiveSet(drawElements);
			v= (float*) verts->getDataPointer();
			for (int k=0; k<drawElements->getNumIndices(); k+=3) {
				int j1= drawElements->getElement(k);
				int j2= drawElements->getElement(k+1);
				int j3= drawElements->getElement(k+2);
				osg::Vec3 p1(v[j1*3],v[j1*3+1],v[j1*3+2]);
				osg::Vec3 p2(v[j2*3],v[j2*3+1],v[j2*3+2]);
				osg::Vec3 p3(v[j3*3],v[j3*3+1],v[j3*3+2]);
				osg::Vec3 cross= (p2-p1)^(p3-p1);
				cross.normalize();
				sum+= cross;
			}
		}
		geometry->setVertexArray(verts);
		geometry->setColorArray(patchGeom->getColorArray());
		geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
		geometry->setNormalArray(normals);
		geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
		sum.normalize();
//		fprintf(stderr,"transfer normal %f %f %f\n",
//		  sum[0],sum[1],sum[2]);
		osg::Vec2Array* texCoords= new osg::Vec2Array;
		float* xyz= (float*) verts->getDataPointer();
		for (int j=0; j<verts->getNumElements(); j++) {
			int j3= j*3;
			float u= xyz[j3]/w + .5;
			float v= -xyz[j3+2]/h + .5;
			texCoords->push_back(osg::Vec2(u,v));
//			for (int k=0; k<3; k++)
//				xyz[j3+k]-= .03*sum[k];
		}
		geometry->setTexCoordArray(0,texCoords);
		osg::StateSet* stateSet= geometry->getOrCreateStateSet();
		stateSet->setTextureAttributeAndModes(0,t,
		  osg::StateAttribute::ON);
	//	stateSet->setAttribute(mat,osg::StateAttribute::ON);
		stateSet->setAttributeAndModes(mat,osg::StateAttribute::ON);
//		stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//		stateSet->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
		stateSet->setRenderBinDetails(3,"DepthSortedBin");
		osg::Depth* depth= new osg::Depth(osg::Depth::LEQUAL,0,1,false);
		stateSet->setAttribute(depth,osg::StateAttribute::ON);
		osg::PolygonOffset* offset= new osg::PolygonOffset(-10,-10);
		stateSet->setAttributeAndModes(offset,osg::StateAttribute::ON);
		osg::BlendFunc* bf= new osg::BlendFunc();
		bf->setFunction(osg::BlendFunc::SRC_ALPHA,
		  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
		stateSet->setAttributeAndModes(bf,osg::StateAttribute::ON);
		stateSet->setMode(GL_LIGHTING,osg::StateAttribute::ON);
		geode->addDrawable(geometry);
	}
	return geode;
}

struct Random {
	int value;
	Random(int seed=12345) { value= seed; };
	double next() {
		value= (value*2661+36979) % 175000;
		return value/(double)(175000-1);
	};
};

osg::Geometry* makeBBTree(osg::Texture2D* t, float w, float h, float scale)
{
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec3Array* normals= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::DrawElementsUShort* drawElements=
	  new osg::DrawElementsUShort(GL_QUADS);;
	verts->push_back(osg::Vec3(-w/2*scale,0,0));
	verts->push_back(osg::Vec3(w/2*scale,0,0));
	verts->push_back(osg::Vec3(w/2*scale,h*scale,0));
	verts->push_back(osg::Vec3(-w/2*scale,h*scale,0));
	normals->push_back(osg::Vec3(0,0,1));
	normals->push_back(osg::Vec3(0,0,1));
	normals->push_back(osg::Vec3(0,0,1));
	normals->push_back(osg::Vec3(0,0,1));
	texCoords->push_back(osg::Vec2(0,1));
	texCoords->push_back(osg::Vec2(1,1));
	texCoords->push_back(osg::Vec2(1,0));
	texCoords->push_back(osg::Vec2(0,0));
	drawElements->push_back(3);
	drawElements->push_back(2);
	drawElements->push_back(1);
	drawElements->push_back(0);
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(1,1,1,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(drawElements);
	geometry->setTexCoordArray(0,texCoords);
	osg::StateSet* stateSet= geometry->getOrCreateStateSet();
	stateSet->setTextureAttributeAndModes(0,t,osg::StateAttribute::ON);
//	stateSet->setAttribute(mat,osg::StateAttribute::ON);
//	osg::TexEnvFilter* lodBias= new osg::TexEnvFilter(-3);
//	stateSet->setTextureAttribute(0,lodBias);
	osg::Material* m= new osg::Material;
	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.99,.99,.99,1));
	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.0,.0,.0,1));
//	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
//	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	stateSet->setAttribute(m,osg::StateAttribute::ON);
	osg::AlphaFunc* af= new osg::AlphaFunc(osg::AlphaFunc::GREATER,.6);
	stateSet->setAttributeAndModes(af,osg::StateAttribute::ON);
	stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//	stateSet->setRenderBinDetails(3,"DepthSortedBin");
	return geometry;
}

int addCrossTree(int vIndex, float w, float h, float scale,
  float x, float y, float z, osg::Vec3Array* verts, osg::Vec3Array* normals,
  osg::Vec2Array* texCoords, osg::DrawElementsUShort* drawElements)
{
	verts->push_back(osg::Vec3(-w/2*scale+x,y,z));
	verts->push_back(osg::Vec3(w/2*scale+x,y,z));
	verts->push_back(osg::Vec3(w/2*scale+x,h*scale+y,z));
	verts->push_back(osg::Vec3(-w/2*scale+x,h*scale+y,z));
	normals->push_back(osg::Vec3(0,0,1));
	normals->push_back(osg::Vec3(0,0,1));
	normals->push_back(osg::Vec3(0,0,1));
	normals->push_back(osg::Vec3(0,0,1));
	texCoords->push_back(osg::Vec2(0,1));
	texCoords->push_back(osg::Vec2(1,1));
	texCoords->push_back(osg::Vec2(1,0));
	texCoords->push_back(osg::Vec2(0,0));
	drawElements->push_back(3+vIndex);
	drawElements->push_back(2+vIndex);
	drawElements->push_back(1+vIndex);
	drawElements->push_back(0+vIndex);
	drawElements->push_back(0+vIndex);
	drawElements->push_back(1+vIndex);
	drawElements->push_back(2+vIndex);
	drawElements->push_back(3+vIndex);
	vIndex+= 4;
	verts->push_back(osg::Vec3(x,y,-w/2*scale+z));
	verts->push_back(osg::Vec3(x,y,w/2*scale+z));
	verts->push_back(osg::Vec3(x,h*scale+y,w/2*scale+z));
	verts->push_back(osg::Vec3(x,h*scale+y,-w/2*scale+z));
	normals->push_back(osg::Vec3(1,0,0));
	normals->push_back(osg::Vec3(1,0,0));
	normals->push_back(osg::Vec3(1,0,0));
	normals->push_back(osg::Vec3(1,0,0));
	texCoords->push_back(osg::Vec2(0,1));
	texCoords->push_back(osg::Vec2(1,1));
	texCoords->push_back(osg::Vec2(1,0));
	texCoords->push_back(osg::Vec2(0,0));
	drawElements->push_back(3+vIndex);
	drawElements->push_back(2+vIndex);
	drawElements->push_back(1+vIndex);
	drawElements->push_back(0+vIndex);
	drawElements->push_back(0+vIndex);
	drawElements->push_back(1+vIndex);
	drawElements->push_back(2+vIndex);
	drawElements->push_back(3+vIndex);
	vIndex+= 4;
	return vIndex;
}

osg::Node* MSTSRoute::makeForest(MSTSFileNode* transfer,
  Tile* tile, MSTSFileNode* pos, MSTSFileNode* qdir)
{
	MSTSFileNode* treeTexture= transfer->children->find("TreeTexture");
	MSTSFileNode* scaleRange= transfer->children->find("ScaleRange");
	MSTSFileNode* area= transfer->children->find("Area");
	MSTSFileNode* size= transfer->children->find("TreeSize");
	MSTSFileNode* population= transfer->children->find("Population");
	if (transfer==NULL || pos==NULL || qdir==NULL || treeTexture==NULL ||
	  scaleRange==NULL || area==NULL || size==NULL || population==NULL) {
//		fprintf(stderr,"forest %p %p %p %p %p %p %p %p\n",transfer,
//		  pos,qdir,treeTexture,scaleRange,area,size,population);
		return NULL;
	}
	float x0= 2048*(tile->x-centerTX);
	float z0= 2048*(tile->z-centerTZ);
	double a,x,y,z;
	osg::Quat(
	  atof(qdir->getChild(0)->value->c_str()),
	  atof(qdir->getChild(1)->value->c_str()),
	  atof(qdir->getChild(2)->value->c_str()),
	  atof(qdir->getChild(3)->value->c_str())).getRotate(a,x,y,z);
	osg::Quat q;
	q.makeRotate(a,x,y,z);
	osg::Matrix rot(q);
	q.makeRotate(-a,x,y,z);
	osg::Matrix rot1(q);
	osg::Vec3 center= osg::Vec3(atof(pos->getChild(0)->value->c_str()),
	  atof(pos->getChild(1)->value->c_str()),
	  atof(pos->getChild(2)->value->c_str()));
	Tile* t12= findTile(tile->x,tile->z-1);
	Tile* t21= findTile(tile->x+1,tile->z);
	Tile* t22= findTile(tile->x+1,tile->z-1);
	float a0= center[1];//getAltitude(center[0],center[2],tile,t12,t21,t22);
	float scale= atof(scaleRange->getChild(0)->value->c_str());
	float range= atof(scaleRange->getChild(1)->value->c_str());
	float areaW= atof(area->getChild(0)->value->c_str());
	float areaH= atof(area->getChild(1)->value->c_str());
	float w= atof(size->getChild(0)->value->c_str());
	float h= atof(size->getChild(1)->value->c_str());
	int pop= atoi(population->getChild(0)->value->c_str());
//	fprintf(stderr,"forest %s %.2f %.2f %.2f %.2f %.2f %.2f %d\n",
//	  treeTexture->getChild(0)->value->c_str(),
//	  scale,range,areaW,areaH,w,h,pop);
	osg::Texture2D* texture= new osg::Texture2D;
//	texture->setDataVariance(osg::Object::DYNAMIC);
	string path= rTexturesDir+dirSep+
	  treeTexture->getChild(0)->value->c_str();
	osg::Image* image= readMSTSACE(path.c_str());
	if (image != NULL)
		texture->setImage(image);
	texture->setWrap(osg::Texture2D::WRAP_S,
	  osg::Texture2D::CLAMP_TO_EDGE);
	texture->setWrap(osg::Texture2D::WRAP_T,
	  osg::Texture2D::CLAMP_TO_EDGE);
	osg::Billboard* bb= new osg::Billboard();
	bb->setMode(osg::Billboard::POINT_ROT_EYE);
	bb->setNormal(osg::Vec3f(0,0,1));
	bb->setAxis(osg::Vec3f(0,1,0));
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec3Array* normals= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::DrawElementsUShort* drawElements=
	  new osg::DrawElementsUShort(GL_QUADS);;
	int vIndex= 0;
	Random random;
	int nh= (int)ceil(sqrt(pop*areaH/areaW));
	int nv= (int)ceil(pop/(double)nh);
#ifdef ORMETHOD
	nh= pop;
	nv= 1;
#endif
	for (int i=0; i<nh; i++) {
		if (i==nh-1 && nv>pop)
			nv= pop;
//		fprintf(stderr,"forest %d %d %d %d\n",i,nh,nv,pop);
		for (int j=0; j<nv; j++) {
#ifdef ORMETHOD
			float s= random.next();
			float t= random.next();
#else
			float s= (i+.5+.9*(random.next()-.5))/(float)nh;
			float t= (j+.5+.9*(random.next()-.5))/(float)nv;
#endif
//			float t= (i+.5+.6*(random.next()-.5))/(float)nh;
//			float s= (j+.5+.6*(random.next()-.5))/(float)nv;
			float size= scale + (range-scale)*random.next();
//			float x= (s-.5)*areaW/2;
//			float z= (t-.5)*areaH/2;
#ifdef ORMETHOD
			float x= (s-.5)*(areaW-size);
			float z= (t-.5)*(areaH-size);
#else
			float x= (s-.5)*(areaW>size?areaW-size:0);
			float z= (t-.5)*(areaH>size?areaH-size:0);
#endif
			osg::Vec3 p= rot*osg::Vec3(x,0,z) + center;
			float a= getAltitude(p[0],p[2],tile,t12,t21,t22);
			bb->addDrawable(makeBBTree(texture,w,h,size),
			  osg::Vec3f(x,a-a0,z));
			vIndex= addCrossTree(vIndex,w,h,size,x,a-a0,z,
			  verts,normals,texCoords,drawElements);
//			fprintf(stderr,"forest %d %d %f %f %d\n",i,j,s,t,pop);
			if (pop-- == 0)
				break;
		}
	}
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(1,1,1,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->setNormalArray(normals);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(drawElements);
	geometry->setTexCoordArray(0,texCoords);
	osg::StateSet* stateSet= geometry->getOrCreateStateSet();
	stateSet->setTextureAttributeAndModes(0,texture,
	  osg::StateAttribute::ON);
//	osg::TexEnvFilter* lodBias= new osg::TexEnvFilter(-3);
//	stateSet->setTextureAttribute(0,lodBias);
	osg::Material* m= new osg::Material;
	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.99,.99,.99,1));
	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.0,.0,.0,1));
//	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
//	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	stateSet->setAttribute(m,osg::StateAttribute::ON);
	stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::AlphaFunc* af= new osg::AlphaFunc(osg::AlphaFunc::GREATER,.6);
	stateSet->setAttributeAndModes(af,osg::StateAttribute::ON);
	osg::Geode* geode= new osg::Geode;
	geode->addDrawable(geometry);
	osg::LOD* lod= new osg::LOD();
#if 1
	lod->addChild(bb,0,500);
	lod->addChild(geode,500,4000);
#else
	lod->addChild(geode,0,4000);
#endif
	return lod;
}

//	makes 3D models for dynamic track
osg::Node* MSTSRoute::makeDynTrack(MSTSFileNode* dynTrack)
{
	MSTSFileNode* sections= dynTrack->children->find("TrackSections");
	if (sections == NULL)
		return NULL;
	bool bridge= false;
	MSTSFileNode* staticFlags= dynTrack->children->find("StaticFlags");
	if (staticFlags != NULL) {
		MSTSFileNode* p= staticFlags->getChild(0);
		if (p && p->value &&
		  strtol(p->value->c_str(),NULL,16)>0x10000000)
			bridge= true;
//		fprintf(stderr,"bridge %d\n",bridge);
	}
	TrackSections trackSections;
	for (MSTSFileNode* node=sections->children; node!=NULL;
	  node=node->next) {
		if (node->value == NULL)
			continue;
		MSTSFileNode* p= node->next->getChild(2);
		if (p==NULL || p->value==NULL)
			continue;
		int id= atoi(p->value->c_str());
		if (id < 0)
			continue;
		p= node->next->getChild(3);
		if (p==NULL || p->value==NULL)
			continue;
		float d= atof(p->value->c_str());
		if (d == 0)
			continue;
		p= node->next->getChild(4);
		if (p==NULL || p->value==NULL)
			continue;
		float r= atof(p->value->c_str());
		trackSections.push_back(TrackSection(d,r));
	}
	return makeDynTrack(trackSections,bridge);
}

osg::Node* MSTSRoute::makeDynTrack(TrackSections& trackSections, bool bridge)
{
	if (dynTrackBase == NULL && srDynTrack)
		makeSRDynTrackShapes();
	else if (dynTrackBase == NULL && ustDynTrack)
		makeUSTDynTrackShapes();
	else if (dynTrackBase==NULL && makeDynTrackShapes())
		;
	else if (dynTrackBase==NULL && makeUSTDynTrackShapes())
		;
	Track track;
	Track::Vertex* pv= track.addVertex(Track::VT_SIMPLE,0,0,0);
	float x= 0;
	float y= 0;
	float dx= 1;
	float dy= 0;
	for (int n=0; n<trackSections.size(); n++) {
		float d= trackSections[n].dist;
		float r= trackSections[n].radius;
		if (r == 0) {
			x+= dx*d;
			y+= dy*d;
		} else {
			float x0= dx;
			float y0= dy;
			float cs= .9998477;
			float sn= -.0174524;
			//float sn= d>0 ? .0174524 : -.0174524;
			float sr= d>0 ? r : -r;
			int m= (int)(abs(d/sn));
			for (int i=0; i<m; i++) {
				float x1= cs*x0 + sn*y0;
				float y1= cs*y0 - sn*x0;
				Track::Vertex* v=
				  track.addVertex(Track::VT_SIMPLE,
				  y+sr*(1-x1),x+r*y1,0);
				track.addEdge(Track::ET_STRAIGHT,pv,
				  n==0?0:1,v,0);
				x0= x1;
				y0= y1;
				pv= v;
			}
			if (d < 0) {
				x+= r*sin(-d);
				y-= r*(1-cos(d));
			} else {
				x+= r*sin(d);
				y+= r*(1-cos(d));
			}
			cs= cos(-d);
			sn= sin(-d);
			x0= cs*dx + sn*dy;
			y0= cs*dy - sn*dx;
			dx= x0;
			dy= y0;
		}
		Track::Vertex* v= track.addVertex(Track::VT_SIMPLE,y,x,0);
		track.addEdge(Track::ET_STRAIGHT,pv,n==0?0:1,v,0);
		pv= v;
	}
	osg::Geode* geode= new osg::Geode();
	track.shape= dynTrackRails;
	osg::Geometry* g= track.makeGeometry();
	osg::StateSet* stateSet= g->getOrCreateStateSet();
	osg::Material* m= new osg::Material;
	m->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
	m->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	m->setSpecular(osg::Material::FRONT_AND_BACK,osg::Vec4(1,1,1,1));
//	m->setShininess(osg::Material::FRONT_AND_BACK,4.);
	m->setShininess(osg::Material::FRONT_AND_BACK,128.);
	stateSet->setAttribute(m,osg::StateAttribute::ON);
	stateSet->setMode(GL_LIGHTING,osg::StateAttribute::ON);
	stateSet->setRenderBinDetails(5,"RenderBin");
	addShaders(stateSet,.3);
	geode->addDrawable(g);
	if (!bridge) {
		track.shape= dynTrackBase;
		g= track.makeGeometry();
		stateSet= g->getOrCreateStateSet();
		m= new osg::Material;
		m->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.6,.6,.6,1));
		m->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.4,.4,.4,1));
		stateSet->setAttribute(m,osg::StateAttribute::ON);
		addShaders(stateSet,1);
		if (!ustDynTrack) {
		stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
//		osg::TexEnvFilter* lodBias= new osg::TexEnvFilter(-3);
//		stateSet->setTextureAttribute(0,lodBias);
		osg::BlendFunc* bf= new osg::BlendFunc();
		bf->setFunction(osg::BlendFunc::SRC_ALPHA,
		  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
		stateSet->setAttributeAndModes(bf,osg::StateAttribute::ON);
		}
		geode->addDrawable(g);
		if (bermHeight > 0) {
			track.shape= dynTrackBerm;
			g= track.makeGeometry();
			geode->addDrawable(g);
		}
	} else if (bridgeBase) {
		track.shape= dynTrackBridge;
		g= track.makeGeometry();
		geode->addDrawable(g);
	}
	if (!bridge && dynTrackTies) {
		track.shape= dynTrackTies;
		g= track.makeGeometry();
		stateSet= g->getOrCreateStateSet();
		m= new osg::Material;
		m->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.6,.6,.6,1));
		m->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.4,.4,.4,1));
		stateSet->setAttribute(m,osg::StateAttribute::ON);
//		stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
		stateSet->setRenderBinDetails(11,"DepthSortedBin");
//		osg::TexEnvFilter* lodBias= new osg::TexEnvFilter(-3);
//		stateSet->setTextureAttribute(0,lodBias);
		osg::BlendFunc* bf= new osg::BlendFunc();
		bf->setFunction(osg::BlendFunc::SRC_ALPHA,
		  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
		stateSet->setAttributeAndModes(bf,osg::StateAttribute::ON);
		addShaders(stateSet,1);
		geode->addDrawable(g);
	}
	if (wireHeight > 0) {
		track.shape= dynTrackWire;
		g= track.makeGeometry();
		geode->addDrawable(g);
	}
	osg::MatrixTransform* mt= new osg::MatrixTransform;
	mt->setMatrix(osg::Matrix(1,0,0,0, 0,0,1,0, 0,1,0,0, 0,0,0,1));
	mt->addChild(geode);
	return mt;
}

//	makes profile information for dynamic track
bool MSTSRoute::makeDynTrackShapes()
{
	string path= rTexturesDir+dirSep+"acleantrack1.ace";
	osg::Texture2D* t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"acleantrack1.ace";
		t= readCacheACEFile(path.c_str());
	}
	if (t == NULL)
		return false;
	dynTrackBase= new TrackShape;
	dynTrackBase->texture= new Texture;
	dynTrackBase->texture->texture= t;
	t->ref();
//	dynTrackBase->offsets.push_back(TrackShape::Offset(-2.5,-.2));
//	dynTrackBase->offsets.push_back(TrackShape::Offset(2.5,-.2));
//	dynTrackBase->surfaces.push_back(
//	  TrackShape::Surface(0,1,-.1389,.862,6,4));
//	OR uses -.153916 .862105
//	default MSTS track uses -.139 .862 and .2 per meter
	dynTrackBase->offsets.push_back(TrackShape::Offset(-2.5,-.18));
	dynTrackBase->offsets.push_back(TrackShape::Offset(-1.5,-.2));
	dynTrackBase->offsets.push_back(TrackShape::Offset(1.5,-.2));
	dynTrackBase->offsets.push_back(TrackShape::Offset(2.5,-.18));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(0,1,-.139,.062,5,4));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(1,2,.062,.662,5,4));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(2,3,.662,.862,5,4));
	dynTrackBase->matchOffsets();
	path= rTexturesDir+dirSep+"acleantrack2.ace";
	t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"acleantrack2.ace";
		t= readCacheACEFile(path.c_str());
	}
	dynTrackBridge= new TrackShape;
	dynTrackBridge->texture= new Texture;
	dynTrackBridge->texture->texture= t;
	t->ref();
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.5,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.5,-.2));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1.5,-.2));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1.5,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1,.4));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1,.4));
	if (bermHeight > 0) {
		dynTrackBridge->offsets.push_back(
		  TrackShape::Offset(-1.1-.2*bermHeight,.4+bermHeight));
		dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.1,.4));
		dynTrackBridge->offsets.push_back(TrackShape::Offset(1.1,.4));
		dynTrackBridge->offsets.push_back(
		  TrackShape::Offset(1.1+.2*bermHeight,.4+bermHeight));
	}
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(0,1,-.139,.062,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(1,2,.062,.662,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(2,3,.662,.862,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(3,0,.062,.662,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(4,5,-.139,.062,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(6,7,.662,.862,5,4));
	if (bermHeight > 0) {
		dynTrackBridge->surfaces.push_back(
		  TrackShape::Surface(8,9,-.139,.062,5,4));
		dynTrackBridge->surfaces.push_back(
		  TrackShape::Surface(10,11,.662,.862,5,4));
	}
	dynTrackBridge->matchOffsets();
	dynTrackRails= new TrackShape;
	dynTrackRails->texture= new Texture;
	dynTrackRails->texture->texture= t;
	t->ref();
	dynTrackRails->offsets.push_back(TrackShape::Offset(.718,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.868,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.718,-.2));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.868,-.2));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.718,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.868,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.718,-.2));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.868,-.2));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(0,1,.12,.22,-4,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(2,0,.01,.11,-4,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(1,3,.11,.01,-4,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(5,4,.12,.22,-4,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(4,6,.01,.11,-4,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(7,5,.11,.01,-4,4));
	dynTrackRails->matchOffsets();
	if (bermHeight > 0) {
		string path= rTexturesDir+dirSep+"acleantrackbase.ace";
		osg::Texture2D* t= readCacheACEFile(path.c_str());
		dynTrackBerm= new TrackShape;
		dynTrackBerm->texture= new Texture;
		dynTrackBerm->texture->texture= t;
		t->ref();
		dynTrackBerm->offsets.push_back(TrackShape::Offset(
		  -2.75-1.5*bermHeight,bermHeight));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(-2.75,.1));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(2.75,.1));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(
		  2.75+1.5*bermHeight,bermHeight));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(0,1,0,.4,50,4));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(1,2,.4,.6,50,4));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(2,3,.6,1,50,4));
		dynTrackBerm->matchOffsets();
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(0,0,.1));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(3,1,.1));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(2,.6,.2));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(1,.4,.2));
	}
	if (wireHeight <= 0)
		return true;
	dynTrackWire= new TrackShape;
	float r= .015;
	dynTrackWire->offsets.push_back(TrackShape::Offset(0,-wireHeight));
	dynTrackWire->offsets.push_back(TrackShape::Offset(r,-wireHeight-r));
	dynTrackWire->offsets.push_back(TrackShape::Offset(0,-wireHeight-2*r));
	dynTrackWire->offsets.push_back(TrackShape::Offset(-r,-wireHeight-r));
	dynTrackWire->surfaces.push_back(TrackShape::Surface(0,1,0,0,0,0));
	dynTrackWire->surfaces.push_back(TrackShape::Surface(1,2,0,0,0,0));
	dynTrackWire->surfaces.push_back(TrackShape::Surface(2,3,0,0,0,0));
	dynTrackWire->surfaces.push_back(TrackShape::Surface(3,0,0,0,0,0));
	dynTrackWire->color= osg::Vec4(.2,.3,.2,1);
	return true;
}

//	Changes the navigable water to match the route
//	can also adjust the route's terrain to match the water depth
void MSTSRoute::adjustWater(int setTerrain)
{
	if (waterList.size() == 0)
		return;
	for (WaterList::iterator i=waterList.begin(); i!=waterList.end(); ++i) {
		Water* w= *i;
		w->waterLevel= waterLevelDelta;
		for (Water::VertexMap::iterator i=w->vertexMap.begin();
		  i!=w->vertexMap.end(); ++i) {
			Water::Vertex* v= i->second;
			double x,y;
			ll2xy(v->xy[1],v->xy[0],&x,&y);
			v->xy[0]= x;
			v->xy[1]= y;
		}
		for (Water::TriangleList::iterator j=w->triangleList.begin();
		  j!=w->triangleList.end(); ++j) {
			Water::Triangle* t= *j;
			t->area= Water::area(t->v[0],t->v[1],t->v[2]);
		}
	}
	if (!setTerrain)
		return;
	for (TileMap::iterator i=tileMap.begin(); i!=tileMap.end(); ++i) {
		Tile* t= i->second;
//		fprintf(stderr,"%d %d %f %f\n",t->x,t->z,t->floor,t->scale);
		readTerrain(t);
		float x0= 2048*(t->x-centerTX);
		float z0= 2048*(t->z-centerTZ);
//		fprintf(stderr,"%f %f\n",x0,z0);
#if 0
		Water::Location loc;
		for (int j=0; j<256; j++) {
			fprintf(stderr,"%d",j);
			for (int k=0; k<256; k++) {
				double x= x0+8*(k-128);
				double z= z0+8*(128-j);
				if (!loc.update(x,z))
					continue;
				float d= waterLevelDelta - loc.depth;
				t->terrain->y[j][k]= d<t->floor ? 0 :
				  (int)((d-t->floor)/t->scale);
				fprintf(stderr,"%d %d %d %f\n",
				  j,k,t->terrain->y[j][k],d);
			}
		}
#else
		for (WaterList::iterator j=waterList.begin();
		  j!=waterList.end(); ++j) {
			Water* w= *j;
			int n= 0;
			for (Water::TriangleList::iterator
			  k=w->triangleList.begin();
			  k!=w->triangleList.end(); ++k) {
				Water::Triangle* tri= *k;
				double minX= 1e10;
				double maxX= -1e10;
				double minY= 1e10;
				double maxY= -1e10;
				for (int i1=0; i1<3; i1++) {
					Water::Vertex* v= tri->v[i1];
					if (minX > v->xy[0])
						minX= v->xy[0];
					if (minY > v->xy[1])
						minY= v->xy[1];
					if (maxX < v->xy[0])
						maxX= v->xy[0];
					if (maxY < v->xy[1])
						maxY= v->xy[1];
				}
//				fprintf(stderr,"%lf %lf %lf %lf\n",
//				  minX,maxX,minY,maxY);
				if (maxX<x0-1024 || minX>x0+1024 ||
				  maxY<z0-1024 || minY>z0+1024)
					continue;
				int minK= (int)((minX-x0)/8 + 128);
				int maxK= (int)((maxX-x0)/8 + 128);
				int minJ= (int)(128 - (maxY-z0)/8);
				int maxJ= (int)(128 - (minY-z0)/8);
//				fprintf(stderr,"%d %d %d %d\n",
//				  minJ,maxJ,minK,maxK);
				if (minJ < 0)
					minJ= 0;
				if (maxJ > 255)
					maxJ= 255;
				if (minK < 0)
					minK= 0;
				if (maxK > 255)
					maxK= 255;
				n++;
				Water::Location loc;
				for (int j1=minJ; j1<=maxJ; j1++) {
					for (int k1=minK; k1<=maxK; k1++) {
						double x= x0+8*(k1-128);
						double z= z0+8*(128-j1);
						loc.triangle= tri;
						if (!loc.update(x,z))
							continue;
						float d= waterLevelDelta -
						  loc.depth;
						t->terrain->y[j1][k1]= 
						  d<t->floor ? 0 :
						  (int)((d-t->floor)/
						  t->scale);
//						fprintf(stderr,"%d %d %d %f\n",
//						  j1,k1,t->terrain->y[j1][k1],
//						  d);
					}
				}
			}
			fprintf(stderr,"n=%d\n",n);
		}
#endif
		writeTerrain(t);
		t->freeTerrain();
	}
}

//	makes an MSTS marker file to match the navigable water's shore line
void MSTSRoute::saveShoreMarkers(const char* filename)
{
	for (WaterList::iterator i=waterList.begin(); i!=waterList.end(); ++i) {
		Water* w= *i;
		for (Water::VertexMap::iterator j=w->vertexMap.begin();
		  j!=w->vertexMap.end(); ++j) {
			Water::Vertex* v= j->second;
			v->label= 0;
		}
		for (Water::TriangleList::iterator j=w->triangleList.begin();
		  j!=w->triangleList.end(); ++j) {
			Water::Triangle* t= *j;
			for (int k=0; k<3; k++) {
				if (t->adj[k] != NULL)
					continue;
				t->v[k]->label= 1;
				int k1= (k+1)%3;
				t->v[k1]->label= 1;
			}
		}
	}
	FILE* out= fopen(filename,"w");
	if (out == NULL) {
		fprintf(stderr,"cannot create %s\n",filename);
		return;
	}
	fprintf(out,"SIMISA@@@@@@@@@@JINXOIOt______\n");
	int n= 0;
	for (WaterList::iterator i=waterList.begin(); i!=waterList.end(); ++i) {
		Water* w= *i;
		for (Water::VertexMap::iterator j=w->vertexMap.begin();
		  j!=w->vertexMap.end(); ++j) {
			Water::Vertex* v= j->second;
			if (v->label == 0)
				continue;
			double x= v->xy[0] + centerTX*2048 - 20015000 +
			  16385*2048 - 1024;
			double y= v->xy[1] + centerTZ*2048 + 8673000 -
			  16385*2048 + 3072;
			double lat,lng;
			ghProj.xy2ll(x,y,&lat,&lng);
			fprintf(out,"Marker ( %f %f %d 2 )\n",lng,lat,n++);
		}
	}
	fclose(out);
}

//	makes 3D models for each patch in a tile
void MSTSRoute::makeTerrainPatches(Tile* tile)
{
	if (tile->terrModel != NULL)
		return;
	readTerrain(tile);
//	calcTileAO(tile);
//	fprintf(stderr,"makeTerrain %d %d %f %f\n",
//	  tile->x,tile->z,tile->floor,tile->scale);
	Tile* t12= findTile(tile->x,tile->z-1);
	if (t12 != NULL)
		readTerrain(t12);
	Tile* t21= findTile(tile->x+1,tile->z);
	if (t21 != NULL)
		readTerrain(t21);
	Tile* t22= findTile(tile->x+1,tile->z-1);
	if (t22 != NULL)
		readTerrain(t22);
	vector<osg::Texture2D*> textures;
	for (int i=0; i<tile->textures.size()/2; i++) {
		osg::Texture2D* t= new osg::Texture2D;
		t->ref();
		t->setDataVariance(osg::Object::DYNAMIC);
		textures.push_back(t);
		string path= terrtexDir+dirSep+tile->textures[i];
		if (strstr(tile->textures[i].c_str(),".ace") ||
		  strstr(tile->textures[i].c_str(),".ACE")) {
			osg::Image* img= readMSTSACE(path.c_str());
			if (img == NULL) {
				path= path.substr(0,path.size()-4)+".png";
//				fprintf(stderr,"trying %s\n",path.c_str());
				img= osgDB::readImageFile(path.c_str());
//				if (img == NULL)
//					fprintf(stderr,"no png");
			}
			if (img != NULL)
				t->setImage(img);
		} else {
			osg::Image* img= osgDB::readImageFile(path.c_str());
			if (img != NULL)
				t->setImage(img);
		}
//		t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
//		t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
		t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
		t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
	}
	vector<osg::Texture2D*> microTextures;
	for (int i=0; i<tile->microTextures.size(); i++) {
		osg::Texture2D* t= new osg::Texture2D;
		t->ref();
		t->setDataVariance(osg::Object::DYNAMIC);
		microTextures.push_back(t);
		string path= terrtexDir+dirSep+tile->microTextures[i];
		osg::Image* image= readMSTSACE(path.c_str());
		if (image != NULL)
			t->setImage(image);
		t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
		t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
	}
	osg::TexEnvCombine* tec= new osg::TexEnvCombine();
	tec->setCombine_RGB(osg::TexEnvCombine::MODULATE);
	tec->setSource0_RGB(osg::TexEnvCombine::PREVIOUS);
	tec->setSource1_RGB(osg::TexEnvCombine::TEXTURE1);
	tec->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
	tec->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
	tec->setScale_RGB(2.);
	tec->ref();
//	osg::Material* mat= new osg::Material;
//	mat->setAmbient(osg::Material::FRONT_AND_BACK,osg::Vec4(.6,.6,.6,1));
//	mat->setDiffuse(osg::Material::FRONT_AND_BACK,osg::Vec4(.4,.4,.4,1));
	Patch* patch= tile->patches;
	osg::Geode* geode= new osg::Geode;
	tile->terrModel= geode;
	geode->setNodeMask(0x1);
//	fprintf(stderr,"tile %d %d\n",tile->x,tile->z);
	geode->ref();
	for (int i=0; i<16; i++) {
		for (int j=0; j<16; j++) {
			if ((patch->flags&1)!=0) {
//				fprintf(stderr,"skip patch %d %d %d %d %x\n",
//				  tile->x,tile->z,i,j,patch->flags);
				patch++;
				continue;
			}
//			fprintf(stderr,"patchflags %x\n",patch->flags);
			osg::Geometry* geometry=
			  loadPatchGeoFile(patch,i,j,tile);
			if (geometry == NULL)
				geometry= makePatch(patch,i*16,j*16,
				  tile,t12,t21,t22);
			osg::Material* mat= new osg::Material;
			mat->setAmbient(osg::Material::FRONT_AND_BACK,
			  osg::Vec4(.6,.6,.6,1));
			mat->setDiffuse(osg::Material::FRONT_AND_BACK,
			  osg::Vec4(.4,.4,.4,1));
			mat->setSpecular(osg::Material::FRONT_AND_BACK,
			  osg::Vec4(0,0,0,1));
			osg::StateSet* stateSet=
			  geometry->getOrCreateStateSet();
			stateSet->setMode(GL_LIGHTING,osg::StateAttribute::ON);
			stateSet->setAttributeAndModes(mat,
			  osg::StateAttribute::ON);
			stateSet->setTextureAttributeAndModes(0,
			  textures[patch->texIndex],osg::StateAttribute::ON);
			if (patch->texIndex<microTextures.size() &&
			  microTextures[patch->texIndex]) {
				stateSet->setTextureAttributeAndModes(1,
				  microTextures[patch->texIndex],
				  osg::StateAttribute::ON);
				stateSet->setTextureAttributeAndModes(1,tec,
				  osg::StateAttribute::ON);
			}
			stateSet->setAttribute(mat,osg::StateAttribute::ON);
#if 0
			osg::PolygonMode* pm= new osg::PolygonMode;
			pm->setMode(osg::PolygonMode::FRONT_AND_BACK,
			  osg::PolygonMode::LINE);
			stateSet->setAttribute(pm);
#endif
			geode->addDrawable(geometry);
//			fprintf(stderr," %d.%x",
//			  patch->detail,patch->edgeDetail);
			patch++;
		}
//		fprintf(stderr,"\n");
	}
	for (int i=0; i<textures.size(); i++)
		textures[i]->unref();
	for (int i=0; i<microTextures.size(); i++)
		microTextures[i]->unref();
	tec->unref();
//	osgUtil::Simplifier simplifier(.5,4);
//	geode->accept(simplifier);
}

//	makes a 3D model for a single patch
//	uses TrinagleGrid to simplify terrain far from the tracks
osg::Geometry* MSTSRoute::makePatch(Patch* patch, int i0, int j0,
  Tile* tile, Tile* t12, Tile* t21, Tile* t22)
{
	float x0= 2048*(tile->x-centerTX);
	float z0= 2048*(tile->z-centerTZ);
	int sz= 1;
	int dij= 16;
	for (int i=0; i<patch->detail; i++) {
		sz*= 2;
		dij/= 2;
	}
	sz++;
	float uvmult= tile->microTexUVMult;
	//fprintf(stderr,"%d %d %d\n",patch->detail,sz,dij);
	//sz= 17;
	//dij= 1;
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::Vec2Array* microTexCoords= new osg::Vec2Array;
	osg::Vec3Array* norms= new osg::Vec3Array;
	osg::DrawElementsUInt* drawElements=
	  new osg::DrawElementsUInt(GL_TRIANGLES);;
	if (patch->detail>=4 && patch->edgeDetail==0) {
		for (int i=0; i<=16; i++) {
			int k= i*17;
			for (int j=0; j<=16; j++) {
				float a=
				  getAltitude(i+i0,j+j0,tile,t12,t21,t22);
//				int ni=
//				  getNormalIndex(i+i0,j+j0,tile,t12,t21,t22);
				osg::Vec3f normal=
				  getNormal(i+i0,j+j0,tile,t12,t21,t22);
				verts->push_back(
				  osg::Vec3(x0+8*(j0+j-128),z0+8*(128-i-i0),a));
				float u= patch->u0+patch->dudx*j+patch->dudz*i;
				float v= patch->v0+patch->dvdx*j+patch->dvdz*i;
				texCoords->push_back(osg::Vec2(u,v));
				microTexCoords->push_back(
				  osg::Vec2(uvmult*u,uvmult*v));
//				norms->push_back(terrainNormals[ni]);
				norms->push_back(normal);
				float h00=
				  getVertexHidden(i+i0,j+j0,tile,t12,t21,t22);
				if (i<16 && j<16) {
					float a11= getAltitude(i+i0+1,j+j0+1,
					  tile,t12,t21,t22);
					float a01= getAltitude(i+i0,j+j0+1,
					  tile,t12,t21,t22);
					float a10= getAltitude(i+i0+1,j+j0,
					  tile,t12,t21,t22);
					float h11= getVertexHidden(
					  i+i0+1,j+j0+1,tile,t12,t21,t22);
					float h01= getVertexHidden(
					  i+i0,j+j0+1,tile,t12,t21,t22);
					float h10= getVertexHidden(
					  i+i0+1,j+j0,tile,t12,t21,t22);
					int kj= k+j/dij;
					if (fabs(a11-a) < fabs(a10-a01)) {
						if (!h00 && !h10 && !h11) {
						drawElements->push_back(kj);
						drawElements->push_back(kj+17);
						drawElements->push_back(kj+17+1);
						}
						if (!h00 && !h01 && !h11) {
						drawElements->push_back(kj+17+1);
						drawElements->push_back(kj+1);
						drawElements->push_back(kj);
						}
					} else {
						if (!h00 && !h10 && !h01) {
						drawElements->push_back(kj);
						drawElements->push_back(kj+17);
						drawElements->push_back(kj+1);
						}
						if (!h11 && !h01 && !h10) {
						drawElements->push_back(kj+1);
						drawElements->push_back(kj+17);
						drawElements->push_back(kj+17+1);
						}
					}
				}
			}
		}
	} else {
		TriangleGrid tg(17,patch->edgeDetail);
		for (int i=0; i<=16; i+=dij) {
			for (int j=0; j<=16; j+=dij) {
				if (getVertexHidden(
				  i+i0,j+j0,tile,t12,t21,t22)) {
					fprintf(stderr,
					  "vertex hidden %p %f %f %d %d %x\n",
					  tile,x0,z0,i+i0,j+j0,
					  0);//tile->terrain->f[i+i0][j+j0]);
					tg.hide(i,j);
				}
			}
		}
		tg.makeList(2*patch->detail);
		list<int>& tlist= tg.getList();
		map<int,int> vmap;
		for (list<int>::iterator i=tlist.begin(); i!=tlist.end(); ++i) {
			map<int,int>::iterator j= vmap.find(*i);
			if (j != vmap.end()) {
				drawElements->push_back(j->second);
			} else {
				int i1= (*i)/17;
				int j1= (*i)%17;
				float a=
				  getAltitude(i1+i0,j1+j0,tile,t12,t21,t22);
//				int ni=
//				  getNormalIndex(i1+i0,j1+j0,tile,t12,t21,t22);
				osg::Vec3f normal=
				  getNormal(i1+i0,j1+j0,tile,t12,t21,t22);
				verts->push_back(osg::Vec3(
				  x0+8*(j0+j1-128),z0+8*(128-i1-i0),a));
				float u=
				  patch->u0+patch->dudx*j1+patch->dudz*i1;
				float v=
				  patch->v0+patch->dvdx*j1+patch->dvdz*i1;
				texCoords->push_back(osg::Vec2(u,v));
				microTexCoords->push_back(osg::Vec2(32*u,32*v));
//				norms->push_back(terrainNormals[ni]);
				norms->push_back(normal);
				int vi= vmap.size();
				vmap[*i]= vi;
				drawElements->push_back(vi);
			}
		}
	}
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(osg::Vec4(1,1,1,1));
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	geometry->setNormalArray(norms);
	geometry->setNormalBinding(osg::Geometry::BIND_PER_VERTEX);
	geometry->setTexCoordArray(0,texCoords);
	geometry->setTexCoordArray(1,microTexCoords);
	geometry->addPrimitiveSet(drawElements);
	return geometry;
}

//	returns the altitude of terrain point i,j
float MSTSRoute::getAltitude(int i, int j,
  Tile* tile, Tile* t12, Tile* t21, Tile* t22)
{
	if (tile==NULL || tile->terrain==NULL)
		return 0;
	if (i < 0)
		i= 0;
	if (i<256 && j<256)
		return tile->floor + tile->scale*tile->terrain->y[i][j];
	else if (i<256 && j>=256)
	  if (t21!=NULL && t21->terrain!=NULL)
		return t21->floor+ t21->scale*t21->terrain->y[i][j-256];
	  else
		return tile->floor+ tile->scale*tile->terrain->y[i][255];
	else if (i>=256 && j<256)
	  if (t12!=NULL && t12->terrain!=NULL)
		return t12->floor+ t12->scale*t12->terrain->y[i-256][j];
	  else
		return tile->floor+ tile->scale*tile->terrain->y[255][j];
	else if (i>=256 && j>=256)
	  if (t22!=NULL && t22->terrain!=NULL)
		return t22->floor+ t22->scale*t22->terrain->y[i-256][j-256];
	  else
		return tile->floor+ tile->scale*tile->terrain->y[255][255];
	return 0;
}

float MSTSRoute::getAltitude(float x, float z,
  Tile* tile, Tile* t12, Tile* t21, Tile* t22)
{
	int j= (int)floor(x/8) + 128;
	int i= 128 - (int)floor(z/8);
	if (j < 0) {
		j+= 256;
		x+= 2048;
		t21= tile;
		t22= t12;
		tile= findTile(t21->x-1,t21->z);
		t12= findTile(t21->x-1,t21->z-1);
	}
	if (i < 0) {
		i+= 256;
		z-= 2048;
		t12= tile;
		t22= t21;
		tile= findTile(t12->x,t12->z+1);
		t21= findTile(t12->x+1,t12->z+1);
	}
	float x0= 8*(j-128);
	float z0= 8*(128-i);
	float wx= (x-x0)/8;
	float wz= (z-z0)/8;
	float a00= getAltitude(i,j,tile,t12,t21,t22);
	float a01= getAltitude(i-1,j,tile,t12,t21,t22);
	float a11= getAltitude(i-1,j+1,tile,t12,t21,t22);
	float a10= getAltitude(i,j+1,tile,t12,t21,t22);
	float a= (1-wx)*(1-wz)*a00 + wx*(1-wz)*a10 + wx*wz*a11 + (1-wx)*wz*a01;
//	fprintf(stderr,"getalt %f %f %f %f %d %d %f %f %f %f\n",
//	  x,z,wx,wz,i,j,a00,a10,a11,a01);
	if (tile->terrModel) {
		float tx0= 2048*(tile->x-centerTX);
		float tz0= 2048*(tile->z-centerTZ);
		osgSim::LineOfSight::Intersections hits=
		  osgSim::LineOfSight::computeIntersections(tile->terrModel,
		   osg::Vec3(x+tx0,z+tz0,1e7),
		   osg::Vec3(x+tx0,z+tz0,0),0x7fffffff);
		if (hits.size() > 0)
			a= hits[0][2];
//		for (int i=0; i<hits.size(); i++)
//			fprintf(stderr,
//			  "getalt %f %f %f %f %d %d %f %f %f %f %f %f %f\n",
//			  x,z,wx,wz,i,j,a00,a10,a11,a01,
//			  hits[i][0],hits[i][1],hits[i][2]);
	}
	return a;
}

//	gets a terrain normal
osg::Vec3f MSTSRoute::getNormal(int i, int j,
  Tile* tile, Tile* t12, Tile* t21, Tile* t22)
{
#if 0
	float a00= getAltitude(i,j,tile,t12,t21,t22);
	float a01= getAltitude(i+1,j,tile,t12,t21,t22);
	float a10= getAltitude(i,j+1,tile,t12,t21,t22);
	osg::Vec3f n= osg::Vec3f(a00-a10,a01-a00,8);
#else
	float a0m= getAltitude(i-1,j,tile,t12,t21,t22);
	float am0= getAltitude(i,j-1,tile,t12,t21,t22);
	float a0p= getAltitude(i+1,j,tile,t12,t21,t22);
	float ap0= getAltitude(i,j+1,tile,t12,t21,t22);
	osg::Vec3f n= osg::Vec3f(am0-ap0,a0p-a0m,16);
#endif
	n.normalize();
//	fprintf(stderr,"normal %f %f %f %f %f %f\n",
//	  a00,a10,a01,n.x(),n.y(),n.z());
	return n;
}

//	gets a normal index
int MSTSRoute::getNormalIndex(int i, int j,
  Tile* tile, Tile* t12, Tile* t21, Tile* t22)
{
	if (i<256 && j<256)
		return tile->terrain->n[i][j];
	else if (i<256 && j>=256)
	  if (t21 != NULL)
		return t21->terrain->n[i][j-256];
	  else
		return tile->terrain->n[i][255];
	else if (i>=256 && j<256)
	  if (t12 != NULL)
		return t12->terrain->n[i-256][j];
	  else
		return tile->terrain->n[255][j];
	else if (i>=256 && j>=256)
	  if (t22 != NULL)
		return t22->terrain->n[i-256][j-256];
	  else
		return tile->terrain->n[255][255];
	return 0;
}

//	returns true if terrain vertex is hidden
bool MSTSRoute::getVertexHidden(int i, int j,
  Tile* tile, Tile* t12, Tile* t21, Tile* t22)
{
	if (ignoreHiddenTerrain)
		return false;
	if (i<256 && j<256)
		return (tile->terrain->f[i][j]&0x04)!=0;
	else if (i<256 && j>=256)
	  if (t21 != NULL)
		return (t21->terrain->f[i][j-256]&0x04)!=0;
	  else
		return (tile->terrain->f[i][255]&0x04)!=0;
	else if (i>=256 && j<256)
	  if (t12 != NULL)
		return (t12->terrain->f[i-256][j]&0x04)!=0;
	  else
		return (tile->terrain->f[255][j]&0x04)!=0;
	else if (i>=256 && j>=256)
	  if (t22 != NULL)
		return (t22->terrain->f[i-256][j-256]&0x04)!=0;
	  else
		return (tile->terrain->f[255][255]&0x04)!=0;
	return false;
}

//	marks the patch near x,z as haveing track and needing high detail
void MSTSRoute::saveTrackLocation(Tile* tile,float x,float z)
{
	if (tile == NULL)
		return;
	int i= ((int)(z+1024))/128;
	int j= ((int)(x+1024))/128;
//	fprintf(stderr,"%d %d %f %f %d %d\n",tile->x,tile->z,x,z,i,j);
	if (i<0 || i>15 || j<0 || j>15)
		return;
	Patch* patch= &tile->patches[16*(15-i)+j];
	patch->detail= 5;
}

//	calculates patch detail information used to simplify terrain
void MSTSRoute::setPatchDetail()
{
	for (int i=5; i>0; i--)
		setPatchDetail(i);
	for (TileMap::iterator k=tileMap.begin(); k!=tileMap.end(); ++k) {
		Tile* t= k->second;
		for (int i=0; i<256; i++) {
			if (t->patches[i].detail >= 4) {
				t->patches[i].detail= 4;
				t->patches[i].edgeDetail= 0;
			}
		}
	}
}

void MSTSRoute::setPatchDetail(int detail)
{
	int d1= detail-1;
	for (TileMap::iterator k=tileMap.begin(); k!=tileMap.end(); ++k) {
		Tile* t= k->second;
		Tile* t1= findTile(t->x,t->z-1);
		Tile* t2= findTile(t->x-1,t->z);
		Tile* t3= findTile(t->x,t->z+1);
		Tile* t4= findTile(t->x+1,t->z);
		for (int i=0; i<16; i++) {
			for (int j=0; j<16; j++) {
				Patch* p= &t->patches[16*(15-i)+j];
				if (p->detail != detail)
					continue;
				if (i > 0) {
					p= &t->patches[16*(15-(i-1))+j];
					setPatchDetail(p,d1,
					  TriangleGrid::NEDGE);
				} else if (t1 != NULL) {
					p= &t1->patches[j];
					setPatchDetail(p,d1,
					  TriangleGrid::NEDGE);
				}
				if (j > 0) {
					p= &t->patches[16*(15-i)+j-1];
					setPatchDetail(p,d1,
					  TriangleGrid::EEDGE);
				} else if (t2 != NULL) {
					p= &t2->patches[16*(15-i)+15];
					setPatchDetail(p,d1,
					  TriangleGrid::EEDGE);
				}
				if (i < 15) {
					p= &t->patches[16*(15-(i+1))+j];
					setPatchDetail(p,d1,
					  TriangleGrid::SEDGE);
				} else if (t3 != NULL) {
					p= &t3->patches[16*15+j];
					setPatchDetail(p,d1,
					  TriangleGrid::SEDGE);
				}
				if (j < 15) {
					p= &t->patches[16*(15-i)+j+1];
					setPatchDetail(p,d1,
					  TriangleGrid::WEDGE);
				} else if (t4 != NULL) {
					p= &t4->patches[16*(15-i)];
					setPatchDetail(p,d1,
					  TriangleGrid::WEDGE);
				}
			}
		}
	}
}

void MSTSRoute::setPatchDetail(Patch* patch, int detail, int edge)
{
	if (patch->detail < detail) {
		patch->detail= detail;
		patch->edgeDetail= edge;
	}
	if (patch->detail == detail)
		patch->edgeDetail|= edge;
}

void findPassingPoints(Track::Path* aiPath, Track::Path* playerPath)
{
	typedef set<Track::SSEdge*> SSEdgeSet;
	SSEdgeSet ssEdgeSet;
	for (Track::Path::Node* pn=playerPath->firstNode; pn!=NULL;
	  pn=pn->next)
		if (pn->nextSSEdge)
			ssEdgeSet.insert(pn->nextSSEdge);
	bool overlap= true;
	for (Track::Path::Node* pn=aiPath->firstNode; pn!=NULL; pn=pn->next) {
		if (pn->nextSSEdge == NULL)
			continue;
		SSEdgeSet::iterator i= ssEdgeSet.find(pn->nextSSEdge);
		if (i!=ssEdgeSet.end() && !overlap) {
			pn->type= Track::Path::MEET;
			fprintf(stderr,"  aimeet %p %d\n",pn,pn->type);
		}
		overlap= i!=ssEdgeSet.end();
	}
}

void MSTSRoute::loadActivity(osg::Group* root, int activityFlags)
{
	if (activityName.size()==0) {
		simTime= 12*3600;
		loadExploreConsist(root);
		return;
	}
	Activity activity;
	string path= routeDir+dirSep+"ACTIVITIES"+dirSep+activityName;
//	fprintf(stderr,"path=%s\n",path.c_str());
	activity.readFile(path.c_str());
	simTime= activity.startTime;
	if (timeTable) {
		tt::Station* start= timeTable->findStation("start");
		if (start == NULL)
			start= timeTable->addStation("start");
		for (Traffic* t=activity.traffic; t!=NULL; t=t->next) {
			fprintf(stderr,"traffic %s %d\n",
			  t->service.c_str(),t->startTime);
			Track::Path* path= loadService(t->service,root,false,
			  t->id);
			tt::Train* train= timeTable->addTrain(t->service);
			train->setSchedTime(start,
			  t->startTime-60,t->startTime,0);
			train->path= path;
		}
	}
	if ((activityFlags&01) != 0) {
		Track::Path* playerPath=
		  loadService(activity.playerService,root,true,0);
		if (playerPath && timeTable) {
			for (int i=0; i<timeTable->getNumTrains(); i++) {
				tt::Train* train= timeTable->getTrain(i);
				if (train->getPrevTrain()==NULL)
					findPassingPoints(train->path,
					  playerPath);
			}
		}
	}
	for (LooseConsist* c=activity.consists; c!=NULL; c=c->next) {
//		fprintf(stderr,"consist %d %d %d %d %f %f\n",
//		  c->id,c->direction,c->tx,c->tz,c->x,c->z);
//		for (Wagon* w=c->wagons; w!=NULL; w=w->next)
//			fprintf(stderr," wagon %s %s\n",
//			  w->dir.c_str(),w->name.c_str());
		loadConsist(c,root);
	}
	for (Event* e=activity.events; e!=NULL; e=e->next) {
		eventMap[e->id]= e;
	}
}

void MSTSRoute::loadExploreConsist(osg::Group* root)
{
	string trainsDir= fixFilenameCase(mstsDir+dirSep+"TRAINS");
	string trainsetDir= fixFilenameCase(trainsDir+dirSep+"TRAINSET");
	string consistsDir= fixFilenameCase(trainsDir+dirSep+"CONSISTS");
	string path= fixFilenameCase(consistsDir+dirSep+consistName);
//	fprintf(stderr,"path=%s\n",path.c_str());
	MSTSFile conFile;
	conFile.readFile(path.c_str());
//	fprintf(stderr,"first=%p\n",conFile.getFirstNode());
	MSTSFileNode* cfg= conFile.getFirstNode()->get("TrainCfg");
//	fprintf(stderr,"cfg=%p\n",cfg);
	Train* train= new Train;
	for (MSTSFileNode* node=cfg->get(0); node!=NULL; node=node->next) {
		string dir= trainsetDir+dirSep;;
		string file= "";
		int id;
		int rev= 0;
		if (node->value && *(node->value)=="Engine") {
			MSTSFileNode* data= node->get("EngineData");
			dir+= data->get(1)->c_str();
			file+= data->get(0)->c_str();
			file+= ".eng";
			MSTSFileNode* uid= node->get("UiD");
			if (uid)
				id= atoi(uid->get(0)->c_str());
			MSTSFileNode* flip= node->get("Flip");
			if (flip)
				rev= atoi(flip->get(0)->c_str());
		} else if (node->value && *(node->value)=="Wagon") {
			MSTSFileNode* data= node->get("WagonData");
			dir+= data->get(1)->c_str();
			file+= data->get(0)->c_str();
			file+= ".wag";
			MSTSFileNode* uid= node->get("UiD");
			if (uid)
				id= atoi(uid->get(0)->c_str());
			MSTSFileNode* flip= node->get("Flip");
			if (flip)
				rev= atoi(flip->get(0)->c_str());
		}
		if (file == "")
			continue;
		dir= fixFilenameCase(dir);
		RailCarDef* def= findRailCarDef(file,false);
		if (def == NULL) {
			def= readMSTSWag(dir.c_str(),file.c_str());
			if (def)
				railCarDefMap[file]= def;
			else
				def= findRailCarDef(file,true);
			fprintf(stderr,"loaded %s %s\n",
			  dir.c_str(),file.c_str());
		}
		if (def == NULL) {
			fprintf(stderr,"cannot load %s %s\n",
			  dir.c_str(),file.c_str());
			continue;
		}
		RailCarInst* car= new RailCarInst(def,root,70,"K");
		car->setLoad(0);
		car->prev= train->lastCar;
		car->rev= rev;
		if (train->lastCar == NULL)
			train->firstCar= car;
		else
			train->lastCar->next= car;
		train->lastCar= car;
	}
	if (train->firstCar == NULL) {
		fprintf(stderr,"empty train\n");
		delete train;
	}
	train->name= "explore";;
	trainMap[train->name]= train;
	train->setModelsOff();
	float initAux= 50;
	float initCyl= 50;
	float initEqRes= 50;
	train->connectAirHoses();
	if (train->engAirBrake != NULL)
		train->engAirBrake->setEqResPressure(initEqRes);
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next) {
		if (car->airBrake != NULL) {
			car->airBrake->setCylPressure(initCyl);
			car->airBrake->setAuxResPressure(initAux);
			car->airBrake->setEmergResPressure(70);
			car->airBrake->setPipePressure(initEqRes);
		}
	}
	train->calcPerf();
}

void MSTSRoute::loadConsist(LooseConsist* consist, osg::Group* root)
{
	string trainsDir= fixFilenameCase(mstsDir+dirSep+"TRAINS");
	string trainsetDir= fixFilenameCase(trainsDir+dirSep+"TRAINSET");
	Train* train= new Train;
	float initPipe= 0;
	float initAux= 50;
	float initCyl= 50;
	float initEqRes= 50;
	for (Wagon* w=consist->wagons; w!=NULL; w=w->next) {
		string dir= fixFilenameCase(trainsetDir+"/"+w->dir);
		string file= w->name+(w->isEngine?".eng":".wag");
		RailCarDef* def= findRailCarDef(file,false);
		if (def == NULL) {
			def= readMSTSWag(dir.c_str(),file.c_str());
			if (def)
				railCarDefMap[file]= def;
			else
				def= findRailCarDef(file,true);
		}
		if (def == NULL) {
			fprintf(stderr,"cannot load %s %s\n",
			  dir.c_str(),file.c_str());
			continue;
		}
		RailCarInst* car= new RailCarInst(def,root,70);
		car->setLoad(0);
		car->prev= train->lastCar;
		if (train->lastCar == NULL)
			train->firstCar= car;
		else
			train->lastCar->next= car;
		train->lastCar= car;
		char buf[20];
		sprintf(buf,"%d-%d",consist->id,w->id);
		string carid= buf;
		car->addWaybill(carid,1,0,0,1);
	}
	if (train->firstCar == NULL) {
		fprintf(stderr,"empty train\n");
		delete train;
		return;
	}
	trainList.push_back(train);
	Track* track= trackMap[routeID];
	track->findLocation(convX(consist->tx,consist->x),
	  convZ(consist->tz,consist->z),&train->location);
	train->location.rev= consist->direction!=0;
	float len= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next)
		len+= car->def->length;
//	train->location.move(len/2,1,0);
	train->location.edge->occupied++;
	train->endLocation= train->location;
	train->endLocation.move(-len,1,-1);
	train->connectAirHoses();
	if (train->engAirBrake != NULL) {
		train->engAirBrake->setEqResPressure(initEqRes);
		initPipe= initEqRes;
	}
	if (initCyl > initAux)
		initCyl= initAux;
	float x= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next) {
		car->setLocation(x-car->def->length/2,&train->location);
		x-= car->def->length;
		if (car->airBrake != NULL) {
			car->airBrake->setCylPressure(initCyl);
			car->airBrake->setAuxResPressure(initAux);
			car->airBrake->setPipePressure(initPipe);
			car->airBrake->setEmergResPressure(initPipe);
		}
		if (initCyl == 0)
			car->handBControl= 1;
	}
	train->calcPerf();
	WLocation loc;
	train->location.getWLocation(&loc);
	fprintf(stderr,"consist %d at %lf %lf %f\n",consist->id,
	  loc.coord[0],loc.coord[1],loc.coord[2]);
}

Track::Path* MSTSRoute::loadService(string filename, osg::Group* root,
  bool player, int serviceId)
{
	string servicesDir= fixFilenameCase(routeDir+dirSep+"SERVICES");
	string path= fixFilenameCase(servicesDir+dirSep+filename+".srv");
//	fprintf(stderr,"path=%s\n",path.c_str());
	Service serv;
	serv.readFile(path.c_str());
	fprintf(stderr," service consist %s %s\n",serv.consistName.c_str(),
	  serv.pathName.c_str());
	Track::Path* trackPath= loadPath(serv.pathName+".pat",true);
	if (trackPath == NULL) {
		fprintf(stderr,"no path\n");
		return NULL;
	}
	string trainsDir= fixFilenameCase(mstsDir+dirSep+"TRAINS");
	string trainsetDir= fixFilenameCase(trainsDir+dirSep+"TRAINSET");
	string consistsDir= fixFilenameCase(trainsDir+dirSep+"CONSISTS");
	path= fixFilenameCase(consistsDir+dirSep+serv.consistName+".con");
//	fprintf(stderr,"path=%s\n",path.c_str());
	MSTSFile conFile;
	conFile.readFile(path.c_str());
//	fprintf(stderr,"first=%p\n",conFile.getFirstNode());
	MSTSFileNode* cfg= conFile.getFirstNode()->get("TrainCfg");
//	fprintf(stderr,"cfg=%p\n",cfg);
	Train* train= new Train;
	float initAux= 50;
	float initCyl= 50;
	float initEqRes= 50;
	for (MSTSFileNode* node=cfg->get(0); node!=NULL; node=node->next) {
		string dir= trainsetDir+dirSep;;
		string file= "";
		int id;
		int rev= 0;
		if (node->value && *(node->value)=="Engine") {
			MSTSFileNode* data= node->get("EngineData");
			dir+= data->get(1)->c_str();
			file+= data->get(0)->c_str();
			file+= ".eng";
			MSTSFileNode* uid= node->get("UiD");
			if (uid)
				id= atoi(uid->get(0)->c_str());
			MSTSFileNode* flip= node->get("Flip");
			if (flip)
				rev= atoi(flip->get(0)->c_str());
		} else if (node->value && *(node->value)=="Wagon") {
			MSTSFileNode* data= node->get("WagonData");
			dir+= data->get(1)->c_str();
			file+= data->get(0)->c_str();
			file+= ".wag";
			MSTSFileNode* uid= node->get("UiD");
			if (uid)
				id= atoi(uid->get(0)->c_str());
			MSTSFileNode* flip= node->get("Flip");
			if (flip)
				rev= atoi(flip->get(0)->c_str());
		}
		if (file == "")
			continue;
		dir= fixFilenameCase(dir);
		RailCarDef* def= findRailCarDef(file,false);
		if (def == NULL) {
			def= readMSTSWag(dir.c_str(),file.c_str());
			if (def)
				railCarDefMap[file]= def;
			else
				def= findRailCarDef(file,true);
			fprintf(stderr,"loaded %s %s\n",
			  dir.c_str(),file.c_str());
		}
		if (def == NULL) {
			fprintf(stderr,"cannot load %s %s\n",
			  dir.c_str(),file.c_str());
			continue;
		}
		RailCarInst* car= new RailCarInst(def,root,70);
		car->setLoad(0);
		car->prev= train->lastCar;
		car->rev= rev;
		if (train->lastCar == NULL)
			train->firstCar= car;
		else
			train->lastCar->next= car;
		train->lastCar= car;
		char buf[20];
		sprintf(buf,"%d-%d",serviceId,id);
		string carid= buf;
		car->addWaybill(carid,1,0,0,1);
	}
	if (train->firstCar == NULL) {
		fprintf(stderr,"empty train\n");
		delete train;
		return NULL;
	}
	train->name= filename;
	trainMap[filename]= train;
	trainList.push_back(train);
	if (!player) {
		train->targetSpeed= 8.9;
		train->bControl= 1;
		train->modelCouplerSlack= 0;
	}
	Track* track= trackMap[routeID];
	train->endLocation= trackPath->firstNode->loc;
	float len= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next)
		len+= car->def->length;
	train->endLocation.edge->occupied++;
	train->location= train->endLocation;
	train->location.move(len,1,1);
	train->connectAirHoses();
	if (train->engAirBrake != NULL)
		train->engAirBrake->setEqResPressure(initEqRes);
	if (initCyl > initAux)
		initCyl= initAux;
	float x= 0;
	for (RailCarInst* car=train->firstCar; car!=NULL; car=car->next) {
		car->setLocation(x-car->def->length/2,&train->location);
		x-= car->def->length;
//		fprintf(stderr,"%f\n",x);
		if (car->airBrake != NULL) {
			car->airBrake->setCylPressure(initCyl);
			car->airBrake->setAuxResPressure(initAux);
		}
		if (initCyl == 0)
			car->handBControl= 1;
	}
	train->calcPerf();
	WLocation loc;
	train->location.getWLocation(&loc);
	fprintf(stderr,"service %s at %lf %lf %f\n",filename.c_str(),
	  loc.coord[0],loc.coord[1],loc.coord[2]);
	currentPerson.setLocation(loc.coord);
	return trackPath;
}

//	loads an MSTS path file and converts into a Path structure
Track::Path* MSTSRoute::loadPath(string filename, bool align)
{
	TrackPath trackPath;
	string pathsDir= fixFilenameCase(routeDir+dirSep+"PATHS");
	string filepath= fixFilenameCase(pathsDir+dirSep+filename);
//	fprintf(stderr,"filepath=%s\n",filepath.c_str());
	trackPath.readFile(filepath.c_str());
	vector<Track::Path::Node*> pathNodes;
	pathNodes.reserve(trackPath.nNodes);
	for (int i=0; i<trackPath.nNodes; i++)
		pathNodes[i]= new Track::Path::Node;
	bool print= false;
//	for (int i=0; i<trackPath.nPDPs; i++)
//		if ((trackPath.pdps[i].type2&0x8) != 0)
//			print= true;
	Track* track= trackMap[routeID];
	Track::Path* path= new Track::Path;
	path->firstNode= pathNodes[0];
	for (int i=0; i<trackPath.nNodes; i++) {
		Track::Path::Node* p= pathNodes[i];
		p->sw= NULL;
		p->next= NULL;
		p->nextSiding= NULL;
		TrPathNode* tpn= &trackPath.nodes[i];
		if (tpn->next != NULL)
			p->next= pathNodes[tpn->next-trackPath.nodes];
		if (tpn->nextSiding != NULL)
			p->nextSiding=
			  pathNodes[tpn->nextSiding-trackPath.nodes];
		if ((tpn->flags & 02) != 0) {
			p->type= Track::Path::STOP;
			p->value= ((tpn->flags>>16)&0xffff);
			if (p->value>=20000 && p->value<30000) {
				p->type= Track::Path::UNCOUPLE;
				p->value-= 20000;
			} else if (p->value>=30000 && p->value<40000) {
				p->type= Track::Path::COUPLE;
				p->value-= 30000;
			}
		} else if ((tpn->flags & 01) != 0) {
			p->type= Track::Path::REVERSE;
			p->value= 0;
		} else {
			p->type= Track::Path::OTHER;
			p->value= 0;
		}
		double x= convX(tpn->pdp->tx,tpn->pdp->x);
		double z= convZ(tpn->pdp->tz,tpn->pdp->z);
		double y= tpn->pdp->y;
		track->findLocation(x,z,y,&p->loc);
		if (tpn->pdp->type1 == 2)
			p->sw= track->findSwitch(x,z,y);
		if (print) {
			fprintf(stderr," %d %f %f %f %d %d %p\n",
			  i,x,z,y,p->type,p->value,p->sw);
			if (p->sw)
				fprintf(stderr,"  %f %f %f\n",
				  p->sw->location.coord[0],
				  p->sw->location.coord[1],
				  p->sw->location.coord[2]);
		}
	}
	track->orient(path);
	if (align)
		track->alignSwitches(path->firstNode,NULL,false);
	return path;
}

void MSTSRoute::addSwitchStands(double offset, double zoffset,
  osg::Node* model, osg::Group* rootNode, double pOffset)
{
	Track* track= trackMap[routeID];
	for (Track::VertexList::iterator i=track->vertexList.begin();
	  i!=track->vertexList.end(); ++i) {
		Track::Vertex* v= *i;
		if (v->type != Track::VT_SWITCH)
			continue;
		Track::SwVertex* sw= (Track::SwVertex*) v;
		Track::Vertex* v1= sw->swEdges[0]->otherV(v);
		Track::Vertex* v2= sw->swEdges[1]->otherV(v);
#if 0
		fprintf(stderr,"switchstand %f %f %f %f %f\n",triArea(
		  v->location.coord[0],v->location.coord[1],
		  v1->location.coord[0],v1->location.coord[1],
		  v2->location.coord[0],v2->location.coord[1]),
		  sw->swEdges[0]->length,sw->swEdges[0]->curvature,
		  sw->swEdges[1]->length,sw->swEdges[1]->curvature);
#endif
		double fOffset= pOffset;
		if (sw->swEdges[0]->curvature==0 &&
		  sw->swEdges[1]->curvature==0)
			fOffset+= fmin(sw->swEdges[0]->length,
			  sw->swEdges[1]->length);
		track->addSwitchStand(sw->id,sw->mainEdge==1?offset:-offset,
		  zoffset,model,rootNode,fOffset);
	}
}

osg::Node* MSTSRoute::attachSwitchStand(Tile* tile, osg::Node* model,
  double x, double y, double z)
{
	Track::SwVertex* swVertex= NULL;
	double bestd= 1000;
	for (SwVertexMap::iterator i= tile->swVertexMap.begin();
	  i!=tile->swVertexMap.end(); i++) {
		Track::SwVertex* v= i->second;
		WLocation loc= v->location;
		double dx= loc.coord[0] - x;
		double dy= loc.coord[1] - y;
		double dz= loc.coord[2] - z;
		double d= dx*dx + dy*dy + dz*dz;
		if (bestd > d) {
			bestd= d;
			swVertex= v;
		}
	}
	if (swVertex) {
		fprintf(stderr,"attach %p %f %f %f %f\n",swVertex,bestd,x,y,z);
		osg::Node* copy= (osg::Node*)
		  model->clone(osg::CopyOp::DEEP_COPY_NODES);
		SetSwVertexVisitor visitor(swVertex);
		copy->accept(visitor);
		return copy;
	}
	return model;
}

MSTSSignal* MSTSRoute::findSignalInfo(MSTSFileNode* node)
{
	if (!createSignals)
		return NULL;
	MSTSFileNode* subobj= node->get("SignalSubObj");
	MSTSFileNode* units= node->get("SignalUnits");
//	for (int i=0; subobj->get(i); i++) {
//		fprintf(stderr," subobj %d %s\n",
//		  i,subobj->get(i)->c_str());
//	}
	MSTSSignal* signal= new MSTSSignal;
	for (int i=0; units->get(i); i++) {
		MSTSFileNode* n= units->get(i)->get("TrItemId");
		if (n == NULL)
			continue;
		i++;
		const char* id= n->get(1)->c_str();
		SignalMap::iterator j= signalMap.find(id);
		if (j!=signalMap.end()) {
			Signal* sig= j->second;
//		TrItem* item= &trackDB.trItems[atoi(id)];
//		fprintf(stderr," id %d %s %p\n",i,id,sig);
			signal->units.push_back(sig);
		}
	}
	return signal;
}

osg::Geometry* MSTSRoute::loadPatchGeoFile(Patch* patch, int ti, int tj,
  Tile* tile)
{
	return NULL;
	float x0= 2048*(tile->x-centerTX);
	float z0= 2048*(tile->z-centerTZ);
	char buf[50];
	sprintf(buf,"_%d_%d.obj",ti,tj);
	string path= tilesDir+dirSep+tile->tFilename+buf;
//	fprintf(stderr,"trying patchgeom %s\n",path.c_str());
	osg::Node* model= osgDB::readNodeFile(path);
	if (model == NULL)
		return NULL;
//	fprintf(stderr,"got patchgeom %s\n",path.c_str());
	osg::Group* group= dynamic_cast<osg::Group*>(model);
	if (group == NULL) {
		fprintf(stderr,"patchgeom %s not group\n",path.c_str());
		model->unref();
		return NULL;
	}
	osg::Geode* geode= dynamic_cast<osg::Geode*>(group->getChild(0));
	if (geode == NULL) {
		fprintf(stderr,"patchgeom %s no geode\n",path.c_str());
		model->unref();
		return NULL;
	}
	osg::Geometry* geom=
	  dynamic_cast<osg::Geometry*>(geode->getDrawable(0));
	if (geom == NULL) {
		fprintf(stderr,"patchgeom %s no geom\n",path.c_str());
		model->unref();
		return NULL;
	}
	geom->ref();
	model->unref();
//	fprintf(stderr,"patchgeom %s\n",path.c_str());
	osg::Vec3Array* pVerts= (osg::Vec3Array*) geom->getVertexArray();
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::Vec2Array* microTexCoords= new osg::Vec2Array;
	float* v= (float*) pVerts->getDataPointer();
	for (int j=0; j<pVerts->getNumElements(); j++) {
		int j3= j*3;
		float x= v[j3];
		float y= v[j3+2];
		float z= -v[j3+1];
		float vj= x/8 - tj*16 + 128;
		float vi= 128 - y/8 - ti*16;
//		if (j == 0)
//			fprintf(stderr," %f %f %f %f %f\n",
//			  v[j3],v[j3+1],v[j3+2],vj,vi);
		v[j3]= x0+x;
		v[j3+1]= z0+y;
		v[j3+2]= z;
		float u= patch->u0 + patch->dudx*vj + patch->dudz*vi;
		float v= patch->v0 + patch->dvdx*vj + patch->dvdz*vi;
		texCoords->push_back(osg::Vec2(u,v));
		microTexCoords->push_back(osg::Vec2(32*u,32*v));
	}
	geom->setTexCoordArray(0,texCoords);
	geom->setTexCoordArray(1,microTexCoords);
	return geom;
}

//	makes profile information for scalerail dynamic track
bool MSTSRoute::makeSRDynTrackShapes()
{
	string path= rTexturesDir+dirSep+"sr_track_w1a.ace";
	osg::Texture2D* t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"sr_track_w1a.ace";
		t= readCacheACEFile(path.c_str());
	}
	if (t == NULL)
		return false;
	dynTrackBase= new TrackShape;
	dynTrackBase->texture= new Texture;
	dynTrackBase->texture->texture= t;
	t->ref();
//	dynTrackBase->offsets.push_back(TrackShape::Offset(-2.5,-.2));
//	dynTrackBase->offsets.push_back(TrackShape::Offset(2.5,-.2));
//	dynTrackBase->surfaces.push_back(
//	  TrackShape::Surface(0,1,-.1389,.862,6,4));
//	OR uses -.153916 .862105
//	default MSTS track uses -.139 .862 and .2 per meter
	dynTrackBase->offsets.push_back(TrackShape::Offset(-2.4384,.2));
	dynTrackBase->offsets.push_back(TrackShape::Offset(-1.4478,-.16));
	dynTrackBase->offsets.push_back(TrackShape::Offset(1.4478,-.16));
	dynTrackBase->offsets.push_back(TrackShape::Offset(2.4384,.2));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(0,1,0.,.203,5,4));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(1,2,.203,.797,5,4));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(2,3,.797,1.,5,4));
	dynTrackBase->matchOffsets();
	path= rTexturesDir+dirSep+"sr_track_w3a.ace";
	t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"sr_track_w3a.ace";
		t= readCacheACEFile(path.c_str());
	}
	dynTrackBridge= new TrackShape;
	dynTrackBridge->texture= new Texture;
	dynTrackBridge->texture->texture= t;
	t->ref();
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.5,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.5,-.2));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1.5,-.2));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1.5,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1,.4));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1,.4));
	if (bermHeight > 0) {
		dynTrackBridge->offsets.push_back(
		  TrackShape::Offset(-1.1-.2*bermHeight,.4+bermHeight));
		dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.1,.4));
		dynTrackBridge->offsets.push_back(TrackShape::Offset(1.1,.4));
		dynTrackBridge->offsets.push_back(
		  TrackShape::Offset(1.1+.2*bermHeight,.4+bermHeight));
	}
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(0,1,-.139,.062,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(1,2,.062,.662,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(2,3,.662,.862,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(3,0,.062,.662,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(4,5,-.139,.062,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(6,7,.662,.862,5,4));
	if (bermHeight > 0) {
		dynTrackBridge->surfaces.push_back(
		  TrackShape::Surface(8,9,-.139,.062,5,4));
		dynTrackBridge->surfaces.push_back(
		  TrackShape::Surface(10,11,.662,.862,5,4));
	}
	dynTrackBridge->matchOffsets();
	dynTrackRails= new TrackShape;
	dynTrackRails->texture= new Texture;
	dynTrackRails->texture->texture= t;
	t->ref();
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7175,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7826,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7175,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7826,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7175,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7826,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7175,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7826,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7501,-.330));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7501,-.330));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.6850,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7387,-.2000));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7614,-.2000));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.8152,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.6850,-.1822));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7387,-.2000));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7614,-.2000));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.8152,-.1822));
//	dynTrackRails->surfaces.push_back(
//	  TrackShape::Surface(0,1,.43,.57,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(0,8,.43,.50,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(8,1,.50,.57,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(2,0,.97,.57,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(1,3,.43,.03,2,4));
//	dynTrackRails->surfaces.push_back(
//	  TrackShape::Surface(5,4,.57,.43,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(5,9,.57,.50,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(9,4,.50,.43,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(4,6,.57,.97,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(7,5,.03,.43,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(10,11,1.,.8,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(12,13,.2,.0,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(15,14,1.,.8,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(17,16,.2,.0,2,4));
	dynTrackRails->matchOffsets();
	if (bermHeight > 0) {
		string path= rTexturesDir+dirSep+"acleantrackbase.ace";
		osg::Texture2D* t= readCacheACEFile(path.c_str());
		dynTrackBerm= new TrackShape;
		dynTrackBerm->texture= new Texture;
		dynTrackBerm->texture->texture= t;
		t->ref();
		dynTrackBerm->offsets.push_back(TrackShape::Offset(
		  -2.75-1.5*bermHeight,bermHeight));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(-2.75,.1));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(2.75,.1));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(
		  2.75+1.5*bermHeight,bermHeight));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(0,1,0,.4,50,4));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(1,2,.4,.6,50,4));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(2,3,.6,1,50,4));
		dynTrackBerm->matchOffsets();
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(0,0,.1));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(3,1,.1));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(2,.6,.2));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(1,.4,.2));
	}
	if (wireHeight > 0) {
		dynTrackWire= new TrackShape;
		float r= .015;
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(0,-wireHeight));
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(r,-wireHeight-r));
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(0,-wireHeight-2*r));
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(-r,-wireHeight-r));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(0,1,0,0,0,0));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(1,2,0,0,0,0));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(2,3,0,0,0,0));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(3,0,0,0,0,0));
		dynTrackWire->color= osg::Vec4(.2,.3,.2,1);
	}
	path= rTexturesDir+dirSep+"sr_track_w2a.ace";
	t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"sr_track_w2a.ace";
		t= readCacheACEFile(path.c_str());
	}
	dynTrackTies= new TrackShape;
	dynTrackTies->texture= new Texture;
	dynTrackTies->texture->texture= t;
	t->ref();
	dynTrackTies->offsets.push_back(TrackShape::Offset(-1.2954,-.16));
	dynTrackTies->offsets.push_back(TrackShape::Offset(-1.2954,-.1822));
	dynTrackTies->offsets.push_back(TrackShape::Offset(1.2954,-.1822));
	dynTrackTies->offsets.push_back(TrackShape::Offset(2.2954,-.16));
//	dynTrackTies->surfaces.push_back(
//	  TrackShape::Surface(0,1,.215,.235,5,4));
	dynTrackTies->surfaces.push_back(
	  TrackShape::Surface(1,2,.235,.766,5,4));
//	dynTrackTies->surfaces.push_back(
//	  TrackShape::Surface(2,3,.766,.786,5,4));
	dynTrackTies->matchOffsets();
	return true;
}

//	makes profile information for US tracks dynamic track
bool MSTSRoute::makeUSTDynTrackShapes()
{
	string path= rTexturesDir+dirSep+"US_Track3.ace";
	osg::Texture2D* t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"US_Track3.ace";
		t= readCacheACEFile(path.c_str());
	}
	if (t == NULL)
		return false;
	dynTrackBase= new TrackShape;
	dynTrackBase->texture= new Texture;
	dynTrackBase->texture->texture= t;
	t->ref();
	dynTrackBase->offsets.push_back(TrackShape::Offset(-2.6,.2));
	dynTrackBase->offsets.push_back(TrackShape::Offset(-1.7,-.136));
	dynTrackBase->offsets.push_back(TrackShape::Offset(1.7,-.136));
	dynTrackBase->offsets.push_back(TrackShape::Offset(2.6,.2));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(0,1,.862,.7158,5,4));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(1,2,.7158,.0342,5,4));
	dynTrackBase->surfaces.push_back(
	  TrackShape::Surface(2,3,.0342,-.1389,5,4));
	dynTrackBase->matchOffsets();
	path= rTexturesDir+dirSep+"US_Rails3.ace";
	t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"US_Rails3.ace";
		t= readCacheACEFile(path.c_str());
	}
	dynTrackBridge= new TrackShape;
	dynTrackBridge->texture= new Texture;
	dynTrackBridge->texture->texture= t;
	t->ref();
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.5,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.5,-.2));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1.5,-.2));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1.5,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1,.4));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(-1,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1,0));
	dynTrackBridge->offsets.push_back(TrackShape::Offset(1,.4));
	if (bermHeight > 0) {
		dynTrackBridge->offsets.push_back(
		  TrackShape::Offset(-1.1-.2*bermHeight,.4+bermHeight));
		dynTrackBridge->offsets.push_back(TrackShape::Offset(-1.1,.4));
		dynTrackBridge->offsets.push_back(TrackShape::Offset(1.1,.4));
		dynTrackBridge->offsets.push_back(
		  TrackShape::Offset(1.1+.2*bermHeight,.4+bermHeight));
	}
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(0,1,-.139,.062,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(1,2,.062,.662,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(2,3,.662,.862,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(3,0,.062,.662,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(4,5,-.139,.062,5,4));
	dynTrackBridge->surfaces.push_back(
	  TrackShape::Surface(6,7,.662,.862,5,4));
	if (bermHeight > 0) {
		dynTrackBridge->surfaces.push_back(
		  TrackShape::Surface(8,9,-.139,.062,5,4));
		dynTrackBridge->surfaces.push_back(
		  TrackShape::Surface(10,11,.662,.862,5,4));
	}
	dynTrackBridge->matchOffsets();
	dynTrackRails= new TrackShape;
	dynTrackRails->texture= new Texture;
	dynTrackRails->texture->texture= t;
	t->ref();
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7175,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7895,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7175,-.185));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7895,-.185));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7175,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7895,-.325));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7175,-.185));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7895,-.185));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7501,-.330));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7501,-.330));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.6785,-.170));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7535,-.187));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.7535,-.187));
	dynTrackRails->offsets.push_back(TrackShape::Offset(.8285,-.170));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.6785,-.170));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7535,-.187));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.7535,-.187));
	dynTrackRails->offsets.push_back(TrackShape::Offset(-.8285,-.170));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(0,1,-.9336,-.9883,-.5,4));
//	dynTrackRails->surfaces.push_back(
//	  TrackShape::Surface(0,8,.43,.50,-2,4));
//	dynTrackRails->surfaces.push_back(
//	  TrackShape::Surface(8,1,.50,.57,-2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(2,0,-.8359,-.9336,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(1,3,-.9336,-.8359,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(5,4,-.9883,-.9336,-.5,4));
//	dynTrackRails->surfaces.push_back(
//	  TrackShape::Surface(5,9,.57,.50,2,4));
//	dynTrackRails->surfaces.push_back(
//	  TrackShape::Surface(9,4,.50,.43,2,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(4,6,-.9336,-.8359,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(7,5,-.8359,-.9336,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(10,11,-.793,-.8414,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(12,13,-.8414,-.793,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(15,14,-.793,-.8414,-.5,4));
	dynTrackRails->surfaces.push_back(
	  TrackShape::Surface(17,16,-.8414,-.793,-.5,4));
	dynTrackRails->matchOffsets();
	if (bermHeight > 0) {
		string path= rTexturesDir+dirSep+"acleantrackbase.ace";
		osg::Texture2D* t= readCacheACEFile(path.c_str());
		dynTrackBerm= new TrackShape;
		dynTrackBerm->texture= new Texture;
		dynTrackBerm->texture->texture= t;
		t->ref();
		dynTrackBerm->offsets.push_back(TrackShape::Offset(
		  -2.75-1.5*bermHeight,bermHeight));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(-2.75,.1));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(2.75,.1));
		dynTrackBerm->offsets.push_back(TrackShape::Offset(
		  2.75+1.5*bermHeight,bermHeight));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(0,1,0,.4,50,4));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(1,2,.4,.6,50,4));
		dynTrackBerm->surfaces.push_back(
		  TrackShape::Surface(2,3,.6,1,50,4));
		dynTrackBerm->matchOffsets();
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(0,0,.1));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(3,1,.1));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(2,.6,.2));
		dynTrackBerm->endVerts.push_back(TrackShape::EndVert(1,.4,.2));
	}
	if (wireHeight > 0) {
		dynTrackWire= new TrackShape;
		float r= .015;
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(0,-wireHeight));
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(r,-wireHeight-r));
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(0,-wireHeight-2*r));
		dynTrackWire->offsets.push_back(
		  TrackShape::Offset(-r,-wireHeight-r));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(0,1,0,0,0,0));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(1,2,0,0,0,0));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(2,3,0,0,0,0));
		dynTrackWire->surfaces.push_back(
		  TrackShape::Surface(3,0,0,0,0,0));
		dynTrackWire->color= osg::Vec4(.2,.3,.2,1);
	}
	path= rTexturesDir+dirSep+"US_Track3s.ace";
	t= readCacheACEFile(path.c_str());
	if (t == NULL) {
		path= gTexturesDir+dirSep+"US_Track3.ace";
		t= readCacheACEFile(path.c_str());
	}
	dynTrackTies= new TrackShape;
	dynTrackTies->texture= new Texture;
	dynTrackTies->texture->texture= t;
	t->ref();
	dynTrackTies->offsets.push_back(TrackShape::Offset(-1.35,-.152));
	dynTrackTies->offsets.push_back(TrackShape::Offset(0,-.154));
	dynTrackTies->offsets.push_back(TrackShape::Offset(1.35,-.152));
	dynTrackTies->surfaces.push_back(
	  TrackShape::Surface(0,1,.6457,.375,5,4));
	dynTrackTies->surfaces.push_back(
	  TrackShape::Surface(1,2,.375,.1043,5,4));
	dynTrackTies->matchOffsets();
	return true;
}

void MSTSRoute::createWater(float waterLevel)
{
	Water* water= new Water;
	water->waterLevel= waterLevel;
	water->mstsRoute= this;
	waterList.push_back(water);
}

float MSTSRoute::getWaterDepth(double x, double y)
{
	int tx= centerTX + (int)rint(x/2048);
	int tz= centerTZ + (int)rint(y/2048);
	float dx= x - 2048*(tx-centerTX);
	float dz= y - 2048*(tz-centerTZ);
	int tx1= dx>128 ? tx+1 : dx<-128 ? tx-1 : tx;
	int tz1= dz>128 ? tz+1 : dz<-128 ? tz-1 : tz;
	Tile* tile= findTile(tx,tz);
	if (tile == NULL)
		return 0;
	Tile* t12= findTile(tile->x,tile->z-1);
	Tile* t21= findTile(tile->x+1,tile->z);
	Tile* t22= findTile(tile->x+1,tile->z-1);
	float a0= getAltitude(dx,dz,tile,t12,t21,t22);
	int j= (int)floor(dx/8) + 128;
	int i= 128 - (int)floor(dz/8);
//	fprintf(stderr,"getWaterDepth %f %f %d %d %f %f\n",
//	  x,y,i,j,a0,tile->getWaterLevel(i,j));
	return tile->getWaterLevel(i,j)-a0;
}

bool MSTSRoute::ignoreShape(string* filename, double x, double y, double z)
{
	int n= ignorePolygon.size();
	if (n > 0) {
		bool inside= true;
		for (int i=0; i<n-3 && inside; i+=2) {
			if (triArea(ignorePolygon[i],
			  ignorePolygon[i+1],ignorePolygon[i+2],
			  ignorePolygon[i+3],x,y)<0)
				inside= false;
		}
		if (inside) {
			fprintf(stderr,"ignore %s %.3f %.3f %.3f\n",
			  filename->c_str(),x,y,z);
			return true;
		}
	}
	for (multimap<string,osg::Vec3d>::iterator
	  i= ignoreShapeMap.lower_bound(*filename);
	  i!=ignoreShapeMap.end() && i->first==*filename; i++) {
		if (x-.001<i->second[0] && i->second[0]<x+.001 &&
		  y-.001<i->second[1] && i->second[1]<y+.001 &&
		  z-.001<i->second[2] && i->second[2]<z+.001)
			return true;
	}
	return false;
}

void MSTSRoute::calcTileAO(Tile* tile)
{
	fprintf(stderr,"tile AO %d %d\n",tile->x,tile->z);
	float x0= 2048*(tile->x-centerTX);
	float z0= 2048*(tile->z-centerTZ);
	float horizon[8];
#if 0
	for (int i=0; i<256; i++) {
		for (int j=0; j<256; j++) {
#else
	for (int i=128; i<130; i++) {
		for (int j=128; j<130; j++) {
#endif
			double x= x0+8*(i-128);
			double z= z0+8*(128-j);
			float a= getAltitude(i,j,tile,NULL,NULL,NULL);
			findHorizon(tile,x,a,z,horizon,8,1000);
			float sum= 0;
			for (int k=0; k<8; k++) {
				float cs= cos(atan(horizon[k]));
				sum+= cs*cs;
			}
			sum/= 8;
			fprintf(stderr," horizon %d %d %.0f %.0f ",i,j,x,z);
			for (int k=0; k<8; k++)
				fprintf(stderr," %.2f",horizon[k]);
			fprintf(stderr,"  %.3f\n",sum);
		}
	}
	fprintf(stderr,"tile AO %d %d\n",tile->x,tile->z);
}

float MSTSRoute::calcAO(double x, double y)
{
	int tx= centerTX + (int)rint(x/2048);
	int tz= centerTZ + (int)rint(y/2048);
	float dx= x - 2048*(tx-centerTX);
	float dz= y - 2048*(tz-centerTZ);
	Tile* tile= findTile(tx,tz);
	if (tile == NULL)
		return 0;
	float horizon[8];
	float a= getAltitude(dx,dz,tile,NULL,NULL,NULL);
	findHorizon(tile,x,a,y,horizon,8,2000);
	float sum= 0;
	for (int k=0; k<8; k++) {
		float cs= cos(atan(horizon[k]));
		sum+= cs*cs;
	}
	sum/= 8;
	fprintf(stderr,"horizon ");
	for (int k=0; k<8; k++)
		fprintf(stderr," %.2f",horizon[k]);
	fprintf(stderr,"  %.3f\n",sum);
	return 1-sum;
}

void MSTSRoute::findHorizon(Tile* tile, double px, double py, double pz,
  float* horizon, int nSegments, float radius)
{
	for (int i=0; i<nSegments; i++)
		horizon[i]= 0;
	for (TileMap::iterator i=tileMap.begin(); i!=tileMap.end(); ++i) {
		Tile* t= i->second;
		if (t->terrain == NULL)
			continue;
		if (t->x<tile->x-1 || t->x>tile->x+1 ||
		  t->z<tile->z-1 || t->z>tile->z+1)
			continue;
		float x0= 2048*(t->x-centerTX);
		float z0= 2048*(t->z-centerTZ);
		for (int j=0; j<256; j++) {
			for (int k=0; k<256; k++) {
				double x= x0+8*(k-128);
				double z= z0+8*(128-j);
				float a= getAltitude(j,k,t,NULL,NULL,NULL);
				if (a <= py)
					continue;
				float dx= px-x;
				float dz= pz-z;
				float d= sqrt(dx*dx+dz*dz);
				if (d<5 || d>radius)
					continue;
				float h= (a-py)/d;
				int hi= (int)floor((atan2(dz,dx)+M_PI)*
				  nSegments/(2*M_PI));
				if (hi >= nSegments)
					hi= nSegments-1;
				if (horizon[hi] < h)
					horizon[hi]= h;
			}
		}
	}
}
