//	class for managing/reading MSTS route information
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
#ifndef MSTSROUTE_H
#define MSTSROUTE_H

#include "trackdb.h"
#include "activity.h"
#include "ghproj.h"

#include "signal.h"

struct MSTSRoute {
	string mstsDir;
	string routeID;
	string dirSep;
	string routeDir;
	string fileName;
	string gShapesDir;
	string gTexturesDir;
	string tilesDir;
	string terrtexDir;
	string worldDir;
	string rShapesDir;
	string rTexturesDir;
	string activityName;
	string consistName;
	int centerTX;
	int centerTZ;
	float centerLat;
	float centerLong;
	double cosCenterLat;
	struct Terrain {
		unsigned short y[256][256];
		unsigned char n[256][256];
		unsigned char f[256][256];
	};
	struct Patch {
		int flags;
		short texIndex;
		short detail;
		short edgeDetail;
		float u0;
		float v0;
		float dudx;
		float dudz;
		float dvdx;
		float dvdz;
		float centerX;
		float centerZ;
	};
	typedef map<int,Track::SwVertex*> SwVertexMap;
	struct Tile {
		int x;
		int z;
		float floor;
		float scale;
		float swWaterLevel;
		float seWaterLevel;
		float neWaterLevel;
		float nwWaterLevel;
		string tFilename;
		osg::Group* models;
		osg::Geode* terrModel;
		osg::PagedLOD* plod;
		Terrain* terrain;
		Patch patches[256];
		SwVertexMap swVertexMap;
		std::vector<string> textures;
		std::vector<string> microTextures;
		float microTexUVMult;
		void freeTerrain();
		Tile(int tx, int tz) {	
			x= tx;
			z= tz;
			terrain= NULL;
			models= NULL;
			terrModel= NULL;
			microTexUVMult= 32;
		};
		float getWaterLevel(int i, int j);
	};
	int tileID(int tx, int tz) {
		return ((0xffff&tx)<<16) + (0xffff&tz);
	};
	struct TrackSection {
		float dist;
		float radius;
		TrackSection(float d, float r) {
			dist= d;
			radius= r;
		}
	};
	typedef std::vector<TrackSection> TrackSections;
	typedef map<int,Tile*> TileMap;
	typedef map<string,Tile*> TerrainTileMap;
	TileMap tileMap;
	TerrainTileMap terrainTileMap;
	typedef list<Tile*> TileList;
	TileList loadedTiles;
	typedef map<string,osg::Node*> ModelMap;
	ModelMap trackModelMap;
	ModelMap staticModelMap;
	TrackShape* dynTrackBase;
	TrackShape* dynTrackRails;
	TrackShape* dynTrackWire;
	TrackShape* dynTrackBerm;
	TrackShape* dynTrackBridge;
	TrackShape* dynTrackTies;
	float bermHeight;
	float wireHeight;
	bool bridgeBase;
	bool srDynTrack;
	bool ustDynTrack;
	string wireModelsDir;
	bool ignoreHiddenTerrain;
	typedef map<int,Event*> EventMap;
	EventMap eventMap;
	MSTSRoute(const char* mstsDir, const char* routeID);
	~MSTSRoute();
	void findCenter(TrackDB* trackDB);
	double convX(int tx, float x) {
		return 2048*(double)(tx-centerTX) + x;
	}
	double convZ(int tz, float z) {
		return 2048*(double)(tz-centerTZ) + z;
	}
	GHProjection ghProj;
	void ll2xy(double lat, double lng, double* x, double *y);
	void xy2ll(double lat, double lng, double* x, double *y);
	void makeTrack(int smoothGradeIterations, float smoothGradesDistance);
	void addSwitchStands(double offset, double zoffset,
	  osg::Node* model, osg::Group* rootNode, double poffset);
	void adjustWater(int setTerrain);
	const char* tFile(int x, int z);
	void tFileToXZ(char* filename, int *xp, int * zp);
	void readTiles();
	osg::Group* loadTiles(double x, double z);
	int readTFile(const char* path, Tile* tile);
	void readTerrain(Tile* tile);
	void writeTerrain(Tile* tile);
	Tile* findTile(int tx, int tz);
	void makeTileMap(osg::Group* root);
	void loadModels(Tile* tile);
	int readBinWFile(const char* filename, Tile* tile, float x0, float z0);
	void loadTerrainData(Tile* tile);
	osg::Node* loadTrackModel(string* filename, Track::SwVertex* sw);
	void overrideTrackModel(string& shapename, string& model);
	osg::Node* loadStaticModel(string* filename, MSTSSignal* signal=NULL);
	osg::Node* loadHazardModel(string* filename);
	osg::Node* attachSwitchStand(Tile* tile, osg::Node* model,
	  double x, double y, double z);
	void cleanStaticModelMap();
	osg::Node* makeDynTrack(MSTSFileNode* dynTrack);
	osg::Node* makeDynTrack(TrackSections& trackSections,bool bridge);
	osg::Node* makeTransfer(MSTSFileNode* transfer, string* filename,
	  Tile* tile, MSTSFileNode* pos, MSTSFileNode* qdir);
	osg::Node* makeTransfer(string* filename, Tile* tile,
	  osg::Vec3 center, osg::Quat quat, float w, float h);
	osg::Node* makeForest(MSTSFileNode* transfer,
	  Tile* tile, MSTSFileNode* pos, MSTSFileNode* qdir);
	void makeTerrainModel(Tile* tile);
	void makeWater(Tile* tile, float dl, const char* texture,
	  int renderBin);
	bool makeDynTrackShapes();
	bool makeSRDynTrackShapes();
	bool makeUSTDynTrackShapes();
	void createWater(float waterLevel);
	void saveShoreMarkers(const char* filename);
	int drawWater;
	float waterLevelDelta;
	void makeTerrainPatches(Tile* tile);
	osg::Geometry* makePatch(Patch* patch, int i0, int j0,
	  Tile* tile, Tile* t12, Tile* t21, Tile* t22);
	osg::Geometry* loadPatchGeoFile(Patch* patch, int i0, int j0,
	  Tile* tile);
	float getAltitude(int i, int j, Tile* tile,
	  Tile* t12, Tile* t21, Tile* t22);
	float getAltitude(float x, float z, Tile* tile,
	  Tile* t12, Tile* t21, Tile* t22);
	int getNormalIndex(int i, int j, Tile* tile,
	  Tile* t12, Tile* t21, Tile* t22);
	osg::Vec3f getNormal(int i, int j, Tile* tile,
	  Tile* t12, Tile* t21, Tile* t22);
	bool getVertexHidden(int i, int j, Tile* tile,
	  Tile* t12, Tile* t21, Tile* t22);
	std::vector<osg::Vec3f> terrainNormals;
	void saveTrackLocation(Tile* tile, float x, float z);
	void setPatchDetail(int detail);
	void setPatchDetail();
	void setPatchDetail(Patch* patch, int detail, int edge);
	void loadActivity(osg::Group* root, int activityFlags);
	void loadConsist(LooseConsist* consist, osg::Group* root);
	void loadExploreConsist(osg::Group* root);
	Track::Path* loadPath(string filename, bool align);
	Track::Path* loadService(string filename, osg::Group* root,
	  bool player, int id);
	bool signalSwitchStands;
	bool createSignals;
	MSTSSignal* findSignalInfo(MSTSFileNode* node);
	float getWaterDepth(double x, double y);
	std::vector<double> ignorePolygon;
	std::multimap<std::string,osg::Vec3d> ignoreShapeMap;
	bool ignoreShape(string* filename, double x, double y, double z);
	void calcTileAO(Tile* tile);
	float calcAO(double x, double y);
	void findHorizon(Tile* tile, double px, double py, double pz,
	  float* horizon, int nSegments, float radius);
};

#endif
