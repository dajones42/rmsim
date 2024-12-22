//	class for reading MSTS shape files
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
#ifndef MSTSSHAPE_H
#define MSTSSHAPE_H

struct MSTSSignal;

struct MSTSShape {
	string filename;
	string directory;
	string directory2;
	vector<int> shaders;
	struct Point {
		float x;
		float y;
		float z;
		int index;
		Point(float x, float y, float z) {
			this->x= x;
			this->y= y;
			this->z= z;
		};
	};
	vector<Point> points;
	struct UVPoint {
		float u;
		float v;
		int index;
		UVPoint(float u, float v) {
			this->u= u;
			this->v= v;
		};
	};
	vector<UVPoint> uvPoints;
	struct Normal {
		float x;
		float y;
		float z;
		int index;
		Normal(float x, float y, float z) {
			this->x= x;
			this->y= y;
			this->z= z;
		};
	};
	vector<Normal> normals;
	struct Matrix {
		string name;
		int part;
		osg::Matrixd matrix;
		osg::Geode* geode;
		osg::MatrixTransform* transform;
		Matrix(string& s) {
			name= s;
			geode= NULL;
			transform= NULL;
			part= -1;
		};
	};
	vector<Matrix> matrices;
	vector<string> images;
	struct Texture {
		int imageIndex;
		osg::Texture2D* texture;
		Texture(int i) {
			imageIndex= i;
			texture= NULL;
		};
	};
	vector<Texture> textures;
	struct VTXState {
		int matrixIndex;
		int lightMaterialIndex;
		VTXState(int i, int j) {
			matrixIndex= i;
			lightMaterialIndex= j;
		};
	};
	vector<VTXState> vtxStates;
	struct PrimState {
		vector<int> texIdxs;
		int vStateIndex;
		int shaderIndex;
		int alphaTestMode;
		int zBufMode;
		PrimState(int vsi, int si) {
			vStateIndex= vsi;
			shaderIndex= si;
			alphaTestMode= 0;
			zBufMode= 1;
		};
	};
	vector<PrimState> primStates;
	struct Vertex {
		int pointIndex;
		int normalIndex;
		int uvIndex;
		int index;
		Vertex(int p, int n, int uv) {
			pointIndex= p;
			normalIndex= n;
			uvIndex= uv;
		};
	};
	struct VertexSet {
		int stateIndex;
		int startIndex;
		int nVertex;
		VertexSet(int state, int start, int n) {
			stateIndex= state;
			startIndex= start;
			nVertex= n;
		};
	};
	struct TriList {
		int primStateIndex;
		vector<int> vertexIndices;
		vector<int> normalIndices;
		TriList(int psi) {
			primStateIndex= psi;
		};
	};
	struct SubObject {
		vector<Vertex> vertices;
		vector<VertexSet> vertexSets;
		vector<TriList> triLists;
	};
	struct DistLevel {
		float dist;
		vector<int> hierarchy;
		vector<SubObject> subObjects;
		
	};
	vector<DistLevel> distLevels;
	struct AnimNode {
		string name;
		map<int,osg::Vec3f> positions;
		map<int,osg::Quat> quats;
		AnimNode(string s) {
			name=s;
		};
	};
	struct Animation {
		int nFrames;
		int frameRate;
		vector<AnimNode> nodes;
		Animation(int n, int r) {
			nFrames= n;
			frameRate= r;
		};
	};
	vector<Animation> animations;
	int readBinFile(const char* filename);
	void readFile(const char* filename, const char* texDir1=NULL,
	 const char* texDir2=NULL);
	void createRailCar(RailCarDef* def, bool saveNames=false);
	void makeGeometry(SubObject& subObject, TriList& triList,
	  int& transparentBin, bool incTransparentBin= false);
	void makeSplitGeometry(SubObject& subObject, TriList& triList,
	  int transparentBin);
	osg::Node* createModel(int transform=1, int transparentBin=10,
	  bool saveNames=false, bool incTransparentBin= false);
	void readACEFiles();
	osg::Texture2D* readACEFile(string& path);
	void makeLOD();
	void printSubobjects();
	void fixTop();
	osg::Vec3d* signalLightOffset;
	static bool useAO;
	MSTSShape() {
		signalLightOffset= NULL;
	}
	void calcAO(SubObject& subObject, TriList& triList);
	float getAO(int pointIndex);
	struct AOPoint {
		osg::Vec3f point;
		osg::Vec3f normal;
		osg::Vec3f bentNormal;
		float area;
		float occlusion;
		AOPoint() {
			area= 0;
			occlusion= 0;
		}
	};
	std::map<int,AOPoint> aoPoints;
};
void printTree(osg::Node* node, int depth);

#endif
