//	track profile code for making 3D model for track
//Copyright 2009 Doug Jones
#include "rmsim.h"

#include <map>
#include <vector>

using namespace std;
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>

TrackShapeMap trackShapeMap;

struct VInfo {
	int id;
	osg::Vec3 normal;
	int nEdges;
	VInfo(int i) {
		id= i;
		normal= osg::Vec3(0,0,0);
		nEdges= 0;
	};
	void addNormal(float dx, float dy);
};
typedef map<Track::Vertex*,VInfo> VInfoMap;

int sameDirection(float x1, float y1, float x2, float y2)
{
	float x= x1+x2;
	float y= y1+y2;
	float d1= x*x+y*y;
	x= x1-x2;
	y= y1-y2;
	float d2= x*x+y*y;
	return d1 > d2;
}

void VInfo::addNormal(float dx, float dy)
{
	if (nEdges==0 || sameDirection(normal[1],normal[0],dx,-dy))
		normal+= osg::Vec3(-dy,dx,0);
	else
		normal+= osg::Vec3(dy,-dx,0);
	nEdges++;
}

osg::Geometry* Track::makeGeometry()
{
	if (shape == NULL)
		return NULL;
	VInfoMap vInfoMap;
	int nv= 0;
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		vInfoMap.insert(make_pair(v,VInfo(nv++)));
	}
	for (EdgeList::iterator i=edgeList.begin(); i!=edgeList.end();
	  ++i) {
		Edge* e= *i;
		float dx= (e->v1->location.coord[0]-e->v2->location.coord[0]) /
		  e->length;
		float dy= (e->v1->location.coord[1]-e->v2->location.coord[1]) /
		  e->length;
		VInfoMap::iterator vi= vInfoMap.find(e->v1);
		vi->second.addNormal(dx,dy);
		vi= vInfoMap.find(e->v2);
		vi->second.addNormal(dx,dy);
	}
	int no= shape->offsets.size();
	vector<osg::Vec3> vertv;
	osg::Vec3Array* verts2= new osg::Vec3Array;
	for (VertexList::iterator i=vertexList.begin(); i!=vertexList.end();
	  ++i) {
		Vertex* v= *i;
		VInfoMap::iterator vi= vInfoMap.find(v);
//		if (vi->second.nEdges == 0)
//			continue;
		vi->second.normal.normalize();
		if (v->occupied)
		fprintf(stderr,"v %d %d %lf %lf %lf\n",
		  vi->second.id,vi->second.nEdges,vi->second.normal[0],
		  vi->second.normal[1],vi->second.normal[2]);
		float nx= vi->second.normal[0];
		float ny= vi->second.normal[1];
		for (int j=0; j<shape->offsets.size(); j++) {
			int j1= j;
			if (v->occupied && j%2==1)
				j1--;
			osg::Vec3 p= v->location.coord+
			  osg::Vec3(nx*shape->offsets[j1].x,
			  ny*shape->offsets[j1].x,-shape->offsets[j1].y);
			if (v->occupied)
			fprintf(stderr,"%d %f %f %f\n",
			  vi->second.id*no+j,p[0],p[1],p[2]);
			vertv.push_back(p);
		}
	}
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	int nvi= 0;
	for (EdgeList::iterator i=edgeList.begin(); i!=edgeList.end();
	  ++i) {
		Edge* e= *i;
		float edx= (e->v1->location.coord[0]-e->v2->location.coord[0])
		  / e->length;
		float edy= (e->v1->location.coord[1]-e->v2->location.coord[1])
		  / e->length;
		VInfoMap::iterator vi1= vInfoMap.find(e->v1);
		VInfoMap::iterator vi2= vInfoMap.find(e->v2);
		int v1o= vi1->second.id*no;
		int flip1= !sameDirection(vi1->second.normal[1],
		  vi1->second.normal[0],edx,-edy);
		int v2o= vi2->second.id*no;
		int flip2= !sameDirection(vi2->second.normal[1],
		  vi2->second.normal[0],edx,-edy);
		for (int j=0; j<shape->surfaces.size(); j++) {
			int flags= shape->surfaces[j].flags;
			if (e->occupied && flags && (e->occupied&flags)==0)
				continue;
			int i1= shape->surfaces[j].vo1;
			int o1= shape->offsets[i1].other;
			int i2= shape->surfaces[j].vo2;
			int o2= shape->offsets[i2].other;
			if (flip1) {
				verts2->push_back(vertv[v1o+o1]);
				verts2->push_back(vertv[v1o+o2]);
			} else {
				verts2->push_back(vertv[v1o+i1]);
				verts2->push_back(vertv[v1o+i2]);
			}
			if (flip2) {
				verts2->push_back(vertv[v2o+o2]);
				verts2->push_back(vertv[v2o+o2]);
				verts2->push_back(vertv[v2o+o1]);
			} else {
				verts2->push_back(vertv[v2o+i2]);
				verts2->push_back(vertv[v2o+i2]);
				verts2->push_back(vertv[v2o+i1]);
			}
			if (flip1) {
				verts2->push_back(vertv[v1o+o1]);
			} else {
				verts2->push_back(vertv[v1o+i1]);
			}
			nvi+= 6;
			float dx= shape->offsets[i2].x-shape->offsets[i1].x;
			float dy= shape->offsets[i2].y-shape->offsets[i1].y;
//			fprintf(stderr,
//			  "%p %f %f %f %d %d %d %d %d %d %d %d %d %d\n",e,
//			  normal[0],normal[1],normal[2],
//			  v1o,i1,v2o,i2,
//			  v1o+i1,v1o+i2,v2o+i2,v2o+i2,v2o+i1,v1o+i1);
			if (shape->texture != NULL) {
				float top= e->length/shape->surfaces[j].meters;
				float u1= shape->surfaces[j].u1;
				float u2= shape->surfaces[j].u2;
//				fprintf(stderr,"%f %f %f\n",u1,u2,top);
				if (top < 0) {
					texCoords->push_back(osg::Vec2(0,u1));
					texCoords->push_back(osg::Vec2(0,u2));
					texCoords->push_back(osg::Vec2(top,u2));
					texCoords->push_back(osg::Vec2(top,u2));
					texCoords->push_back(osg::Vec2(top,u1));
					texCoords->push_back(osg::Vec2(0,u1));
				} else {
					texCoords->push_back(osg::Vec2(u1,0));
					texCoords->push_back(osg::Vec2(u2,0));
					texCoords->push_back(osg::Vec2(u2,top));
					texCoords->push_back(osg::Vec2(u2,top));
					texCoords->push_back(osg::Vec2(u1,top));
					texCoords->push_back(osg::Vec2(u1,0));
				}
			}
		}
	}
	if (shape->endVerts.size() > 0) {
		int eo0= shape->endVerts[0].offset;
		float u0= shape->endVerts[0].u;
		float v0= shape->endVerts[0].v;
		for (VertexList::iterator i=vertexList.begin();
		  i!=vertexList.end(); ++i) {
			Vertex* v= *i;
			VInfoMap::iterator vi= vInfoMap.find(v);
			if (vi->second.nEdges != 1)
				continue;
			int vo= vi->second.id*no;
			int flip= vi->second.id>0;
			for (int j=2; j<shape->endVerts.size(); j++) {
				int eo1= shape->endVerts[j-1].offset;
				int eo2= shape->endVerts[j].offset;
				float u1= shape->endVerts[j-1].u;
				float v1= shape->endVerts[j-1].v;
				float u2= shape->endVerts[j].u;
				float v2= shape->endVerts[j].v;
				verts2->push_back(vertv[vo+eo0]);
				texCoords->push_back(osg::Vec2(u0,v0));
				if (flip) {
					verts2->push_back(vertv[vo+eo2]);
					verts2->push_back(vertv[vo+eo1]);
					texCoords->push_back(osg::Vec2(u2,v2));
					texCoords->push_back(osg::Vec2(u1,v1));
				} else {
					verts2->push_back(vertv[vo+eo1]);
					verts2->push_back(vertv[vo+eo2]);
					texCoords->push_back(osg::Vec2(u1,v1));
					texCoords->push_back(osg::Vec2(u2,v2));
				}
				nvi+= 3;
			}
		}
	}
	osg::Geometry* geometry= new osg::Geometry;
	geometry->setVertexArray(verts2);
	osg::Vec4Array* colors= new osg::Vec4Array;
	colors->push_back(shape->color);
	geometry->setColorArray(colors);
	geometry->setColorBinding(osg::Geometry::BIND_OVERALL);
	if (shape->texture != NULL) {
		geometry->setTexCoordArray(0,texCoords);
	}
	geometry->addPrimitiveSet(new osg::DrawArrays(
	  osg::PrimitiveSet::TRIANGLES,0,nvi));
	osgUtil::SmoothingVisitor::smooth(*geometry,.5);
	if (shape->texture != NULL) {
		osg::Texture2D* t= shape->texture->texture;
		if (t == NULL) {
			t= new osg::Texture2D;
			shape->texture->texture= t;
			t->setDataVariance(osg::Object::DYNAMIC);
			osg::Image* image= osgDB::readImageFile(
			  shape->texture->filename);
			if (image == NULL)
				fprintf(stderr,"cannot read image %s\n",
				  shape->texture->filename.c_str());
			t->setImage(image);
			t->setWrap(osg::Texture2D::WRAP_T,
			  osg::Texture2D::REPEAT);
			t->setWrap(osg::Texture2D::WRAP_S,
			  osg::Texture2D::REPEAT);
		}
//		fprintf(stderr,"t=%p %s\n",t,shape->texture->filename.c_str());
		osg::StateSet* stateSet= geometry->getOrCreateStateSet();
		stateSet->setTextureAttributeAndModes(0,t,
		  osg::StateAttribute::ON);
	}
	return geometry;
}

void TrackShape::matchOffsets()
{
	for (int i=0; i<offsets.size(); i++) {
		float bestd= 1e10;
		int bestj= i;
		for (int j=0; j<offsets.size(); j++) {
			float dx= offsets[i].x+offsets[j].x;
			float dy= offsets[i].y-offsets[j].y;
			float d= dx*dx+dy*dy;
			if (bestd > d) {
				bestd= d;
				bestj= j;
			}
		}
		offsets[i].other= bestj;
	}
}
