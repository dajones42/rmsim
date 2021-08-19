//	functions for managing navigable water
//	the water is a collection of triangles
//	ships keep track of which triangle they are in
//
//Copyright 2009 Doug Jones

#include "rmsim.h"

WaterList waterList;

Water::Vertex* Water::findVertex(int id)
{
	VertexMap::iterator i= vertexMap.find(id);
	return i==vertexMap.end() ? NULL : i->second;
}

void Water::addVertex(int id, double x, double y, double depth, int edgeID)
{
	Vertex* v= new Vertex();
	v->xy[0]= x;
	v->xy[1]= y;
	v->depth= depth;
	v->label= 0;
	v->color[0]= .7;
	v->color[1]= .8;
	v->color[2]= 1;
	v->next= NULL;
	v->prev= (Vertex*)(edgeID+1);
	vertexMap[id]= v;
}

void Water::addTriangle(int v1id, int v2id, int v3id)
{
	Vertex* v1= findVertex(v1id);
	Vertex* v2= findVertex(v2id);
	Vertex* v3= findVertex(v3id);
	if (v1==NULL || v2==NULL || v3==NULL)
		throw "cannot find vertex";
	Triangle* t= new Triangle;
	t->v[0]= v1;
	t->v[1]= v2;
	t->v[2]= v3;
	t->adj[0]= t->adj[1]= t->adj[2]= NULL;
	t->area= area(v1,v2,v3);
	t->len[0]= t->len[1]= t->len[2]= 0;
	triangleList.push_back(t);
}

struct Edge {
	Water::Vertex* v1;
	Water::Vertex* v2;
	Edge(Water::Vertex* v1, Water::Vertex* v2) {
		if (v1 < v2) {
			this->v1= v1;
			this->v2= v2;
		} else {
			this->v1= v2;
			this->v2= v1;
		}
	};
};
inline bool operator< (const Edge& e1, const Edge& e2)
{
	return e1.v1<e2.v1 || (e1.v1==e2.v1 && e1.v2<e2.v2);
}
struct TriRef {
	Water::Triangle* tri;
	int index;
	TriRef(Water::Triangle* t, int i) {
		tri= t;
		index= i;
	};
};
typedef map<Edge,TriRef> EdgeTriMap;

void Water::matchTriangles()
{
	for (VertexMap::iterator i=vertexMap.begin(); i!=vertexMap.end(); ++i)
		if (i->second->prev != NULL)
			i->second->prev= findVertex((long)(i->second->prev)-1);
	int nMatch= 0;
	EdgeTriMap edgeTriMap;
	for (TriangleList::iterator i=triangleList.begin();
	  i!=triangleList.end(); ++i) {
		Triangle* t= *i;
		for (int j=0; j<3; j++) {
			int j1= (j+1)%3;
			Edge edge(t->v[j],t->v[j1]);
			EdgeTriMap::iterator k= edgeTriMap.find(edge);
			if (k == edgeTriMap.end()) {
				edgeTriMap.insert(make_pair(edge,TriRef(t,j)));
				continue;
			}
			Triangle* t1= k->second.tri;
			t->adj[j]= t1;
			t1->adj[k->second.index]= t;
			nMatch++;
		}
	}
	for (TriangleList::iterator i=triangleList.begin();
	  i!=triangleList.end(); ++i) {
		Triangle* t= *i;
		for (int j=0; j<3; j++) {
			if (t->adj[j] != NULL)
				continue;
			int j1= (j+1)%3;
			t->v[j]->next= t->v[j1];
			t->v[j1]->prev= t->v[j];
		}
	}
	fprintf(stderr,"%d triangles %d inside edges %d outside edges\n",
	  triangleList.size(),2*nMatch,3*triangleList.size()-2*nMatch);
}

int Water::Location::set(double x, double y)
{
	Vertex v;
	v.xy[0]= x;
	v.xy[1]= y;
	for (WaterList::iterator i=waterList.begin(); i!=waterList.end(); ++i) {
		Water* w= *i;
		if (w->mstsRoute) {
			water= w;
			depth= w->mstsRoute->getWaterDepth(x,y);
			return 1;
		}
		for (TriangleList::iterator j=w->triangleList.begin();
		  j!=w->triangleList.end(); ++j) {
			Triangle* t= *j;
			if (t->inside(&v)) {
				triangle= t;
				water= w;
				update(v.xy);
				return 1;
			}
		}
	}
	return 0;
}

void printVertex(Water::Vertex* v)
{
	fprintf(stderr," %.2lf,%.2lf",v->xy[0],v->xy[1]);
}

int Water::Location::update(double x, double y)
{
	if (triangle == NULL)
		return set(x,y);
	Vertex v;
	v.xy[0]= x;
	v.xy[1]= y;
	depth= 0;
	Triangle* t= triangle;
	int k=0;
	double a[3];
	for (int i=0; i<3; i++) {
		a[i]= area(&v,t->v[i],t->v[(i+1)%3]);
		if (a[i] >= 0)
			continue;
		if (t->adj[i] == NULL)
			return 0;
		if (k++ > 10)
			return set(v.xy[0],v.xy[1]);
		t= t->adj[i];
		i= -1;
	}
	triangle= t;
	for (int i=0; i<3; i++) {
		int i1= (i+1)%3;
		int i2= (i+2)%3;
		depth+= a[i]*t->v[i2]->depth;
#if 0
		if (t->len[i] <= 0) {
			float dx= t->v[i]->xy[0] - t->v[i1]->xy[0];
			float dy= t->v[i]->xy[1] - t->v[i1]->xy[1];
			t->len[i]= sqrt(dx*dx+dy*dy);
		}
#endif
	}
	depth/= t->area;
#if 0
	if (k > 0) {
		fprintf(stderr,"newtri %f %f\n",depth,t->area);
		for (int i=0; i<3; i++)
			fprintf(stderr," %f %f %lf %lf %lf %d\n",
			  t->v[i]->depth,t->len[i],a[i],
			  t->v[i]->xy[0],t->v[i]->xy[1],
			  t->adj[i]==NULL);
	}
#endif
#if 0
	if (k > 0) {
		for (int i=0; i<3; i++)
			if (t->adj[i] == NULL)
				fprintf(stderr,"%lf,%lf %lf,%lf\n",
				  t->v[i]->xy[0],t->v[i]->xy[1],
				  t->v[(i+1)%3]->xy[0],t->v[(i+1)%3]->xy[1]);
	}
#endif
	return 1;
}

void Water::setVColor(float* c0, float* c1, float depth)
{
	for (VertexMap::iterator i=vertexMap.begin(); i!=vertexMap.end(); ++i) {
		Vertex* v= i->second;
		float x= v->depth>depth ? 1 : v->depth/depth;
		for (int j=0; j<3; j++)
			v->color[j]= (1-x)*c0[j] + x*c1[j];
	}
}

//	creates an MSTS marker file with points that match the water shore line
void Water::saveShoreLine(const char* filename)
{
	for (VertexMap::iterator i=vertexMap.begin(); i!=vertexMap.end(); ++i) {
		Vertex* v= i->second;
		v->label= 0;
	}
	for (TriangleList::iterator j=triangleList.begin();
	  j!=triangleList.end(); ++j) {
		Triangle* t= *j;
		for (int k=0; k<3; k++) {
			if (t->adj[k] != NULL)
				continue;
			t->v[k]->label= 1;
			int k1= (k+1)%3;
			t->v[k1]->label= 1;
		}
	}
	FILE* out= fopen(filename,"w");
	if (out == NULL) {
		fprintf(stderr,"cannot create %s\n",filename);
		return;
	}
	fprintf(out,"SIMISA@@@@@@@@@@JINXOIOt________\n");
	float clat= 40.7;
	float clng= -74;
	float yMult= 1.319027;
	int n= 0;
	for (VertexMap::iterator i=vertexMap.begin(); i!=vertexMap.end(); ++i) {
		Vertex* v= i->second;
//		if (v->label == 0)
//			continue;
		float lat= clat + v->xy[1]/(yMult*60*1852.216);
		float lng= clng + v->xy[0]/(60*1852.216);
//		if (lng<-74.026576 || lng>-73.988822 ||
//		  lat<40.675561 || lat>40.707368)
//			continue;
		fprintf(out,"Marker ( %f %f %.0f 2 )\n",lng,lat,
		  v->label==0?v->depth:-v->depth);
	}
	fclose(out);
}
