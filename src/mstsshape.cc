//	code to read MSTS shape files and convert them to OSG
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
#include "mstsfile.h"
#include "mstsbfile.h"
#include "ufsets.h"

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Texture>
#include <osg/TexEnvFilter>
#include <osg/FrontFace>
#include <osg/AnimationPath>
#include <osg/BlendFunc>
#include <osg/AlphaFunc>
#include <osg/Material>
#include <osg/LOD>
#include <osgSim/LightPointNode>

struct ShadowCalc {
	osg::Vec3 sum1;
	osg::Vec3 sum2;
	int nTri;
	ShadowCalc() {
		sum1= osg::Vec3(0,0,0);
		sum2= osg::Vec3(0,0,0);
		nTri= 0;
	};
	void addTri(osg::Vec3 p1, osg::Vec3 p2, osg::Vec3 p3) {
		osg::Vec3 cross= (p2-p1)^(p3-p1);
		cross*= .5;
		for (int i=0; i<3; i++) {
			if (cross[i]<0)
				sum2[i]-= cross[i];
			else
				sum1[i]+= cross[i];
		}
		nTri++;
	};
	float getArea() {
		osg::Vec3 sum= sum1;
		for (int i=0; i<3; i++)
			if (sum[i] < sum2[i])
				sum[i]= sum2[i];
		return sum.length();
	};
};

float drawableRadius(osg::Geode* geode)
{
	float max= 0;
	for (int i=0; i<geode->getNumDrawables(); i++) {
		osg::Geometry* geom=
		  dynamic_cast<osg::Geometry*>(geode->getDrawable(i));
		if (!geom)
			continue;
		float r= geom->getBound().radius();
//		if (max < r)
//			max= r;
		osg::Vec3Array* va=
		  (osg::Vec3Array*) geom->getVertexArray();
		float* v= (float*) va->getDataPointer();
		UFSets vGroups(va->getNumElements());
		osg::Vec3 sum= osg::Vec3(0,0,0);
		osg::Vec3 sum1= osg::Vec3(0,0,0);
		int nt= 0;
		for (int j=0; j<geom->getNumPrimitiveSets(); j++) {
			osg::DrawElementsUShort* de= (osg::DrawElementsUShort*)
			  geom->getPrimitiveSet(j)->getDrawElements();
			for (int k=0; k<de->getNumIndices(); k+=3) {
				int j1= de->getElement(k);
				int j2= de->getElement(k+1);
				int j3= de->getElement(k+2);
				osg::Vec3 v1=
				  osg::Vec3(v[3*j1],v[3*j1+1],v[3*j1+2]);
				osg::Vec3 v2=
				  osg::Vec3(v[3*j2],v[3*j2+1],v[3*j2+2]);
				osg::Vec3 v3=
				  osg::Vec3(v[3*j3],v[3*j3+1],v[3*j3+2]);
				osg::Vec3 cross= (v2-v1)^(v3-v1);
				cross*= .5;
				for (int k1=0; k1<3; k1++) {
					if (cross[k1]<0)
						sum1[k1]-= cross[k1];
					else
						sum[k1]+= cross[k1];
				}
				nt++;
				int s1= vGroups.find(j1);
				int s2= vGroups.find(j2);
				if (s1 != s2)
					s1= vGroups.link(s1,s2);
				int s3= vGroups.find(j3);
				if (s1 != s3)
					s1= vGroups.link(s1,s3);
			}
		}
		int ns= 0;
		for (int j=0; j<va->getNumElements(); j++)
			if (vGroups.find(j) == j)
				ns++;
		for (int k1=0; k1<3; k1++)
			if (sum[k1] < sum1[k1])
				sum[k1]= sum1[k1];
		float shadowArea= sum.length();
		float r1= sqrt(shadowArea/M_PI);
		if (max < r1)
			max= r1;
//		if (nt > 100)
//		fprintf(stderr,"  %d %f %f %f %f %d %d\n",
//		  i,r,max,r1,sum.length(),nt,ns);
	}
	return max;
}

//	reads an uncompressed shape file
void MSTSShape::readFile(const char* filename, const char* texDir1,
  const char* texDir2)
{
//	fprintf(stderr,"readshape %s\n",filename);
	if (texDir1 == NULL) {
		char* p= strrchr((char*)filename,'/');
		if (p != NULL)
			directory= string(filename,p-filename+1);
	} else {
		directory= string(texDir1);
		if (texDir2 != NULL)
			directory2= string(texDir2);
	}
	if (readBinFile(filename))
		return;
	MSTSFile file;
	file.readFile(filename);
	MSTSFileNode* shape= file.find("shape");
	if (shape == NULL)
		throw "not a MSTS shape file?";
	MSTSFileNode* node= shape->children->find("shader_names");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("named_shader");
		  p!=NULL; p=p->find("named_shader")) {
			const char* s= p->getChild(0)->value->c_str();
			if (strcmp(s,"TexDiff")==0 || strcmp(s,"Tex")==0) {
				shaders.push_back(0);
			} else if (strcmp(s,"BlendATexDiff")==0 ||
			  strcmp(s,"BlendATex")==0) {
				shaders.push_back(1);
			} else {
				shaders.push_back(0);
//				fprintf(stderr,"named_shader %s\n",s);
			}
		}
	}
	node= shape->children->find("points");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("point");
		  p!=NULL; p=p->find("point")) {
			points.push_back(Point(
			  atof(p->getChild(0)->value->c_str()),
			  atof(p->getChild(1)->value->c_str()),
			  atof(p->getChild(2)->value->c_str())));
		}
	}
	node= shape->children->find("uv_points");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("uv_point");
		  p!=NULL; p=p->find("uv_point")) {
			uvPoints.push_back(UVPoint(
			  atof(p->getChild(0)->value->c_str()),
			  atof(p->getChild(1)->value->c_str())));
		}
	}
	node= shape->children->find("normals");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("vector");
		  p!=NULL; p=p->find("vector")) {
			normals.push_back(Normal(
			  atof(p->getChild(0)->value->c_str()),
			  atof(p->getChild(1)->value->c_str()),
			  atof(p->getChild(2)->value->c_str())));
		}
	}
	node= shape->children->find("matrices");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("matrix");
		  p!=NULL; p=p->find("matrix")) {
			int i= matrices.size();
			matrices.push_back(Matrix(*(p->value)));
			matrices[i].matrix= osg::Matrixd(
			  atof(p->next->getChild(0)->value->c_str()),
			  atof(p->next->getChild(1)->value->c_str()),
			  atof(p->next->getChild(2)->value->c_str()),
			  0,
			  atof(p->next->getChild(3)->value->c_str()),
			  atof(p->next->getChild(4)->value->c_str()),
			  atof(p->next->getChild(5)->value->c_str()),
			  0,
			  atof(p->next->getChild(6)->value->c_str()),
			  atof(p->next->getChild(7)->value->c_str()),
			  atof(p->next->getChild(8)->value->c_str()),
			  0,
			  atof(p->next->getChild(9)->value->c_str()),
			  atof(p->next->getChild(10)->value->c_str()),
			  atof(p->next->getChild(11)->value->c_str()),
			  1);
		}
	}
	node= shape->children->find("images");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("image");
		  p!=NULL; p=p->find("image")) {
			images.push_back(*(p->getChild(0)->value));
		}
	}
	node= shape->children->find("textures");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("texture");
		  p!=NULL; p=p->find("texture")) {
			textures.push_back(
			  Texture(atoi(p->getChild(0)->value->c_str())));
		}
	}
	node= shape->children->find("vtx_states");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("vtx_state");
		  p!=NULL; p=p->find("vtx_state")) {
			vtxStates.push_back(
			  VTXState(atoi(p->getChild(1)->value->c_str()),
			    atoi(p->getChild(2)->value->c_str())));
		}
	}
	node= shape->children->find("prim_states");
	if (node != NULL) {
		for (MSTSFileNode* p= node->children->find("prim_state");
		  p!=NULL; p=p->find("prim_state")) {
			if (p->children == NULL)
				p= p->next;
			int i= primStates.size();
			primStates.push_back(PrimState(
			  atoi(p->getChild(5)->value->c_str()),
			  atoi(p->getChild(1)->value->c_str())));
			MSTSFileNode* p1= p->children->find("tex_idxs");
			int n= atoi(p1->getChild(0)->value->c_str());
			if (n >= 1)
				primStates[i].texIdxs.push_back(
				  atoi(p1->getChild(1)->value->c_str()));
		}
	}
	node= shape->children->find("lod_controls");
	for (MSTSFileNode* lod= node->children->find("lod_control");
	  lod!=NULL; lod=lod->find("lod_control")) {
	for (MSTSFileNode* dls= lod->children->find("distance_levels");
	  dls!=NULL; dls=dls->find("distance_levels")) {
	for (MSTSFileNode* dl= dls->children->find("distance_level");
	  dls!=NULL; dls=dls->find("distance_level")) {
		int i= distLevels.size();
		distLevels.push_back(DistLevel());
		MSTSFileNode* dlh= dl->children->find("distance_level_header");
		MSTSFileNode* p= dlh->children->find("dlevel_selection");
		distLevels[i].dist= atof(p->getChild(0)->value->c_str());
		p= dlh->children->find("hierarchy");
		int n= atoi(p->getChild(0)->value->c_str());
		for (int j=1; j<=n; j++)
			distLevels[i].hierarchy.push_back(
			  atoi(p->getChild(j)->value->c_str()));
		MSTSFileNode* subs= dl->children->find("sub_objects");
		for (MSTSFileNode* so= subs->children->find("sub_object");
		  so!=NULL; so=so->find("sub_object")) {
			int j= distLevels[i].subObjects.size();
			distLevels[i].subObjects.push_back(SubObject());
			node= so->children->find("vertices");
			if (node == NULL)
				continue;
			for (MSTSFileNode* p= node->children->find("vertex");
			  p!=NULL; p=p->find("vertex")) {
				MSTSFileNode* p1=
				  p->children->find("vertex_uvs");
				int n= p1==NULL?0:
				  atoi(p1->getChild(0)->value->c_str());
				distLevels[i].subObjects[j].vertices.push_back(
				  Vertex(
				    atoi(p->getChild(1)->value->c_str()),
				    atoi(p->getChild(2)->value->c_str()),
				    n==0?0:
				    atoi(p1->getChild(1)->value->c_str())));
			}
			node= so->children->find("vertex_sets");
			for (MSTSFileNode* p=
			  node->children->find("vertex_set");
			  p!=NULL; p=p->find("vertex_set")) {
				distLevels[i].subObjects[j].vertexSets.
				  push_back(VertexSet(
				    atoi(p->getChild(0)->value->c_str()),
				    atoi(p->getChild(1)->value->c_str()),
				    atoi(p->getChild(2)->value->c_str())));
			}
			node= so->children->find("primitives");
			int primStateIndex= -1;
			for (MSTSFileNode* p=node->children; p!=NULL;
			  p=p->next) {
				if (p->value!=NULL &&
				  *(p->value)=="prim_state_idx") {
					primStateIndex= atoi(p->next->
					  getChild(0)->value->c_str());
					continue;
				}
				if (p->value==NULL ||
				  *(p->value)!="indexed_trilist")
					continue;
				int k= distLevels[i].subObjects[j].
				  triLists.size();
				distLevels[i].subObjects[j].triLists.push_back(
				  TriList(primStateIndex));
				MSTSFileNode* p1=
				  p->next->children->find("vertex_idxs");
				int n= atoi(p1->getChild(0)->value->c_str());
				for (int j1=1; j1<=n; j1++)
					distLevels[i].subObjects[j].triLists[k].
					  vertexIndices.push_back(atoi(
					   p1->getChild(j1)->value->c_str()));
				p1= p->next->children->find("normal_idxs");
				n= atoi(p1->getChild(0)->value->c_str());
				for (int j1=1; j1<=n; j1++)
					distLevels[i].subObjects[j].triLists[k].
					  normalIndices.push_back(atoi(
					   p1->getChild(j1)->value->c_str()));
			}
		}
	}
	}
	}	
//	fprintf(stderr,"animations\n");
	node= shape->children->find("animations");
	if (node != NULL) {
	 for (MSTSFileNode* a= node->children->find("animation");
	   a!=NULL; a=a->find("animation")) {
	 	int i= animations.size();
	 	animations.push_back(Animation(
	 	  atoi(a->getChild(0)->value->c_str()),
	 	  atof(a->getChild(1)->value->c_str())));
	 	for (MSTSFileNode* anodes= a->children->find("anim_nodes");
	 	  anodes!=NULL; anodes=anodes->find("anim_nodes")) {
		 	for (MSTSFileNode* an=
			  anodes->children->find("anim_node");
		 	  an!=NULL; an=an->find("anim_node")) {
				int j= animations[i].nodes.size();
				animations[i].nodes.push_back(
				  AnimNode(*(an->value)));
			 	for (MSTSFileNode* c=
				  an->next->children->find("controllers");
			 	  c!=NULL; c=c->find("controllers")) {
			 	 for (MSTSFileNode* r=
				   c->children->find("tcb_rot");
			 	   r!=NULL; r=r->find("tcb_rot")) {
			 	  for (MSTSFileNode* k=
				    r->children->find("tcb_key");
			 	    k!=NULL; k=k->find("tcb_key")) {
					int frame=
					  atoi(k->getChild(0)->value->c_str());
					if (frame>=animations[i].nFrames &&
					  animations[i].nodes[j].quats.size()==
					  animations[i].nFrames)
						continue;
					animations[i].nodes[j].quats[frame]=
					  osg::Quat(
					  -atof(k->getChild(1)->value->c_str()),
					  -atof(k->getChild(2)->value->c_str()),
					  -atof(k->getChild(3)->value->c_str()),
					  atof(k->getChild(4)->value->c_str()));
				  }
			 	  for (MSTSFileNode* k=
				    r->children->find("slerp_rot");
			 	    k!=NULL; k=k->find("slerp_rot")) {
					int frame=
					  atoi(k->getChild(0)->value->c_str());
					if (frame>=animations[i].nFrames &&
					  animations[i].nodes[j].quats.size()==
					  animations[i].nFrames)
						continue;
					animations[i].nodes[j].quats[frame]=
					  osg::Quat(
					  -atof(k->getChild(1)->value->c_str()),
					  -atof(k->getChild(2)->value->c_str()),
					  -atof(k->getChild(3)->value->c_str()),
					  atof(k->getChild(4)->value->c_str()));
				  }
				 }
			 	 for (MSTSFileNode* p=
				   c->children->find("linear_pos");
			 	   p!=NULL; p=p->find("linear_pos")) {
			 	  for (MSTSFileNode* k=
				    p->children->find("linear_key");
			 	    k!=NULL; k=k->find("linear_key")) {
					int frame=
					  atoi(k->getChild(0)->value->c_str());
					if (frame>=animations[i].nFrames &&
					  animations[i].nodes[j].positions.
					  size()==animations[i].nFrames)
						continue;
					animations[i].nodes[j].
					  positions[frame]= osg::Vec3f(
					  atof(k->getChild(1)->value->c_str()),
					  atof(k->getChild(2)->value->c_str()),
					  atof(k->getChild(3)->value->c_str()));
				  }
				 }
				}
			}
	 	}
	 }
	}
}

//	reads a binary shape file
int MSTSShape::readBinFile(const char* filename)
{
	MSTSBFile reader;
	if (reader.open(filename))
		return 0;
//	fprintf(stderr,"%s is binary %d\n",filename,reader.compressed);
	Byte buf[16];
	reader.getBytes(buf,16);
	int dli= 0;
	int soi= 0;
	int psi= 0;
	for (;;) {
		int code= reader.getInt();
		int len= reader.getInt();
		if (code == 0)
			break;
//		fprintf(stderr,"%d %d\n",code,len);
		switch (code) {
		 case 70: // shape_header
		 case 68: // volumes
		 case 74: // texture_filter_names
		 case 76: // sort_vectors
		 case 11: // colours
		 case 18: // light_materials
		 case 79: // light_model_cfgs
		 case 33: // distance_levels_header
		 case 40: // sub_object_header
		 case 52: // vertex_sets
		 case 6: // normal_idxs
		 case 64: // flags
			reader.getBytes(NULL,len);
			break;
		 case 72: // shader_names
		 case 7: // points
		 case 9: // uv_points
		 case 5: // normals
		 case 66: // matrices
		 case 14: // images
		 case 16: // textures
		 case 47: // vtx_states
		 case 55: // prim_states
		 case 31: // lod_controls
		 case 36: // distance_levels
		 case 38: // sub_objects
		 case 50: // vertices
		 case 53: // primitives
		 case 27: // anim_nodes
			reader.getString();
			reader.getInt();
			break;
		 case 71: // shape
		 case 32: // lod_control
		 case 34: // distance_level_header
		 case 60: // indexed_trilist
			reader.getString();
			break;
		 case 129: // named_shader
			reader.getString();
			{
				int n= reader.getShort();
				const char* s= reader.getString(n).c_str();
				if (strcmp(s,"TexDiff")==0 ||
				  strcmp(s,"Tex")==0) {
					shaders.push_back(0);
				} else if (strcmp(s,"BlendATexDiff")==0 ||
				  strcmp(s,"BlendATex")==0) {
					shaders.push_back(1);
				} else {
					shaders.push_back(0);
//					fprintf(stderr,"named_shader %s\n",s);
				}
			}
			break;
		 case 2: // point
			reader.getString();
			{
				float x= reader.getFloat();
				float y= reader.getFloat();
				float z= reader.getFloat();
				points.push_back(Point(x,y,z));
			}
			break;
		 case 8: // uv_point
			reader.getString();
			{
				float u= reader.getFloat();
				float v= reader.getFloat();
				uvPoints.push_back(UVPoint(u,v));
			}
			break;
		 case 3: // vector
			reader.getString();
			{
				float x= reader.getFloat();
				float y= reader.getFloat();
				float z= reader.getFloat();
				normals.push_back(Normal(x,y,z));
			}
			break;
		 case 65: // matrix
			{
				string name= reader.getString();
				float m1= reader.getFloat();
				float m2= reader.getFloat();
				float m3= reader.getFloat();
				float m4= reader.getFloat();
				float m5= reader.getFloat();
				float m6= reader.getFloat();
				float m7= reader.getFloat();
				float m8= reader.getFloat();
				float m9= reader.getFloat();
				float m10= reader.getFloat();
				float m11= reader.getFloat();
				float m12= reader.getFloat();
				int i= matrices.size();
				matrices.push_back(Matrix(name));
				matrices[i].matrix= osg::Matrixd(m1,m2,m3,0,
				  m4,m5,m6,0,m7,m8,m9,0,m10,m11,m12,1);
//				fprintf(stderr,"matrix %s %f %f %f"
//				  " %f %f %f %f %f %f %f %f %f\n",name.c_str(),
//				m1,m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12);
			}
			break;
		 case 13: // image
			{
				reader.getString();
				int n= reader.getShort();
				string s= reader.getString(n);
				images.push_back(s);
			}
			break;
		 case 15: // texture
			reader.getString();
			textures.push_back(Texture(reader.getInt()));
			reader.getInt();
			reader.getFloat();
			if (len > 13)
				reader.getInt();
			break;
		 case 46: // vtx_state
			{
				int read= reader.read;
				reader.getString();
				reader.getInt();
				int matIdx= reader.getInt();
				int lightMatIdx= reader.getInt();
				vtxStates.push_back(
				  VTXState(matIdx,lightMatIdx));
				if (reader.read < read+len)
					reader.getBytes(NULL,
					  read+len-reader.read);
			}
			break;
		 case 54: // prim_state
			{ 
				int read= reader.read;
				int i= primStates.size();
				primStates.push_back(PrimState(0,0));
				reader.getString();
				reader.getInt();
				primStates[i].shaderIndex= reader.getInt();
				reader.getInt(); // tex_idxs
				int len1= reader.getInt();
				reader.getString();
				int n= reader.getInt();
//				fprintf(stderr,"tex_idxs %d %d\n",len1,n);
				for (int j=0; j<n; j++)
					primStates[i].texIdxs.push_back(
					  reader.getInt());
				reader.getInt();
				primStates[i].vStateIndex= reader.getInt();
				if (reader.read < read+len) {
					n= read+len-reader.read;
					if (n >= 4) {
						primStates[i].alphaTestMode=
						  reader.getInt();
						n-= 4;
					}
					if (n >= 4) {
						reader.getInt();
						n-= 4;
					}
					if (n >= 4) {
						primStates[i].zBufMode=
						  reader.getInt();
						n-= 4;
					}
					if (n > 0)
						reader.getBytes(NULL,n);
				}
			}
			break;
		 case 37: // distance_level
			reader.getString();
			dli= distLevels.size();
			distLevels.push_back(DistLevel());
			break;
		 case 35: // dlevel_selection
			reader.getString();
			distLevels[dli].dist= reader.getFloat();
			break;
		 case 67: // heirarchy
			{
				reader.getString();
				int n= reader.getInt();
				for (int j=1; j<=n; j++)
					distLevels[dli].hierarchy.push_back(
					  reader.getInt());
			}
			break;
		 case 39: // sub_object
			reader.getString();
			soi= distLevels[dli].subObjects.size();
			distLevels[dli].subObjects.push_back(SubObject());
			break;
		 case 48: // vertex
			{
				int read= reader.read;
				reader.getString();
				reader.getInt();
				int pi= reader.getInt();
				int ni= reader.getInt();
				reader.getInt();
				reader.getInt();
				reader.getInt(); // vertex_uvs
				int uvlen= reader.getInt();
				reader.getString();
				int nuv= reader.getInt();
				int uvi= 0;
				for (int k=0; k<nuv; k++)
					uvi= reader.getInt();
				distLevels[dli].subObjects[soi].vertices.
				  push_back(Vertex(pi,ni,uvi));
				if (reader.read < read+len)
					reader.getBytes(NULL,
					  read+len-reader.read);
			}
			break;
		 case 56: // prim_state_index
			reader.getString();
			psi= reader.getInt();
			break;
		 case 63: // vertex_idxs
			{
				reader.getString();
				int n= reader.getInt();
				int k= distLevels[dli].subObjects[soi].
				  triLists.size();
				distLevels[dli].subObjects[soi].triLists.
				  push_back(TriList(psi));
				for (int j=0; j<n; j++)
					distLevels[dli].subObjects[soi].
					  triLists[k].vertexIndices.
					  push_back(reader.getInt());
			}
			break;
		 case 29: // animations
			reader.getString();
			reader.getInt();
			break;
		 case 28: // animation
			{
				reader.getString();
				int n= reader.getInt();
				int r= reader.getInt();
				animations.push_back(Animation(n,r));
			}
			break;
		 case 26: // anim_node
			{
				string name= reader.getString();
				int i= animations.size()-1;
				animations[i].nodes.push_back(AnimNode(name));
			}
			break;
		 case 25: // controllers
			reader.getString();
			reader.getInt();
			break;
		 case 21: // linear_pos
			reader.getString();
			reader.getInt();
			//fprintf(stderr,"linear_pos\n");
			break;
		 case 22: // tcb_pos
			fprintf(stderr,"tcb_pos %d %d\n",code,len);
			reader.getBytes(NULL,len);
			break;
		 case 23: // slerp_rot
			{
				reader.getString();
				int frame= reader.getInt();
				float x= reader.getFloat();
				float y= reader.getFloat();
				float z= reader.getFloat();
				float w= reader.getFloat();
				//fprintf(stderr,
				//  "%d %f %f %f %f\n",
				//  frame,x,y,z,w);
				int i= animations.size()-1;
				int j= animations[i].nodes.size()-1;
				if (frame<animations[i].nFrames ||
				  animations[i].nodes[j].quats.size()<
				  animations[i].nFrames)
					animations[i].nodes[j].quats[frame]=
					  osg::Quat(-x,-y,-z,w);
			}
			break;
		 case 24: // tcb_rot
			reader.getString();
			reader.getInt();
			//fprintf(stderr,"tcb_rot\n");
			break;
		 case 19: // linear_key
			{
				reader.getString();
				int frame= reader.getInt();
				float x= reader.getFloat();
				float y= reader.getFloat();
				float z= reader.getFloat();
				//fprintf(stderr,"%d %f %f %f\n",frame,x,y,z);
				int i= animations.size()-1;
				int j= animations[i].nodes.size()-1;
				if (frame<animations[i].nFrames ||
				  animations[i].nodes[j].positions.size()<
				  animations[i].nFrames)
				animations[i].nodes[j].positions[frame]=
				  osg::Vec3f(x,y,z);
			}
			break;
		 case 20: // tcb_key
			{
				reader.getString();
				int frame= reader.getInt();
				float x= reader.getFloat();
				float y= reader.getFloat();
				float z= reader.getFloat();
				float w= reader.getFloat();
				float tension= reader.getFloat();
				float continuity= reader.getFloat();
				float bias= reader.getFloat();
				float in= reader.getFloat();
				float out= reader.getFloat();
				//fprintf(stderr,
				//  "%d %f %f %f %f\n",
				//  frame,x,y,z,w);
				int i= animations.size()-1;
				int j= animations[i].nodes.size()-1;
				if (frame<animations[i].nFrames ||
				  animations[i].nodes[j].quats.size()<
				  animations[i].nFrames)
					animations[i].nodes[j].quats[frame]=
					  osg::Quat(-x,-y,-z,w);
			}
			break;
		 case 1: // comment
			fprintf(stderr,"%d %d\n",code,len);
			for (int i=0; i<len; i++) {
				Byte b;
				reader.getBytes(&b,1);
				fprintf(stderr," %d",0xff&b);
				if (i%16==15)
					fprintf(stderr,"\n");
			}
			fprintf(stderr,"\n");
			break;
		 default:
			fprintf(stderr,"%d %d\n",code,len);
			reader.getBytes(NULL,len);
			break;
		}
	}
	return 1;
}

//	makes OSG geometry for triangle list
void MSTSShape::makeGeometry(SubObject& subObject, TriList& triList,
  int& transparentBin, bool incTransparentBin)
{
	for (int i=0; i<subObject.vertices.size(); i++)
		subObject.vertices[i].index= -1;
	int nv= 0;
	int npt= 0;
	int nuv= 0;
	int nn= 0;
	for (int i=0; i<triList.vertexIndices.size(); i++) {
		int j= triList.vertexIndices[i];
		if (subObject.vertices[j].index < 0)
			subObject.vertices[j].index= nv++;
	}
	osg::Vec3Array* verts= new osg::Vec3Array;
	osg::Vec2Array* texCoords= new osg::Vec2Array;
	osg::Vec3Array* norms= new osg::Vec3Array;
	nv= 0;
	for (int i=0; i<subObject.vertices.size(); i++) {
		if (subObject.vertices[i].index < 0)
			continue;
		subObject.vertices[i].index= nv++;
		int j= subObject.vertices[i].pointIndex;
		if (j >= points.size())
			fprintf(stderr,"pointIndex too big\n");
		verts->push_back(osg::Vec3(
		  points[j].x,points[j].y,points[j].z));
		j= subObject.vertices[i].uvIndex;
		if (j<0 || j >= uvPoints.size())
//			fprintf(stderr,"uvIndex too big\n");
			texCoords->push_back(osg::Vec2(0,0));
		else
		  texCoords->push_back(osg::Vec2(uvPoints[j].u,uvPoints[j].v));
		j= subObject.vertices[i].normalIndex;
		if (j >= normals.size())
			fprintf(stderr,"normalIndex too big\n");
		norms->push_back(osg::Vec3(
		  normals[j].x,normals[j].y,normals[j].z));
	}
	osg::DrawElementsUShort* drawElements=
	  new osg::DrawElementsUShort(GL_TRIANGLES);;
	for (int i=0; i<triList.vertexIndices.size(); i++) {
		int j= triList.vertexIndices[i];
		drawElements->push_back(subObject.vertices[j].index);
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
	PrimState* ps= &primStates[triList.primStateIndex];
	osg::Material* mat= NULL;
//	fprintf(stderr,"vsi %d\n",ps->vStateIndex);
//	fprintf(stderr,"lmi %d\n",vtxStates[ps->vStateIndex].lightMaterialIndex);
	switch (vtxStates[ps->vStateIndex].lightMaterialIndex) {
#if 1
	 case -5://normal
		mat= new osg::Material;
		mat->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.6,.6,.6,1));
		mat->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.4,.4,.4,1));
		break;
#endif
	 case -6: //spec25
		mat= new osg::Material;
		mat->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.6,.6,.6,1));
		mat->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.3,.3,.3,1));
		mat->setSpecular(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.1,.1,.1,1));
//		mat->setShininess(osg::Material::FRONT_AND_BACK,1.);
		mat->setShininess(osg::Material::FRONT_AND_BACK,4.);
//		mat->setEmission(osg::Material::FRONT_AND_BACK,
//		  osg::Vec4(1,1,1,1));
		break;
	 case -7: //spec750
		mat= new osg::Material;
		mat->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.6,.6,.6,1));
		mat->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.3,.3,.3,1));
		mat->setSpecular(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.1,.1,.1,1));
		mat->setShininess(osg::Material::FRONT_AND_BACK,8.);
		break;
	 case -8: // full bright
		mat= new osg::Material;
		mat->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(1.,1.,1.,1));
		mat->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(0,0,0,1));
		break;
	 case -11: // half bright
		mat= new osg::Material;
		mat->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.75,.75,.75,1));
		mat->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(0,0,0,1));
		break;
	 case -12: // dark bright
		mat= new osg::Material;
		mat->setAmbient(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(.5,.5,.5,1));
		mat->setDiffuse(osg::Material::FRONT_AND_BACK,
		  osg::Vec4(0,0,0,1));
		break;
	 case -10: // emissive
		mat= new osg::Material;
		mat->setEmission(osg::Material::FRONT_AND_BACK,
		osg::Vec4(1,1,1,1));
		break;
	 default:
		break;
	}
	if (ps->texIdxs.size()>0) {
		int ti= ps->texIdxs[0];
		if (textures[ti].texture != NULL) {
			osg::StateSet* stateSet=
			  geometry->getOrCreateStateSet();
			stateSet->setTextureAttributeAndModes(0,
			  textures[ti].texture,osg::StateAttribute::ON);
			stateSet->setAttribute(new osg::FrontFace(
			  osg::FrontFace::CLOCKWISE));
			if (mat)
				stateSet->setAttributeAndModes(mat,
				  osg::StateAttribute::ON);
			stateSet->setMode(GL_LIGHTING,osg::StateAttribute::ON);
			if ((ps->alphaTestMode || shaders[ps->shaderIndex]==1)
			  && transparentBin!=0) {
//				stateSet->setRenderingHint(
//				  osg::StateSet::TRANSPARENT_BIN);
				int& tbin= transparentBin;
				if (incTransparentBin &&
				  ps->zBufMode==3 && ps->alphaTestMode==0)
					tbin++;
				else if (incTransparentBin && ps->zBufMode==1)
					tbin+= 2;
				stateSet->setRenderBinDetails(tbin,
				  "DepthSortedBin");
#if 0
				if (incTransparentBin)
					fprintf(stderr,"tbin %d %d %s\n",
					  tbin,ti,
					  images[textures[ti].imageIndex].c_str());
#endif
				osg::TexEnvFilter* lodBias=
				  new osg::TexEnvFilter(-3);
				stateSet->setTextureAttribute(0,lodBias);
				if (ps->alphaTestMode) {
					osg::AlphaFunc* af= new osg::AlphaFunc(
					  osg::AlphaFunc::GREATER,.6);
					stateSet->setAttributeAndModes(af,
					  osg::StateAttribute::ON);
				} else {
				osg::BlendFunc* bf= new osg::BlendFunc();
				bf->setFunction(osg::BlendFunc::SRC_ALPHA,
				  osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
				stateSet->setAttributeAndModes(bf,
				  osg::StateAttribute::ON);
				}
			} else {
//				stateSet->setRenderingHint(
//				  osg::StateSet::OPAQUE_BIN);
				stateSet->setRenderBinDetails(5,
				  "RenderBin");
			}
		}
	}
	int vsi= primStates[triList.primStateIndex].vStateIndex;
	int mi= vtxStates[vsi].matrixIndex;
#if 0
	if (incTransparentBin)
	fprintf(stderr,"psi=%d vsi=%d mi=%d ti=%d sonv=%d nv=%d tnv=%d si=%d atest=%d zbuf=%d\n",
	  triList.primStateIndex,vsi,mi,
	  primStates[triList.primStateIndex].texIdxs[0],
	  subObject.vertices.size(),nv,
	  triList.vertexIndices.size(),
	  ps->shaderIndex,ps->alphaTestMode,ps->zBufMode);
#endif
	if (matrices[mi].geode == NULL)
		matrices[mi].geode= new osg::Geode;
	matrices[mi].geode->addDrawable(geometry);
}

bool isOpaque(osg::Image* img, float minU, float maxU, float minV, float maxV)
{
	if (!img || img->getPixelFormat()!=GL_RGBA ||
	  img->getDataType()!=GL_UNSIGNED_BYTE)
		return false;
	while (minU < 0)
		minU+= 1;
	while (maxU < minU)
		maxU+= 1;
	while (minV < 0)
		minV+= 1;
	while (maxV < minV)
		maxV+= 1;
	int w= img->s();
	int h= img->t();
	int minS= (int)floor(minU*(w-1));
	int maxS= (int)ceil(maxU*(w-1));
	int minT= (int)floor(minV*(h-1));
	int maxT= (int)ceil(maxV*(h-1));
	if (minS < 0)
		minS= 0;
	if (maxS >= w)
		maxS= w-1;
	if (minT < 0)
		minT= 0;
	if (maxT >= h)
		maxT= h-1;
	unsigned char* data= img->data();
//	fprintf(stderr," %d %d %d %d %d %d\n",w,h,minS,maxS,minT,maxT);
	for (int i=minS; i<=maxS; i++) {
		for (int j=minT; j<=maxT; j++) {
			unsigned char a= data[4*(i+j*w)+3];
//			fprintf(stderr,"  %d %d %d\n",i,j,a);
			if (a < 200)
				return false;
		}
	}
	return true;
}

void MSTSShape::makeSplitGeometry(SubObject& subObject, TriList& triList,
  int transparentBin)
{
	PrimState* ps= &primStates[triList.primStateIndex];
	if (shaders[ps->shaderIndex]==0 || transparentBin<=10) {
		makeGeometry(subObject,triList,transparentBin);
		return;
	}
	float minX= 1e10;
	float minZ= 1e10;
	float maxX= -1e10;
	float maxZ= -1e10;
	for (int i=0; i<triList.vertexIndices.size(); i++) {
		int j= triList.vertexIndices[i];
		int k= subObject.vertices[j].pointIndex;
		if (k > points.size())
			continue;
		if (minX > points[k].x)
			minX= points[k].x;
		if (maxX < points[k].x)
			maxX= points[k].x;
		if (minZ > points[k].z)
			minZ= points[k].z;
		if (maxZ < points[k].z)
			maxZ= points[k].z;
	}
	osg::Image* img= NULL;
	if (ps->texIdxs.size() > 0) {
		int ti= ps->texIdxs[0];
		if (textures[ti].texture) {
			img= textures[ti].texture->getImage();
		}
	}
	TriList tl0(triList.primStateIndex);
	TriList tl1(triList.primStateIndex);
	TriList tl2(triList.primStateIndex);
	TriList tl3(triList.primStateIndex);
	TriList tl4(triList.primStateIndex);
	for (int i=0; i<triList.vertexIndices.size(); i+=3) {
		int cx= 0;
		int cz= 0;
		float minU= 1e10;
		float minV= 1e10;
		float maxU= -1e10;
		float maxV= -1e10;
		for (int j=0; j<3; j++) {
			int k= triList.vertexIndices[i+j];
			int k1= subObject.vertices[k].pointIndex;
			if (k1 >= points.size())
				continue;
			cx+= points[k1].x;
			cz+= points[k1].z;
			k1= subObject.vertices[k].uvIndex;
			if (k1 >= uvPoints.size())
				continue;
			if (minU > uvPoints[k1].u)
				minU= uvPoints[k1].u;
			if (maxU < uvPoints[k1].u)
				maxU= uvPoints[k1].u;
			if (minV > uvPoints[k1].v)
				minV= uvPoints[k1].v;
			if (maxV < uvPoints[k1].v)
				maxV= uvPoints[k1].v;
		}
		if (isOpaque(img,minU,maxU,minV,maxV)) {
			for (int j=0; j<3; j++) {
				int k= triList.vertexIndices[i+j];
				tl0.vertexIndices.push_back(k);
			}
			continue;
		}
		cx/= 3;
		cz/= 3;
		for (int j=0; j<3; j++) {
			int k= triList.vertexIndices[i+j];
			if (cx>0 && cz*maxX<cx*maxZ && cz*maxX<cx*minZ)
				tl1.vertexIndices.push_back(k);
			else if (cx<0 && cz*minX<cx*maxZ && cz*minX<cx*minZ)
				tl2.vertexIndices.push_back(k);
			else if (cz>0)
				tl3.vertexIndices.push_back(k);
			else
				tl4.vertexIndices.push_back(k);
		}
	}
//	int vsi= primStates[triList.primStateIndex].vStateIndex;
//	int mi= vtxStates[vsi].matrixIndex;
//	fprintf(stderr,"makesplit %s %d %d %d %d\n",
//	  matrices[mi].name.c_str(),
//	  tl1.vertexIndices.size(),tl2.vertexIndices.size(),
//	  tl3.vertexIndices.size(),tl4.vertexIndices.size());
	if (tl0.vertexIndices.size() > 0) {
		int tbin= 0;
		makeGeometry(subObject,tl0,tbin,false);
	}
	if (tl1.vertexIndices.size() > 0)
		makeGeometry(subObject,tl1,transparentBin);
	if (tl2.vertexIndices.size() > 0)
		makeGeometry(subObject,tl2,transparentBin);
	if (tl3.vertexIndices.size() > 0)
		makeGeometry(subObject,tl3,transparentBin);
	if (tl4.vertexIndices.size() > 0)
		makeGeometry(subObject,tl4,transparentBin);
}

void printTree(osg::Node* node, int depth)
{
	fprintf(stderr,"printTree %d %p %d\n",
	  depth,node,node->referenceCount());
	osg::Geode* geode= dynamic_cast<osg::Geode*>(node);
	if (geode) {
		fprintf(stderr,"geode\n");
		for (int i=0; i<geode->getNumDrawables(); i++) {
			osg::Geometry* geom=
			  dynamic_cast<osg::Geometry*>(geode->getDrawable(i));
			if (!geom)
				continue;
			osg::Vec3Array* pVerts=
			  (osg::Vec3Array*) geom->getVertexArray();
			float* v= (float*) pVerts->getDataPointer();
			float miny= 1e10;
			float maxy= -1e10;
			for (int j=0; j<pVerts->getNumElements(); j++) {
				int j3= j*3;
				if (miny > v[j3+1])
					miny= v[j3+1];
				if (maxy < v[j3+1])
					maxy= v[j3+1];
			}
			fprintf(stderr," drawable %d %f %f\n",i,miny,maxy);
		}
	} else {
		osg::MatrixTransform* mt=
		  dynamic_cast<osg::MatrixTransform*>(node);
		osg::Group* group=
		  dynamic_cast<osg::Group*>(node);
		if (mt) {
			osg::Vec3d trans= mt->getMatrix().getTrans();
			osg::Quat q= mt->getMatrix().getRotate();
			double a,x,y,z;
			q.getRotate(a,x,y,z);
			fprintf(stderr,"mt %d %lf %lf %lf %lf %lf %lf %lf\n",
//			  depth,trans[0],trans[1],trans[2],a,x,y,z);
			  depth,trans[0],trans[1],trans[2],q[0],q[1],q[2],q[3]);
			double roll= atan2(2*(q[0]*q[1]+q[2]*q[3]),
			  1-2*(q[1]*q[1]+q[2]*q[2]));
			double sinp= 2*(q[0]*q[2]-q[1]*q[3]);
			double pitch= sinp>=1 ? M_PI/2 : sinp<=-1 ? -M_PI/2 :
			  asin(sinp);
			double yaw= atan2(2*(q[0]*q[3]+q[1]*q[2]),
			  1-2*(q[2]*q[2]+q[3]*q[3]));
			fprintf(stderr," rpy %lf %lf %lf %lf %lf %lf\n",
			  roll,pitch,yaw,
			  180*roll/M_PI,180*pitch/M_PI,180*yaw/M_PI);
			for (int i=0; i<mt->getNumChildren(); i++)
				printTree(mt->getChild(i),depth+1);
		} else if (group) {
			fprintf(stderr,"group %d %d\n",depth,
			  group->getNumChildren());
			for (int i=0; i<group->getNumChildren(); i++)
				printTree(group->getChild(i),depth+1);
		} else {
			fprintf(stderr,"unknown %s\n",node->className());
		}
	}
}

//	reads all the ACE files needs for the shape
void MSTSShape::readACEFiles()
{
	for (int i=0; i<textures.size(); i++) {
		string filename= directory+images[textures[i].imageIndex];
		osg::Texture2D* t= readCacheACEFile(filename.c_str(),
		  directory2.size()==0);
		if (t==NULL && directory2.size()>0) {
			filename= directory2+images[textures[i].imageIndex];
			t= readCacheACEFile(filename.c_str());
		}
		textures[i].texture= t;
	}
}

//	creates an OSG subgraph for the shape file
osg::Node* MSTSShape::createModel(int transform, int transparentBin,
  bool saveNames, bool incTransparentBin)
{
//	makeLOD();
	readACEFiles();
	bool hasWheels= false;
	for (int j=0; j<matrices.size(); j++) {
		if (strncasecmp(matrices[j].name.c_str(),"WHEELS",6) == 0)
			hasWheels= true;
	}
	for (int i=0; i<distLevels.size(); i++) {
		DistLevel& dl= distLevels[i];
#if 0
		fprintf(stderr,"dl %d %f\n",i,dl.dist);
		for (int j=0; j<dl.hierarchy.size(); j++) {
			int parent= dl.hierarchy[j];
			fprintf(stderr,"h %d %s %d\n",
			  j,matrices[j].name.c_str(),dl.hierarchy[j]);
		}
#endif
	}
	for (int i=0; i<distLevels.size(); i++) {
		DistLevel& dl= distLevels[i];
#if 0
		fprintf(stderr,"dl %d %f\n",i,dl.dist);
		for (int j=0; j<dl.hierarchy.size(); j++) {
			int parent= dl.hierarchy[j];
			fprintf(stderr,"h %d %s %d\n",
			  j,matrices[j].name.c_str(),dl.hierarchy[j]);
			double* m= matrices[j].matrix.ptr();
			for (int k=12; k<16; k++)
				fprintf(stderr," %f",m[k]);
			fprintf(stderr,"\n");
		}
#endif
#if 0
		int n= 0;
		int m=  0;
		for (int j=0; j<dl.subObjects.size(); j++) {
			SubObject& so= dl.subObjects[j];
			for (int k=0; k<so.triLists.size(); k++) {
				TriList& tl= so.triLists[k];
//				fprintf(stderr,"trilist %d %d %d\n",
//				  j,k,tl.vertexIndices.size());
				m+= tl.vertexIndices.size();
				n++;
			}
		}
		fprintf(stderr,"dl %d %f %d %d\n",i,dl.dist,n,m);
#endif
		if (i > 0)
			continue;
		for (int j=0; j<matrices.size(); j++) {
			matrices[j].geode= NULL;
			matrices[j].transform= NULL;
		}
		for (int j=0; j<dl.subObjects.size(); j++) {
			SubObject& so= dl.subObjects[j];
			for (int k=0; k<so.triLists.size(); k++) {
				TriList& tl= so.triLists[k];
				if (transparentBin == 10 ||
				  (incTransparentBin && transparentBin>=10))
					makeGeometry(so,tl,transparentBin,
					  incTransparentBin);
				else
					makeSplitGeometry(so,tl,transparentBin);
			}
		}
#if 0
		for (int j=0; j<matrices.size(); j++) {
			double* m= matrices[j].matrix.ptr();
			fprintf(stderr,"matrix %d %s:\n",
			  j,matrices[j].name.c_str());
			for (int k=0; k<16; k++) {
				fprintf(stderr," %f",m[k]);
				if (k%4==3)
					fprintf(stderr,"\n");
			}
		}
#endif
		for (int j=0; j<matrices.size(); j++) {
			osg::MatrixTransform* mt= new osg::MatrixTransform;
			mt->setMatrix(matrices[j].matrix);
			if (matrices[j].geode != NULL) {
				float r1=
				  matrices[j].geode->getBound().radius();
				float r2= drawableRadius(matrices[j].geode);
#if 0
//				if (m > 1000)
//				fprintf(stderr," %d %s %f %f\n",
//				  j,matrices[j].name.c_str(),r1,r2);
				osg::LOD* lod= new osg::LOD();
				lod->addChild(matrices[j].geode,0,600*r2);
				mt->addChild(lod);
#else
				mt->addChild(matrices[j].geode);
#endif
				if (saveNames)
					matrices[j].geode->setName(
					  matrices[j].name);
			}
			matrices[j].transform= mt;
		}
	}
	int nSigHead= 0;
	if (signalLightOffset) {
		for (int k=0; k<matrices.size(); k++) {
			Matrix& m= matrices[k];
			if (strncasecmp(m.name.c_str(),"head",4) == 0)
				nSigHead++;
		}
	}
	for (int i=0; i<animations.size(); i++) {
		Animation& a= animations[i];
		if (a.nFrames < 2)
			continue;
#if 0
		fprintf(stderr,"animnodes %d %d\n",
		  a.nodes.size(),matrices.size());
		for (int k=0; k<matrices.size(); k++) {
			Matrix& m= matrices[k];
			fprintf(stderr,"mat %s %s\n",m.name.c_str(),
			  a.nodes[k].name.c_str());
		}
#endif
		for (int j=0; j<a.nodes.size(); j++) {
			AnimNode& n= a.nodes[j];
			if (n.positions.size()<2 && n.quats.size()<2)
				continue;
#if 0
			fprintf(stderr,"anim %s %d %d\n",
			  n.name.c_str(),a.nFrames,a.frameRate);
			for (map<int,osg::Vec3f>::iterator
			  k=n.positions.begin(); k!=n.positions.end(); ++k)
				fprintf(stderr," pos %d %f %f %f\n",k->first,
				  k->second[0],k->second[1],k->second[2]);
			for (map<int,osg::Quat>::iterator k=n.quats.begin();
			  k!=n.quats.end(); ++k)
				fprintf(stderr," quat %d %f %f %f %f\n",
				  k->first,k->second[0],k->second[1],
				  k->second[2],k->second[3]);
#endif
//			if (strncasecmp(n.name.c_str(),"WHEELS",6) == 0)
//				continue;
			osg::MatrixTransform* mt= NULL;
			if (j < matrices.size()) {
				Matrix& m= matrices[j];
				if (m.transform!=NULL && m.name==n.name)
					mt= m.transform;
			}
			if (mt == NULL) {
				for (int k=0; k<matrices.size(); k++) {
					Matrix& m= matrices[k];
					if (m.transform==NULL || m.name!=n.name)
						continue;
					mt= m.transform;
					break;
				}
			}
			if (mt==NULL && j < matrices.size()) {
				Matrix& m= matrices[j];
//				fprintf(stderr," anim j %s %s\n",
//				  m.name.c_str(),n.name.c_str());
				if (m.transform!=NULL)
					mt= m.transform;
			}
			if (mt == NULL)
				continue;
			osg::AnimationPath* ap= new osg::AnimationPath;
			ap->setLoopMode(osg::AnimationPath::NO_LOOPING);
			if (n.quats.size() < n.positions.size()) {
				osg::Quat q= mt->getMatrix().getRotate();
				for (map<int,osg::Vec3f>::iterator
				  k=n.positions.begin(); k!=n.positions.end();
				  ++k)
					ap->insert(k->first,osg::AnimationPath::
					  ControlPoint(k->second,q));
			} else if(n.positions.size() < n.quats.size()) {
				osg::Vec3d p= mt->getMatrix().getTrans();
				for (map<int,osg::Quat>::iterator
				  k=n.quats.begin(); k!=n.quats.end(); ++k)
					ap->insert(k->first,osg::AnimationPath::
					  ControlPoint(p,k->second));
			} else {
				for (map<int,osg::Quat>::iterator
				  k=n.quats.begin(); k!=n.quats.end(); ++k) {
					map<int,osg::Vec3f>::iterator k1=
					  n.positions.find(k->first);
					if (k1 != n.positions.end())
						ap->insert(k->first,osg::
						  AnimationPath::ControlPoint(
						  k1->second,k->second));
				}
			}
			if (strncasecmp(n.name.c_str(),
			  "Pantograph",10) == 0) {
				int len= n.name.length();
				TwoStateAnimPathCB* apc=
				  new TwoStateAnimPathCB(NULL,ap,atoi(
				   n.name.substr(len>16?16:(len>13?13:10)).
				   c_str()));
				mt->setUpdateCallback(apc);
			} else if (strncasecmp(n.name.c_str(),"Door",4)==0 ||
			  strncasecmp(n.name.c_str(),"Mirror",6)==0) {
				;//ignore animation
			} else if (hasWheels && (a.nFrames==16 ||
			  strncasecmp(n.name.c_str(),"WHEELS",6) == 0 ||
			  strncasecmp(n.name.c_str(),"ROD",3) == 0)) {
				RodAnimPathCB* apc=
				  new RodAnimPathCB(NULL,ap,a.nFrames);
				mt->setUpdateCallback(apc);
			} else if (strncasecmp(n.name.c_str(),"coupler",
			  7) == 0) {
				CouplerAnimPathCB* apc=
				  new CouplerAnimPathCB(NULL,ap,a.frameRate>0);
				mt->setUpdateCallback(apc);
			} else if (strncasecmp(n.name.c_str(),"head",4) == 0) {
				int unit= n.name[4]-'1';
//				fprintf(stderr,"signalanim %s %d\n",
//				  n.name.c_str(),n);
				SignalAnimPathCB* apc=
				  new SignalAnimPathCB(NULL,ap,unit);
				mt->setUpdateCallback(apc);
			} else {
//				fprintf(stderr,"twostateanim %s\n",
//				  n.name.c_str());
				TwoStateAnimPathCB* apc=
				  new TwoStateAnimPathCB(NULL,ap,-1);
				mt->setUpdateCallback(apc);
			}
		}
	}
	osg::MatrixTransform* top= NULL;
	for (int i=0; i<distLevels.size(); i++) {
		DistLevel& dl= distLevels[i];
		for (int j=0; j<dl.hierarchy.size(); j++) {
			int parent= dl.hierarchy[j];
			osg::MatrixTransform* mt= matrices[j].transform;
			if (mt == NULL)
				continue;
			if (parent < 0)
				top= mt;
			else if (matrices[j].part < 0 ||
			  mt->getUpdateCallback())
				matrices[parent].transform->addChild(mt);
#if 0
			osg::Vec3d trans=
			  matrices[j].transform->getMatrix().getTrans();
			fprintf(stderr,"h %d %s %d %lf %lf %lf %d %p\n",
			  j,matrices[j].name.c_str(),dl.hierarchy[j],
			  trans[0],trans[1],trans[2],
			  matrices[j].part,
			  mt->getUpdateCallback());
#endif
		}
	}
	if (top == NULL) {
		fprintf(stderr,"no top for model %d %d\n",matrices.size(),
		  distLevels.size());
		for (int i=0; i<distLevels.size(); i++) {
			DistLevel& dl= distLevels[i];
			if (i > 0)
				continue;
			for (int j=0; j<dl.hierarchy.size(); j++) {
				osg::Vec3d trans=
				  matrices[j].transform->getMatrix().getTrans();
				fprintf(stderr,"h %d %s %d %lf %lf %lf\n",
				  j,matrices[j].name.c_str(),dl.hierarchy[j],
				  trans[0],trans[1],trans[2]);
			}
		}
		return NULL;
	}
	if (signalLightOffset) {
		osgSim::LightPointNode* node= new osgSim::LightPointNode;
		node->setUpdateCallback(new SignalLightUCB(NULL));
		for (int i=0; i<nSigHead; i++) {
			osgSim::LightPoint lp;
			node->addLightPoint(lp);
		}
		for (int k=0; k<matrices.size(); k++) {
			Matrix& m= matrices[k];
			if (strncasecmp(m.name.c_str(),"head",4) == 0) {
				int unit= m.name[4]-'1';
				int state= 1;
				if (unit!=0 && nSigHead==3)
					state= 0;
				osgSim::LightPoint& lp=
				  node->getLightPoint(unit);
				lp._on= true;
				osg::Vec3d trans=
				  m.transform->getMatrix().getTrans();
//				fprintf(stderr,"unit %d %f %f %f\n",
//				  unit,trans[0],trans[1],trans[2]);
				lp._position= trans+*signalLightOffset;
				if (state)
					lp._color= osg::Vec4d(0,1,0,1);
				else
					lp._color= osg::Vec4d(1,0,0,1);
				lp._radius= fabs((*signalLightOffset)[2]);
			}
		}
		top->addChild(node);
	}
	if (transform) {
		osg::MatrixTransform* mt= new osg::MatrixTransform;
		mt->setMatrix(osg::Matrix(0,-1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1));
		mt->addChild(top);
		top= mt;
	}
//	printTree(top,0);
#if 0
	if (distLevels.size() > 0) {
		DistLevel& dl= distLevels[0];
		osg::LOD* lod= new osg::LOD();
		lod->addChild(top,0,dl.dist);
		top= lod;
	}
#endif
	return top;
}

//	devides the shape file parts into railcar parts
void MSTSShape::createRailCar(RailCarDef* car, bool saveNames)
{
	car->parts.clear();
	map<int,int> partMap;
	for (int j=0; j<matrices.size(); j++) {
		if (strncasecmp(matrices[j].name.c_str(),"WHEELS",6) == 0) {
			int id= atoi(matrices[j].name.c_str()+6);
			if (partMap.find(id) == partMap.end())
				partMap[id]= j;
			else
				fprintf(stderr,"duplicate wheel %s %s\n",
				  car->name.c_str(),matrices[j].name.c_str());
		}
	}
	for (map<int,int>::iterator i=partMap.begin(); i!=partMap.end(); ++i) {
		matrices[i->second].part= car->parts.size();
		car->parts.push_back(RailCarPart(-1,0,0));
	}
	car->axles= partMap.size();
//	fprintf(stderr,"axles=%d\n",car->axles);
	partMap.clear();
	for (int j=0; j<matrices.size(); j++)
		if (matrices[j].name.size()==6 &&
		  strncasecmp(matrices[j].name.c_str(),"BOGIE",5) == 0)
			partMap[atoi(matrices[j].name.c_str()+5)]= j;
	if (partMap.size() < 2)
		partMap.clear();
	for (map<int,int>::iterator i=partMap.begin(); i!=partMap.end(); ++i) {
		matrices[i->second].part= car->parts.size();
		car->parts.push_back(RailCarPart(-1,0,0));
	}
	for (int i=0; i<matrices.size(); i++) {
		if (strncasecmp(matrices[i].name.c_str(),"COUPLER",7) == 0) {
			const double* m= matrices[i].matrix.ptr();
//			fprintf(stderr,"%s %f %f %f\n",matrices[i].name.c_str(),
//			  m[12],m[13],m[14]);
			float sign= m[14]<0 ? -1 : 1;
		 	int j= animations.size();
		 	animations.push_back(Animation(2,m[14]));
			int k= animations[j].nodes.size();
			animations[j].nodes.push_back(
			  AnimNode(matrices[i].name.c_str()));
			animations[j].nodes[k].positions[0]=
			  osg::Vec3f(m[12],m[13],m[14]);
			animations[j].nodes[k].positions[1]=
			  osg::Vec3f(m[12],m[13],
			  m[14]+.5*(car->maxSlack-car->couplerGap)*sign);
		}
	}
	DistLevel& dl= distLevels[0];
	for (int j=0; j<dl.hierarchy.size(); j++) {
		int parent= dl.hierarchy[j];
		if (parent < 0)
			continue;
		if (matrices[j].part>=0 && matrices[parent].part<0) {
			const double* m1= matrices[parent].matrix.ptr();
			const double* m2= matrices[j].matrix.ptr();
			int p= matrices[j].part;
			car->parts[p].xoffset= m1[14]+m2[14];
			car->parts[p].zoffset= m1[13]+m2[13];//-.05;
			car->parts[p].parent= car->parts.size();
//			fprintf(stderr,"p1 %d %s %d %f %f\n",
//			  j,matrices[j].name.c_str(),p,
//			  car->parts[p].xoffset,car->parts[p].zoffset);
		}
	}
	for (int j=0; j<dl.hierarchy.size(); j++) {
		int parent= dl.hierarchy[j];
		if (parent < 0)
			continue;
		if (matrices[j].part>=0 && matrices[parent].part>=0) {
			const double* m= matrices[j].matrix.ptr();
			int pp= matrices[parent].part;
			int p= matrices[j].part;
			car->parts[p].xoffset= m[14]+car->parts[pp].xoffset;
			car->parts[p].zoffset= m[13]+car->parts[pp].zoffset;
			car->parts[p].parent= pp;
//			fprintf(stderr,"p2 %d %s %d %d %f %f\n",
//			  j,matrices[j].name.c_str(),p,pp,
//			  car->parts[p].xoffset,car->parts[p].zoffset);
		}
	}
	int i= car->parts.size();
	car->parts.push_back(RailCarPart(-1,0,0));
	car->parts[i].model= createModel(1,11,saveNames,true);
	if (car->parts[i].model)
		car->parts[i].model->ref();
	if (car->headlights.size() > 0) {
		osgSim::LightPointNode* node= new osgSim::LightPointNode;
		for (list<HeadLight>::iterator j=car->headlights.begin();
		  j!=car->headlights.end(); j++) {
			osgSim::LightPoint lp;
			lp._on= false;
			lp._position= osg::Vec3d(j->x,j->y,
			  j->z+(j->z>0?.005:-.005));
			lp._color= osg::Vec4d((j->color>>16)&0xff,
			  (j->color>>8)&0xff,j->color&0xff,
			  (j->color>>24)&0xff);
			lp._radius= j->radius>.5?.1*j->radius:.2*j->radius;
			if (j->unit == 2)
				lp._radius= .15;
			else
				lp._radius= .06;
			node->addLightPoint(lp);
//			fprintf(stderr,"headlight %f %f %f %f %d %x\n",
//			  j->x,j->y,j->z,j->radius,j->unit,j->color);
		}
		osg::MatrixTransform* mt=
		  (osg::MatrixTransform*)car->parts[i].model;
		if (mt != NULL)
			mt->addChild(node);
	}
	for (int j=0; j<matrices.size(); j++) {
#if 0
		if (matrices[j].geode) {
			osg::BoundingBox bb=
			  matrices[j].geode->getBoundingBox();
			fprintf(stderr," %d %s %f %f %f %f\n",
			  j,matrices[j].name.c_str(),
			  matrices[j].geode->getBound().radius(),
			   bb.xMax()-bb.xMin(),
			   bb.yMax()-bb.yMin(),
			   bb.zMax()-bb.zMin());
		}
#endif
		int p= matrices[j].part;
		if (p<0 || matrices[j].transform->getUpdateCallback())
			continue;
		osg::Matrixd m= matrices[j].transform->getMatrix();
		double* mp= m.ptr();
//		fprintf(stderr,"%d %d %lf %lf\n",j,p,mp[13],mp[14]);
		mp[13]= 0;
		mp[14]= 0;
		matrices[j].transform->setMatrix(m);
		osg::MatrixTransform* mt= new osg::MatrixTransform;
		mt->setMatrix(osg::Matrix(0,-1,0,0, 0,0,1,0, 1,0,0,0, 0,0,0,1));
		mt->addChild(matrices[j].transform);
		car->parts[p].model= mt;
	}
	for (int i=0; i<car->parts.size(); i++)
		if (car->parts[i].model)
			car->parts[i].model->ref();
#if 0
	for (int i=0; i<car->parts.size(); i++) {
		fprintf(stderr,"part %d %d %f %f %p\n",i,car->parts[i].parent,
		  car->parts[i].xoffset,car->parts[i].zoffset,
		  car->parts[i].model);
	}
#endif
}

void MSTSShape::printSubobjects()
{
	for (int i=0; i<distLevels.size(); i++) {
		DistLevel& dl= distLevels[i];
		printf("dl %d %f\n",i,dl.dist);
		for (int j=0; j<dl.subObjects.size(); j++) {
			SubObject& so= dl.subObjects[j];
			for (int k=0; k<so.triLists.size(); k++) {
				TriList& tl= so.triLists[k];
				PrimState* ps= &primStates[tl.primStateIndex];
				int vsi=
				  primStates[tl.primStateIndex].vStateIndex;
				int mi= vtxStates[vsi].matrixIndex;
				printf(" %d.%d.%d %s %d",i,j,k,
				  matrices[mi].name.c_str(),
				  tl.vertexIndices.size());
				if (ps->texIdxs.size()>0) {
					int ti= ps->texIdxs[0];
					printf(" %s %d",images[textures[ti].
					  imageIndex].c_str(),
					  shaders[ps->shaderIndex]);
				}
				printf("\n");
			}
		}
	}
}

void MSTSShape::fixTop()
{
	if (matrices.size() == 1) {
		double* m= matrices[0].matrix.ptr();
		if (m[12]!=0 || m[13]!=0 || m[14]!=0) {
//			fprintf(stderr,"nonzero top %f %f %f\n",
//			  m[12],m[13],m[14]);
			m[12]= 0;
			m[13]= 0;
			m[14]= 0;
		}
	}
}

void MSTSShape::makeLOD()
{
	DistLevel& dl= distLevels[0];
#if 0
		fprintf(stderr,"dl %d %f\n",i,dl.dist);
		for (int j=0; j<dl.hierarchy.size(); j++) {
			int parent= dl.hierarchy[j];
			fprintf(stderr,"h %d %s %d\n",
			  j,matrices[j].name.c_str(),dl.hierarchy[j]);
		}
#endif
	float dist1= 600;
	multimap<float,int> distMap;
	int n= 0;
	int m= 0;
	for (int j=0; j<dl.subObjects.size(); j++) {
		SubObject& so= dl.subObjects[j];
		for (int k=0; k<so.triLists.size(); k++) {
			TriList& tl= so.triLists[k];
//			fprintf(stderr,"trilist %d %d %d\n",
//			  j,k,tl.vertexIndices.size());
			m+= tl.vertexIndices.size();
			n++;
			UFSets pGroups(points.size());
			set<int> pointSet;
			for (int i=0; i<tl.vertexIndices.size(); i+=3) {
				int vi1= tl.vertexIndices[i];
				int vi2= vi1+1;
				int vi3= vi1+2;
				int pi1= so.vertices[vi1].pointIndex;
				int pi2= so.vertices[vi2].pointIndex;
				int pi3= so.vertices[vi3].pointIndex;
				int g1= pGroups.find(pi1);
				int g2= pGroups.find(pi2);
				if (g1 != g2)
					g1= pGroups.link(g1,g2);
				int g3= pGroups.find(pi3);
				if (g1 != g3)
					g1= pGroups.link(g1,g3);
				pointSet.insert(pi1);
				pointSet.insert(pi2);
				pointSet.insert(pi3);
			}
			int ng= 0;
			for (set<int>::iterator i=pointSet.begin();
			  i!=pointSet.end(); i++) {
				int pi= *i;
				if (pi != pGroups.find(pi))
					continue;
				ng++;
				ShadowCalc sc;
				for (int i1=0; i1<tl.vertexIndices.size();
				  i1+=3) {
					int vi1= tl.vertexIndices[i1];
					int vi2= vi1+1;
					int vi3= vi1+2;
					int pi1= so.vertices[vi1].pointIndex;
					int pi2= so.vertices[vi2].pointIndex;
					int pi3= so.vertices[vi3].pointIndex;
					int g1= pGroups.find(pi1);
					if (g1 != pi)
						continue;
					sc.addTri(osg::Vec3(points[pi1].x,
					  points[pi1].y,points[pi1].z),
					  osg::Vec3(points[pi2].x,
					  points[pi2].y,points[pi2].z),
					  osg::Vec3(points[pi3].x,
					  points[pi3].y,points[pi3].z));
				}
				float r= sqrt(sc.getArea()/M_PI);
				float d= r*dist1;
				fprintf(stderr,"  %d %d %f %f %f\n",pi,sc.nTri,
				  sc.getArea(),r,d);
				distMap.insert(make_pair(d,sc.nTri));
			}
			int vsi= primStates[tl.primStateIndex].vStateIndex;
			int mi= vtxStates[vsi].matrixIndex;
			fprintf(stderr," %d %d %d %d %s\n",
			  j,k,ng,pointSet.size(),matrices[mi].name.c_str());
		}
	}
	fprintf(stderr,"dl %f %d %d\n",dl.dist,n,m);
	int s= 0;
	for (multimap<float,int>::iterator i=distMap.begin(); i!=distMap.end();
	  i++) {
		s+= i->second;
		fprintf(stderr," %f %d %d\n",i->first,i->second,s);
	}
}

#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/fstream>

class MSTSSReaderWriter : public osgDB::ReaderWriter
{
  public:
	MSTSSReaderWriter()
	{
		supportsExtension("s","MSTS Shape");
	}
	
	virtual const char* className() const { return "MSTS Shape Reader"; }

	virtual ReadResult readNode(const std::string& file,
	  const Options* options) const
	{
		std::string ext = osgDB::getFileExtension(file);
		if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

		std::string fileName = osgDB::findDataFile( file, options );
		osg::notify(osg::INFO) << "osgDB MSTS Shape reader: starting reading \"" << fileName << "\"" << std::endl;
	
//		fprintf(stderr,"loading %s\n",fileName.c_str());
		MSTSShape shape;
		shape.readFile(fileName.c_str(),NULL,NULL);	
		ReadResult result(shape.createModel(1,10,false));
		return result;
	}
	virtual ReadResult readNode(std::istream& fin,
	  const Options* options) const
	{
		return ReadResult::FILE_NOT_HANDLED;
	}
	virtual WriteResult writeNode(const osg::Node& node,
	  const std::string& fileName, const Options* /*options*/) const
	{
		return WriteResult::FILE_NOT_HANDLED;
	}
	
	virtual WriteResult writeNode(const osg::Node& node,
	  std::ostream& fout, const Options* opts) const
	{
		return WriteResult::FILE_NOT_HANDLED;
	}
};

REGISTER_OSGPLUGIN(s, MSTSSReaderWriter)
