//	code to read MSTS ACE files into OSG Images
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

#include <osgDB/ReadFile>

#include "mstsbfile.h"

osg::Image* readMSTSACE(const char* path)
{
	MSTSBFile reader;
	if (reader.open(path)) {
//		fprintf(stderr,"cannot read ace %s\n",path);
		return NULL;
	}
	reader.getInt();
	int flags= reader.getInt();
	int wid= reader.getInt();
	int ht= reader.getInt();
	int code= reader.getInt();
	int colors= reader.getInt();
	reader.seek(168+16*colors);
	int offset= reader.getInt();
//	fprintf(stderr,"ace %s %x %d %d %d %d\n",
//	  path,flags,wid,ht,colors,offset);
	int i= 2;
//	if ((flags&01)) {
		for (; i<12; i++)
			if (wid == (1<<i))
				break;
//	}
	if (wid<4|| wid>4096 || ht<4 || ht>4096 || wid!=ht || i>=12) {
		fprintf(stderr,"bad ace %s %d %d %d %d %d\n",
		  path,flags,wid,ht,colors,offset);
		return NULL;
	}
	int sz= (flags&020)!=0 ? wid*ht/2 : wid*ht*(colors==3?3:4);
	int size= sz;
	if ((flags&01) != 0) {
		int s= sz;
		int w= wid;
		int h= ht;
		for (;;) {
			w/= 2;
			h/= 2;
			if (w<4 || h<4)
				break;
			s/= 4;
			size+= s;
		}
	}
	reader.seek(offset+16);
//	fprintf(stderr,"size=%d %d\n",size,wid);
	GLubyte* data= (GLubyte*) malloc(size);
	if (data == NULL)
		return NULL;
	GLubyte* row= (GLubyte*) malloc(wid*5);
	if (row == NULL) {
		free(data);
		return NULL;
	}
	offset= 0;
	int w= wid;
	int h= ht;
	osg::Image::MipmapDataType mipmapData;
	for (;;) {
		if ((flags&020) != 0) {
			int len= reader.getInt();
//			fprintf(stderr,"len=%d %d %d %d\n",len,offset,w,h);
			reader.getBytes(data+offset,len);
		} else {
			int rsz= 3*w;
			if (colors > 3)
				rsz+= w<8 ? 1 : w/8;
			if (colors > 4)
				rsz+= w;
			GLubyte* dp= data+offset;
			for (int j=0; j<h; j++) {
				reader.getBytes(row,rsz);
				GLubyte* rp= row;
				GLubyte* gp= rp+w;
				GLubyte* bp= gp+w;
				GLubyte* tp= bp+w;
				GLubyte* ap= tp+w/8;
				for (int k=0; k<w; k++) {
					*dp++= *rp++;
					*dp++= *gp++;
					*dp++= *bp++;
					if (colors == 4) {
						int j= k%8;
						//*dp++= (*tp&(1<<j))==0?0:255;
						*dp++=
						  (*tp&(1<<(7-j)))==0?0:255;
						if (j == 7)
							tp++;
					} else if (colors == 5) {
						*dp++= *ap++;
					}
				}
			}
		}
		if ((flags&01)==0 || w<=4 || h<=4)
			break;
		w/= 2;
		h/= 2;
		offset+= sz;
		sz/= 4;
		mipmapData.push_back(offset);
	}
	free(row);
	if (offset > size)
		fprintf(stderr,"bad mipmap %d %d\n ace %s %d %d %d %d %d\n",
		  offset,size,
		  path,flags,wid,ht,colors,offset);
	osg::Image* image= new osg::Image;
	int format= colors>3 ? GL_RGBA : GL_RGB;
	if ((flags&020)!=0)
		format= colors>3 ? GL_COMPRESSED_RGBA_S3TC_DXT1_EXT :
		  GL_COMPRESSED_RGB_S3TC_DXT1_EXT;
	image->setImage(wid,ht,1,format,format,GL_UNSIGNED_BYTE,data,
	  osg::Image::USE_MALLOC_FREE,0);
	image->setMipmapLevels(mipmapData);
	return image;
}

typedef map<string,osg::Texture2D*> ACEMap;
static ACEMap aceMap;

void cleanACECache()
{
	for (ACEMap::iterator i=aceMap.begin(); i!=aceMap.end(); i++) {
		if (i->second == NULL)
			continue;
		if (i->second->referenceCount() <= 1) {
//			fprintf(stderr,"unused %s %d\n",
//			  i->first.c_str(),i->second->referenceCount());
			i->second->unref();
			i->second= NULL;
		}
	}
}

//	reads an ACE file and saves image for future calls
osg::Texture2D* readCacheACEFile(const char* path, bool tryPNG)
{
	ACEMap::iterator i= aceMap.find(path);
	if (i != aceMap.end() && i->second)
		return i->second;
	osg::Image* image= NULL;
	if (strstr(path,".ace") || strstr(path,".ACE")) {
		image= readMSTSACE(path);
		if (image == NULL) {
			string ddsPath(path);
			ddsPath= ddsPath.substr(0,ddsPath.size()-4)+".dds";
//			fprintf(stderr,"trying %s\n",ddsPath.c_str());
			image= osgDB::readImageFile(ddsPath.c_str());
		}
		if (image==NULL && tryPNG) {
			string pngPath(path);
			pngPath= pngPath.substr(0,pngPath.size()-4)+".png";
//			fprintf(stderr,"trying %s\n",pngPath.c_str());
			image= osgDB::readImageFile(pngPath.c_str());
			if (!image)
				fprintf(stderr,"tried %s\n",pngPath.c_str());
		}
	} else {
		image= osgDB::readImageFile(path);
	}
	if (image == NULL)
		return NULL;
	osg::Texture2D* t= new osg::Texture2D;
	t->setDataVariance(osg::Object::DYNAMIC);
	t->setImage(image);
	t->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::REPEAT);
	t->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::REPEAT);
	t->setMaxAnisotropy(16);
	t->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
	if (image->getNumMipmapLevels() > 2) {
		t->setFilter(osg::Texture2D::MIN_FILTER,
		  osg::Texture2D::LINEAR_MIPMAP_LINEAR);
		t->setMaxLOD(image->getNumMipmapLevels()-2);
	} else {
		t->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
	}
	aceMap[path]= t;
	t->ref();
	return t;
}

static void writeInt(FILE* out, int n)
{
	fwrite(&n,4,1,out);
}

static float mipmapColor(const osg::Image& image, int color, int i, int j,
  int samples)
{
	int n= 0;
	float sum= 0;
	i*= samples;
	j*= samples;
	for (int i1=0; i1<samples; i1++) {
		for (int j1=0; j1<samples; j1++) {
			osg::Vec4f c= image.getColor(i+i1,j+j1);
			sum+= c[color];
			n++;
		}
	}
	return sum/n;
}

bool writeMSTSACE(const osg::Image& image, const std::string& fileName)
{
	bool hasAlpha= image.getPixelFormat()!=GL_RGB;
//	fprintf(stderr,"%d %d %d\n",hasAlpha,image.getPixelFormat(),GL_RGB);
	int levels= 1;
	if (image.s() == image.t()) {
		int i= 0;
		for (; i<12; i++)
			if (image.s() == (1<<i))
				break;
		if (i < 12)
			levels= i+1;
	}
//	levels= 1;
//	fprintf(stderr,"%d %d %d %d\n",hasAlpha,levels,image.s(),image.t());
	FILE* out= fopen(fileName.c_str(),"w");
	if (out == NULL) {
		fprintf(stderr,"cannot write %s\n",fileName.c_str());
		return false;
	}
	fprintf(out,"SIMISA@@@@@@@@@@");
	writeInt(out,1);
	writeInt(out,levels>1?1:0);//flags
	writeInt(out,image.s());
	writeInt(out,image.t());
	writeInt(out,0xe);
	writeInt(out,hasAlpha?5:3);//colors
	for (int i=0; i<128; i++)
		fputc(0,out);
	writeInt(out,8); writeInt(out,0); writeInt(out,3); writeInt(out,0);
	writeInt(out,8); writeInt(out,0); writeInt(out,4); writeInt(out,0);
	writeInt(out,8); writeInt(out,0); writeInt(out,5); writeInt(out,0);
	if (hasAlpha) {
	  writeInt(out,1); writeInt(out,0); writeInt(out,2); writeInt(out,0);
	  writeInt(out,8); writeInt(out,0); writeInt(out,6); writeInt(out,0);
	}
	int offset= ftell(out)-16;
	int h= image.t();
	for (int level=1; level<=levels; level++) {
		offset+= h*4;
		h/= 2;
	}
//	fprintf(stderr,"offset0 %d\n",offset);
	int w= image.s();
	h= image.t();
	for (int level=1; level<=levels; level++) {
		for (int i=0; i<h; i++) {
			writeInt(out,offset);
			offset+= 3*w;
			if (hasAlpha) {
				offset+= w;
				if (w > 8)
					offset+= w/8;
				else
					offset++;
			}
		}
		w/= 2;
		h/= 2;
	}
//	fprintf(stderr,"ftell %d %d %d\n",ftell(out),w,h);
	w= image.s();
	h= image.t();
	int samples= 1;
	for (int level=1; level<=levels; level++) {
//		fprintf(stderr,"level %d %d %d %d\n",level,w,h,samples);
		for (int i=0; i<h; i++) {
			for (int k=0; k<(hasAlpha?4:3); k++) {
				if (k == 3) {
					for (int j=0; j<w; j+=8)
						fputc(0xff,out);
				}
				for (int j=0; j<w; j++) {
					float c=
					  mipmapColor(image,k,j,i,samples);
					fputc((int)(c*255),out);
				}
			}
		}
		w/= 2;
		h/= 2;
		samples*= 2;
	}
	fclose(out);
	return true;
}

#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/fstream>

class MSTSACEReaderWriter : public osgDB::ReaderWriter
{
  public:
	MSTSACEReaderWriter()
	{
		supportsExtension("ace","MSTS ACE");
	}
	
	virtual const char* className() const { return "MSTS ACE Reader"; }

	virtual ReadResult readImage(const std::string& file,
	  const Options* options) const
	{
		std::string ext = osgDB::getFileExtension(file);
		if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;

		std::string fileName = osgDB::findDataFile( file, options );
		osg::notify(osg::INFO) << "osgDB MSTS Shape reader: starting reading \"" << fileName << "\"" << std::endl;
	
		osg::Image* img= readMSTSACE(fileName.c_str());
		ReadResult result(img);
		return result;
	}
	virtual ReadResult readImage(std::istream& fin,
	  const Options* options) const
	{
		return ReadResult::FILE_NOT_HANDLED;
	}
	virtual WriteResult writeImage(const osg::Image& image,
	  const std::string& fileName, const Options* /*options*/) const
	{
		if (writeMSTSACE(image,fileName))
			return WriteResult::FILE_SAVED;
		else
			return WriteResult::FILE_NOT_HANDLED;
	}
	
	virtual WriteResult writeImage(const osg::Image& image,
	  std::ostream& fout, const Options* opts) const
	{
		return WriteResult::FILE_NOT_HANDLED;
	}
};

REGISTER_OSGPLUGIN(ace, MSTSACEReaderWriter)
