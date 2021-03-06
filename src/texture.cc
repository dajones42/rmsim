//	old 2D texture code
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

TextureMap textureMap;

static int getByte(FILE* in)
{
	return fgetc(in);
}

static int getLEShort(FILE* in)
{
	int b1= getByte(in);
	int b2= getByte(in);
	return b2*256+b1;
}

#if 0
int Texture::load()
{
	return;
	GLuint id;
	glGenTextures(1,&id);
	glBindTexture(GL_TEXTURE_2D,id);
	FILE* in= fopen(filename.c_str(),"rb");
	if (in == NULL) {
		fprintf(stderr,"cannot read texture %s\n",filename.c_str());
		return id;
	}
	int imageIDSize= getByte(in);
	int colorMapType= getByte(in);
	int imageType= getByte(in);
	int colorMapOrigin= getLEShort(in);
	int colorMapSize= getLEShort(in);
	int colorMapEntrySize= getByte(in);
	int xStart= getLEShort(in);
	int yStart= getLEShort(in);
	int xSize= getLEShort(in);
	int ySize= getLEShort(in);
	int pixelSize= getByte(in);
	int imageDescriptor= getByte(in);
	if (imageType != 2) {
		fprintf(stderr,"%s: unhandle image type %d\n",
		  filename.c_str(),imageType);
		fclose(in);
		return id;
	}
	if (pixelSize!=24 && pixelSize!=32) {
		fprintf(stderr,"%s: unhandle pixel size %d\n",
		  filename.c_str(),pixelSize);
		fclose(in);
		return id;
	}
	fseek(in,imageIDSize,SEEK_CUR);
	GLubyte* colorMap= NULL;
	if (colorMapType != 0) {
		colorMap= (GLubyte*) malloc(colorMapEntrySize/8*colorMapSize);
		if (colorMap == NULL) {
			fclose(in);
			return id;
		}
		fread(colorMap,colorMapEntrySize/8*colorMapSize,1,in);
	}
	GLubyte* image= (GLubyte*) malloc(pixelSize/8*xSize*ySize);
	if (image == NULL) {
		fclose(in);
		return id;
	}
	fread(image,pixelSize/8*xSize*ySize,1,in);
	fclose(in);
//	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glTexParameteri(GL_TEXTURE_2D,GL_GENERATE_MIPMAP,GL_TRUE);
	glTexImage2D(GL_TEXTURE_2D,0,pixelSize==24?GL_RGB:GL_RGBA,
	  xSize,ySize,0,pixelSize==24?GL_BGR_EXT:GL_BGRA_EXT,
	  GL_UNSIGNED_BYTE,image);
	free(image);
	if (colorMap)
		free(colorMap);
	glTexEnvi(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,
	  GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
	fprintf(stderr,"loaded %s %d %d %d\n",
	  filename.c_str(),xSize,ySize,pixelSize);
	return id;
}
#endif
