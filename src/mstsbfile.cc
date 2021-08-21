//	code to read MSTS binary file
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mstsbfile.h"
#include "mstsfile.h"

using namespace std;
#include <string>

MSTSBFile::MSTSBFile()
{
	compressed= 0;
	cBuf= NULL;
	uBuf= NULL;
	in= NULL;
	read= 0;
}

//	opens the specified file and determines type
int MSTSBFile::open(const char* filename)
{
	in= fopen(filename,"r");
	if (in == NULL) {
		string fixed= fixFilenameCase(filename);
		if (fixed.size() > 0)
			in= fopen(fixed.c_str(),"r");
	}
	if (in == NULL)
		return 1;
	char magic[16];
	if (fread(magic,1,16,in) != 16)
		return 1;
	if (strncmp(magic,"SIMISA",6) != 0)
		return 1;
	read= 16;
	if (magic[7] == 'F') {
		compressed= 1;
		cBuf= (Byte*) malloc(BUFSZ);
		uBuf= (Byte*) malloc(BUFSZ);
		if (cBuf==NULL || uBuf==NULL)
			return 1;
		strm.zalloc= Z_NULL;
		strm.zfree= Z_NULL;
		strm.opaque= Z_NULL;
		strm.next_in= cBuf;
		strm.avail_in= 0;
		strm.next_out= uBuf;
		strm.avail_out= 0;
		next= uBuf;
		int err= inflateInit(&strm);
		if (err < 0) {
			fprintf(stderr,"inflate error %d %s\n",err,strm.msg);
			return 1;
		}
	}
	return 0;
}

MSTSBFile::~MSTSBFile()
{
	if (in != NULL)
		fclose(in);
	if (compressed)
		inflateEnd(&strm);
	if (cBuf)
		free(cBuf);
	if (uBuf)
		free(uBuf);
}

//	fills bytes with the next n bytes from the file
int MSTSBFile::getBytes(Byte* bytes, int n)
{
	if (!compressed & bytes!=NULL) {
		read+= n;
		return fread(bytes,1,n,in);
	}
	if (!compressed) {
		Byte b;
		for (int i=0; i<n; i++)
			fread(&b,1,1,in);
		read+= n;
		return n;
	}
	int i=0;
	for (;;) {
		if (next != strm.next_out) {
			if (bytes != NULL)
				bytes[i]= *next;
			next++;
			read++;
			i++;
			if (i >= n)
				return n;
			continue;
		}
		next= uBuf;
		strm.next_out= uBuf;
		strm.avail_out= BUFSZ;
		if (strm.avail_in == 0) {
			strm.next_in= cBuf;
			strm.avail_in= fread(cBuf,1,BUFSZ,in);
			if (strm.avail_in <= 0)
				return i;
		}
		int err= inflate(&strm,Z_NO_FLUSH);
		if (err < 0) {
			fprintf(stderr,"inflate error %d %s\n",err,strm.msg);
			strm.avail_out= 0;
		}
//		fprintf(stderr,"inflated %p %p %d %d\n",
//		  strm.next_out,uBuf,strm.avail_out,strm.avail_in);
//		for (int i=0; i<16; i++)
//			fprintf(stderr," %x",0xff&uBuf[i]);
//		fprintf(stderr,"\n");
	}
}

//	skips forward in the file
void MSTSBFile::seek(int offset)
{
	if (!compressed) {
		fseek(in,offset,SEEK_SET);
		return;
	}
	getBytes(NULL,offset-read);
}

int MSTSBFile::getInt()
{
	Byte b[4];
	if (getBytes(b,4) != 4)
		return 0;
//	fprintf(stderr,"getInt %x %x %x %x\n",b[0],b[1],b[2],b[3]);
	return b[0] + 256*(b[1] + 256*(b[2] + 256*b[3]));
}

float MSTSBFile::getFloat()
{
	union {
		int i;
		float f;
	} i2f;
	i2f.i= getInt();
	return i2f.f;
}

Byte MSTSBFile::getByte()
{
	Byte b;
	if (getBytes(&b,1) != 1)
		return 0;
	return b;
}

short MSTSBFile::getShort()
{
	Byte b[2];
	if (getBytes(b,2) != 2)
		return 0;
//	fprintf(stderr,"getInt %x %x\n",b[0],b[1]);
	return b[0] + 256*b[1];
}

std::string MSTSBFile::getString()
{
	Byte n= getByte();
	return getString(n);
}

std::string MSTSBFile::getString(int n)
{
	std::string s;
	for (int i=0; i<n; i++) {
		int c= getShort();
		if (c <= 0x7f) {
			s.push_back(c);
		} else if (c <= 0x7ff) {
			s.push_back(0xc0 + ((c>>6)&0x1f));
			s.push_back(0x80 + (c&0x3f));
		} else {
			s.push_back(0xe0 + ((c>>12)&0xf));
			s.push_back(0x80 + ((c>>6)&0x3f));
			s.push_back(0x80 + ((c>>6)&0x3f));
		}
	}
	return s;
}
