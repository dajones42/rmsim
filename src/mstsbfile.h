//	class for reading MSTS binary files
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
#ifndef MTSTBFILE_H
#define MTSTBFILE_H

#include <zlib.h>
#include <string>

#define BUFSZ 4096

struct MSTSBFile {
	int compressed;
	z_stream strm;
	Byte* cBuf;
	Byte* uBuf;
	Byte* next;
	FILE* in;
	int read;
	MSTSBFile();
	~MSTSBFile();
	int open(const char* filename);
	int getBytes(Byte* bytes, int n);
	Byte getByte();
	short getShort();
	int getInt();
	float getFloat();
	std::string getString();
	std::string getString(int n);
	void seek(int offset);
};

#endif
