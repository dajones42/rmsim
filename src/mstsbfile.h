//	class for reading MSTS binary files
//
//Copyright 2009 Doug Jones
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
