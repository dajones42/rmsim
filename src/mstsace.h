//	functions for reading MSTS ACE files
//
//Copyright 2009 Doug Jones
#ifndef MSTSACE_H
#define MSTSACE_H

osg::Image* readMSTSACE(const char* path);
osg::Texture2D* readCacheACEFile(const char* path, bool tryPNG=false);
void cleanACECache();

#endif
