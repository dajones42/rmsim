//	wrapper for 2D textures
//	should be removed
//
//Copyright 2009 Doug Jones
#ifndef TEXTURE_H
#define TEXTURE_H

#include <string>
#include <map>
#include <osg/Texture2D>

struct Texture {
	std::string filename;
//	int load();
	osg::Texture2D* texture;
	Texture() { texture= NULL; }
};
typedef std::map<std::string,Texture*> TextureMap;
extern TextureMap textureMap;

#endif
