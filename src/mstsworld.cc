//	code to read MSTS world file
//
//Copyright 2017 Doug Jones
#include "rmsim.h"

#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/Texture>
#include <osg/TexEnvFilter>
#include <osg/FrontFace>
#include <osg/AnimationPath>
#include <osg/BlendFunc>

#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/ReadFile>
#include <osgDB/FileUtils>
#include <osgDB/fstream>

extern MSTSRoute* mstsRoute;

class MSTSWorldReaderWriter : public osgDB::ReaderWriter
{
  public:
	MSTSWorldReaderWriter()
	{
		supportsExtension("world","MSTS World");
	}
	
	virtual const char* className() const { return "MSTS World Reader"; }

	virtual ReadResult readNode(const std::string& file,
	  const Options* options) const
	{
		std::string ext = osgDB::getFileExtension(file);
		if (!acceptsExtension(ext))
			return ReadResult::FILE_NOT_HANDLED;
		if (mstsRoute == NULL)
			return ReadResult::FILE_NOT_HANDLED;
		MSTSRoute::TerrainTileMap::iterator i=
		  mstsRoute->terrainTileMap.find(file.substr(0,9).c_str());
		if (i == mstsRoute->terrainTileMap.end())
			return ReadResult::FILE_NOT_HANDLED;
		MSTSRoute::Tile* tile= i->second;
//		fprintf(stderr,"world %s %d %d %p\n",file.c_str(),
//		  tile->x,tile->z,tile->models);
		if (tile->models == NULL)
			mstsRoute->loadModels(tile);
		ReadResult result(tile->models);
		return result;
#if 0
		std::string fileName = osgDB::findDataFile( file, options );
		osg::notify(osg::INFO)<<
		  "osgDB MSTS World reader: starting reading \""<<
		  fileName<<"\""<<std::endl;
		MSTSShape shape;
		shape.readFile(fileName.c_str(),NULL,NULL);	
		return ReadResult::FILE_NOT_HANDLED;
#endif
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

REGISTER_OSGPLUGIN(world, MSTSWorldReaderWriter)
