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

class MSTSTerrainReaderWriter : public osgDB::ReaderWriter
{
  public:
	MSTSTerrainReaderWriter()
	{
		supportsExtension("raw","MSTS Shape");
	}
	
	virtual const char* className() const { return "MSTS Terrain Reader"; }

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
//		fprintf(stderr,"terrain %s %d %d %p\n",
//		  file.c_str(),tile->x,tile->z,tile->terrModel);
		if (tile->terrModel == NULL)
			mstsRoute->makeTerrainPatches(tile);
		ReadResult result(tile->terrModel);
		return result;
#if 0
		std::string ext = osgDB::getFileExtension(file);
		if (!acceptsExtension(ext)) return ReadResult::FILE_NOT_HANDLED;
		std::string fileName = osgDB::findDataFile( file, options );
		osg::notify(osg::INFO)<<
		  "osgDB MSTS Terrain reader: starting reading \""<<
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

REGISTER_OSGPLUGIN(raw, MSTSTerrainReaderWriter)
