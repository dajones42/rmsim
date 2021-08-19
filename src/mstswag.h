//	function for reading MSTS wag file
//
//Copyright 2009 Doug Jones
#ifndef MSTSWAG_H
#define MSTSWAG_H

RailCarDef* readMSTSWag(const char* dir, const char* file,
  bool saveNames=false);

#endif
