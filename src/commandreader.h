//	Parser interfaces 
//
//Copyright 2009 Doug Jones
#ifndef COMMANDREADER_H
#define COMMANDREADER_H

#include <string>

struct CommandBlockHandler;

struct CommandReader {
	virtual int getCommand() { return 0; };
	virtual int getNumTokens() { return 0; };
	virtual std::string getString(int index) { return ""; };
	virtual int getInt(int index, int min, int max,
	  int dflt=0x80000000) { return dflt; };
	virtual double getDouble(int index, double min, double max,
	  double dflt=1e30) { return dflt; };
	virtual void printError(const char* message) { };
	virtual void parseBlock(CommandBlockHandler* handler) { };
};

struct CommandBlockHandler {
	virtual bool handleCommand(CommandReader& reader) { };
	virtual void handleBeginBlock(CommandReader& reader) { };
	virtual void handleEndBlock(CommandReader& reader) { };
};

#endif
