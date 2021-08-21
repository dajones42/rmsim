//	Parser interfaces 
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
