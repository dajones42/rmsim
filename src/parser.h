//	Parser for reading ascii files of commands
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
#ifndef PARSER_H
#define PARSER_H

#include <string>
#include <vector>
#include <set>
#include <stack>
#include <fstream>
#include <stdexcept>
#include "commandreader.h"

struct Parser : public CommandReader {
	typedef std::vector<std::string> DirStack;
	DirStack dirStack;
	std::string dir;
	struct FileInfo {
		std::string name;
		int lineNumber;
		std::ifstream file;
		FileInfo(const char* nm);
		~FileInfo();
	};
	typedef std::vector<FileInfo*> FileStack;
	FileStack fileStack;
	typedef std::set<std::string> FileSet;
	FileSet fileSet;
	std::string delimiters;
	std::string line;
	std::vector<std::string> tokens;
	const char* makePath();
	void pushFile(const char* name, int require);
	Parser();
	~Parser();
	void splitLine();
	void setCommand(const char* s) {
		line= std::string(s);
		splitLine();
	};
	void setCommand(std::string& s) {
		line= s;
		splitLine();
	};
	void setDelimiters(std::string& s) {
		delimiters= s;
	};
	void setDelimiters(const char* s) {
		delimiters= s;
	};
	virtual int getCommand();
	virtual int getNumTokens() { return tokens.size(); }
	virtual std::string getString(int index);
	virtual int getInt(int index, int min, int max,
	  int dflt=0x80000000);
	virtual double getDouble(int index, double min, double max,
	  double dflt=1e30);
	virtual void printError(const char* message);
	virtual void parseBlock(CommandBlockHandler* handler);
	typedef set<string> StringSet;
	StringSet symbols;
	stack<int> ifStack;
};

#endif
