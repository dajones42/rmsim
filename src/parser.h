//	Parser for reading ascii files of commands
//
//Copyright 2009 Doug Jones
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
