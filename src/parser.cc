//	file parser for rmsim
//	uses python to evaluate expressions that are expected
//	to be numeric but aren't
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
#include "parser.h"
#include <string.h>

#include <Python.h>
#include <graminit.h>
#include <pythonrun.h>
PyObject* pdict;

void strlcpy(char* s1, const char* s2, int n)
{
	strncpy(s1,s2,n-1);
	s1[n-1]= '\0';
}

Parser::Parser()
{
	Py_Initialize();
	pdict= PyDict_New();
	PyDict_SetItemString(pdict,"__builtins__",PyEval_GetBuiltins());
	delimiters= " ";
}

Parser::~Parser()
{
	Py_DECREF(pdict);
}

Parser::FileInfo::FileInfo(const char* nm) {
	name= string(nm);
	lineNumber= 0;
	file.open(nm);
	if (! file)
		throw std::invalid_argument("cannot read file");
}

Parser::FileInfo::~FileInfo() {
	file.close();
}

#define PATHSZ 512

const char* Parser::makePath()
{
	if (tokens.size() < 2)
		throw std::invalid_argument("filename missing");
	const char* name= tokens[1].c_str();
	static char path[PATHSZ];
	if (dir.size()==0 || name[0]=='/' || name[1]==':')
		strlcpy(path,name,PATHSZ);
	else if (dir.size()+strlen(name)+2 > PATHSZ)
		throw std::invalid_argument("path too long");
	else
		ulMakePath(path,dir.c_str(),name);
	return path;
}

void Parser::printError(const char* message)
{
	FileInfo* fi= fileStack.back();
	fprintf(stderr,"%s\n %s line %d\n %s\n",
	  message,fi->name.c_str(),fi->lineNumber,line.c_str());
}

std::string Parser::getString(int index)
{
	if (tokens.size() <= index) {
		char error[100];
		sprintf(error,"field %d missing",index);
		throw std::out_of_range(error);
	}
	return tokens[index];
}

int Parser::getInt(int index, int min, int max, int dflt)
{
	if (tokens.size() <= index) {
		if (dflt != 0x80000000)
			return dflt;
		char error[100];
		sprintf(error,"field %d missing",index);
		throw std::out_of_range(error);
	}
	char* end;
	const char* s= tokens[index].c_str();
	int v= strtol(s,&end,0);
	if (*end != '\0') {
		PyObject* pv= PyRun_String(s,eval_input,pdict,pdict);
		if (pv==NULL || PyArg_Parse(pv,"i",&v)==0) {
			char error[100];
			sprintf(error,"evaluation failed on field %d",index);
			throw std::invalid_argument(error);
		}
		Py_DECREF(pv);
	}
	if (v<min || v>max) {
		char error[100];
		sprintf(error,"field %d must between %d and %d",index,min,max);
		throw std::out_of_range(error);
	}
	return v;
}

double Parser::getDouble(int index, double min, double max, double dflt)
{
	if (tokens.size() <= index) {
		if (dflt < 1e30)
			return dflt;
		char error[100];
		sprintf(error,"field %d missing",index);
		throw std::out_of_range(error);
	}
	char* end;
	const char* s= tokens[index].c_str();
	double v= strtod(s,&end);
	if (*end != '\0') {
		PyObject* pv= PyRun_String(s,eval_input,pdict,pdict);
		if (pv==NULL || PyArg_Parse(pv,"d",&v)==0) {
			char error[100];
			sprintf(error,"evaluation failed on field %d",index);
			throw std::invalid_argument(error);
		}
		Py_DECREF(pv);
	}
	if (v<min || v>max) {
		char error[100];
		sprintf(error,"field %d must between %g and %g",index,min,max);
		throw std::out_of_range(error);
	}
	return v;
}

void Parser::pushFile(const char* name, int require)
{
	if (require) {
		FileSet::iterator i= fileSet.find(name);
		if (i != fileSet.end())
			return;
		fileSet.insert(name);
	}
	FileInfo* fi= new FileInfo(name);
	fileStack.push_back(fi);
}

void Parser::splitLine()
{
	tokens.clear();
	string token;
	bool quote= false;
	for (int i=0; i<line.size(); i++) {
		char c= line[i];
		if (!quote && ((0<=c && c<' ') ||
		  delimiters.find(c)!=string::npos)) {
			if (token.size() > 0) {
				tokens.push_back(token);
				token.clear();
			}
		} else if (c=='\\' && i<line.size()-1) {
			i++;
			token+= line[i];
		} else if (c=='"' && quote) {
			tokens.push_back(token);
			token.clear();
			quote= false;
		} else if (c=='"' && token.size()==0) {
			quote= true;
		} else {
			token+= c;
		}
	}
	if (token.size() > 0)
		tokens.push_back(token);
}

int Parser::getCommand()
{
	FileInfo* fi= fileStack.back();
	while (getline(fi->file,line)) {
		fi->lineNumber++;
		splitLine();
		if (tokens.size() == 0)
			continue;
		const char* cmd= tokens[0].c_str();
		try {
			if (cmd[0] == '#') {
				continue;
			} else if (strcasecmp(cmd,"if") == 0) {
				int t= 0;
				for (int i=1; i<tokens.size(); i++)
					if (symbols.find(tokens[i])!=
					  symbols.end())
						t= 3;
				ifStack.push(t);
//				fprintf(stderr,"if %d\n",ifStack.top());
			} else if (strcasecmp(cmd,"elif") == 0) {
				int t= ifStack.top() & 2;
				for (int i=1; t==0 && i<tokens.size(); i++)
					if (symbols.find(tokens[i])!=
					  symbols.end())
						t= 3;
				ifStack.pop();
				ifStack.push(t);
//				fprintf(stderr,"elif %d\n",ifStack.top());
			} else if (strcasecmp(cmd,"else") == 0) {
				int t= ifStack.top() & 2;
				if (t == 0)
					t= 3;
				ifStack.pop();
				ifStack.push(t);
//				fprintf(stderr,"else %d\n",ifStack.top());
			} else if (strcasecmp(cmd,"endif") == 0) {
				ifStack.pop();
//				fprintf(stderr,"endif %d\n",ifStack.size());
			} else if (ifStack.size()>0 && ifStack.top()!=3) {
				continue;
			} else if (strcasecmp(cmd,"dir") == 0) {
				dir= string(makePath());
			} else if (strcasecmp(cmd,"pushdir") == 0) {
				dirStack.push_back(dir);
				dir= string(makePath());
			} else if (strcasecmp(cmd,"popdir") == 0) {
				if (dirStack.size() > 0) {
					dir= dirStack[dirStack.size()-1];
					dirStack.pop_back();
				}
			} else if (strcasecmp(cmd,"include") == 0) {
				pushFile(makePath(),0);
				fi= fileStack.back();
			} else if (strcasecmp(cmd,"require") == 0) {
				pushFile(makePath(),1);
				fi= fileStack.back();
			} else if (strcasecmp(cmd,"python") == 0) {
				if (tokens.size() < 2)
					throw "two or more fields required";
				string s(tokens[1]);
				for (int i=2; i<tokens.size(); i++)
					s+= " "+tokens[i];
				PyObject* pv= PyRun_String(s.c_str(),file_input,
				  pdict,pdict);
				if (pv == NULL)
					throw "python failed";
				Py_DECREF(pv);
			} else if (strcasecmp(cmd,"delimiters") == 0) {
				if (tokens.size() < 2)
					throw "two fields required";
				delimiters= tokens[1];
			} else {
				return 1;
			}
		} catch (const char* message) {
			printError(message);
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	fileStack.pop_back();
	delete fi;
	if (fileStack.size() > 0)
		return getCommand();
	return 0;
}

void Parser::parseBlock(CommandBlockHandler* handler)
{
	handler->handleBeginBlock((CommandReader&)*this);
	while (getCommand()) {
		try {
			if (getString(0) == "end")
				break;
			if (!handler->handleCommand((CommandReader&)*this))
				printError("unknown command");
		} catch (const std::exception& error) {
			printError(error.what());
		}
	}
	handler->handleEndBlock((CommandReader&)*this);
}
