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
#ifndef MSTSFILE_H
#define MSTSFILE_H

using namespace std;
#include <string>

struct MSTSFileNode {
	string* value;
	MSTSFileNode* children;
	MSTSFileNode* next;
	MSTSFileNode() {
		value= NULL;
		children= NULL;
		next= NULL;
	}
	MSTSFileNode* getFirstChild() { return children; }
	MSTSFileNode* getChild(int n) {
		MSTSFileNode* p= children;
		for (int i=0; i<n && p!=NULL; i++)
			p= p->next;
		return p;
	}
	MSTSFileNode* getNextSibling() { return next; }
	MSTSFileNode* find(const char*);
	MSTSFileNode* get(int n) {
		if (this==NULL)
			return NULL;
		if (children)
			return getChild(n);
		if (next)
			return next->getChild(n);
		return NULL;
	}
	MSTSFileNode* get(const char* name) {
		if (this == NULL)
			return NULL;
		if (children)
			return children->find(name);
		if (next && next->children)
			return next->children->find(name);
		return NULL;
	}
	const char* c_str() {
		if (this==NULL || value==NULL)
			return "";
		return value->c_str();
	}
	string catChildren() {
		if (this==NULL || children==NULL)
			return "";
		string s= "";
		for (MSTSFileNode* p=children; p!=NULL; p=p->next) {
			if (p->value && *(p->value)!="+")
				s+= *(p->value);
		}
		return s;
	}
};

class MSTSFile {
	int hiByte;
	int loByte;
	FILE* inFile;
	int getChar();
	int getToken(string& token);
	int parseList(MSTSFileNode* parent);
	void freeList(MSTSFileNode* first);
	MSTSFileNode* firstNode;
 public:
	MSTSFile() { inFile= NULL; firstNode= NULL; }
	~MSTSFile() { closeFile(); freeList(firstNode); }
	MSTSFileNode* getFirstNode() { return firstNode; }
	MSTSFileNode* find(const char* s) {
		return firstNode==NULL ? NULL : firstNode->find(s);
	}
	void readFile(const char* path);
	void openFile(const char* path);
	int getLine(string& token);
	void closeFile();
	void printTree(MSTSFileNode* node, string indent);
	void printTree() {
		printTree(firstNode,"");
	}
};

std::string fixFilenameCase(const char* path);

#endif
