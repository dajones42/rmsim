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
};

class MSTSFile {
	int hiByte;
	int loByte;
	FILE* inFile;
	int getChar();
	int getToken(string& token);
	void parseList(MSTSFileNode* parent);
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
