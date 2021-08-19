#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <plib/ul.h>

using namespace std;
#include <string>

#include "mstsfile.h"

MSTSFileNode* MSTSFileNode::find(const char* value)
{
	for (MSTSFileNode* p=this; p!=NULL; p=p->next)
		if (p->value!=NULL && p->value->compare(value)==0)
			return p->next;
	return NULL;
}

int MSTSFile::getChar()
{
	unsigned char bytes[2];
	if (fread(bytes,1,2,inFile) != 2)
		return EOF;
	if (bytes[hiByte] != 0)
		return '?';
	return bytes[loByte];
}

int MSTSFile::getToken(string& token)
{
	static int savedC= '\0';
	token.erase();
	if (savedC != '\0') {
		token+= (char) savedC;
		savedC= '\0';
		return 1;
	}
	savedC= '\0';
	int c;
	for (;;) {
		c= getChar();
		if (c == EOF)
			return 0;
		if (c!=' ' && c!='\t' && c!='\n' && c!='\r')
			break;
	}
	if (c=='(' || c==')') {
		token+= (char) c;
	} else if (c == '"') {
		for (c=getChar(); c!='"'; c=getChar()) {
			if (c == '\\') {
				c= getChar();
				if (c == 'n')
					c= '\n';
			}
			token+= (char) c;
		}
	} else {
		while (c!=' ' && c!='\t' && c!='\n' && c!='\r') {
			if (c=='(' || c==')') {
				savedC= c;
				break;
			}
			token+= (char) c;
			c= getChar();
		}
	}
	return 1;
}

void MSTSFile::parseList(MSTSFileNode* parent)
{
	MSTSFileNode* last= NULL;
	string token;
	while (getToken(token)) {
		if (token == ")")
			return;
		MSTSFileNode* n= new MSTSFileNode();
		if (last == NULL)
			parent->children= n;
		else
			last->next= n;
		last= n;
		if (token == "(") {
			parseList(n);
		} else {
			n->value= new string(token);
		}
	}
}

void MSTSFile::openFile(const char* path)
{
	inFile= fopen(path,"r");
	if (inFile == NULL) {
		string fixed= fixFilenameCase(path);
		if (fixed.size() > 0)
			inFile= fopen(fixed.c_str(),"r");
	}
	if (inFile == NULL) {
		fprintf(stderr,"cannot open %s\n",path);
		throw "MSTSFile: cannot open file";
	}
	char mark[2];
	fread(mark,1,2,inFile);
	if (mark[0] == '\377') {
		loByte= 0;
		hiByte= 1;
	} else {
		loByte= 1;
		hiByte= 0;
	}
}

void MSTSFile::readFile(const char* path)
{
	firstNode= NULL;
	openFile(path);
	string token;
	getToken(token);
	if (token.compare(0,6,"SIMISA") != 0) {
		closeFile();
		throw "MSTSFile: heading not found";
	}
	MSTSFileNode* last= NULL;
	while (getToken(token)) {
		MSTSFileNode* n= new MSTSFileNode();
		if (last == NULL)
			firstNode= n;
		else
			last->next= n;
		last= n;
		if (token == "(") {
			parseList(n);
		} else {
			n->value= new string(token);
		}
	}
	closeFile();
}

int MSTSFile::getLine(string& line)
{
	line.erase();
	int c;
	for (;;) {
		c= getChar();
		if (c == EOF)
			return -1;
		if (c == '\n')
			break;
		if (c == '\r')
			continue;
		line+= (char) c;
	}
	return line.length();
}

void MSTSFile::closeFile()
{
	if (inFile != NULL)
		fclose(inFile);
	inFile= NULL;
}

void MSTSFile::freeList(MSTSFileNode* first)
{
	while (first != NULL) {
		MSTSFileNode* t= first->next;
		if (first->value != NULL)
			delete first->value;
		else
			freeList(first->children);
		delete first;
		first= t;
	}
}

string fixFilenameCase(const char* path)
{
	char* p= strrchr((char*)path,'/');
	if (p == NULL)
		return "";
	string dirPath(path,p-path);
	ulDir* dir= ulOpenDir(dirPath.c_str());
	if (dir == NULL) {
		fprintf(stderr,"cannot read directory %s\n",
		  dirPath.c_str());
		return "";
	}
	for (ulDirEnt* ent=ulReadDir(dir); ent!=NULL; ent=ulReadDir(dir)) {
		if (strcasecmp(p+1,ent->d_name) == 0) {
			string result= dirPath+"/"+ent->d_name;
			ulCloseDir(dir);
			return result;
		}
	}
	ulCloseDir(dir);
	return "";
}

void MSTSFile::printTree(MSTSFileNode* node, string indent)
{
	if (node == NULL)
		return;
	for (MSTSFileNode* n1= node; n1!=NULL; n1=n1->getNextSibling()) {
		fprintf(stderr,"%s",indent.c_str());
		if (n1->value)
			fprintf(stderr,"node value %s\n",n1->value->c_str());
		else
			fprintf(stderr,"no value\n");
		printTree(n1->getFirstChild(),indent+" ");
	}
}
