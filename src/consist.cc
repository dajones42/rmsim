#include "mstsfile.h"
#include "consist.h"
#include <stdio.h>

MSTSConsist::MSTSConsist()
{
}

MSTSConsist::~MSTSConsist()
{
}

void printTree(MSTSFileNode* node, string indent)
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

void MSTSConsist::readFile(const char* path)
{
	MSTSFile conFile;
	conFile.readFile(path);
#if 0
	MSTSFileNode* def= conFile.find("Service_Definition");
	if (def==NULL)
		throw "bad service file format";
	MSTSFileNode* p=def->children->find("PathID");
	if (p != NULL)
		pathName= *p->getChild(0)->value;
	p=def->children->find("Train_Config");
	if (p != NULL)
		consistName= *p->getChild(0)->value;
#else
	//printTree(conFile.getFirstNode(),"");
#endif
}
