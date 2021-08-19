#include "mstsfile.h"
#include "service.h"

Service::Service()
{
}

Service::~Service()
{
}

void Service::readFile(const char* path)
{
	MSTSFile srvFile;
	srvFile.readFile(path);
//	srvFile.printTree();
	MSTSFileNode* def= srvFile.find("Service_Definition");
	if (def==NULL)
		throw "bad service file format";
	MSTSFileNode* p=def->children->find("PathID");
	if (p != NULL)
		pathName= *p->getChild(0)->value;
	p=def->children->find("Train_Config");
	if (p != NULL)
		consistName= *p->getChild(0)->value;
}
