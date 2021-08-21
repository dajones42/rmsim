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
