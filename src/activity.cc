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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mstsfile.h"
#include "activity.h"

Activity::Activity()
{
	consists= NULL;
}

Activity::~Activity()
{
	clear();
}

void Activity::clear()
{
	while (consists != NULL) {
		LooseConsist* t= consists->next;
		delete consists;
		consists= t;
	}
}

void Activity::readFile(const char* path)
{
	clear();
	MSTSFile file;
	file.readFile(path);
//	file.printTree();
	MSTSFileNode* act= file.find("Tr_Activity");
	if (act == NULL)
		return;
	MSTSFileNode* trActFile= act->children->find("Tr_Activity_File");
	if (trActFile != NULL) {
		MSTSFileNode* player=
		  trActFile->children->find("Player_Service_Definition");
		if (player) {
			playerService= *player->getChild(0)->value;
			startTime= 0;
			MSTSFileNode* def=
			  player->children->find("Player_Traffic_Definition");
			if (def != NULL)
				startTime=
				  atoi(def->getChild(0)->value->c_str());
		}
		MSTSFileNode* traf=
		  trActFile->children->find("Traffic_Definition");
		traffic= NULL;
		if (traf) {
			for (MSTSFileNode*
			  sd=traf->children->find("Service_Definition");
			  sd!=NULL; sd=sd->find("Service_Definition")) {
				Traffic* t= new Traffic;
				t->service= *sd->getChild(0)->value;
				t->startTime=
				  atoi(sd->getChild(1)->value->c_str());
				t->next= traffic;
				traffic= t;
			}
		}
		MSTSFileNode* trActObjs=
		  trActFile->children->find("ActivityObjects");
		if (trActObjs != NULL)
			for (MSTSFileNode*
			  ao=trActObjs->children->find("ActivityObject");
			  ao!=NULL; ao=ao->find("ActivityObject"))
				saveConsist(ao->children);
	}
	Traffic* traf= traffic;
	traffic= NULL;
	while (traf != NULL) {
		Traffic* t= traf->next;
		traf->next= traffic;
		traffic= traf;
		traf= t;
	}
	MSTSFileNode* trActHdr= act->children->find("Tr_Activity_Header");
	if (trActHdr != NULL) {
		MSTSFileNode* briefing= trActHdr->children->find("Briefing");
		if (briefing) {
			fprintf(stderr,"Briefing:\n");
			for (MSTSFileNode* line=briefing->getFirstChild();
			  line!=NULL; line=line->next) {
				if (line->value && *(line->value)!="+")
					fprintf(stderr,"%s",
					  line->value->c_str());
			}
		}
	}
}

void Activity::saveConsist(MSTSFileNode* list)
{
	MSTSFileNode* type= list->find("ObjectType");
	if (type==NULL || type->children->value==NULL ||
	  type->children->value->compare("WagonsList")!=0)
		return;
	LooseConsist* consist= new LooseConsist();
	consist->wagons= NULL;
	MSTSFileNode* id= list->find("ID");
	if (id != NULL)
		consist->id= atoi(id->getChild(0)->value->c_str());
	MSTSFileNode* dir= list->find("Direction");
	if (dir != NULL)
		consist->direction= atoi(dir->getChild(0)->value->c_str());
	MSTSFileNode* tile= list->find("Tile");
	if (tile != NULL) {
		consist->tx= atoi(tile->getChild(0)->value->c_str());
		consist->tz= atoi(tile->getChild(1)->value->c_str());
		consist->x= atoi(tile->getChild(2)->value->c_str());
		consist->z= atoi(tile->getChild(3)->value->c_str());
	}
	MSTSFileNode* trainconfig= list->find("Train_Config");
	if (trainconfig != NULL) {
		MSTSFileNode* traincfg= trainconfig->children->find("TrainCfg");
		if (traincfg != NULL) {
			int i= 0;
			for (MSTSFileNode* n=traincfg->getChild(i++); n!=NULL;
			  n=traincfg->getChild(i++)) {
			  if (n->value && *(n->value)=="Wagon") {
//			for (MSTSFileNode*
//			  w=traincfg->children->find("Wagon");
//			  w!=NULL; w=w->find("Wagon")) {
				MSTSFileNode* wd=
				  n->next->children->find("WagonData");
				if (wd != NULL) {
					Wagon* wagon= new Wagon;
					wagon->dir= *wd->getChild(1)->value;
					wagon->name= *wd->getChild(0)->value;
					wagon->next= consist->wagons;
					consist->wagons= wagon;
					wagon->isEngine= false;
				}
			  } else if (n->value && *(n->value)=="Engine") {
				MSTSFileNode* ed=
				  n->next->children->find("EngineData");
				if (ed != NULL) {
					Wagon* wagon= new Wagon;
					wagon->dir= *ed->getChild(1)->value;
					wagon->name= *ed->getChild(0)->value;
					wagon->next= consist->wagons;
					consist->wagons= wagon;
					wagon->isEngine= true;
				}
			  }
			}
		}
	}
	consist->next= consists;
	consists= consist;
	Wagon* w= consist->wagons;
	consist->wagons= NULL;
	while (w != NULL) {
		Wagon* t= w->next;
		w->next= consist->wagons;
		consist->wagons= w;
		w= t;
	}
}
