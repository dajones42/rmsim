#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "mstsfile.h"
#include "trackdb.h"
#include "tsection.h"

TrackDB::TrackDB()
{
	nodes= NULL;
	nNodes= 0;
	trItems= NULL;
	nTrItems= 0;
}

TrackDB::~TrackDB()
{
	freeMem();
}

void TrackDB::freeMem()
{
	for (int i=0; i<nTrItems; i++)
		if (trItems[i].name != NULL)
			delete trItems[i].name;
	if (nodes != NULL)
		free(nodes);
	if (trItems != NULL)
		free(trItems);
	nNodes= 0;
	nTrItems= 0;
}

void TrackDB::readFile(const char* path, int readGlobalTSection,
  int readRouteTSection)
{
	TSection tSection;
	if (readGlobalTSection) {
		char* p= strstr((char*)path,"\\ROUTES");
		if (p != NULL) {
			string filename(path,p-path);
			filename+= "\\global\\tsection.dat";
			tSection.readGlobalFile(filename.c_str());
		}
	}
	if (readRouteTSection) {
		char* p= strrchr((char*)path,'\\');
		if (p != NULL) {
			string filename(path,p-path);
			filename+= "\\tsection.dat";
			tSection.readRouteFile(filename.c_str());
		}
	}
	readFile(path,&tSection);
}

void TrackDB::readFile(const char* path, TSection* tSection)
{
	freeMem();
	MSTSFile tdbFile;
	tdbFile.readFile(path);
	MSTSFileNode* tdb= tdbFile.find("TrackDB");
	if (tdb == NULL)
		return;
	MSTSFileNode* trackNodes= tdb->children->find("TrackNodes");
	if (trackNodes == NULL)
		return;
	nNodes= atoi(trackNodes->children->value->c_str());
	nodes= (TrackNode*) calloc(nNodes,sizeof(struct TrackNode));
	MSTSFileNode* trItemTable= tdb->children->find("TrItemTable");
	if (trItemTable != NULL) {
		nTrItems= atoi(trItemTable->children->value->c_str());
		trItems= (TrItem*) calloc(nTrItems,sizeof(struct TrItem));
	}
	for (MSTSFileNode* tn=trackNodes->children->find("TrackNode");
	  tn!=NULL; tn=tn->find("TrackNode"))
		saveTrackNode(tn->children);
	if (trItemTable != NULL) {
		for (MSTSFileNode* i=trItemTable->getChild(0);
		  i!=NULL; i=i->next) {
			if (i->value == NULL)
				continue;
			if (i->value->compare("SidingItem") == 0)
				saveTrItem(0,i->next->children);
			else if (i->value->compare("PlatformItem") == 0)
				saveTrItem(1,i->next->children);
			else if (i->value->compare("SignalItem") == 0)
				saveTrItem(2,i->next->children);
		}
	}
	for (int i=0; i<nNodes; i++) {
		TrackNode* node= &nodes[i];
		if (node->nSections <= 0)
			continue;
		TrSection* s= &node->sections[node->nSections];
		s->tx= node->pin[1]->tx;
		s->tz= node->pin[1]->tz;
		s->x= node->pin[1]->x;
		s->y= node->pin[1]->y;
		s->z= node->pin[1]->z;
		for (int j=0; j<node->nSections; j++) {
			TrSection* s1= &node->sections[j];
			TrSection* s2= &node->sections[j+1];
			double x1= 2048*s1->tx + s1->x;
			double z1= 2048*s1->tz + s1->z;
			double x2= 2048*s2->tx + s2->x;
			double z2= 2048*s2->tz + s2->z;
			double d= sqrt((x1-x2)*(x1-x2)+(z1-z2)*(z1-z2)+
			  (s1->y-s2->y)*(s1->y-s2->y));
			Curve* c= tSection==NULL ? NULL :
			  tSection->findCurve(s1->sectionID);
			if (c == NULL) {
				s1->length= d;
			} else {
				s1->radius= c->radius;
				s1->angle= c->angle;
				s1->length= s1->radius*s1->angle*3.14159/180;
				if (s1->angle < 0)
					s1->length= -s1->length;
				double r= sqrt(c->radius*c->radius-.25*d*d);
				s1->cx= .5*(x1+x2);
				s1->cz= .5*(z1+z2);
				if (c->angle < 0) {
					s1->cx+= r*(z1-z2)/d;
					s1->cz+= r*(x2-x1)/d;
				} else {
					s1->cx+= r*(z2-z1)/d;
					s1->cz+= r*(x1-x2)/d;
				}
			}
			if (d > 0)
				s1->grade= s1->y>s2->y ? (s1->y-s2->y)/d :
				  (s2->y-s1->y)/d;
		}
		node->nSections++;
	}
}

void TrackDB::saveTrackNode(MSTSFileNode* list)
{
	int id= atoi(list->value->c_str());
	TrackNode* node= &nodes[id-1];
	MSTSFileNode* vNode= list->find("TrVectorNode");
	if (vNode != NULL)
		vNode= vNode->children->find("TrVectorSections");
	if (vNode != NULL) {
		node->nSections= atoi(vNode->getChild(0)->value->c_str());
		node->sections=
		  (TrSection*) calloc(node->nSections+1,sizeof(TrSection));
		MSTSFileNode* p= vNode->getChild(1);
		for (int i=0; i<node->nSections; i++) {
			TrSection* s= &(node->sections[i]);
			s->sectionID= atoi(p->value->c_str());
			p= p->next;
			s->shapeID= atoi(p->value->c_str());
			for (int j=0; j<7; j++)
				p= p->next;
			s->tx= atoi(p->value->c_str());
			p= p->next;
			s->tz= atoi(p->value->c_str());
			p= p->next;
			s->x= atof(p->value->c_str());
			p= p->next;
			s->y= atof(p->value->c_str());
			p= p->next;
			s->z= atof(p->value->c_str());
			p= p->next;
			for (int j=0; j<3; j++)
				p= p->next;
		}
	}
	MSTSFileNode* uid= list->find("UiD");
	if (uid != NULL) {
		node->id= atoi(uid->getChild(2)->value->c_str());
		node->tx= atoi(uid->getChild(4)->value->c_str());
		node->tz= atoi(uid->getChild(5)->value->c_str());
		node->x= atof(uid->getChild(6)->value->c_str());
		node->y= atof(uid->getChild(7)->value->c_str());
		node->z= atof(uid->getChild(8)->value->c_str());
	}
	MSTSFileNode* trPins= list->find("TrPins");
	if (trPins != NULL) {
		for (MSTSFileNode* p=trPins->children->find("TrPin");
		  p!=NULL; p=p->find("TrPin")) {
			int i= atoi(p->children->value->c_str());
			if (i<1 || i>nNodes) {
				node->pin[node->nPins]= NULL;
			} else {
				node->pin[node->nPins]= &nodes[i-1];
				node->pinEnd[node->nPins]= 
				  atoi(p->getChild(1)->value->c_str());
			}
			node->nPins++;
		}
	}
	MSTSFileNode* jNode= list->find("TrJunctionNode");
	if (jNode != NULL) {
		node->shape= atoi(jNode->getChild(1)->value->c_str());
	}
}

void TrackDB::saveTrItem(int type, MSTSFileNode* list)
{
	MSTSFileNode* idNode= list->find("TrItemId");
	if (idNode == NULL)
		return;
	int id= atoi(idNode->getChild(0)->value->c_str());
	TrItem* item= &trItems[id];
	item->id= id;
	item->type= type;
	MSTSFileNode* rData= list->find("TrItemRData");
	if (rData != NULL) {
		item->x= atof(rData->getChild(0)->value->c_str());
		item->y= atof(rData->getChild(1)->value->c_str());
		item->z= atof(rData->getChild(2)->value->c_str());
		item->tx= atoi(rData->getChild(3)->value->c_str());
		item->tz= atoi(rData->getChild(4)->value->c_str());
	}
	MSTSFileNode* sData= list->find("TrItemSData");
	if (sData != NULL)
		item->offset= atof(sData->getChild(0)->value->c_str());
	if (type == 0) {
		MSTSFileNode* iData= list->find("SidingTrItemData");
		if (iData != NULL) {
			item->otherID= atoi(iData->getChild(1)->value->c_str());
			item->flags= strtol(iData->getChild(0)->value->c_str(),
			  NULL,16);
		}
		MSTSFileNode* name= list->find("SidingName");
		if (name != NULL)
			item->name= new string(*name->getChild(0)->value);
	} else if (type == 1) {
		MSTSFileNode* iData= list->find("PlatformTrItemData");
		if (iData != NULL) {
			item->otherID= atoi(iData->getChild(1)->value->c_str());
			item->flags= strtol(iData->getChild(0)->value->c_str(),
			  NULL,16);
		}
		MSTSFileNode* station= list->find("Station");
		if (station != NULL)
			item->name= new string(*station->getChild(0)->value);
		MSTSFileNode* name= list->find("PlatformName");
		if (name!=NULL && item->name==NULL) {
			item->name= new string(*name->getChild(0)->value);
		} else if (name!=NULL && item->name!=NULL &&
		  (*item->name)!=(*name->getChild(0)->value)) {
			(*item->name)+= " ";
			(*item->name)+= *name->getChild(0)->value;
		}
	} else if (type == 2) {
		MSTSFileNode* sType= list->find("TrSignalType");
		if (sType != NULL) {
			item->flags= strtol(sType->getChild(0)->value->c_str(),
			  NULL,16);
			item->dir= atoi(sType->getChild(1)->value->c_str());
			//item->sigdata1= atof(sType->getChild(2)->value->c_str());
			item->name= new string(*sType->getChild(3)->value);
		}
		// TrSignalDirs ? (tracknode,data1,linklrpath,data3)
	}
}
