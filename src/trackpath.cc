#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mstsfile.h"
#include "trackpath.h"

TrackPath::TrackPath()
{
	nodes= NULL;
	nNodes= 0;
	pdps= NULL;
	nPDPs= 0;
}

TrackPath::~TrackPath()
{
	if (nodes != NULL)
		free(nodes);
	if (pdps != NULL)
		free(pdps);
}

void TrackPath::readFile(const char* path)
{
	if (nodes != NULL)
		free(nodes);
	if (pdps != NULL)
		free(pdps);
	nodes= NULL;
	nNodes= 0;
	pdps= NULL;
	nPDPs= 0;
	MSTSFile patFile;
	patFile.readFile(path);
	MSTSFileNode* trackPDPs= patFile.find("TrackPDPs");
	MSTSFileNode* trackPath= patFile.find("TrackPath");
	if (trackPDPs==NULL || trackPath==NULL)
		throw "bad path file format";
	for (MSTSFileNode* p=trackPDPs->children->find("TrackPDP"); p!=NULL;
	  p=p->find("TrackPDP"))
		nPDPs++;
	if (nPDPs <= 0)
		throw "no TrackPDPs";
	pdps= (TrackPDP*) calloc(nPDPs,sizeof(TrackPDP));
	nPDPs= 0;
//	fprintf(stderr,"pdps\n");
	for (MSTSFileNode* p=trackPDPs->children->find("TrackPDP"); p!=NULL;
	  p=p->find("TrackPDP")) {
		TrackPDP* pdp= &pdps[nPDPs++];
		pdp->tx= atoi(p->getChild(0)->value->c_str());
		pdp->tz= atoi(p->getChild(1)->value->c_str());
		pdp->x= atof(p->getChild(2)->value->c_str());
		pdp->y= atof(p->getChild(3)->value->c_str());
		pdp->z= atof(p->getChild(4)->value->c_str());
		pdp->type1= atoi(p->getChild(5)->value->c_str());
		pdp->type2= atoi(p->getChild(6)->value->c_str());
	}
//	fprintf(stderr,"alloc nodes\n");
	MSTSFileNode* trPathNodes= trackPath->children->find("TrPathNodes");
//	fprintf(stderr,"tpn %p\n",trPathNodes);
	nNodes= atoi(trPathNodes->getChild(0)->value->c_str());
//	fprintf(stderr,"n %d\n",nNodes);
	nodes= (TrPathNode*) calloc(nNodes,sizeof(TrPathNode));
	int n= 0;
//	fprintf(stderr,"nodes\n");
	for (MSTSFileNode* p=trPathNodes->children->find("TrPathNode"); p!=NULL;
	  p=p->find("TrPathNode")) {
		TrPathNode* node= &nodes[n];
//		fprintf(stderr,"%d %p",n,node);
		node->next= NULL;
		node->prev= NULL;
		node->nextSiding= NULL;
		n++;
		sscanf(p->getChild(0)->value->c_str(),"%x",&node->flags);
//		fprintf(stderr," %x\n",node->flags);
		int i= atoi(p->getChild(1)->value->c_str());
//		fprintf(stderr," %dn",i);
		if (i>=0 && i<nNodes) {
			node->next= &nodes[i];
			node->next->prev= node;
		}
		i= atoi(p->getChild(2)->value->c_str());
//		fprintf(stderr," %d",i);
		if (i>=0 && i<nNodes) {
			node->nextSiding= &nodes[i];
		}
		i= atoi(p->getChild(3)->value->c_str());
//		fprintf(stderr," %d\n",i);
		node->pdp= &pdps[i];
	}
}
