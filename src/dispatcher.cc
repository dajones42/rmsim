//	classes for dispatching AI trains

#include "rmsim.h"

//	looks for passing sections in AI train paths and divides the track
//	into blocks that can be reserved for AI trains
void Dispatcher::findBlocks()
{
	track= trackMap.begin()->second;
#if 1
	int n= 0;
	for (Track::VertexList::iterator i=track->vertexList.begin();
	  i!=track->vertexList.end(); ++i) {
		Track::Vertex* v= *i;
		if (v->type != Track::VT_SWITCH)
			continue;
		Track::SwVertex* sw= (Track::SwVertex*)v;
		for (int j=0; j<3; j++) {
			if (sw->ssEdges[j]->block < 0)
				sw->ssEdges[j]->block= ++n;
			else if (n < sw->ssEdges[j]->block)
				n= sw->ssEdges[j]->block;
		}
	}
#else
	for (Track::VertexList::iterator i=track->vertexList.begin();
	  i!=track->vertexList.end(); ++i) {
		Track::Vertex* v= *i;
		if (v->type != Track::VT_SWITCH)
			continue;
		Track::SwVertex* sw= (Track::SwVertex*)v;
		for (int j=0; j<3; j++) {
			sw->ssEdges[j]->block= 0;
			sw->ssEdges[j]->occupied= 0;
		}
	}
	for (TrainInfoMap::iterator i=trainInfoMap.begin();
	  i!=trainInfoMap.end(); ++i) {
		for (Track::Path::Node* p=i->second.path->firstNode;
		  p!=NULL; p=p->next) {
			if (p->nextSSEdge != NULL)
				p->nextSSEdge->occupied= 1;
			if (p->nextSidingSSEdge != NULL)
				p->nextSidingSSEdge->occupied= 1;
			for (Track::Path::Node* p1=p->nextSiding;
			  p1!=NULL; p1=p1->nextSiding)
				if (p1->nextSidingSSEdge != NULL)
					p1->nextSidingSSEdge->occupied= 1;
		}
	}
	for (Track::VertexList::iterator i=track->vertexList.begin();
	  i!=track->vertexList.end(); ++i) {
		Track::Vertex* v= *i;
		if (v->type != Track::VT_SWITCH)
			continue;
		Track::SwVertex* sw= (Track::SwVertex*)v;
		sw->dist= 0;
		for (int j=0; j<3; j++)
			if (sw->ssEdges[j]->occupied > 0)
				sw->dist+= 1;
	}
	int n= 0;
	for (TrainInfoMap::iterator i=trainInfoMap.begin();
	  i!=trainInfoMap.end(); ++i) {
		int m= 0;
		for (Track::Path::Node* p=i->second.path->firstNode;
		  p!=NULL; p=p->next) {
			if (p->sw!=NULL && p->sw->dist>2.5)
				m= 0;
			Track::SSEdge* e= p->nextSSEdge;
			if (e != NULL) {
				if (e->block > 0)
					m= e->block;
				else if (m > 0)
					e->block= m;
				else
					m= e->block= ++n;
			}
			e= p->nextSidingSSEdge;
			if (e != NULL) {
				int b1;
				if (e->block > 0)
					b1= e->block;
				else
					b1= e->block= ++n;
				for (Track::Path::Node* p1=p->nextSiding;
				  p1!=NULL; p1=p1->nextSiding) {
					if (p1->sw!=NULL && p1->sw->dist>2.5)
						b1= 0;
					e= p1->nextSidingSSEdge;
					if (e == NULL)
						continue;
					if (e->block > 0)
						b1= e->block;
					else if (b1 > 0)
						e->block= b1;
					else
						b1= e->block= ++n;
				}
			}
		}
	}
#endif
	blockReservations.resize(n+1);
#if 0
	fprintf(stderr,"%d blocks\n",n);
	for (TrainInfoMap::iterator i=trainInfoMap.begin();
	  i!=trainInfoMap.end(); ++i) {
		fprintf(stderr,"train %s\n",i->first->name.c_str());
		for (Track::Path::Node* p=i->second.path->firstNode;
		  p!=NULL; p=p->next) {
			WLocation loc;
			p->loc.getWLocation(&loc);
			fprintf(stderr,"main %d %d %d %.2f %.2f %.2f\n",
			  p->nextSSEdge==NULL?0:p->nextSSEdge->block,
			  p->nextSidingSSEdge==NULL?0:
			  p->nextSidingSSEdge->block,p->type,
			  loc.coord[0],loc.coord[1],loc.coord[2]);
			for (Track::Path::Node* p1=p->nextSiding;
			  p1!=NULL &&p1->nextSiding!=NULL; p1=p1->nextSiding)
				fprintf(stderr," siding %d %d\n",
				  p1->nextSidingSSEdge==NULL?0:
				  p1->nextSidingSSEdge->block,p1->type);
		}
	}
#endif
}

//	Save the path for an AI train for future reference
void Dispatcher::registerPath(Train* train, Track::Path* path)
{
	trainInfoMap.insert(std::make_pair(
	  train,TrainInfo(trainInfoMap.size()+1,path)));
}

Track::Path::Node* Dispatcher::TrainInfo::findBlock(int block)
{
	for (Track::Path::Node* p=firstNode; p!=NULL; ) {
		if (p->nextSSEdge!=NULL && p->nextSSEdge->block==block) {
			firstNode= p;
			return p->next;
		}
		if (p->nextSidingSSEdge!=NULL &&
		  p->nextSidingSSEdge->block==block) {
			firstNode= p;
			return p->nextSiding;
		}
		for (Track::Path::Node* p1=p->nextSiding; p1!=NULL;
		  p1=p1->nextSiding) {
			if (p1->nextSidingSSEdge!=NULL &&
			  p1->nextSidingSSEdge->block==block) {
				firstNode= p1;
				return p1->nextSiding;
			}
		}
		if (p->next==NULL && p->nextSiding!=NULL) {
			while (p->nextSiding != NULL)
				p= p->nextSiding;
		} else {
			p= p->next;
		}
	}
	return NULL;
}

//	Requests movement authorization for an AI train.
//	Checks to see if the blocks up to the next passing point
//	including the siding or main track at that passing point can
//	be reserved.  If the possible the blocks are reserved and the
//	path is setup.  If not the train is told to try again later.
MoveAuth Dispatcher::requestAuth(Train* train)
{
	if (track == NULL)
		findBlocks();
	TrainInfoMap::iterator i= trainInfoMap.find(train);
	if (i == trainInfoMap.end())
		return MoveAuth(0,0);
//	fprintf(stderr,"start reservations\n");
//	for (int i=1; i<blockReservations.size(); i++)
//		if (blockReservations[i] > 0)
//			fprintf(stderr," %d %d\n",i,blockReservations[i]);
	TrainInfo* ti= &(i->second);
	BlockList& blocks= ti->blocks;
	int block= train->location.edge->ssEdge->block;
	int endBlock= train->endLocation.edge->ssEdge->block;
//	fprintf(stderr,"blocks %d %d %d %d\n",block,endBlock,
//	  blockReservations[block],blockReservations[endBlock]);
	if ((blockReservations[block]!=0 &&
	  blockReservations[block]!=ti->id) ||
	  (endBlock!=block && blockReservations[endBlock]!=0 &&
	  blockReservations[endBlock]!=ti->id))
		return MoveAuth(0,60);
	unreserve(&blocks);
	blocks.clear();
	blocks.add(endBlock);
	blocks.add(block);
	reserve(ti->id,&blocks);
	bool checkOtherTrains= true;
	if (ti->stopNode != NULL) {
		float d= 1e30;
		float mind= -.8;
		if (ti->stopNode->type != Track::Path::COUPLE) {
			d= train->location.dDistance(&ti->stopNode->loc);
			mind= -train->getLength()-.8;
		} else if (train->location.edge->occupied > 1) {
			d= train->otherDist(&train->location);
		} else if (train->endLocation.edge->occupied > 1) {
			d= train->otherDist(&train->endLocation);
		}
		fprintf(stderr,"stop node dist %f %f %d %d\n",d,mind,
		  ti->stopNode->type,ti->stopNode->value);
		if (mind<d && d<.8) {
			switch (ti->stopNode->type) {
			 case Track::Path::COUPLE:
				train->coupleOther();
				checkOtherTrains= false;
			 case Track::Path::UNCOUPLE:
				switch (ti->stopNode->value/1000) {
				 case 0:
					train->uncouple(
					  ti->stopNode->value%1000,false);
					break;
				 case 1:
					train->uncouple(
					  -(ti->stopNode->value%1000),false);
					break;
				 case 2:
					train->uncouple(
					  ti->stopNode->value%1000,true);
					break;
				 case 3:
					train->uncouple(
					  -(ti->stopNode->value%1000),true);
					break;
				 default:
					break;
				}
				break;
			 default:
				break;
			}
			ti->firstNode= ti->stopNode;
			ti->stopNode= NULL;
		}
	}
	Track::Path::Node* pn= ti->findBlock(block);
	fprintf(stderr,"find block %d %p\n",block,pn);
	BlockList bl1;
	int hasInterlocking= 0;
	while (pn != NULL) {
		if (pn->sw)
			hasInterlocking|= pn->sw->hasInterlocking;
		if (pn->type == Track::Path::COUPLE)
			checkOtherTrains= false;
		if (pn->next != NULL) {
			if (pn->nextSiding != NULL)
				break;
			bl1.add(pn->nextSSEdge->block);
//			fprintf(stderr,"adding %d %d\n",
//			  pn->nextSSEdge->block,
//			  blockReservations[pn->nextSSEdge->block]);
			pn= pn->next;
		} else if (pn->nextSiding != NULL) {
			bl1.add(pn->nextSidingSSEdge->block);
			pn= pn->nextSiding;
		} else {
			break;
		}
	}
	fprintf(stderr,"new blocks %d %d %p\n",
	  bl1.size(),canReserve(ti->id,&bl1,checkOtherTrains),pn);
	if (pn == NULL) {
		return MoveAuth(0,0);
	}
	if (!hasInterlocking && !canReserve(ti->id,&bl1,checkOtherTrains))
		return MoveAuth(0,60);
	PathAuth pathAuth;
	track->findSPT(train->location,false,ti->path);
	BlockList bl2;
	BlockList bl3;
	if (pn->type == Track::Path::SIDINGSTART) {
		fprintf(stderr,"start siding %d %d\n",
		  pn->nextSSEdge->block,pn->nextSidingSSEdge->block);
		bl2.add(pn->nextSSEdge->block);
		bl3.add(pn->nextSidingSSEdge->block);
		pathAuth.sidingNode= pn;
		Track::Path::Node* p=pn->next;
		for (; p!=NULL && p->type!=Track::Path::SIDINGEND; p=p->next) {
			if (p->nextSSEdge != NULL)
				bl2.add(p->nextSSEdge->block);
			if (p->type == Track::Path::COUPLE)
				checkOtherTrains= false;
		}
		pathAuth.endNode= p;
		fprintf(stderr,"bl2 %d %d\n",
		  bl2.size(),canReserve(ti->id,&bl2,checkOtherTrains));
		for (Track::Path::Node* p=pn->nextSiding;
		  p!=NULL && p->type!=Track::Path::SIDINGEND; p=p->nextSiding) {
			if (p->nextSidingSSEdge != NULL)
				bl3.add(p->nextSidingSSEdge->block);
			if (p->type == Track::Path::COUPLE)
				checkOtherTrains= false;
		}
		fprintf(stderr,"bl3 %d %d\n",
		  bl3.size(),canReserve(ti->id,&bl3,checkOtherTrains));
		bool canUseMain= canReserve(ti->id,&bl2,checkOtherTrains);
		bool canUseSiding= canReserve(ti->id,&bl3,checkOtherTrains);
		if (canUseSiding && (pn->sw->dist>501 || !canUseMain))
			bl2.clear();
		else if (canUseMain)
			bl3.clear();
		else
			return MoveAuth(0,60);
		pathAuth.takeSiding= bl2.size()==0;
	} else {
		pathAuth.endNode= pn;
	}
	blocks.add(bl1);
	blocks.add(bl2);
	blocks.add(bl3);
	if (!hasInterlocking)
		reserve(ti->id,&blocks);
//	fprintf(stderr,"reservations\n");
//	for (int i=1; i<blockReservations.size(); i++)
//		if (blockReservations[i] > 0)
//			fprintf(stderr," %d %d\n",i,blockReservations[i]);
	MoveAuth auth(pn->loc.getDist(),0);
	for (Track::Path::Node* p=ti->firstNode; p!=NULL; p=p->next) {
		if (p->loc.edge->v1->dist<1e10 && (auth.farVertex==NULL ||
		  p->loc.edge->v1->dist>auth.farVertex->dist))
			auth.farVertex= p->loc.edge->v1;
		if (p->loc.edge->v2->dist<1e10 && (auth.farVertex==NULL ||
		  p->loc.edge->v2->dist>auth.farVertex->dist))
			auth.farVertex= p->loc.edge->v2;
//		fprintf(stderr,"farv %p %f %f\n",auth.farVertex,
//		  p->loc.edge->v1->dist,p->loc.edge->v2->dist);
	}
	if (train->location.edge == pn->loc.edge)
		auth.distance= train->location.dDistance(&(pn->loc));
	if (auth.distance<0 || auth.distance>1e10)
		auth.distance= 0;
	if (pn->next==NULL && pn->nextSiding==NULL &&
	  auth.distance>-.5 && auth.distance<.5)
		auth.distance= 0;
	bool beforeSiding= true;
	for (Track::Path::Node* p=ti->firstNode; p!=NULL && p!=pathAuth.endNode;
	  p= p->nextSiding!=NULL ? p->nextSiding : p->next) {
		if (p->type == Track::Path::SIDINGSTART) {
			p= pathAuth.takeSiding ? p->nextSiding : p->next;
			beforeSiding= false;
		}
		if (p!=ti->firstNode && p->type!=Track::Path::OTHER &&
		  p->type!=Track::Path::SIDINGEND) {
			auth.nextNode= p;
			break;
		}
		
	}
	if (pn->type == Track::Path::SIDINGSTART) {
		if (pn->sw->dist>501 && ti->state==TAKESIDING) {
			auth.distance= ti->nextSwitch->dist +
			  train->getLength() + 1;
			auth.updateDistance= 0;
			auth.waitTime= 10;
			train->alignSwitches(pn->sw);
			ti->nextSwitch= pn->sw;
			ti->state= BETWEEN;
			fprintf(stderr,"leave siding %f\n",pn->sw->dist);
		} else if (pn->sw->dist>501) {
			auth.distance= pn->sw->dist-1;
			if (ti->nextSwitch != NULL)
				auth.updateDistance= pn->sw->dist -
				  ti->nextSwitch->dist - train->getLength() - 2;
			if (ti->nextSwitch==NULL || auth.updateDistance<500)
				auth.updateDistance= 500;
			train->alignSwitches(pn->sw);
			ti->nextSwitch= pn->sw;
			ti->state= BETWEEN;
			fprintf(stderr,"main %f\n",pn->sw->dist);
		} else if (pn->sw->dist>10 && bl3.size()>0) {
			auth.distance= pn->sw->dist-1;
			auth.waitTime= 30;
			train->alignSwitches(pn->sw);
			ti->nextSwitch= pn->sw;
			ti->state= BETWEEN;
			fprintf(stderr,"take siding %f\n",pn->sw->dist);
		} else {
			float d= pn->sw->dist;
			Track::SSEdge* sse= pn->nextSSEdge;
			if (bl2.size() > 0) {
				for (;pn->type!=Track::Path::SIDINGEND;
				  pn=pn->next)
					sse= pn->nextSSEdge;
				ti->state= HOLDMAIN;
			} else {
				for (;pn->type!=Track::Path::SIDINGEND;
				  pn=pn->nextSiding)
					sse= pn->nextSidingSSEdge;
				ti->state= TAKESIDING;
			}
			ti->nextSwitch= pn->sw;
			Track::Edge* e= pn->sw->ssEdges[1]==sse ?
			  pn->sw->swEdges[0] : pn->sw->swEdges[1];
			Track::Vertex* v= e->v1==pn->sw ? e->v2 : e->v1;
			auth.distance= v->dist + e->length - 50;
			auth.updateDistance=
			  auth.distance-d-50-train->getLength();
			train->alignSwitches(v);
			fprintf(stderr,"end siding %f %f %f\n",auth.distance,
			  v->dist,e->length);
		}
	}
	if (auth.nextNode == NULL)
		return auth;
	track->alignSwitches(ti->firstNode,auth.nextNode,pathAuth.takeSiding);
//	float d= auth.nextNode->loc.getDist();
//	if (train->location.edge == auth.nextNode->loc.edge)
	float d= train->location.dDistance(&(auth.nextNode->loc));
	if (d>=1e10 || d<0) {
		fprintf(stderr,"try reverse %f %f %f\n",d,
		  train->location.dDistance(&(auth.nextNode->loc)),
		  train->getLength());
//		d= train->location.dDistance(&(auth.nextNode->loc));
		if (d<0 && -d<train->getLength()) {
			d= 0;
		} else if (train->endLocation.edge == auth.nextNode->loc.edge) {
			d= train->endLocation.dDistance(&(auth.nextNode->loc)) +
			  train->getLength();
			fprintf(stderr," same edge %f\n",d);
		} else {
			Track::Location loc= train->endLocation;
			loc.rev= 1-loc.rev;
			track->findSPT(loc,false,ti->path);
			d= -auth.nextNode->loc.getDist();
			fprintf(stderr," diff edge %f\n",d);
		}
		if (d>0 || d<-1e10)
			d= 0;
	}
	fprintf(stderr," %f %f %d\n",auth.distance,d,beforeSiding);
	if (auth.distance>d || beforeSiding) {
		auth.distance= d;
		auth.updateDistance= 0;
		ti->stopNode= auth.nextNode;
		switch (auth.nextNode->type) {
		 case Track::Path::STOP:
			auth.waitTime= auth.nextNode->value;
			break;
		 case Track::Path::REVERSE:
			auth.waitTime= 5;
			break;
		 case Track::Path::COUPLE:
			auth.waitTime= 60;
			for (TrainList::iterator i=trainList.begin();
			  i!=trainList.end(); ++i) {
				Train* t= *i;
				if (t == train)
					continue;
				TrainInfoMap::iterator j= trainInfoMap.find(t);
				if (j != trainInfoMap.end())
					continue;
				if (t->location.edge->ssEdge==
				  ti->stopNode->loc.edge->ssEdge) {
					d= t->location.getDist()+.5;
					fprintf(stderr,"couple dist %f %f\n",
					  auth.distance,d);
					if (auth.distance > d)
						auth.distance= d;
					else if (auth.distance < -d)
						auth.distance= -d;
				}
				if (t->endLocation.edge->ssEdge==
				  ti->stopNode->loc.edge->ssEdge) {
					d= t->endLocation.getDist()+.5;
					fprintf(stderr,"couple dist %f %f\n",
					  auth.distance,d);
					if (auth.distance > d)
						auth.distance= d;
					else if (auth.distance < -d)
						auth.distance= -d;
				}
			}
			break;
		 default:
			auth.waitTime= 60;
			break;
		}
		if (auth.waitTime == 0)
			auth.waitTime= 1;
		//train->alignSwitches(auth.nextNode->loc.edge->v1);
		fprintf(stderr,"next stop %f %d\n",
		  auth.distance,auth.nextNode->type);
		fprintf(stderr," %f %f %f\n",
		  train->location.dDistance(&(auth.nextNode->loc)),
		  train->endLocation.dDistance(&(auth.nextNode->loc)),
		  train->getLength());
	} else {
		auth.nextNode= NULL;
	}
	return auth;
}

void Dispatcher::release(Train* train)
{
	TrainInfoMap::iterator i= trainInfoMap.find(train);
	if (i == trainInfoMap.end())
		return;
	fprintf(stderr,"release %s %d\n",train->name.c_str(),i->second.id);
	unreserve(&i->second.blocks);
	i->second.blocks.clear();
}

void Dispatcher::BlockList::add(int block)
{
	list.insert(block);
}

bool Dispatcher::BlockList::contains(int block)
{
	iterator i= list.find(block);
	return i!=list.end();
}

//	tests to see if the blocks in list bl can be reserved for train id
//	checks to see if blocks are already reserved for another AI train
//	and also to make sure no non-AI trains are on the block
bool Dispatcher::canReserve(int id, BlockList* bl, bool checkOtherTrains)
{
	if (bl->size() == 0)
		return true;
	for (BlockList::iterator i=bl->begin(); i!=bl->end(); ++i)
		if (blockReservations[*i]!=0 && blockReservations[*i]!=id)
			return false;
	if (ignoreOtherTrains || !checkOtherTrains)
		return true;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		TrainInfoMap::iterator j= trainInfoMap.find(t);
		if (j != trainInfoMap.end())
			continue;
		if (bl->contains(t->location.edge->ssEdge->block) ||
		  bl->contains(t->endLocation.edge->ssEdge->block))
			return false;
	}
	return true;
}

//	reserves the blocks in list bl for train id
void Dispatcher::reserve(int id, BlockList* bl)
{
	for (BlockList::iterator i=bl->begin(); i!=bl->end(); ++i)
		  blockReservations[*i]= id;
}

//	unreserves the blocks in list bl
void Dispatcher::unreserve(BlockList* bl)
{
	for (BlockList::iterator i=bl->begin(); i!=bl->end(); ++i) {
//		fprintf(stderr,"unreserve %d was %d\n",
//		  *i,blockReservations[*i]);
		blockReservations[*i]= 0;
	}
}

//	add the blocks in list other to this list
void Dispatcher::BlockList::add(Dispatcher::BlockList& other)
{
	for (BlockList::iterator i=other.list.begin(); i!=other.list.end(); ++i)
		list.insert(*i);
}

//	returns true if the train is on a block reserved for another train
//	used to warn the player
bool Dispatcher::isOnReservedBlock(Train* train)
{
	TrainInfoMap::iterator i= trainInfoMap.find(train);
	int id= i==trainInfoMap.end() ? 0 : i->second.id;
	int block= train->location.edge->ssEdge->block;
	int endBlock= train->endLocation.edge->ssEdge->block;
	return block<blockReservations.size() &&
	  endBlock<blockReservations.size() &&
	  ((blockReservations[block]!=0 && blockReservations[block]!=id) ||
	  (blockReservations[endBlock]!=0 && blockReservations[endBlock]!=id));
}
