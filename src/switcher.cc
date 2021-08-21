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
#include "rmsim.h"
#include "switcher.h"

void Switcher::update()
{
	if (train==NULL || train->speed!=0 ||
	  train->tControl>0 || train->bControl<1)
		return;
	if (targetCar == NULL)
		findSetoutCar();
	if (targetCar == NULL)
		findPickupCar();
	if (targetCar == NULL) {
		fprintf(stderr,"%d moves\n",moves);
		train->targetSpeed= targetSpeed;
		train= NULL;
		return;
	}
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		for (RailCarInst* car=t->firstCar; car!=NULL; car=car->next) {
			if (car == targetCar)
				targetTrain= t;
		}
	}
	findSPT(NULL,false);
	Track::Vertex* v= NULL;
	Track::Edge* pe= NULL;
	float d1= train->location.getDist();
	float d2= train->endLocation.getDist();
	fprintf(stderr,"tdist %f %f %f %f %d %s\n",d1,d2,d2-d1,train->length,
	  train==targetTrain,targetCar->waybill->destination.c_str());
	float throwDist= 30;
	bool sameWaybill= true;
	for (RailCarInst* c= train->firstCar; c!=NULL; c=c->next)
		if (c->waybill==NULL || c->waybill->priority==0 ||
		  c->waybill->destination!=targetCar->waybill->destination)
			sameWaybill= false;
	if (!sameWaybill && targetTrain == train) {
		RailCarInst* c= train->firstCar;
		for (; c!=NULL && c!=targetCar && !isEngine(c); c=c->next)
			;
		fprintf(stderr,"eng %p %p %p %p\n",
		  c,targetCar,train->firstCar,train->lastCar);
		if (d2<d1 && targetCar!=train->lastCar && c!=targetCar) {
			Track::Edge* e= train->endLocation.edge;
			Track::Vertex* avoid=
			  e->v1->dist<e->v2->dist ? e->v1 : e->v2;
			fprintf(stderr,"avoidr %p %f\n",avoid,avoid->dist);
			findSPT(avoid,true);
			throwDist= 1e5;
		} else if (d1<d2 && targetCar!=train->firstCar &&
		  c==targetCar) {
			Track::Edge* e= train->location.edge;
			Track::Vertex* avoid=
			  e->v1->dist<e->v2->dist ? e->v1 : e->v2;
			fprintf(stderr,"avoidf %p %f\n",avoid,avoid->dist);
			findSPT(avoid,true);
			throwDist= 1e5;
		}
	} else if (d2<d1 && !sameWaybill && !isEngine(train->lastCar)) {
		Track::Edge* e= train->endLocation.edge;
		Track::Vertex* avoid=
		  e->v1->dist<e->v2->dist ? e->v1 : e->v2;
		fprintf(stderr,"avoidr %p %f\n",avoid,avoid->dist);
		findSPT(avoid,true);
		throwDist= 1e5;
	} else if (d1<d2 && !sameWaybill && !isEngine(train->firstCar)) {
		Track::Edge* e= train->location.edge;
		Track::Vertex* avoid=
		  e->v1->dist<e->v2->dist ? e->v1 : e->v2;
		fprintf(stderr,"avoidf %p %f\n",avoid,avoid->dist);
		findSPT(avoid,true);
		throwDist= 1e5;
	}
	d1= train->location.getDist();
	if (d1 >= 1e10) {
		fprintf(stderr,"no avoid path\n");
		findSPT(NULL,false);
		d1= train->location.getDist();
	}
	d2= train->endLocation.getDist();
	fprintf(stderr,"newdist %f %f\n",d1,d2);
	if (d1 < d2) {
		int n= 0;
		pe= train->location.edge;
		v= train->location.rev ? pe->v1 : pe->v2;
		Track::Vertex* pv= v;
		for (;;) {
			Track::Edge* e= v->inEdge;
			if (e==NULL || e==pe)
				break;
			if (v->type==Track::VT_SWITCH &&
			  v->edge1!=e && v->edge1!=pe) {
				n++;
			}
			pv= v;
			v= e->v1==v ? e->v2 : e->v1;
			pe= e;
		}
		if (n%2 == 0) {
			fprintf(stderr,"even dir changes %d\n",n);
			findSPT(pv,false);
			throwDist= 1e5;
			d1= train->location.getDist();
			if (d1 >= 1e10) {
				findSPT(NULL,false);
				d1= train->location.getDist();
			}
			d2= train->endLocation.getDist();
			fprintf(stderr,"newdist %f %f %f\n",d1,d2,d2-d1);
		}
	}
#if 0
	if (d1 < d2) {
		pe= train->location.edge;
		train->nextStopDist= train->location.rev ?
		  train->location.offset : pe->length-train->location.offset;
		v= train->location.rev ? pe->v1 : pe->v2;
		if (targetTrain == train)
			uncoupleTarget(true);
		else
			uncouplePower(true);
	} else {
		pe= train->endLocation.edge;
		train->nextStopDist= !train->endLocation.rev ?
		  train->endLocation.offset :
		  pe->length-train->endLocation.offset;
		v= !train->endLocation.rev ? pe->v1 : pe->v2;
		if (targetTrain == train)
			uncoupleTarget(false);
		else
			uncouplePower(false);
	}
#else
	pe= train->endLocation.edge;
	if (pe->v1->dist < pe->v2->dist) {
		v= pe->v1;
		train->nextStopDist= train->endLocation.offset;
	} else {
		v= pe->v2;
		train->nextStopDist= pe->length - train->endLocation.offset;
	}
#endif
	float clearDist= 0;
	for (;;) {
#if 0
		fprintf(stderr,"v %d %d %.2f %.2f %p %p %p %p\n",
		  v->type,
		  v->type==Track::VT_SWITCH?((Track::SwVertex*)v)->id:0,
		  v->dist,train->nextStopDist,
		  pe,v->edge1,v->edge2,v->inEdge);
#endif
		if (v->type==Track::VT_SWITCH && pe!=NULL &&
		  v->edge1!=pe && v->edge2!=pe) {
			if (train->nextStopDist >
			  throwDist+(d1<d2?train->length:0)) {
				train->nextStopDist-= 10;
				if (d1 < d2)
					train->nextStopDist-= train->length;
				pe= NULL;
				fprintf(stderr,"throw pe too far\n");
				break;
			}
			((Track::SwVertex*)v)->throwSwitch(pe,false);
		}
		Track::Edge* e= v->inEdge;
		if (e==NULL || e==pe)
			break;
		if (v->type==Track::VT_SWITCH && v->edge1!=e && v->edge1!=pe) {
			train->nextStopDist+= 1;//train->length+1;
			pe= NULL;
			clearDist= 50;
			break;
		}
		if (v->type==Track::VT_SWITCH && v->edge1!=e && v->edge2!=e) {
			if (train->nextStopDist >
			  throwDist+(d1<d2?train->length:0)) {
				train->nextStopDist-= 1;
				if (d1 < d2)
					train->nextStopDist-= train->length;
				pe= NULL;
				fprintf(stderr,"throw e too far\n");
				break;
			}
			((Track::SwVertex*)v)->throwSwitch(e,false);
		}
		v= e->v1==v ? e->v2 : e->v1;
		pe= e;
		train->nextStopDist+= e->length;
	}
	if (pe == NULL && d2 < d1)
		train->nextStopDist= -train->nextStopDist;
	if (pe != NULL && targetTrain == train) {
		train->nextStopDist= targetDistance();
	} else if (pe != NULL && d1 < d2) {
//		d1= train->location.dDistance(&targetTrain->location);
//		d2= train->location.dDistance(&targetTrain->endLocation);
//		train->nextStopDist= d1<d2 ? d1+1 : d2+1;
		train->nextStopDist= train->coupleDistance(false);
	} else if (pe != NULL) {
//		d1= train->endLocation.dDistance(&targetTrain->location);
//		d2= train->endLocation.dDistance(&targetTrain->endLocation);
//		train->nextStopDist= d1>d2 ? d1-1 : d2-1;
		train->nextStopDist= train->coupleDistance(true);
	}
	if (train->nextStopDist > 1) {
		float len1= train->length;
		float len2= uncoupledLength();
		if (len1-1>len2 && train->nextStopDist-(len1-len2)<clearDist) {
			fprintf(stderr,"backup %f %f %f %f %f\n",
			  train->nextStopDist,len1,len2,clearDist,
			  train->nextStopDist-len1+len2);
			train->nextStopDist=
			  train->nextStopDist-(len1-len2)-clearDist-1;
		} else {
			if (targetTrain == train)
				uncoupleTarget(true);
			else
				uncouplePower(true);
			train->nextStopDist-= len1-train->length;
		}
	}
	train->targetSpeed= 4.47;
	if (targetTrain==train &&
	  -1<train->nextStopDist && train->nextStopDist<1) {
		targetCar= NULL;
	} else {
		moves++;
		fprintf(stderr,"move %d %f %s %d\n",moves,train->nextStopDist,
		  targetCar->waybill->destination.c_str(),
		  targetTrain==train);
	}
}

void Switcher::findSPT(Track::Vertex* avoid, bool fix)
{
#if 0
	if (avoid == NULL) {
		for (TrainList::iterator i=trainList.begin();
		  i!=trainList.end(); ++i) {
			Train* t= *i;
			for (RailCarInst* car=t->firstCar; car!=NULL;
			  car=car->next) {
				if (car->waybill==NULL ||
				  car->waybill->destination!=train->name)
					continue;
				if (car->waybill->priority == 0) {
					Track::Edge* e= t->endLocation.edge;
					if (e->v1->occupied)
						avoid= e->v2;
					else
						avoid= e->v1;
					fprintf(stderr,"avoid %p\n",avoid);
				}
			}
		}
	}
#endif
	if (targetTrain == train) {
		track->findSPT(destination,100,2,avoid);
	} else {
		track->findSPT(train->location,100,2,avoid);
		float d1= targetTrain->location.getDist();
		float d2= targetTrain->endLocation.getDist();
		fprintf(stderr,"findSPT %p %f %f\n",avoid,d1,d2);
		if (d1 < d2)
			track->findSPT(targetTrain->location,100,2,avoid);
		else
			track->findSPT(targetTrain->endLocation,100,2,avoid);
	}
	if (avoid && fix) {
		Track::Edge* e= avoid->nextEdge(avoid->inEdge);
		if (e) {
			Track::Vertex* v= e->otherV(avoid);
			if (v->inEdge) {
				avoid->dist= v->dist+e->length;
				avoid->inEdge= e;
			}
		}
	}
}

void Switcher::findSetoutCar()
{
	targetCar= NULL;
	targetTrain= NULL;
	float bestD= 1e3;
	int bestP= 0x7fffffff;
	float bestC= -1;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		for (RailCarInst* car=t->firstCar; car!=NULL;
		  car=car->next) {
			if (car->waybill==NULL)
				continue;
			Track::Location loc;
			if (car->waybill->priority<=bestP &&
			  track->findLocation(car->waybill->destination,&loc)) {
				float cap= checkCapacity(t,
				  car->waybill->destination);
				if (cap < 0)
					continue;
				float d= findCarDist(loc,car,t);
				if (d < 0)
					continue;
				if (t != train) {
					float d1= findCarDist(
					  train->endLocation,car,t);
					if (d1 >= 0)
						d= 100*(d+d1);
				}
//				fprintf(stderr,"waybill dist %s %f %d\n",
//				  car->waybill->destination.c_str(),d,
//				  car->waybill->priority);
				if (car->waybill->priority==bestP && d>bestD)
					continue;
				bestD= d;
				bestP= car->waybill->priority;
				bestC= cap;
				targetCar= car;
				destination= loc;
				targetTrain= t;
			}
		}
	}
#if 0
	if (targetCar == NULL)
		return;
	track->findSPT(targetTrain->location,100,2);
	bestD= destination.getDist();
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t==targetTrain || t==train)
			continue;
		float sum= 0;
		for (RailCarInst* car=t->firstCar; car!=NULL;
		  car=car->next) {
			if (car->waybill==NULL ||
			  car->waybill->priority!=bestP ||
			  car->waybill->destination!=
			   targetCar->waybill->destination) {
				sum= 0;
				break;
			}
			sum+= car->def->length;
		}
		if (sum<=0 || sum>bestC)
			continue;
		float d1= t->location.getDist();
		float d2= t->endLocation.getDist();
		if (d1<d2 && d1<bestD) {
			destination= t->location;
			destination.move(-1,0,0);
			bestD= d1;
		} else if (d2<d1 && d2<bestD) {
			destination= t->endLocation;
			destination.move(1,0,0);
			bestD= d2;
		}
	}
#endif
}

void Switcher::findPickupCar()
{
	targetCar= NULL;
	targetTrain= NULL;
	float bestD= 1e3;
	int bestP= 0x7fffffff;
	Train* cTrain= NULL;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		for (RailCarInst* car=t->firstCar; car!=NULL; car=car->next) {
			if (car->waybill==NULL ||
			  car->waybill->destination!=train->name)
				continue;
			if (car->waybill->priority == 0) {
				cTrain= t;
				destination= car->wheels[0].location;
			}
		}
	}
	if (cTrain == NULL)
		return;
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t == cTrain)
			continue;
		for (RailCarInst* car=t->firstCar; car!=NULL;
		  car=car->next) {
			if (car->waybill==NULL ||
			  car->waybill->destination!=train->name)
				continue;
			fprintf(stderr,"waybill pri %s %d %d\n",
			  car->waybill->destination.c_str(),
			  car->waybill->priority,bestP);
			if (car->waybill->priority <= bestP) {
				float d= findCarDist(destination,car,t);
				fprintf(stderr,"waybill dist %s %f\n",
				  car->waybill->destination.c_str(),d);
				if (d < 0)
					continue;
				if (car->waybill->priority==bestP && d>bestD)
					continue;
				bestD= d;
				bestP= car->waybill->priority;
				targetCar= car;
				targetTrain= t;
			}
		}
	}
}

float Switcher::uncoupledLength()
{
	float s= 0;
	if (targetTrain == train) {
		for (RailCarInst* car=train->lastCar;
		  car!=NULL && !isEngine(car) && (car->waybill==NULL ||
		  car->waybill->destination!=targetCar->waybill->destination);
		  car= car->prev)
			s+= car->def->length;
		s= train->length-s;
	} else {
		for (RailCarInst* car=train->firstCar;
		  car!=NULL && isEngine(car); car= car->next)
			s+= car->def->length;
	}
	return s;
}

void Switcher::uncouplePower(bool rear)
{
	if (rear) {
		RailCarInst* car= train->firstCar;
		while (car->next!=NULL && isEngine(car->next))
			car= car->next;
		if (car->next != NULL)
			train->uncouple(car,false);
	} else if (!isEngine(train->firstCar)) {
		RailCarInst* car= train->firstCar;
		while (car->next!=NULL && !isEngine(car->next))
			car= car->next;
		if (car->next != NULL)
			train->uncouple(car,true);
	}
}

void Switcher::uncoupleTarget(bool rear)
{
	if (!rear) {
		RailCarInst* car= train->firstCar;
		while (car!=NULL && !isEngine(car) && (car->waybill==NULL ||
		  car->waybill->destination!=targetCar->waybill->destination))
			car= car->next;
		fprintf(stderr,"uncouple target %d %p %p\n",
		  rear,car,train->firstCar);
		if (car!=NULL && car!=train->firstCar)
			train->uncouple(car->prev,true);
	} else {
		RailCarInst* car= train->lastCar;
		while (car!=NULL && !isEngine(car) && (car->waybill==NULL ||
		  car->waybill->destination!=targetCar->waybill->destination))
			car= car->prev;
		fprintf(stderr,"uncouple target %d %p %p\n",
		  rear,car,train->lastCar);
		if (car!=NULL && car!=train->lastCar)
			train->uncouple(car,false);
	}
}

float Switcher::targetDistance()
{
	float len1= -1;
	float len2= 0;
	float len3= 0;
	float sum= 0;
	for (RailCarInst* car= targetTrain->firstCar; car!=NULL;
	  car=car->next) {
		if (isEngine(car))
			len3= sum;
		if (car->waybill!=NULL &&
		  car->waybill->destination==targetCar->waybill->destination) {
			len2= sum+car->def->length;
			if (len1 < 0)
				len1= sum;
		}
		sum+= car->def->length;
	}
	float mind= train->endLocation.dDistance(&destination);
	fprintf(stderr,"target end dist %f\n",mind);
	if (mind > 1e10) {
		mind= train->location.dDistance(&destination) + train->length;
		fprintf(stderr,"target front dist %f %f\n",mind,train->length);
		if (mind > 1e10) {
			fprintf(stderr,"target no reachable\n");
			return 0;
		}
	}
	if (targetCar->waybill->destination == train->name) {
		float d= train->location.dDistance(&destination);
		if ((mind<0 && d>0) || (mind>0 && d<0))
			return 0;
	}
	if (len3 < len1)
		mind-= (train->length-len1)/2;
	else
		mind-= len2/2;
	fprintf(stderr,"targetdist %.3f %.3f %.3f %.3f %.3f\n",
	  len1,len2,len3,sum,mind);
	if (mind < 0) {
		float d= train->endLocation.maxDistance(true)-.5;
		fprintf(stderr,"max %f %f\n",-d,mind);
		if (mind < -d)
			mind= -d;
		for (TrainList::iterator i=trainList.begin();
		  i!=trainList.end(); ++i) {
			Train* t= *i;
			if (t == train)
				continue;
			float d= train->endLocation.dDistance(&t->location);
			if (d<0 && mind < d-1)
				mind= d-1;
			d= train->endLocation.dDistance(&t->endLocation);
			if (d<0 && mind < d-1)
				mind= d-1;
		}
	} else {
		float d= train->location.maxDistance(false)-.5;
		fprintf(stderr,"max %f %f\n",d,mind);
		if (mind > d)
			mind= d;
		for (TrainList::iterator i=trainList.begin();
		  i!=trainList.end(); ++i) {
			Train* t= *i;
			if (t == train)
				continue;
			float d= train->location.dDistance(&t->location);
			if (d>0 && d<1e10 && mind > d+1)
				mind= d+1;
			d= train->location.dDistance(&t->endLocation);
			if (d>0 && d<1e10 && mind > d+1)
				mind= d+1;
		}
	}
	return mind;
}

float Switcher::checkCapacity(Train* testTrain, string& dest)
{
	Track::Location loc1;
	Track::Location loc2;
	if (track->findLocation(dest,0,&loc1) == 0)
		return 0;
	if (track->findLocation(dest,1,&loc2) == 0)
		return 0;
	float d= loc1.dDistance(&loc2);
	float avail= d<0 ? -d : d;
//	printf("dest %s len %f\n",dest.c_str(),d);
	for (TrainList::iterator i=trainList.begin(); i!=trainList.end(); ++i) {
		Train* t= *i;
		if (t == train)
			continue;
		float d1= loc1.dDistance(&t->location);
		float d2= loc1.dDistance(&t->endLocation);
		if (d<0 && ((d1<d && d2<d) || (d1>0 && d2>0)))
			continue;
		if (d>0 && ((d1<0 && d2<0) || (d1>d && d2>d)))
			continue;
		//printf(" dest train %s %f %f %f\n",
		//  t->name.c_str(),d1,d2,t->length);
		avail-= t->length;
	}
	for (RailCarInst* car= testTrain->firstCar; car!=NULL;
	  car=car->next)
		if (car->waybill!=NULL && car->waybill->destination==dest)
			avail-= car->def->length;
//	printf(" avail %f\n",avail);
	return avail;
}

float Switcher::findCarDist(Track::Location& loc, RailCarInst* car, Train* t)
{
	track->findSPT(loc,100,2);
	float d1= t->location.getDist();
	float d2= t->endLocation.getDist();
//	fprintf(stderr,"car dist %s %.2f %.2f %.2f %.2f %p %p\n",
//	  car->waybill->destination.c_str(),d1,d2,d2-d1,t->length,car,t);
	if (fabs(d2-d1) < t->length-1)
		return -1;
	if (t == train) {
		RailCarInst* c= t->firstCar;
		for (; c!=NULL && c!=car && c->engine==NULL; c=c->next)
			;
		if (d2<d1 && car!=t->lastCar && c!=car) {
			Track::Edge* e= t->endLocation.edge;
			Track::Vertex* avoid=
			  e->v1->dist<e->v2->dist ? e->v1 : e->v2;
//			fprintf(stderr,"avoid %p %f\n",avoid,avoid->dist);
//			track->findSPT(loc,100,2,avoid);
		} else if (d1<d2 && car!=t->firstCar && c==car) {
			Track::Edge* e= t->location.edge;
			Track::Vertex* avoid=
			  e->v1->dist<e->v2->dist ? e->v1 : e->v2;
//			fprintf(stderr,"avoid %p %f\n",avoid,avoid->dist);
//			track->findSPT(loc,100,2,avoid);
		}
	}
	float d= 0;
	int n= 0;
	for (int j=0; j<car->wheels.size(); j++) {
		if (loc.edge->ssEdge == car->wheels[j].location.edge->ssEdge)
			continue;
		d+= car->wheels[j].location.getDist();
		n++;
	}
//	fprintf(stderr,"car dist %s %f %d %f\n",
//	  car->waybill->destination.c_str(),d,n,d/n);
	if (n == 0)
		return -1;
	return d/n;
}
