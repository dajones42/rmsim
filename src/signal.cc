//	code for interlocking (user controled) signals
//
//Copyright 2009 Doug Jones
#include "signal.h"
#include <stdexcept>

SignalMap signalMap;

Signal::Signal(int s)
{
	state= s;
	indication= s;
	trainDistance= 0;
	distant= false;
};

void Signal::addTrack(Track::Location* loc)
{
	tracks.push_back(*loc);
}

void Signal::update()
{
	WLocation loc1;
	tracks[0].getWLocation(&loc1);
//	fprintf(stderr,"update %p %d %f %f\n",this,state,
//	  loc1.coord[0],loc1.coord[1]);
	if (state == STOP) {
		setIndication(STOP);
		return;
	}
	Track::Edge* e= tracks[0].edge;
	Track::Vertex* v= tracks[0].rev ? e->v1 : e->v2;
	Signal* nextSignal= NULL;
	int occupied= e->occupied;
	bool mainRoute= true;
	int nSw= 0;
	float d= 0;
	while (e!=NULL && nextSignal==NULL) {
		e= v->nextEdge(e);
		if (e == NULL) {
			if (v->type == Track::VT_SWITCH)
				occupied= 100;
			break;
		}
		if (v->type == Track::VT_SWITCH) {
			nSw++;
			Track::SwVertex* sw= (Track::SwVertex*)v;
			if (d<500 && sw->edge2!=sw->swEdges[sw->mainEdge])
				mainRoute= false;
		}
		d+= e->length;
		if (e->occupied) {
//			fprintf(stderr,"occupied %p %d %f\n",e,e->occupied,d);
			occupied= e->occupied;
		}
		int dir= e->v2==v;
		v= e->otherV(v);
		for (SignalList::iterator i=e->signals.begin();
		  i!=e->signals.end(); i++) {
			Signal* s= *i;
			if (s->tracks[0].rev==dir) {
				nextSignal= s;
				break;
			} else {
				WLocation loc2;
				s->tracks[0].getWLocation(&loc2);
//				fprintf(stderr," revSignal %p dist %f %f %f\n",
//				  s,s->tracks[0].dDistance(&tracks[0]),
//				  loc2.coord[0],loc2.coord[1]);
			}
		}
	}
#if 0
	fprintf(stderr," %d %d %d %f\n",occupied,mainRoute,nSw,d);
	if (nextSignal) {
		WLocation loc2;
		nextSignal->tracks[0].getWLocation(&loc2);
		fprintf(stderr,"nextSignal %p %d dist %f %f %f\n",
		  nextSignal,nextSignal->indication,
		  nextSignal->tracks[0].dDistance(&tracks[0]),
		  loc2.coord[0],loc2.coord[1]);
	}
#endif
	if (occupied)
		setIndication(STOP);
	else if (nextSignal == NULL)
		setIndication(CLEAR);
	else if (!mainRoute && nextSignal->indication==CLEAR)
		setIndication(MEDIUMCLEAR);
	else if (!mainRoute)
		setIndication(RESTRICTING);
	else if (nextSignal->indication == STOP)
		setIndication(APPROACH);
	else if (nextSignal->indication==APPROACH ||
	  nextSignal->indication==MEDIUMCLEAR)
		setIndication(APPROACHMEDIUM);
	else
		setIndication(CLEAR);
}

void Signal::setIndication(int ind)
{
	if (indication == ind)
		return;
	WLocation loc1;
	tracks[0].getWLocation(&loc1);
	fprintf(stderr,"setInd %p %d %f %f\n",
	  this,ind,loc1.coord[0],loc1.coord[1]);
	Track::Edge* e= tracks[0].edge;
	for (std::list<Signal*>::iterator i= e->signals.begin();
	  i!=e->signals.end(); i++) {
		Signal* s= *i;
		if (s->tracks[0].rev == tracks[0].rev &&
		  s->tracks[0].distance(&tracks[0])<1)
			s->indication= ind;
	}
	indication= ind;
	Track::Vertex* v= tracks[0].rev ? e->v2 : e->v1;
#if 1
	Signal* prevSignal= findSignal(v,e);
#else
	Signal* prevSignal= NULL;
	while (e!=NULL && prevSignal==NULL) {
		e= v->nextEdge(e);
		if (e == NULL)
			break;
		int dir= e->v1==v;
		v= e->otherV(v);
		for (SignalList::iterator i=e->signals.begin();
		  i!=e->signals.end(); i++) {
			Signal* s= *i;
			if (s->tracks[0].rev==dir) {
				prevSignal= s;
				break;
			}
		}
	}
#endif
	if (prevSignal) {
#if 0
		WLocation loc2;
		prevSignal->tracks[0].getWLocation(&loc2);
		fprintf(stderr,"prevSignal %p dist %f %f %f\n",prevSignal,
		  prevSignal->tracks[0].dDistance(&tracks[0]),
		  loc2.coord[0],loc2.coord[1]);
#endif
		prevSignal->update();
	}
}

Signal* findSignal(Track::Vertex* v, Track::Edge* e)
{
	for (;;) {
		e= v->nextEdge(e);
		if (e == NULL)
			return NULL;
		int dir= e->v1==v;
		v= e->otherV(v);
		for (SignalList::iterator i=e->signals.begin();
		  i!=e->signals.end(); i++) {
			Signal* s= *i;
			if (s->getTrack(0).rev==dir)
				return s;
		}
	}
}

void SignalParser::handleBeginBlock(CommandReader& reader)
{
	signal= new Signal;
	signalMap[reader.getString(1)]= signal;
};

bool SignalParser::handleCommand(CommandReader& reader)
{
//	fprintf(stderr,"sigparser %s\n",reader.getString(0).c_str());
	if (reader.getString(0) == "track") {
		Track::Location loc;
		findTrackLocation(reader.getDouble(1,-1e10,1e10),
		 reader.getDouble(2,-1e10,1e10),reader.getDouble(3,-1e10,1e10),
		 &loc);
		loc.rev= reader.getInt(4,0,1);
		if (loc.edge != NULL) {
			signal->addTrack(&loc);
			loc.edge->signals.push_back(signal);
		}
		return true;
	}
	if (reader.getString(0) == "distant") {
		signal->distant= true;
		return true;
	}
	return false;
};

void SignalParser::handleEndBlock(CommandReader& reader)
{
};
