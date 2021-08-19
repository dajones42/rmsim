
#include "rmsim.h"
#include "rmssocket.h"
#include "parser.h"
#include "changelog.h"
#include <iostream>
#include <sstream>
#include <iomanip>

bool RMSSocket::initCalled= false;

RMSSocket::RMSSocket(const char* server)
{
	if (!initCalled) {
		netInit();
		initCalled= true;
	}
	isServer= server==NULL;
	open(false);
	if (isServer) {
		bind("",5000);
	} else {
		serverAddress= netAddress(server,5000);
	}
	setBlocking(false);
	lastUpdateTime= 0;
	lastUpdateSimTime= 0;
	lastUpdateRequest= -100;
	roundTripTime= 1;
}

RMSSocket::~RMSSocket()
{
	close();
}

void RMSSocket::update(double elapsed)
{
	const int bufSize= 65536;
	char buf[bufSize];
	netAddress addr;
	int sz= recvfrom(buf,bufSize,0,&addr);
	if (sz > 0) {
//		fprintf(stderr,"got command %s from %s:%d\n",
//		  buf,addr.getHost(),addr.getPort());
		for (char* p=buf; p!=NULL; ) {
			char* p1= strchr(p,'\n');
			if (p1 == NULL)
				break;
			*p1= '\0';
			if (isServer)
				handleServerCommand(p,&addr);
			else
				handleClientCommand(p,elapsed);
			p= p1+1;
		}
	}
	if (!isServer && elapsed-lastUpdateRequest>4*roundTripTime) {
		netAddress toAddr= serverAddress;
		ostringstream os;
		os<<"requestupdate|"<<lastUpdateSimTime<<"\n";
		sendto(os.str().c_str(),os.str().size()+1,0,&toAddr);
		lastUpdateRequest= elapsed;
	}
}

void RMSSocket::handleServerCommand(const char* cmd, netAddress* addr)
{
	Parser parser;
	parser.setDelimiters("|");
	parser.setCommand(cmd);
	try {
		if (parser.getString(0)=="requestupdate") {
			ostringstream os;
			os<<"update|"<<std::fixed<<std::setprecision(3)<<
			  simTime<<"|"<<timeMult<<"\n";
#if 0
			Track* track= trackMap.begin()->second;
			os<<"switchupdate";
			for (Track::SwitchMap::iterator
			  i=track->switchMap.begin();
			  i!=track->switchMap.end(); i++) {
				Track::SwVertex* sw= i->second;
				os<<"|"<<(sw->edge2==sw->swEdges[1]);
			}
			os<<"\n";
#else
			ChangeLog* log= ChangeLog::instance();
			for (ChangeLog::iterator
			  i=log->find(parser.getDouble(1,0,1e10)-1);
			  i!=log->end(); i++)
				os<<i->second<<"\n";
#endif
			for (TrainList::iterator i=trainList.begin();
			  i!=trainList.end(); i++) {
				Train* t= *i;
				os<<"trainupdate|"<<t->id;
				os<<"|"<<t->location.edge->ssEdge->block;
				os<<"|"<<t->location.edge->ssOffset+
				  t->location.offset;
				os<<"|"<<t->location.rev;
				os<<"|"<<t->speed<<"|"<<t->accel;
				os<<"|"<<t->tControl;
				os<<"\n";
//				fprintf(stderr,"%s %f %f %f\n",
//				  t->name.c_str(),t->location.offset,
//				  t->location.edge->ssOffset,
//				  t->location.edge->length);
			}
			string s= os.str();
			sendto(s.c_str(),s.size()+1,0,addr);
		}
	} catch (const char* msg) {
		cerr<< msg<<"\n";
	} catch (const std::exception& error) {
		cerr<< error.what()<<"\n";
	}
}

void RMSSocket::handleClientCommand(const char* cmd, double elapsed)
{
	Parser parser;
	parser.setDelimiters("|");
	parser.setCommand(cmd);
	try {
		if (parser.getString(0)=="update") {
			simTime= parser.getDouble(1,0,1e20);
			timeMult= parser.getInt(2,0,128);
			lastUpdateSimTime= simTime;
			lastUpdateTime= elapsed;
			roundTripTime= .9*roundTripTime +
			  .1*(lastUpdateTime-lastUpdateRequest);
		} else if (parser.getString(0)=="trainupdate") {
			Train* t=
			  Train::findTrain(parser.getInt(1,0,0x7fffffff));
			if (t == NULL)
				throw std::invalid_argument(
				  "cannot find train");
			t->remoteControl= 1;
			t->speed= parser.getDouble(5,-1000,1000);
			t->accel= parser.getDouble(6,-1000,1000);
			t->tControl= parser.getDouble(7,0,1);
			Track::SSEdge* sse=
			  findTrackSSEdge(parser.getInt(2,0,100000));
			if (sse == NULL)
				throw std::invalid_argument(
				  "cannot find SSEdge");
			t->setPositionError(sse,parser.getDouble(3,0,1e10),
			  parser.getInt(4,0,1));
		} else if (parser.getString(0)=="switchupdate") {
			Track* track= trackMap.begin()->second;
			int n= 1;
			for (Track::SwitchMap::iterator
			  i=track->switchMap.begin();
			  i!=track->switchMap.end(); i++) {
				Track::SwVertex* sw= i->second;
				int j= parser.getInt(n++,0,1);
				if (sw->edge2 != sw->swEdges[j])
					sw->throwSwitch(sw->swEdges[j],true);
			}
		} else if (parser.getString(0)=="throw") {
			Track* track= trackMap.begin()->second;
			Track::SwitchMap::iterator i=track->switchMap.find(
			  parser.getInt(1,0,0x7fffffff));
			if (i == track->switchMap.end())
				throw std::invalid_argument(
				  "cannot find switch");
			Track::SwVertex* sw= i->second;
			int j= parser.getInt(2,0,1);
			if (sw->edge2 != sw->swEdges[j])
				sw->throwSwitch(sw->swEdges[j],true);
		} else if (parser.getString(0)=="create") {
			Train* t=
			  Train::findTrain(parser.getInt(1,0,0x7fffffff));
			if (t == NULL)
				return;
			trainList.push_back(t);
			listener.addTrain(t);
			t->setModelsOn();
			t->setOccupied();
		} else if (parser.getString(0)=="destroy") {
			Train* t=
			  Train::findTrain(parser.getInt(1,0,0x7fffffff));
			if (t == NULL)
				throw std::invalid_argument(
				  "cannot find train");
			trainList.remove(t);
			listener.removeTrain(t);
			t->setModelsOff();
			t->clearOccupied();
		} else if (parser.getString(0)=="couple") {
			Train* t1=
			  Train::findTrain(parser.getInt(1,0,0x7fffffff));
			if (t1 == NULL)
				throw std::invalid_argument(
				  "cannot find train");
			Train* t2=
			  Train::findTrain(parser.getInt(2,0,0x7fffffff));
			if (t2 == NULL)
				return;//assume already coupled
			t1->otherTrain= t2;
			float d= t1->otherDist(&t1->location);
			float d1= t1->otherDist(&t1->endLocation);
			if ((d1>0 && d1<d) || (d1<0 && d1>d))
				d= d1;
			if (t1->otherTrain==t2 && d<1e10) {
				t2->location.move(d,1,1);
				t2->endLocation.move(d,1,-1);
				for (RailCarInst* c=t2->firstCar; c!=NULL;
				  c=c->next)
					c->move(d);
				t1->coupleOther();
			}
		} else if (parser.getString(0)=="uncouple") {
			Train* t1=
			  Train::findTrain(parser.getInt(1,0,0x7fffffff));
			if (t1 == NULL)
				throw std::invalid_argument(
				  "cannot find train");
			int nCars= parser.getInt(2,0,1000);
			int keepRear= parser.getInt(3,0,1);
			int newID= parser.getInt(4,0,0x7fffffff);
			Train* t2= Train::findTrain(newID);
			if (t2 != NULL)
				return;//assume already uncoupled
			t1->uncouple(nCars,keepRear,newID);
		} else if (parser.getString(0)=="randomselection") {
			Train* t=
			  Train::findTrain(parser.getInt(1,0,0x7fffffff));
			if (t == NULL)
				throw std::invalid_argument(
				  "cannot find train");
			int n= parser.getInt(2,0,1000);
			for (RailCarInst* c=t->firstCar; c!=NULL; c=c->next)
				n--;
			if (n != 0)
				return;//done already?
			int nEng= parser.getInt(3,0,1000);
			int nCab= parser.getInt(4,0,1000);
			std::list<int> carList;
			for (int i=5; i<parser.getNumTokens(); i++)
				carList.push_back(parser.getInt(i,0,1000));
			t->clearOccupied();
			t->selectCars(nEng,nCab,carList);
			t->positionCars();
			t->setOccupied();
		}
	} catch (const char* msg) {
		cerr<< msg<<"\n";
	} catch (const std::exception& error) {
		cerr<< error.what()<<"\n";
	}
}
