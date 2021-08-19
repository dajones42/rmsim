//	timetable and train sheet code
//	includes mustWait function for deciding if its safe to move
//	to next station
//
//Copyright 2009 Doug Jones
#include "timetable.h"
#include "mstsroute.h"
extern MSTSRoute* mstsRoute;

#include <stdexcept>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace std;
using namespace tt;

TimeTable::~TimeTable()
{
	for (StationMap::iterator i=stationMap.begin(); i!=stationMap.end();
	 ++i)
		delete i->second;
	for (TrainMap::iterator i=trainMap.begin(); i!=trainMap.end();
	 ++i)
		delete i->second;
}

Station* TimeTable::findStation(string name)
{
	StationMap::iterator i= stationMap.find(name);
	if (i != stationMap.end())
		return i->second;
	for (i=stationMap.begin(); i!=stationMap.end(); ++i)
		if (name == i->second->callSign)
			return i->second;
	return NULL;
}

void TimeTable::addStation(Station* s)
{
	stationMap[s->getName()]= s;
}

Station* TimeTable::addStation(string name)
{
	Station* s= new Station(name);
	addStation(s);
	return s;
}

Train* TimeTable::findTrain(string name)
{
	TrainMap::iterator i= trainMap.find(name);
	return i==trainMap.end() ? NULL : i->second;
}


void TimeTable::addTrain(Train* t)
{
	trainMap[t->getName()]= t;
//	t->name= name;
	t->clas= 1;
	t->readDown= 1;
	t->timeTable= this;
	t->row= 0;
	t->startDelay= 0;
	for (int i=0; i<rows.size(); i++)
		t->times.push_back(TrainTime());
	trains.push_back(t);
}

Train* TimeTable::addTrain(string name)
{
	Train* t= new Train(name);
	addTrain(t);
	return t;
}

Station* Train::getRow(int idx)
{
	return timeTable->rows[idx];
}

void Train::setSchedTime(Station* s, int ar, int lv, int wait)
{
	if (s == NULL)
		return;
	while (timeTable->rows[row] != s) {
		row++;
		if (row >= times.size())
			break;
	}
	if (0<=row && row<times.size()) {
		times[row].schedAr= ar;
		times[row].schedLv= lv;
		times[row].wait= wait;
	}
}

void Train::copyTimes(string& name, int offset)
{
	Train* t= timeTable->findTrain(name);
	if (t == NULL)
		return;
	fprintf(stderr,"copyTimes %s %d %d %d\n",
	  name.c_str(),offset,(int)times.size(),(int)t->times.size());
	for (int i=0; i<t->times.size(); i++) {
		if (t->times[i].schedAr < 0)
			continue;
		times[i].schedAr= t->times[i].schedAr+offset;
		times[i].schedLv= t->times[i].schedLv+offset;
		times[i].wait= t->times[i].wait;
	}
}

void Train::init(int startTime)
{
	int row1= 0;
	while (row1<times.size() && (times[row1].schedLv<0 ||
	   (times[row1].schedAr<startTime && times[row1].schedLv<startTime)))
		row1++;
	if (row1 >= times.size())
		row1= -1;
	int row2= times.size()-1;
	while (row2>0 && (times[row2].schedLv<0 ||
	   (times[row2].schedAr<startTime && times[row2].schedLv<startTime)))
		row2--;
	if (!readDown) {
		int t= row1;
		row1= row2;
		row2= t;
	}
	row= row1;
#if 0
	if (row >= 0) {
		int lv= times[row].schedLv;
		if (row1 < row2) {
			for (int i=row1; i<=row2; i++) {
				if (times[i].schedLv < 0) {
					times[i].schedAr= lv;
					times[i].schedLv= lv;
					times[i].wait= 0;
				} else {
					lv= times[i].schedLv;
				}
					
			}
		} else {
			for (int i=row1; i>=row2; i--) {
				if (times[i].schedLv < 0) {
					times[i].schedAr= lv;
					times[i].schedLv= lv;
					times[i].wait= 0;
				} else {
					lv= times[i].schedLv;
				}
			}
		}
	}
#endif
}

void Train::setArrival(int row, int time)
{
	this->row= row;
	times[row].actualAr= time;
	active= true;
	for (int i=0; i<timeTable->blocks.size(); i++) {
		Block* b= timeTable->blocks[i];
		if (timeTable->rows[row]!=b->station1 &&
		   timeTable->rows[row]!=b->station2)
			continue;
		for (int j=0; j<b->trainTimes.size(); j++) {
			if (b->trainTimes[j].train==this &&
			  b->trainTimes[j].timeEntered>=0 &&
			  b->trainTimes[j].timeCleared<0) {
				b->trainTimes[j].timeCleared= time;
				fprintf(stderr,"block %d cleared %d %d\n",
				  i,j,time);
				return;
			}
		}
	}
}

void Train::setDeparture(int row, int time)
{
	this->row= row;
	times[row].actualLv= time;
	for (int i=0; i<timeTable->blocks.size(); i++) {
		Block* b= timeTable->blocks[i];
		if (timeTable->rows[row]!=b->station1 &&
		   timeTable->rows[row]!=b->station2)
			continue;
		for (int j=0; j<b->trainTimes.size(); j++) {
			if (b->trainTimes[j].train==this &&
			  b->trainTimes[j].timeEntered<0 &&
			  b->trainTimes[j].timeCleared<0) {
				b->trainTimes[j].timeEntered= time;
				fprintf(stderr,"block %d entered %d %d\n",
				  i,j,time);
				return;
			}
		}
	}
}

int Train::getNextRow(float length, int fromRow)
{
	if (fromRow < 0)
		fromRow= row;
	if (readDown) {
		for (int i=fromRow+1; i<times.size(); i++)
			if (times[i].schedAr>=0 && (length==0 ||
			  timeTable->rows[i]->nTracks>1 ||
			  (timeTable->rows[i]->sidings.size()>0 &&
			   timeTable->rows[i]->sidings[0]>=length)))
				return i;
	} else {
		for (int i=fromRow-1; i>=0; i--)
			if (times[i].schedAr>=0 && (length==0 ||
			  timeTable->rows[i]->nTracks>1 ||
			  (timeTable->rows[i]->sidings.size()>0 &&
			  timeTable->rows[i]->sidings[0]>=length)))
				return i;
	}
	active= false;
	return -1;
}

int Train::getAnyRow(int r)
{
	if (r<0 || times[r].schedAr>=0)
		return r;
	Station* s= timeTable->getRow(r);
	for (std::list<int>::iterator i=s->rows.begin(); i!=s->rows.end();
	  ++i) {
		if (times[*i].schedAr >= 0)
			return *i;
	}
	return r;
}

//	returns 1 if train should not leave current location
//	reason for waiting will be set if provided
//	this function only uses information that train crews should know
//	sometimes this causes problems because AI trains aren't as smart
//	as real crews
int Train::mustWait(int time, int fromRow, int toRow, int eta, string* reason)
{
//	fprintf(stderr,"mustWait(%d %d %d %d)\n",time,fromRow,toRow,eta);
	if (row==fromRow && times[fromRow].schedLv > time) {
		if (reason)
			*reason= "rule 92";
		return 1;
	}
	if (toRow < 0)
		return 0;
	int superior= timeTable->downSuperior ? readDown : !readDown;
//	fprintf(stderr,"super %d %d\n",superior,timeTable->downSuperior);
	int meetInferiorAhead= 0;
	for (MeetMap::iterator i=meets.begin(); i!=meets.end(); ++i) {
		if (i->second != timeTable->getRow(fromRow))
			continue;
		Train* t= i->first;
		int fRow= t->getAnyRow(fromRow);
		if (row==fromRow && t->times[fRow].actualAr<0) {
			if (reason)
				*reason= "meet order";
			return 1;
		}
		if (row == fromRow)
			continue;
		if (readDown==t->readDown ? clas>t->clas : !superior)
			return 1;
		meetInferiorAhead= 1;
	}
	if (meetInferiorAhead)
		return 2;
	bool multiTrack= timeTable->getRow(fromRow)->getNumTracks()>1 &&
	  timeTable->getRow(toRow)->getNumTracks()>1;
	int toRow1= readDown ? toRow-1 : toRow+1;
	for (int i=0; i<timeTable->getNumTrains(); i++) {
		Train* t= timeTable->getTrain(i);
		if (t == this)
			continue;
		MeetMap::iterator j= meets.find(t);
		if (j != meets.end())
			continue;
		if (clas<t->clas || t->clas>=EXTRACLASS)
			continue;
//		fprintf(stderr,"other %s\n",t->getName().c_str());
		if (readDown == t->readDown) {
			int fRow= t->getAnyRow(fromRow);
			if (t->times[fRow].actualLv>=0 &&
			  t->times[fRow].actualLv+
			  timeTable->getRule91Delay()>time) {
				int k= toRow;
				while (k != fromRow) {
//					fprintf(stderr,"%d %d %d\n",k,
//					  times[k].schedLv,t->times[k].schedLv);
					if ((times[k].schedLv<0 &&
					  t->times[k].schedLv>=0) ||
					 (times[k].schedLv>=0 &&
					  t->times[k].schedLv<0))
						break;
					k+= (readDown?-1:1);
				}
//				fprintf(stderr,"%d\n",k);
				if (k != fromRow)
					continue;
				if (reason)
					*reason= t->getName()+ " rule 91";
				return 1;
			}
			int tRow= t->getAnyRow(toRow);
			int tRow1= t->getAnyRow(toRow1);
//			fprintf(stderr,"%d %d %d\n",fRow,tRow,tRow1);
			if (t->times[fRow].schedLv>=0 &&
			  t->times[fRow].actualLv<0 &&
			  toRow>=0 && t->times[tRow].schedAr>=0 &&
			  t->times[tRow1].schedAr>=0 &&
			  t->times[tRow1].schedAr<eta &&
			  (clas!=t->clas ||
			   times[fRow].schedLv>t->times[fRow].schedLv)) {
				if (reason)
					*reason= t->getName()+" rule 86";
				fprintf(stderr,"%d %d %d\n",fRow,tRow,tRow1);
				return 1;
			}
		} else {
			if (multiTrack)
				continue;
			if (clas==t->clas && superior)
				continue;
			int fRow= t->getAnyRow(fromRow);
//			fprintf(stderr," not superior\n");
			if (t->times[fRow].actualAr>=0 ||
			  t->times[fRow].schedAr<0)
				continue;
//			fprintf(stderr," not past yet\n");
			int tRow= t->getAnyRow(toRow);
			if (tRow<0 || t->times[tRow].schedLv<0 ||
			  t->times[tRow].schedLv>=eta+(clas==t->clas?0:300))
				continue;
			if (nextTrain == t)
				continue;
			if (reason)
				*reason= t->getName()+" rule S-89";
			return 1;
		}
	}
	return 0;
}

bool Train::hasBlock(int fromRow, int toRow)
{
	fprintf(stderr,"hasblock %d %d\n",fromRow,toRow);
	Block* b= timeTable->findBlock(fromRow,toRow);
	if (b == NULL)
		return true;
	fprintf(stderr,"block %p %s %s %d\n",b,b->station1->name.c_str(),
	  b->station2->name.c_str(),b->trainTimes.size());
	for (int i=0; i<b->trainTimes.size(); i++)
		if (b->trainTimes[i].train == this)
			return true;
	return false;
}

void TimeTable::print(FILE* out)
{
	fprintf(out,"Timetable %s\n",name.c_str());
	for (int i=0; i<trains.size(); i++)
		if (trains[i]->readDown)
			fprintf(out," %5.5s",trains[i]->name.c_str());
	fprintf(out," | %20.20s %2.2s %4.4s |","Stations","CS","Mi");
	for (int i=0; i<trains.size(); i++)
		if (!trains[i]->readDown)
			fprintf(out," %5.5s",trains[i]->name.c_str());
	fprintf(out,"\n");
	for (int i=0; i<rows.size(); i++) {
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (!t->readDown)
				continue;
			if (t->times[i].schedAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].schedLv/3600,
				  t->times[i].schedLv/60%60);
		}
		fprintf(out," | %20.20s %2.2s %4.1f |",
		  rows[i]->name.c_str(),
		  rows[i]->callSign.c_str(),
		  rows[i]->distance);
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (t->readDown)
				continue;
			if (t->times[i].schedAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].schedLv/3600,
				  t->times[i].schedLv/60%60);
		}
		fprintf(out,"\n");
	}
}

void TimeTable::printHorz(FILE* out)
{
	fprintf(out,"Timetable %s\n",name.c_str());
	fprintf(out," %5.5s","CS");
	for (int i=0; i<rows.size(); i++)
		fprintf(out," %5.5s",rows[i]->callSign.c_str());
	fprintf(out,"\n");
	fprintf(out," %5.5s","Miles");
	for (int i=0; i<rows.size(); i++)
		fprintf(out," %5.1f",rows[i]->distance);
	fprintf(out,"\n");
	for (int j=0; j<trains.size(); j++) {
		Train* t= trains[j];
		fprintf(out," %5.5s",t->name.c_str());
		for (int i=0; i<rows.size(); i++) {
			if (t->times[i].schedAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].schedAr/3600,
				  t->times[i].schedAr/60%60);
		}
		fprintf(out,"\n");
		fprintf(out," %5.5s","");
		for (int i=0; i<rows.size(); i++) {
			if (t->times[i].schedAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].schedLv/3600,
				  t->times[i].schedLv/60%60);
		}
		fprintf(out,"\n");
	}
}

void TimeTable::printTimeSheetHorz(FILE* out, bool activeOnly)
{
	fprintf(out,"Timetable %s\n",name.c_str());
	fprintf(out," %5.5s","CS");
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size() != 0)
			fprintf(out," %5.5s",rows[i]->callSign.c_str());
	fprintf(out,"\n");
	fprintf(out," %5.5s","Miles");
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size() != 0)
			fprintf(out," %5.1f",rows[i]->distance);
	fprintf(out,"\n");
	for (int j=0; j<trains.size(); j++) {
		Train* t= trains[j];
		if (activeOnly && !t->active)
			continue;
		fprintf(out," %5.5s",t->name.c_str());
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].actualAr < 0)
				fprintf(out," %5.5s","");
			else if (t->times[i].actualLv < 0)
				fprintf(out," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
		}
		fprintf(out,"\n");
	}
	printBlocks(out);
}

void TimeTable::printTimeSheetHorz2(FILE* out, bool activeOnly)
{
	fprintf(out,"Timetable %s\n",name.c_str());
	fprintf(out," %5.5s","CS");
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size() != 0)
			fprintf(out," %5.5s",rows[i]->callSign.c_str());
	fprintf(out,"\n");
	fprintf(out," %5.5s","Miles");
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size() != 0)
			fprintf(out," %5.1f",rows[i]->distance);
	fprintf(out,"\n");
	for (int j=0; j<trains.size(); j++) {
		Train* t= trains[j];
		if (activeOnly && !t->active)
			continue;
		fprintf(out," %5.5s",t->name.c_str());
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].schedAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].schedLv/3600,
				  t->times[i].schedLv/60%60);
		}
		fprintf(out,"\n");
		fprintf(out," %5.5s","");
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].actualAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
		}
		fprintf(out,"\n");
		fprintf(out," %5.5s","");
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].actualLv < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
		}
		fprintf(out,"\n");
	}
	printBlocks(out);
}

void TimeTable::printTimeSheet(FILE* out, bool activeOnly)
{
	int len= 1;
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size()>0 &&
		  len<rows[i]->name.size())
			len= rows[i]->name.size();
	fprintf(out,"Timetable %s\n",name.c_str());
	for (int i=0; i<trains.size(); i++) {
		Train* t= trains[i];
		if (activeOnly && !t->active)
			continue;
		if (!t->readDown)
			continue;
		fprintf(out," %5.5s",trains[i]->name.c_str());
	}
	fprintf(out," | %*.*s %2.2s %4.4s |",len,len,"Stations","CS","Mi");
	for (int i=0; i<trains.size(); i++) {
		Train* t= trains[i];
		if (activeOnly && !t->active)
			continue;
		if (t->readDown)
			continue;
		fprintf(out," %5.5s",trains[i]->name.c_str());
	}
	fprintf(out,"\n");
	for (int i=0; i<rows.size(); i++) {
		if (rows[i]->callSign.size() == 0)
			continue;
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (!t->readDown)
				continue;
			if (activeOnly && !t->active)
				continue;
			if (t->times[i].actualAr < 0)
				fprintf(out," %5.5s","");
			else if (t->times[i].actualLv < 0)
				fprintf(out," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
		}
		fprintf(out," | %*.*s %2.2s %4.1f |",len,len,
		  rows[i]->name.c_str(),rows[i]->callSign.c_str(),
		  rows[i]->distance);
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (t->readDown)
				continue;
			if (activeOnly && !t->active)
				continue;
			if (t->times[i].actualAr < 0)
				fprintf(out," %5.5s","");
			else if (t->times[i].actualLv < 0)
				fprintf(out," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
		}
		fprintf(out,"\n");
	}
	printBlocks(out);
}

void TimeTable::printTimeSheet2(FILE* out, bool activeOnly)
{
	int len= 1;
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size()>0 &&
		  len<rows[i]->name.size())
			len= rows[i]->name.size();
	fprintf(out,"Timetable %s\n",name.c_str());
	for (int i=0; i<trains.size(); i++) {
		Train* t= trains[i];
		if (activeOnly && !t->active)
			continue;
		if (!t->readDown)
			continue;
		fprintf(out," %5.5s",trains[i]->name.c_str());
	}
	fprintf(out," | %*.*s %2.2s %4.4s |",len,len,"Stations","CS","Mi");
	for (int i=0; i<trains.size(); i++) {
		Train* t= trains[i];
		if (activeOnly && !t->active)
			continue;
		if (t->readDown)
			continue;
		fprintf(out," %5.5s",trains[i]->name.c_str());
	}
	fprintf(out,"\n");
	for (int i=0; i<rows.size(); i++) {
		if (rows[i]->callSign.size() == 0)
			continue;
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (!t->readDown)
				continue;
			if (activeOnly && !t->active)
				continue;
			if (t->times[i].actualAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
		}
		fprintf(out," | %*.*s %2.2s %4.1f |",len,len,
		  rows[i]->name.c_str(),rows[i]->callSign.c_str(),
		  rows[i]->distance);
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (t->readDown)
				continue;
			if (activeOnly && !t->active)
				continue;
			if (t->times[i].actualLv < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
		}
		fprintf(out,"\n");
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (!t->readDown)
				continue;
			if (activeOnly && !t->active)
				continue;
			if (t->times[i].actualLv < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
		}
		fprintf(out," | %*.*s %2.2s %4.4s |",len,len,"","","");
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (t->readDown)
				continue;
			if (activeOnly && !t->active)
				continue;
			if (t->times[i].actualAr < 0)
				fprintf(out," %5.5s","");
			else
				fprintf(out," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
		}
		fprintf(out,"\n");
	}
	printBlocks(out);
}

void TimeTable::printTimeSheet2(string& s)
{
	s= "";
	int len= 1;
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size()>0 &&
		  len<rows[i]->name.size())
			len= rows[i]->name.size();
	for (int i=0; i<trains.size(); i++)
		if (trains[i]->readDown)
			s+= string(6-trains[i]->name.size(),' ')+
			  trains[i]->name;
	s+= " | ";
	s+= string(len-8,' ') + "Stations CS   Mi |";
	for (int i=0; i<trains.size(); i++)
		if (!trains[i]->readDown)
			s+= string(6-trains[i]->name.size(),' ')+
			  trains[i]->name;
	s+= "\n";
	for (int i=0; i<rows.size(); i++) {
		if (rows[i]->callSign.size() == 0)
			continue;
		char buf[10];
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (!t->readDown)
				continue;
			if (t->times[i].actualAr < 0)
				sprintf(buf," %5.5s","");
			else
				sprintf(buf," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
			s+= buf;
		}
		s+= " | ";
		s+= string(len-rows[i]->name.size(),' ') + rows[i]->name;
		s+= string(3-rows[i]->callSign.size(),' ') + rows[i]->callSign;
		sprintf(buf," %4.1f |",rows[i]->distance);
		s+= buf;
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (t->readDown)
				continue;
			if (t->times[i].actualLv < 0)
				sprintf(buf," %5.5s","");
			else
				sprintf(buf," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
			s+= buf;
		}
		s+= "\n";
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (!t->readDown)
				continue;
			if (t->times[i].actualLv < 0)
				sprintf(buf," %5.5s","");
			else
				sprintf(buf," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
			s+= buf;
		}
		s+= " |";
		s+= string(len+10,' ');
		s+= "|";
		for (int j=0; j<trains.size(); j++) {
			Train* t= trains[j];
			if (t->readDown)
				continue;
			if (t->times[i].actualAr < 0)
				sprintf(buf," %5.5s","");
			else
				sprintf(buf," %2.2d.%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
			s+= buf;
		}
		s+= "\n";
	}
}

float TimeTable::distance(int row1, int row2)
{
	float d= rows[row1]->distance - rows[row2]->distance;
	return d<0 ? -d : d;
}

void Station::parse(CommandReader& reader)
{
	if (reader.getString(0) == "station")
		name= reader.getString(1);
	while (reader.getCommand()) {
		try {
			if (reader.getString(0) == "end")
				break;
			if (reader.getString(0) == "call") {
				setCallSign(reader.getString(1));
			} else if (reader.getString(0) == "altname") {
				addAltName(reader.getString(1));
			} else if (reader.getString(0) == "distance") {
				setDistance(reader.getDouble(1,0,1e10));
			} else if (reader.getString(0) == "tracks") {
				setNumTracks(reader.getInt(1,1,100));
			} else if (reader.getString(0) == "siding") {
				addSiding(reader.getDouble(1,0,1e10));
			} else if (reader.getString(0) == "promptforblock") {
				promptForBlock= true;
			} else {
				reader.printError("unknown command");
			}
		} catch (const std::exception& error) {
			reader.printError(error.what());
		}
	}
}

void TimeTable::parse(CommandReader& reader)
{
	if (reader.getString(0) == "timetable")
		name= reader.getString(1);
	while (reader.getCommand()) {
		try {
			if (reader.getString(0) == "end")
				break;
			if (reader.getString(0) == "superior") {
				setSuperior(reader.getString(1)=="down");
			} else if (reader.getString(0) == "ignoreOtherTrains") {
				setIgnoreOther(true);
			} else if (reader.getString(0) == "rule91Delay") {
				setRule91Delay(reader.getInt(1,0,15,10)*60);
				fprintf(stderr,"rule91delay %d\n",
				  getRule91Delay());
			} else if (reader.getString(0) == "station") {
				Station* s= findStation(reader.getString(1));
				if (s == NULL)
					s= addStation(reader.getString(1));
				addRow(s);
				s->parse(reader);
			} else if (reader.getString(0) == "train") {
				Train* t= addTrain(reader.getString(1));
				t->parse(reader);
			} else if (reader.getString(0) == "block") {
				Station* s1= findStation(reader.getString(1));
				Station* s2= findStation(reader.getString(2));
				if (s1==NULL || s2==NULL)
					throw "cannot find station";
				blocks.push_back(
				  new Block(s1,s2,reader.getInt(3,1,6,1)));
			} else {
				reader.printError("unknown command");
			}
		} catch (const std::exception& error) {
			reader.printError(error.what());
		}
	}
}

int parseTime(string s)
{
	int h,m;
	if (sscanf(s.c_str(),"%d:%d",&h,&m) == 2)
		return 3600*h+60*m;
	return 0;
}

void Train::parse(CommandReader& reader)
{
	if (reader.getString(0) == "train")
		name= reader.getString(1);
	while (reader.getCommand()) {
		try {
			if (reader.getString(0) == "end")
				break;
			if (reader.getString(0) == "readup") {
				readDown= 0;
			} else if (reader.getString(0) == "readdown") {
				readDown= 1;
			} else if (reader.getString(0) == "startvisible") {
				startVisible= true;
			} else if (reader.getString(0) == "endvisible") {
				endVisible= true;
			} else if (reader.getString(0) == "class") {
				clas= reader.getInt(1,1,EXTRACLASS);
			} else if (reader.getString(0) == "speed") {
				minSpeed= reader.getDouble(1,0,100,0);
			} else if (reader.getString(0) == "randomstart") {
				int min= reader.getInt(1,0,24*60);
				int max= reader.getInt(2,min,24*60);
				float prob= reader.getDouble(3,0,1,1);
				if (drand48() < prob)
					startDelay=
					  (int)(60*(min+(max-min)*drand48()));
				fprintf(stderr,"%s startdelay %d\n",
				  name.c_str(),startDelay);
			} else if (reader.getString(0) == "offset") {
				int dt= reader.getInt(1,0,24*60);
				for (int i=0; times.size(); i++) {
					if (times[i].schedAr < 0)
						continue;
					times[i].schedAr+= dt;
					times[i].schedLv+= dt;
				}
			} else if (reader.getString(0) == "station") {
				Station* s=
				  timeTable->findStation(reader.getString(1));
				if (s == NULL) {
					reader.printError(
					  "cannot find station");
					continue;
				}
				int lv= parseTime(reader.getString(2));
				int w= reader.getInt(3,0,100000,0);
				int ar= 0;
				if (reader.getNumTokens() == 5)
					ar= parseTime(reader.getString(4));
				if (ar == 0)
					ar= lv-w;
				setSchedTime(s,ar,lv,w);
			} else if (reader.getString(0) == "next") {
				nextTrain=
				  timeTable->findTrain(reader.getString(1));
				if (nextTrain == NULL)
					throw std::invalid_argument(
					  "cannot find train");
				nextTrain->prevTrain= this;
			} else if (reader.getString(0) == "prev") {
				prevTrain=
				  timeTable->findTrain(reader.getString(1));
				if (prevTrain == NULL)
					throw std::invalid_argument(
					  "cannot find train");
				prevTrain->nextTrain= this;
			} else if (reader.getString(0) == "path") {
				if (mstsRoute == NULL)
					throw std::invalid_argument(
					  "msts route required");
				path= mstsRoute->loadPath(reader.getString(1),
				  false);
			} else if (reader.getString(0) == "route") {
				route= reader.getString(1);
			} else if (reader.getString(0) == "copy") {
				string other= reader.getString(1);
				int offset= reader.getInt(2,0,1000);
				if (reader.getNumTokens() == 4)
					offset+= (int)(reader.getInt(3,0,1000)*
					  drand48());
				copyTimes(other,offset);
			} else {
				reader.printError("unknown command");
			}
		} catch (const std::exception& error) {
			reader.printError(error.what());
		}
	}
}

bool TimeTable::addMeet(Train* t1, Train* t2, Station* s)
{
	for (int i=0; i<rows.size(); i++) {
		if (rows[i] != s)
			continue;
		if (t1->times[i].actualLv>=0 ||
		  t2->times[i].actualLv>=0)
			return false;
		if (t1->times[i].wait == 0)
			t1->times[i].wait= 1;
		if (t2->times[i].wait == 0)
			t2->times[i].wait= 1;
	}
	t1->meets[t2]= s;
	t2->meets[t1]= s;
	return true;
}

Block* TimeTable::findBlock(int row1, int row2)
{
	for (int i=0; i<blocks.size(); i++) {
		Block* b= blocks[i];
		if ((row2>row1 && b->station1==rows[row1] &&
		  b->station2==rows[row2]) ||
		  (row2<row1 && b->station1==rows[row2] &&
		  b->station2==rows[row1]))
			return b;
	}
	return NULL;
}

Block* TimeTable::getBlockFor(Train* train, int time)
{
	int row1= train->getCurrentRow();
	int row2= train->getNextRow(0);
	if (row2 < 0)
		return NULL;
	Block* b= findBlock(row1,row2);
	for (int i=0; b==NULL && i<5; i++) {
		row1= row2;
		row2= train->getNextRow(0,row1);
		b= findBlock(row1,row2);
	}
	if (b == NULL)
		return NULL;
	int n= 0;
	for (int i=0; i<b->trainTimes.size(); i++) {
		if (b->trainTimes[i].train == train)
			return b;
		if (b->trainTimes[i].timeCleared == -1)
			n++;
	}
	if (n < b->nTracks) {
		b->trainTimes.push_back(BlockTimes(train,time));
		return b;
	}
	return NULL;
}

void TimeTable::printBlocks(FILE* out)
{
	for (int i=0; i<blocks.size(); i++) {
		Block* b= blocks[i];
		fprintf(out,"Block %s %s\n",b->station1->name.c_str(),
		  b->station2->name.c_str());
		for (int j=0; j<b->trainTimes.size(); j++) {
			fprintf(out,"%s %2.2d:%2.2d",
			  b->trainTimes[j].train->name.c_str(),
			  b->trainTimes[j].timeGiven/3600,
			  b->trainTimes[j].timeGiven/60%60);
			if (b->trainTimes[j].timeEntered >= 0)
				fprintf(out," %2.2d:%2.2d",
				  b->trainTimes[j].timeEntered/3600,
				  b->trainTimes[j].timeEntered/60%60);
			if (b->trainTimes[j].timeCleared >= 0)
				fprintf(out," %2.2d:%2.2d",
				  b->trainTimes[j].timeCleared/3600,
				  b->trainTimes[j].timeCleared/60%60);
			fprintf(out,"\n");
		}
	}
}

void TimeTable::printTimeSheetHtml(string& s)
{
	bool activeOnly= false;
	char buf[100];
	s= "<table>";
#if 0
	s+= <tr><th colspan=2>Timetable ";
	s+= name;
	s+= "</th></tr>\n";
#endif
#if 0
	s+= "<tr><th></th><th>CS</th>";
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size() != 0)
			s+= string("<th>")+rows[i]->callSign+"</th>";
	s+= "</tr>\n";
#endif
#if 0
	s+= "<tr><th></th><th>Miles</th>";
	for (int i=0; i<rows.size(); i++) {
		if (rows[i]->callSign.size() != 0) {
			sprintf(buf,"%.1f",rows[i]->distance);
			s+= string("<th>")+buf+"</th>";
		}
	}
	s+= "</tr>\n";
#endif
	s+= "<tr><th>Train</th><th>Route</th>";
	for (int i=0; i<rows.size(); i++)
		if (rows[i]->callSign.size() != 0)
			s+= string("<th>")+rows[i]->callSign+"</th>";
	s+= "</tr>\n";
	for (int j=0; j<trains.size(); j++) {
		Train* t= trains[j];
		if (activeOnly && !t->active)
			continue;
		if (t->active)
			s+= string("<tr><th rowspan=3>")+t->name+"</th>";
		else
			s+= string("<tr><td rowspan=3>")+t->name+"</td>";
		s+= string("<td rowspan=3>")+t->route+"</td>";
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].schedAr < 0) {
				s+= "<td></td>";
			} else {
				sprintf(buf," %2.2d:%2.2d",
				  t->times[i].schedLv/3600,
				  t->times[i].schedLv/60%60);
				s+= "<td>";
				s+= buf;
				s+= "</td>";
			}
		}
		s+= "</tr>\n";
		s+= "<tr>";
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].actualAr < 0) {
				s+= "<td></td>";
			} else {
				sprintf(buf," %2.2d:%2.2d",
				  t->times[i].actualAr/3600,
				  t->times[i].actualAr/60%60);
				s+= "<td>";
				s+= buf;
				s+= "</td>";
			}
		}
		s+= "</tr>\n";
		s+= "<tr>";
		for (int i=0; i<rows.size(); i++) {
			if (rows[i]->callSign.size() == 0)
				continue;
			if (t->times[i].actualLv < 0) {
				s+= "<td></td>";
			} else {
				sprintf(buf," %2.2d:%2.2d",
				  t->times[i].actualLv/3600,
				  t->times[i].actualLv/60%60);
				s+= "<td>";
				s+= buf;
				s+= "</td>";
			}
		}
		s+= "</tr>\n";
	}
	s+= "</table>\n";
}
