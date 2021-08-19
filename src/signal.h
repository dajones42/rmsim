//	Interlocking signal state information
//
//Copyright 2009 Doug Jones
#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdio.h>
#include <string>
#include <vector>
#include <map>

#include "track.h"
#include "commandreader.h"

class Signal {
 public:
	enum SignalState { CLEAR, STOP, APPROACH, APPROACHMEDIUM, MEDIUMCLEAR,
	  MEDIUMAPPROACH, RESTRICTING };
	enum SignalColor { RED, GREEN, YELLOW };
 private:
	int state;//interlocking state
	int indication;
	std::vector<Track::Location> tracks;
	void setIndication(int ind);
 public:
	Signal(int s=STOP);
	float trainDistance;
	bool distant;
	int getState() { return state; };
	bool isDistant() { return distant; };
	void setState(int s) { state= s; };
	void addTrack(Track::Location* loc);
	int getNumTracks() { return tracks.size(); };
	Track::Location& getTrack(int i) { return tracks[i]; };
	int getIndication() {return indication; }
	void update();
	int getColor(int head) {
		head++;
		if (head == 1) {
			if (indication==CLEAR)
				return GREEN;
			else if (indication==APPROACH ||
			  indication==APPROACHMEDIUM)
				return YELLOW;
			else
				return RED;
		} else if (head == 2) {
			if (indication==CLEAR ||
			  indication==APPROACHMEDIUM ||
			  indication==MEDIUMCLEAR)
				return GREEN;
			else
				return RED;
		} else {
			if (indication==RESTRICTING)
				return YELLOW;
			else
				return RED;
		}
	}
};
typedef std::list<Signal*> SignalList;
typedef std::map<std::string,Signal*> SignalMap;
extern SignalMap signalMap;

struct SignalParser: public CommandBlockHandler {
	Signal* signal;
	SignalParser() {
		signal= NULL;
	};
	virtual bool handleCommand(CommandReader& reader);
	virtual void handleBeginBlock(CommandReader& reader);
	virtual void handleEndBlock(CommandReader& reader);
};

struct MSTSSignal {
	std::vector<Signal*> units;
};

Signal* findSignal(Track::Vertex* v, Track::Edge* e);

#endif
