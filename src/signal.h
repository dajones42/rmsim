//	Interlocking signal state information
//
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
