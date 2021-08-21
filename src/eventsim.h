//	Templates for simple event simulation
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
#ifndef EVENTSIM_H
#define EVENTSIM_H

#include <queue>

namespace tt {

template<class T> class EventSim;

template<class T> struct Event {
	T time;
	virtual void handle(EventSim<T>* sim) = 0;
	Event(T t) {
		time= t;
	};
};

template<class T> struct EventPointer {
	Event<T>* event;
	EventPointer(Event<T>* e) {
		event= e;
	};
};

template<class T> inline bool operator<(const EventPointer<T> ep1,
  const EventPointer<T> ep2) {
	return ep1.event->time > ep2.event->time;
}

template<class T> class EventSim {
	std::priority_queue<EventPointer<T> > events;
 public:
	void processNextEvent() {
		EventPointer<T> ep= events.top();
		events.pop();
		ep.event->handle(this);
		delete ep.event;
	}
	void processEvents(T time) {
		while (events.size()>0 && events.top().event->time<=time)
			processNextEvent();
	}
	void schedule(Event<T>* e) {
		events.push(EventPointer<T>(e));
	}
	T getNextEventTime() {
		return events.size()>0 ? events.top().event->time : 0;
	}
};

}

#endif
