//	Templates for simple event simulation
//
//Copyright 2009 Doug Jones
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
