//	code to draw a line down the center of track
//	used when viewing from a large distance among other things
//	also draws platform marker if there is a timetable
//	and interlocking signal state information

#include "rmsim.h"
#include "trackpathdrawable.h"

osg::BoundingSphere TrackPathDrawable::computeBound() const
{
	osg::BoundingSphere bsphere;
	if (myTrain != NULL) {
		Track::Location& loc= myTrain->location;
		WLocation wl;
		loc.getWLocation(&wl);
		bsphere.set(osg::Vec3(wl.coord[0],wl.coord[1],wl.coord[2]),
		  1000);
	}
	return bsphere;
}

void TrackPathDrawable::drawImplementation(osg::RenderInfo& renderInfo) const
{
	if (draw && timeTable!=NULL) {
		for (int i=0; i<timeTable->getNumRows(); i++) {
			Station* s= (Station*) timeTable->getRow(i);
			glColor3f(1,0,0);
			glBegin(GL_LINES);
			for (int j=0; j<s->locations.size(); j++) {
				WLocation wl;
				s->locations[j].getWLocation(&wl);
				glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]+10);
			}
			glEnd();
		}
	}
	if (drawAll) {
		for (TrackMap::iterator j=trackMap.begin(); j!=trackMap.end();
		  ++j) {
			WLocation wl;
			glColor3f(0,0,0);
			glBegin(GL_LINES);
			for (Track::EdgeList::iterator
			  i=j->second->edgeList.begin();
			  i!=j->second->edgeList.end(); ++i) {
				Track::Edge* e= *i;
				wl= e->v1->location;
				if (e->track->matrix != NULL)
					wl.coord=
					  e->track->matrix->preMult(wl.coord);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]-.1);
				wl= e->v2->location;
				if (e->track->matrix != NULL)
					wl.coord=
					  e->track->matrix->preMult(wl.coord);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]-.1);
			}
			glEnd();
		}
		glPointSize(5.);
		glColor3f(0,0,1);
		glBegin(GL_POINTS);
		for (TrainList::iterator j=trainList.begin();
		  j!=trainList.end(); ++j) {
			Train* t= *j;
			WLocation wl;
			t->location.getWLocation(&wl);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+2);
			t->endLocation.getWLocation(&wl);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+2);
		}
		glEnd();
	}
	if (draw && myTrain!=NULL) {
		Track::Location loc= myTrain->location;
		WLocation wl;
		loc.getWLocation(&wl);
		loc.getWLocation(&wl);
		Track::Vertex* v= loc.rev ? loc.edge->v2 : loc.edge->v1;
		float x= loc.rev ? loc.offset-loc.edge->length : loc.offset;
		glColor3f(0,1,0);
		glBegin(GL_LINE_STRIP);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		for (Track::Edge* e=loc.edge; e!=NULL && x<1000;
		  e=v->nextEdge(e)) {
			x+= e->length;
			v= v==e->v1 ? e->v2 : e->v1;
			if (e->length < .01)
				continue;
			wl= v->location;
			if (e->track->matrix != NULL)
				wl.coord= e->track->matrix->preMult(wl.coord);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+1);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		}
		glEnd();
		loc= myTrain->endLocation;
		loc.getWLocation(&wl);
		v= loc.rev ? loc.edge->v1 : loc.edge->v2;
		x= loc.rev ? loc.offset-loc.edge->length : loc.offset;
		glColor3f(1,0,0);
		glBegin(GL_LINE_STRIP);
		glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		for (Track::Edge* e=loc.edge; e!=NULL && x<1000;
		  e=v->nextEdge(e)) {
			x+= e->length;
			v= v==e->v1 ? e->v2 : e->v1;
			if (e->length < .01)
				continue;
			wl= v->location;
			if (e->track->matrix != NULL)
				wl.coord= e->track->matrix->preMult(wl.coord);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+1);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
		}
		glEnd();
	}
	if (draw)
	for (SignalMap::iterator i=signalMap.begin(); i!=signalMap.end(); ++i) {
	//	fprintf(stderr,"%s %d\n",i->first.c_str(),
	//	  i->second->getNumTracks());
		if (i->second->getState() == Signal::STOP)
			glColor3f(1,0,0);
		else
			glColor3f(0,1,0);
		for (int j=0; j<i->second->getNumTracks(); j++) {
			Track::Location& loc= i->second->getTrack(j);
			WLocation wl;
			loc.getWLocation(&wl);
			glBegin(GL_LINE_STRIP);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]);
			glVertex3d(wl.coord[0],wl.coord[1],wl.coord[2]+1);
			Track::Vertex* v= loc.rev ? loc.edge->v2 : loc.edge->v1;
			float x= loc.rev ? loc.offset-loc.edge->length :
			  -loc.offset;
			for (Track::Edge* e=loc.edge; e!=NULL && x<10;
			  e=v->nextEdge(e)) {
				x+= e->length;
				v= v==e->v1 ? e->v2 : e->v1;
				if (e->length < .01)
					continue;
				wl= v->location;
				if (e->track->matrix != NULL)
					wl.coord= e->track->matrix->preMult(
					  wl.coord);
				glVertex3d(wl.coord[0],wl.coord[1],
				  wl.coord[2]+1);
			}
			glEnd();
		}
	}
}

TrackPathDrawable* trackPathDrawable= NULL;
