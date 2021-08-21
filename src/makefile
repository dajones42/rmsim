all: msts.a ../bin/rmsim airbraketest brakevalvetest

HEADERS = rmsim.h water.h ship.h model2d.h texture.h lcpsolver.h \
	geometry.h engine.h prop.h track.h railcar.h train.h trackshape.h \
	mstsshape.h mstsace.h mstsbfile.h floatbridge.h spline.h mstsroute.h \
	ghproj.h timetable.h ttosim.h eventsim.h listener.h \
	animation.h airbrake.h interlocking.h signal.h morse.h person.h \
	locoeng.h dispatcher.h switcher.h ufsets.h caboverlay.h \
	airtank.h brakevalve.h mstswag.h trackpathdrawable.h camera.h \
	ropedrawable.h waterdrawable.h rmssocket.h trackeditor.h \
	mstsfile.h trackdb.h activity.h tsection.h trackpath.h \
	service.h consist.h commandreader.h parser.h changelog.h \
	trigrid.h
OBJS = water.o ship.o model2d.o texture.o engine.o \
	shipsim.o lcpsolver.o rmparser.o parser.o geometry.o prop.o \
	track.o railcar.o train.o trackshape.o mstsroute.o \
	mstsshape.o mstsace.o mstsbfile.o floatbridge.o mstswag.o \
	ghproj.o timetable.o ttosim.o listener.o animation.o airbrake.o \
	interlocking.o signal.o morse.o person.o trigrid.o locoeng.o \
	dispatcher.o changelog.o switcher.o mststerrain.o mstsworld.o \
	caboverlay.o airtank.o brakevalve.o trackeditor.o \
	trackpathdrawable.o camera.o ropedrawable.o waterdrawable.o \
	rmssocket.o webserver.o

FRAMEWORKS = -losg -losgUtil -losgGA \
	-losgDB -losgManipulator -lOpenThreads \
	-losgFX -losgParticle \
	-losgShadow -losgSim -losgTerrain \
	-losgText -losgViewer -losgEarth -lopenal -lGL -lproj

../bin/rmsim: rmsim.o ${OBJS} msts.a
	g++ -o ../bin/rmsim rmsim.o ${OBJS} msts.a \
	  -lplibpu -lplibssgaux -lplibssg -lplibsg -lplibul -lplibfnt \
	  -lplibsl -lplibnet -lpython3.8 -lm -lz -lmicrohttpd ${FRAMEWORKS}

%.o: %.cc ${HEADERS}
	g++ -c $<

msts.a: mstsfile.o trackdb.o activity.o tsection.o llconv.o trackpath.o \
	service.o consist.o
	ar r msts.a mstsfile.o trackdb.o activity.o tsection.o trackpath.o \
	  llconv.o service.o consist.o
	ranlib msts.a

parser.o: parser.cc ${HEADERS}
	g++ -c -I/usr/include/python3.8 parser.cc

airbraketest: airbraketest.o airbrake.o airtank.o brakevalve.o
	g++ -o airbraketest airbraketest.o airbrake.o airtank.o brakevalve.o -lm

brakevalvetest: brakevalvetest.o airbrake.o airtank.o brakevalve.o
	g++ -o brakevalvetest brakevalvetest.o airbrake.o airtank.o \
	  brakevalve.o -lm

