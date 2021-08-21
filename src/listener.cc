//	sound related functions
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
#include <plib/sl.h>
#include "rmsim.h"

using namespace std;

struct SoundTableEntry {
	float min;
	float max;
	ALuint buffer;
	SoundTableEntry(float mn, float mx, ALuint buf) {
		min= mn;
		max= mx;
		buffer= buf;
	};
};

//	sound control curve for MSTS
struct ControlCurve {
	enum { SPEED, VAR2, VAR2A } control;
	Spline<float> curve;
	float getValue(Train* train) {
		float x;
		switch (control) {
		 case SPEED:
			x= train->speed;
			if (x < 0)
				x= -x;
			break;
		 case VAR2:
			x= train->tControl;
			break;
		 case VAR2A:
			x= 100*train->tControl;
			break;
		}
		return curve(x);
	}
};

struct SoundControl {
	int currentSound;
	vector<SoundTableEntry> soundTable;
	ControlCurve* volCurve;
	ControlCurve* freqCurve;
	SoundControl() {
		volCurve= NULL;
		freqCurve= NULL;
	};
	~SoundControl() {
		if (volCurve)
			delete volCurve;
		if (freqCurve)
			delete freqCurve;
	};
};

Listener listener;

Listener::~Listener()
{
	for (multimap<Train*,RailCarSound>::iterator i=railcars.begin();
	  i!=railcars.end(); ++i) {
		alSourceStop(i->second.source);
		if (i->second.soundControl)
			delete i->second.soundControl;
	}
	if (morseSource) {
		alSourceStop(morseSource);
		cleanupMorse();
		alDeleteSources(1,&morseSource);
	}
	alcMakeContextCurrent(NULL);
	if (context)
		alcDestroyContext(context);
	if (device)
		alcCloseDevice(device);
}

void Listener::init()
{
	device= alcOpenDevice(NULL);
	if (!device)
		return;
	context= alcCreateContext(device,NULL);
	if (!context)
		return;
	alcMakeContextCurrent(context);
	alListenerf(AL_GAIN,1);
}

ALuint Listener::findBuffer(string& file)
{
	map<string,ALuint>::iterator i=bufferMap.find(file);
	if (i != bufferMap.end())
		return i->second;
	slSample sample(file.c_str());
	if (sample.getLength() == 0)
		return 0;
	ALuint buf= makeBuffer(&sample);
	bufferMap[file]= buf;
	return buf;
}
	
ALuint Listener::makeBuffer(slSample* sample)
{
	if (sample->getBps() == 16) {
		int n= sample->getLength()/2;
		unsigned short* b2= (unsigned short*) sample->getBuffer();
		for (int i=0; i<n; i++)
			b2[i]= b2[i] - 32768;
	}
	ALuint buf;
	alGenBuffers(1,&buf);
	ALenum format= AL_FORMAT_MONO8;
	if (sample->getStereo()) {
		if (sample->getBps() == 8)
			format= AL_FORMAT_STEREO8;
		else
			format= AL_FORMAT_STEREO16;
	} else {
		if (sample->getBps() == 8)
			format= AL_FORMAT_MONO8;
		else
			format= AL_FORMAT_MONO16;
	}
	alBufferData(buf,format,sample->getBuffer(),sample->getLength(),
	  sample->getRate());
//	fprintf(stderr,"alBufferData(%d,%d,%p,%d,%d) %d %d\n",
//	  buf,format,sample->getBuffer(),sample->getLength(),
//	  sample->getRate(),sample->getStereo(),sample->getBps());
	return buf;
}

//	sets the location and orientation of the listener and trains
void Listener::update(osg::Vec3d position, float cosa, float sina)
{
	if (!device)
		return;
	ALfloat v[6];
	v[0]= position[0];
	v[1]= position[1];
	v[2]= position[2];
	alListenerfv(AL_POSITION,v);
	v[0]= v[1]= v[2]= 0;
	alListenerfv(AL_VELOCITY,v);
	v[0]= cosa;
	v[1]= sina;
	v[2]= 0;
	v[3]= 0;
	v[4]= 0;
	v[5]= 1;
	alListenerfv(AL_ORIENTATION,v);
	for (multimap<Train*,RailCarSound>::iterator i=railcars.begin();
	  i!=railcars.end(); ++i) {
		RailCarInst* c= i->second.car;
		RailCarInst::LinReg* lr= c->linReg[c->def->parts.size()-1];
		v[0]= lr->ax;
		v[1]= lr->ay;
		v[2]= lr->az;
		alSourcefv(i->second.source,AL_POSITION,v);
		float dx= position[0]-v[0];
		float dy= position[1]-v[1];
		float dz= position[2]-v[2];
		if (dx*dx+dy*dy+dz*dz>1e6) {
			alSourceStop(i->second.source);
			continue;
		}
		v[0]= lr->bx*c->speed;
		v[1]= lr->by*c->speed;
		v[2]= lr->bz*c->speed;
		alSourcefv(i->second.source,AL_VELOCITY,v);
		if (i->second.soundControl) {
			SoundControl* sc= i->second.soundControl;
			float s= c->speed/c->getMainWheelRadius();
			if (s < 0)
				s= -s;
//			fprintf(stderr,"sound speed %f\n",s);
			int j= sc->currentSound;
			while (j>0 && s<sc->soundTable[j].min)
				j--;
			while (j<sc->soundTable.size()-1
			  && s>sc->soundTable[j].max)
				j++;
			if (j != sc->currentSound) {
				alSourcei(i->second.source,AL_LOOPING,AL_FALSE);
				alSourceStop(i->second.source);
			}
			int state;
			alGetSourcei(i->second.source,AL_SOURCE_STATE,&state);
			if (state!=AL_PLAYING && sc->soundTable[j].buffer) {
//				fprintf(stderr,"sound change %d %d %f %f %f\n",
//				  sc->currentSound,j,s,
//				  sc->soundTable[j].min,sc->soundTable[j].max);
				alSourcei(i->second.source,AL_BUFFER,
				  sc->soundTable[j].buffer);
				alSourcei(i->second.source,AL_LOOPING,AL_TRUE);
				alSourcePlay(i->second.source);
				sc->currentSound= j;
			}
			if (sc->volCurve) {
				float g= sc->volCurve->getValue(i->first);
				if (g < .1)
					g= .1;
				alSourcef(i->second.source,AL_GAIN,g);
			}
			if (sc->freqCurve &&
			  sc->soundTable[sc->currentSound].buffer) {
				int fq;
				alGetBufferi(
				  sc->soundTable[sc->currentSound].buffer,
				  AL_FREQUENCY,&fq);
				if (fq > 0)
					alSourcef(i->second.source,AL_PITCH,
					  sc->freqCurve->getValue(i->first)/fq);
//				fprintf(stderr,"freqcurve %f %d\n",
//				  sc->freqCurve->getValue(i->first),fq);
			}
		} else {
			if (i->first->tControl < .05)
				alSourcef(i->second.source,AL_GAIN,.6);
			else
				alSourcef(i->second.source,AL_GAIN,
				  .8+.2*i->first->tControl);
			alSourcef(i->second.source,AL_PITCH,
			  1+.5*i->first->tControl+i->second.pitchOffset);
			int state;
			alGetSourcei(i->second.source,AL_SOURCE_STATE,&state);
			if (state != AL_PLAYING)
				alSourcePlay(i->second.source);
		}
	}
}

void Listener::addTrain(Train* train)
{
	if (!device)
		return;
//	fprintf(stderr,"addTrain %s\n",train->name.c_str());
	int n= 0;
	for (RailCarInst* c=train->firstCar; c!=NULL; c=c->next) {
		if (c->def->soundFile.size() <= 0)
			continue;
		if (c->def->soundFile.find(".sms") != string::npos) {
			readSMS(train,c,c->def->soundFile);
			continue;
		}
		ALuint buf= findBuffer(c->def->soundFile);
		if (buf == 0)
			continue;
		ALuint s;
		alGenSources(1,&s);
		railcars.insert(make_pair(train,RailCarSound(c,s,n*.01,NULL)));
		alSourceQueueBuffers(s,1,&buf);
		alSourcei(s,AL_LOOPING,1);
		alSourcef(s,AL_ROLLOFF_FACTOR,.2);
		alSourcef(s,AL_REFERENCE_DISTANCE,3);
		alSourcef(s,AL_MAX_DISTANCE,10000);
		alSourcef(s,AL_GAIN,c->def->soundGain);
		//alSourcePlay(s);
//		fprintf(stderr,"added source %d\n",s);
		n++;
	}
}

void Listener::removeTrain(Train* train)
{
	if (!device)
		return;
	multimap<Train*,RailCarSound>::iterator i=railcars.find(train);
	while (i!=railcars.end() && i->first==train) {
		ALuint s= i->second.source;
		alDeleteSources(1,&s);
		if (i->second.soundControl)
			delete i->second.soundControl;
		++i;
	}
	railcars.erase(train);
}

MorseConverter* Listener::getMorseConverter()
{
	if (morseConverter != NULL)
		return morseConverter;
	alGenSources(1,&morseSource);
	morseConverter= new MorseConverter;
	return morseConverter;
}

void Listener::playMorse(const char* text)
{
//	fprintf(stderr,"playMorse %s %d\n",text,morseSource);
	morseMessage= text;
	if (morseSource==0 || *text=='\0')
		return;
	cleanupMorse();
	slSample* sample= morseConverter->makeSound(text);
	if (sample == NULL)
		return;
	ALuint buf= makeBuffer(sample);
	delete sample;
	alSourcei(morseSource,AL_SOURCE_RELATIVE,AL_TRUE);
	alSource3f(morseSource,AL_POSITION,1,0,0);
	alSourcef(morseSource,AL_REFERENCE_DISTANCE,10);
	alSourcef(morseSource,AL_GAIN,.5);
	alSourceQueueBuffers(morseSource,1,&buf);
	int state;
	alGetSourcei(morseSource,AL_SOURCE_STATE,&state);
	if (state != AL_PLAYING)
		alSourcePlay(morseSource);
}

bool Listener::playingMorse()
{
	if (morseSource == 0)
		return false;
	int state;
	alGetSourcei(morseSource,AL_SOURCE_STATE,&state);
	return state == AL_PLAYING;
}

void Listener::cleanupMorse()
{
	if (morseSource == 0)
		return;
	ALint state;
	alGetSourcei(morseSource,AL_SOURCE_STATE,&state);
	if (state != AL_STOPPED)
		return;
	ALint n;
	alGetSourcei(morseSource,AL_BUFFERS_QUEUED,&n);
	if (n == 0)
		return;
	ALuint* buffers= (ALuint*) malloc(n*sizeof(ALuint));
	alSourceUnqueueBuffers(morseSource,n,buffers);
	alDeleteBuffers(n,buffers);
	free(buffers);
}

ControlCurve* readSMSCurve(MSTSFileNode* curve)
{
	if (curve->children == NULL)
		return NULL;
	MSTSFileNode* points= curve->children->find("CurvePoints");
	if (points==NULL || points->children==NULL ||
	  points->children->value==NULL)
		return NULL;
	ControlCurve* cc= new ControlCurve;
	if (curve->children->find("SpeedControlled"))
		cc->control= ControlCurve::SPEED;
	else if (curve->children->find("Variable2Controlled"))
		cc->control= ControlCurve::VAR2;
//	fprintf(stderr,"cc %p %d\n",cc,cc->control);
	float maxx= 0;
	for (MSTSFileNode* node=points->children->next; node!=NULL;
	  node=node->next) {
		float x= atof(node->value->c_str());
		node= node->next;
		float y= atof(node->value->c_str());
		cc->curve.add(x,y);
//		fprintf(stderr,"curvepoint %f %f\n",x,y);
		if (maxx < x)
			maxx= x;
	}
	cc->curve.compute();
	if (cc->control==ControlCurve::VAR2 && maxx>1)
		cc->control= ControlCurve::VAR2A;
	return cc;
}

void Listener::readSMS(Train* train, RailCarInst* car, string& file)
{
//	fprintf(stderr,"readSMS %s\n",file.c_str());
	int i= file.rfind("/");
	string dir= file.substr(0,i);
//	fprintf(stderr,"dir %s\n",dir.c_str());
	MSTSFile smsFile;
	try {
		smsFile.readFile(file.c_str());
	} catch (const char* msg) {
		fprintf(stderr,"cannot read %s\n",file.c_str());
		return;
	} catch (const std::exception& error) {
		fprintf(stderr,"cannot read %s\n",file.c_str());
		return;
	}
	MSTSFileNode* sms= smsFile.find("Tr_SMS");
//	fprintf(stderr,"sms %p\n",sms);
	if (sms == NULL)
		return;
	MSTSFileNode* sgroup= sms->children->find("ScalabiltyGroup");
//	fprintf(stderr,"sgroup %p\n",sgroup);
	if (sgroup == NULL)
		return;
	MSTSFileNode* streams= sgroup->children->find("Streams");
//	fprintf(stderr,"streams %p\n",streams);
	if (streams == NULL)
		return;
	for (MSTSFileNode* node=streams->children; node!=NULL;
	  node=node->next) {
//		fprintf(stderr,"node %p\n",node->value);
		if (node->value==NULL || *(node->value)!="Stream")
			continue;
		MSTSFileNode* triggers= node->next->children->find("Triggers");
//		fprintf(stderr,"stream %p\n",triggers);
		if (triggers == NULL)
			continue;
		SoundControl* sc= NULL;
		for (MSTSFileNode* trigger=triggers->children; trigger!=NULL;
		  trigger=trigger->next) {
//			fprintf(stderr,"trigger %p\n",trigger->value);
			if (trigger->value==NULL ||
			  *(trigger->value)!="Variable_Trigger")
				continue;
			MSTSFileNode* v1inc=
			  trigger->next->children->find("Variable1_Inc_Past");
//			fprintf(stderr,"vtrigger %p\n",v1inc);
			if (v1inc==NULL || v1inc->value==NULL ||
			  v1inc->next==NULL || v1inc->next->value==NULL)
				continue;
			float v1= atof(v1inc->value->c_str());
			if (v1 < 0)
				continue;
//			fprintf(stderr,"v1inc %f %s %s\n",
//			  v1,v1inc->value->c_str(),
//			  v1inc->next->value->c_str());
			if (*(v1inc->next->value) == "ReleaseLoopRelease") {
				if (sc)
					sc->soundTable[
					  sc->soundTable.size()-1].max= v1;
			} else if (*(v1inc->next->value) == "StartLoop") {
				MSTSFileNode* fnode=
				  v1inc->next->next->children->find("File");
				if (fnode==NULL || fnode->children==NULL ||
				 fnode->children->value==NULL)
					continue;
//				fprintf(stderr,"file %s\n",
//				  fnode->children->value->c_str());
				string path= dir+"/"+*(fnode->children->value);
				ALuint buf= findBuffer(path);
//				fprintf(stderr,"file %s %d\n",path.c_str(),buf);
				if (buf == 0)
					continue;
				if (sc == NULL) {
					sc= new SoundControl;
					sc->soundTable.push_back(
					  SoundTableEntry(0,v1,0));
				}
				sc->soundTable[sc->soundTable.size()-1].max= v1;
				sc->soundTable.push_back(
				  SoundTableEntry(v1,1e10,buf));
			}
		}
		if (sc == NULL)
			continue;
		MSTSFileNode* curve= node->next->children->find("VolumeCurve");
		if (curve)
			sc->volCurve= readSMSCurve(curve);
		curve= node->next->children->find("FrequencyCurve");
		if (curve)
			sc->freqCurve= readSMSCurve(curve);
//		fprintf(stderr,"adding sms source %p\n",sc);
		ALuint s;
		alGenSources(1,&s);
//		fprintf(stderr,"insert\n");
		railcars.insert(make_pair(train,RailCarSound(car,s,0.,sc)));
//		fprintf(stderr,"queue %d\n",sc->soundTable[0].buffer);
		sc->currentSound= 0;
		if (sc->soundTable[0].buffer)
			alSourceQueueBuffers(s,1,&sc->soundTable[0].buffer);
		alSourcei(s,AL_LOOPING,1);
		alSourcef(s,AL_ROLLOFF_FACTOR,.2);
		alSourcef(s,AL_REFERENCE_DISTANCE,3);
		alSourcef(s,AL_MAX_DISTANCE,10000);
		alSourcef(s,AL_GAIN,1);
		//alSourcePlay(s);
//		fprintf(stderr,"added sms source %d\n",s);
	}
}

void Listener::setGain(float g)
{
	alListenerf(AL_GAIN,g);
}
