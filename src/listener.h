//	wrapper for openAL sound
//
//Copyright 2009 Doug Jones
#ifndef LISTENER_H
#define LISTENER_H

#include <AL/al.h>
#include <AL/alc.h>
#include "morse.h"

struct SoundControl;

struct Listener {
	struct RailCarSound {
		RailCarInst* car;
		ALuint source;
		ALfloat pitchOffset;
		SoundControl* soundControl;
		RailCarSound(RailCarInst* c, ALuint s, ALfloat p,
		  SoundControl* sc) {
			car= c;
			source= s;
			pitchOffset= p;
			soundControl= sc;
		};
	};
	ALCdevice* device;
	ALCcontext* context;
	multimap<Train*,RailCarSound> railcars;
	map<string,ALuint> bufferMap;
	ALuint morseSource;
	MorseConverter* morseConverter;
	std::string morseMessage;
	Listener() {
		device= NULL;
		context= NULL;
		morseSource= 0;
		morseConverter= NULL;
	};
	~Listener();
	void init();
	void update(osg::Vec3d position, float cosa, float sina);
	void addTrain(Train* train);
	void removeTrain(Train* train);
	ALuint findBuffer(string& file);
	ALuint makeBuffer(slSample* sample);
	MorseConverter* getMorseConverter();
	void playMorse(const char* s);
	void cleanupMorse();
	bool playingMorse();
	std::string& getMorseMessage() { return morseMessage; };
	void readSMS(Train* train, RailCarInst* car, string& file);
	void setGain(float g);
};
extern Listener listener;

#endif
