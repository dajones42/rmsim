//	wrapper for openAL sound
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
	void loadWav(const char* filename, slSample* sample);
};
extern Listener listener;

#endif
