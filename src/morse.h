//	class for converting text into morse code sounds
//
//Copyright 2009 Doug Jones
#ifndef MORSE_H
#define MORSE_H

enum MorseCodeType { MCT_AMERICAN, MCT_INTERNATIONAL };
enum MorseSounderType { MST_CW, MST_CLICKCLACK };

struct slSample;

struct MorseConverter {
	int wpm;
	MorseCodeType codeType;
	MorseSounderType sounderType;
	int samplesPerSecond;
	float dashLen;
	float dSpaceLen;
	float mSpaceLen;
	float cSpaceLen;
	float wSpaceLen;
	float lLen;
	float zeroLen;
	int cwFreq;
	slSample* click;
	slSample* clack;
	MorseConverter();
	slSample* makeSound(const char* text);
	void addCWSound(unsigned char* buf, int n);
	void addSilence(unsigned char* buf, int n);
	void addClick(unsigned char* buf, int n);
	void addClack(unsigned char* buf, int n);
	void addSample(unsigned char* buf, int max, slSample* sample);
	float getLength(char c);
	void parse(CommandReader& reader);
};

#endif
