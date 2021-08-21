//	class for converting text into morse code sounds
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
