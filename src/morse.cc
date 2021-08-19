//	morse code to slSample code
//
//Copyright 2009 Doug Jones

#include "rmsim.h"
#include <plib/sl.h>

//	converts ascii character into american morse code
const char* amMorseCode(char c)
{
	switch (c) {
	 case 'A': case 'a': return ".-";
	 case 'B': case 'b': return "-...";
	 case 'C': case 'c': return ".. .";
	 case 'D': case 'd': return "-..";
	 case 'E': case 'e': return ".";
	 case 'F': case 'f': return ".-.";
	 case 'G': case 'g': return "--.";
	 case 'H': case 'h': return "....";
	 case 'I': case 'i': return "..";
	 case 'J': case 'j': return "-.-.";
	 case 'K': case 'k': return "-.-";
	 case 'L': case 'l': return "L";
	 case 'M': case 'm': return "--";
	 case 'N': case 'n': return "-.";
	 case 'O': case 'o': return ". .";
	 case 'P': case 'p': return ".....";
	 case 'Q': case 'q': return "..-.";
	 case 'R': case 'r': return ". ..";
	 case 'S': case 's': return "...";
	 case 'T': case 't': return "-";
	 case 'U': case 'u': return "..-";
	 case 'V': case 'v': return "...-";
	 case 'W': case 'w': return ".--";
	 case 'X': case 'x': return ".-..";
	 case 'Y': case 'y': return ".. ..";
	 case 'Z': case 'z': return "... .";
	 case '0': return "0";
	 case '1': return ".--.";
	 case '2': return "..-..";
	 case '3': return "...-.";
	 case '4': return "....-";
	 case '5': return "---";
	 case '6': return "......";
	 case '7': return "--..";
	 case '8': return "-....";
	 case '9': return "-..-";
	 case '.': return "..--..";
	 case ',': return ".-.-";
	 case '?': return "-..-.";
	 case ' ': return NULL;
	 default: return "";
	}
}

//	converts ascii character into international morse code
const char* intMorseCode(char c)
{
	switch (c) {
	 case 'A': return ".-";
	 case 'B': return "-...";
	 case 'C': return "-.-.";
	 case 'D': return "-..";
	 case 'E': return ".";
	 case 'F': return "..-.";
	 case 'G': return "--.";
	 case 'H': return "....";
	 case 'I': return "..";
	 case 'J': return ".---";
	 case 'K': return "-.-";
	 case 'L': return ".-..";
	 case 'M': return "--";
	 case 'N': return "-.";
	 case 'O': return "---";
	 case 'P': return ".--.";
	 case 'Q': return "--.-";
	 case 'R': return ".-.";
	 case 'S': return "...";
	 case 'T': return "-";
	 case 'U': return "..-";
	 case 'V': return "...-";
	 case 'W': return ".--";
	 case 'X': return "-..-";
	 case 'Y': return "-.--";
	 case 'Z': return "--..";
	 case '0': return "-----";
	 case '1': return ".----";
	 case '2': return "..---";
	 case '3': return "...--";
	 case '4': return "....-";
	 case '5': return ".....";
	 case '6': return "-....";
	 case '7': return "--...";
	 case '8': return "---..";
	 case '9': return "----.";
	 case '.': return ".-.-.-";
	 case ',': return "--..--";
	 case '?': return "..--..";
	 case ' ': return NULL;
	 default: return "";
	}
}

MorseConverter::MorseConverter() {
	wpm= 18;
	codeType= MCT_AMERICAN;
	sounderType= MST_CLICKCLACK;
	samplesPerSecond= 8000;
	dashLen= 3;
	dSpaceLen= 1;
	mSpaceLen= 2;
	cSpaceLen= 3;
	wSpaceLen= 7;
	lLen= 5.6;
	zeroLen= 8.9;
	cwFreq= 500;
	click= NULL;
	clack= NULL;
};

//	returns the relative length of a dot, dash etc.
float MorseConverter::getLength(char c)
{
	switch (c) {
	 case '.': return 1;
	 case '-': return dashLen;
	 case ' ': return mSpaceLen-dSpaceLen;
	 case 'L': return lLen;
	 case '0': return zeroLen;
	 default: return 0;
	}
}

//	adds n bytes of silence to buffer
void MorseConverter::addSilence(Uchar* buf, int n)
{
//	fprintf(stderr,"add silence %d\n",n);
	for (int i=0; i<n; i++)
		buf[i]= 0x80;
}

//	added n bytes of carrier wave sound
void MorseConverter::addCWSound(Uchar* buf, int n)
{
	float a= cwFreq*2*3.14159/samplesPerSecond;
//	fprintf(stderr,"add CW %d %d %f\n",n,cwFreq,a);
	for (int i=0; i<n; i++) {
		float x= sin(a*i);
		if (i < 10)
			x*= .1*i;
		else if (n-i < 10)
			x*= .1*(n-i);
		buf[i]= 0x80 + (int)(0x79*x);
	}
}

//	added at most max bytes from a sound sample
void MorseConverter::addSample(Uchar* buf, int max, slSample* sample)
{
	Uchar* cBuf= sample->getBuffer();
	int n= sample->getLength();
	if (n > max)
		n= max;
//	fprintf(stderr,"add sample %d %d\n",n,max);
	for (int i=0; i<n; i++)
		buf[i]= cBuf[i];
	for (int i=n; i<max; i++)
		buf[i]= 0x80;
}

// adds a click sound to buf
void MorseConverter::addClick(Uchar* buf, int n)
{
	if (sounderType==MST_CW || click==NULL)
		addCWSound(buf,n);
	else
		addSample(buf,n,click);
}

// adds a clack sound to buf
void MorseConverter::addClack(Uchar* buf, int n)
{
	if (sounderType==MST_CW || clack==NULL)
		addSilence(buf,n);
	else
		addSample(buf,n,clack);
}

//	converts a text string to morse code sounds
slSample* MorseConverter::makeSound(const char* text)
{
	int nDot= (int) (samplesPerSecond * 1.2 / wpm);
	int nSamples= 0;
	for (int i=0; text[i]!='\0'; i++) {
		const char* code= codeType==MCT_AMERICAN ?
		  amMorseCode(text[i]) : intMorseCode(text[i]);
		if (code == NULL) {
			nSamples+= (int)(nDot*(wSpaceLen-cSpaceLen-dSpaceLen));
			continue;
		}
		if (i > 0)
			nSamples+= (int)(nDot*(cSpaceLen-dSpaceLen));
		for (int j=0; code[j]!='\0'; j++) {
			if (code[j] == ' ') {
				nSamples+= (int)(nDot*getLength(code[j]));
			} else {
				nSamples+= (int)(nDot*getLength(code[j]));
				nSamples+= (int)(nDot*dSpaceLen);
			}
		}
	}
//	fprintf(stderr,"nSamples=%d %s\n",nSamples,text);
	Uchar* buf= (Uchar*) malloc(nSamples*sizeof(Uchar));
	int offset= 0;
	for (int i=0; text[i]!='\0'; i++) {
		const char* code= codeType==MCT_AMERICAN ?
		  amMorseCode(text[i]) : intMorseCode(text[i]);
		if (code == NULL) {
			int n= (int)(nDot*(wSpaceLen-cSpaceLen-dSpaceLen));
			addSilence(buf+offset,n);
			offset+= n;
			continue;
		}
		if (i > 0) {
			int n= (int)(nDot*(cSpaceLen-dSpaceLen));
			addSilence(buf+offset,n);
			offset+= n;
		}
		for (int j=0; code[j]!='\0'; j++) {
			if (code[j] == ' ') {
				int n= (int)(nDot*getLength(code[j]));
				addSilence(buf+offset,n);
				offset+= n;
			} else {
				int n= (int)(nDot*getLength(code[j]));
				addClick(buf+offset,n);
				offset+= n;
				n= (int)(nDot*dSpaceLen);
				addClack(buf+offset,n);
				offset+= n;
			}
		}
	}
	slSample* sound= new slSample(buf,nSamples);
	free(buf);
	return sound;
}

void MorseConverter::parse(CommandReader& reader)
{
	while (reader.getCommand()) {
		try {
			if (reader.getString(0) == "end")
				break;
			if (reader.getString(0) == "wpm") {
				wpm= reader.getInt(1,1,100);
			} else if (reader.getString(0) == "code") {
				codeType= reader.getString(1)=="int" ?
				 MCT_INTERNATIONAL : MCT_AMERICAN;
			} else if (reader.getString(0) == "sounder") {
				sounderType= reader.getString(1)=="cw" ?
				 MST_CW : MST_CLICKCLACK;
			} else if (reader.getString(0) == "samplespersecond") {
				samplesPerSecond= reader.getInt(1,1,100000);
			} else if (reader.getString(0) == "cwfreq") {
				cwFreq= reader.getInt(1,1,100000);
			} else if (reader.getString(0) == "dashlen") {
				dashLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "ditdashspacelen") {
				dSpaceLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "morsespacelen") {
				mSpaceLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "charspacelen") {
				cSpaceLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "wordspacelen") {
				wSpaceLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "llen") {
				lLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "zerolen") {
				zeroLen= reader.getDouble(1,.1,100);
			} else if (reader.getString(0) == "click") {
				click= new
				  slSample(reader.getString(1).c_str());
			} else if (reader.getString(0) == "clack") {
				clack= new
				  slSample(reader.getString(1).c_str());
			} else {
				reader.printError("unknown command");
			}
		} catch (const std::exception& error) {
			reader.printError(error.what());
		}
	}
}
