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
#ifndef RMSSOCKET_H
#define RMSSOCKET_H

#include <plib/net.h>

class RMSSocket : public netSocket {
	static bool initCalled;
	bool isServer;
	netAddress serverAddress;
	double lastUpdateTime;
	double lastUpdateSimTime;
	double lastUpdateRequest;
	float roundTripTime;
	void handleServerCommand(const char* p, netAddress* addr);
	void handleClientCommand(const char* p, double elapsed);
 public:
	RMSSocket(const char* server);
	~RMSSocket();
	void update(double elapsed);
	float getRTT() { return isServer ? 0 : roundTripTime; };
};

#endif
