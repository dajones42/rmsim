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
