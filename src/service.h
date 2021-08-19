#ifndef SERVICE_H
#define SERVICE_H

struct Service {
 public:
	string consistName;
	string pathName;
	Service();
	~Service();
	void readFile(const char* path);
};

#endif
