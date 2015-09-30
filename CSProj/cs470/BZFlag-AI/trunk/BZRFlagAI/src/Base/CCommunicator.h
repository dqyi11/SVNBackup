/*
 * CCommunicator.h
 *
 *  Created on: Sep 25, 2012
 *      Author: walter
 */

#ifndef CCOMMUNICATOR_H_
#define CCOMMUNICATOR_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <string>
#include <vector>

using namespace std;

const int kBufferSize = 1024;

class CWorldController;
class COccGridState;

class CCommunicator {
	friend class CWorldController;
public:
	CCommunicator(const char *host, int port);
	virtual ~CCommunicator();

	// Self check
	int GetPort(){return nPort;}
	const char *GetHost() {return pcHost;}
	bool GetStatus() {return InitStatus;}

	int Close();

	bool shoot(int index);
	bool speed(int index, double value);
	bool accelx(int index, double value);
	bool angvel(int index, double value);
	bool accely(int index, double value);
	bool occgrid(int index, COccGridState & state);

	int init();
protected:

	int handShake();
	int sendLine(const char *LineText);
	int readReply(char *Reply);
	void readLine(char *LineText);
	void resetReplyBuffer();

	vector <string>  readArr();
	bool readBool();
	void readAck();
	void printLine();

private:
	int sd;
	const char * pcHost;
	int nPort;
	char ReplyBuffer[kBufferSize];
	bool InitStatus;
	int LineFeedPos;
	int Start;
};

class CSplitString {
	vector <string> MyVector;
	string MyString;
public:
	CSplitString(string str);
	vector <string> split() ;
};

#endif /* CCOMMUNICATOR_H_ */
