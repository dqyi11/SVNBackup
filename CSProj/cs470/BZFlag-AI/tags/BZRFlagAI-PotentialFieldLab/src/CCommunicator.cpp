/*
 * CCommunicator.cpp
 *
 *  Created on: Sep 25, 2012
 *      Author: walter
 */

#include "CCommunicator.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


CCommunicator::CCommunicator(const char *host, int port)
{
	// TODO Auto-generated constructor stub
	pcHost = host;
	nPort = port;
	if(init()) {
		cout << "BZRC initialization failed." << endl;
		InitStatus=false;
		Close();
	}
	else {
		InitStatus=true;
	}
}

CCommunicator::~CCommunicator()
{
	// TODO Auto-generated destructor stub
}

int CCommunicator::init()
{
	resetReplyBuffer();
    Start=0;

    struct addrinfo *infop = NULL;
    struct addrinfo hint;

    memset(&hint, 0, sizeof(hint));
    hint.ai_family = AF_INET;
    hint.ai_socktype = SOCK_STREAM;

    char port[10];
    snprintf(port, 10, "%d", nPort);

    if (getaddrinfo(pcHost, port, &hint, &infop) != 0) {
        cerr<<"Couldn't lookup host."<<endl;
    	return 1;
    }

    if ((sd = socket(infop->ai_family, infop->ai_socktype,
    				infop->ai_protocol)) < 0) {
    	cerr<<"Couldn't create socket."<<endl;
    	return 1;
    }

	if (connect(sd, infop->ai_addr, infop->ai_addrlen) < 0) {
		cerr<<"Couldn't connect."<<endl;
		close(sd);
	}

	freeaddrinfo(infop);

	if(handShake()==1) {
		cerr << "Handshake failed!" << endl;
		return 1;
	}

	return 0;

}

// Send line to server
int CCommunicator::sendLine(const char *LineText)
{
	int Length=(int)strlen(LineText);
	char Command[kBufferSize];
	strcpy(Command, LineText);
	Command[Length]='\n';
	Command[Length+1]='\0';
#ifdef BZR_DBG
	cout << Command;
#endif
	if (send(sd, Command, Length+1, 0) >= 0) {
		return 0;
	}
	else {
		return 1;
	}
}

// Read line back from server
int CCommunicator::readReply(char *Reply)
{
	char acReadBuffer[kBufferSize];

	int nNewBytes = recv(sd, acReadBuffer, kBufferSize, 0);
	if (nNewBytes < 0) {
		return -1;
	}
	else if (nNewBytes == 0) {
		cerr << "Connection closed by peer." << endl;
		return 0;
	}

	memcpy(Reply, &acReadBuffer, nNewBytes);
	if(nNewBytes!=kBufferSize) {
		Reply[nNewBytes]='\0';
	}

	return nNewBytes;
}

void CCommunicator::readLine(char *LineText)
{

	memset(LineText, '\0', kBufferSize);
	// Only read more from server when done wiht current ReplyBuffer
	if(strlen(ReplyBuffer)==0) {
		char *Reply;
		Reply = ReplyBuffer;
		readReply(Reply);
	}
	int i=0;
	bool done=false;
	while(!done) {
		for(i=LineFeedPos+1; (i<kBufferSize && ReplyBuffer[i]); i++) {
			if(ReplyBuffer[i]=='\n') {
				LineText[i-LineFeedPos-1+Start]='\0';
				LineFeedPos=i;
				Start=0;
				done=true;
				break;
			}
			LineText[i-LineFeedPos-1+Start]=ReplyBuffer[i];
		}
		if(!done) {
				Start = (int)strlen(LineText);
				resetReplyBuffer();
				char *Reply;
				Reply = ReplyBuffer;
				readReply(Reply);
		}
		else {
			if(ReplyBuffer[i]=='\0') {
				done=true;
				Start=0;
				resetReplyBuffer();
			}
		}
	}
}

// Reset the ReplyBuffer
void CCommunicator::resetReplyBuffer()
{
	memset(ReplyBuffer, '\0', kBufferSize);
	LineFeedPos=-1;
}

int CCommunicator::handShake()
{
	char str[kBufferSize];
	char *LineText;
	LineText=str;
	readLine(LineText);
#ifdef BZR_DGB
	cout << LineText << endl;
#endif
	if (!strcmp(LineText, "bzrobots 1")) {
		const char * Command="agent 1";
		int temp=sendLine(Command);
		if(temp==1)
			return 1;
		else
			resetReplyBuffer();
			return 0;
	}
	else
		return 1;
}

// Read line into vector
vector <string> CCommunicator::readArr()
{
	char str[kBufferSize];
	char *LineText=str;
	// cout << "LineText:" << LineText << endl;
	readLine(LineText);
	if(strlen(LineText)!=0) {
#ifdef BZR_DBG
	    cout << LineText << endl;
#endif
	}
	while(strlen(LineText)==0) {
		readLine(LineText);
#ifdef BZR_DBG
		cout << LineText << endl;
#endif
	}
	CSplitString ss=CSplitString(LineText);
	return ss.split();
}

// Read Acknowledgement
void CCommunicator::readAck()
{
	vector <string> v=readArr();
	if(v.at(0)!="ack") {
		cout << "Didn't receive ack! Exit!" << endl;
		exit(1);
	}
}
// Read "ok"
bool CCommunicator::readBool()
{
	vector <string> v=readArr();
	if(v.at(0)=="ok") {
		return true;
	}
	else if(v.at(0)=="fail"){
#ifdef BZR_DBG
		cout << "Received fail. Exiting!" << endl;
#endif
		return false;
	}
	else {
#ifdef BZR_DBG
		cout << "Something went wrong. Exiting!" << endl;
#endif
		return false;
	}
}

// Receive and print another line
void CCommunicator::printLine()
{
	char str[kBufferSize];
	char *LineText=str;
	readLine(LineText);
#ifdef BZR_DBG
	cout << LineText << endl;
#endif
}

int CCommunicator::Close()
{

	close(sd);

	return 0;
}

bool CCommunicator::shoot(int index)
{
	// Perform a shoot request.
	char char_buff[20];
	sprintf(char_buff, " %d", index);
	string str_buff="shoot";
	str_buff.append(char_buff);
	const char *Command = str_buff.c_str();
	sendLine(Command);
	readAck();
	if(readBool()) {
		return true;
	}
	else {
		return false;
	}
}

bool CCommunicator::speed(int index, double value)
{
	// Set the desired speed to the specified value.
	char char_buff[20];
	sprintf(char_buff, " %d", index);
	string str_buff="speed";
	str_buff.append(char_buff);
	sprintf(char_buff, " %f", value);
	str_buff.append(char_buff);
	const char *Command = str_buff.c_str();
	sendLine(Command);
	readAck();
	if(readBool()) {
		return true;
	}
	else {
		return false;
	}
}


bool CCommunicator::angvel(int index, double value)
{
	// Set the desired angular velocity to the specified value.
	char char_buff[20];
	sprintf(char_buff, " %d", index);
	string str_buff="angvel";
	str_buff.append(char_buff);
	sprintf(char_buff, " %f", value);
	str_buff.append(char_buff);
	const char *Command = str_buff.c_str();
	sendLine(Command);
	readAck();
	if(readBool()) {
		return true;
	}
	else {
		return false;
	}
}

bool CCommunicator::accelx(int index, double value)
{
	// Set the desired accelaration in x axis to the specified value in hovertank mode.
	char char_buff[20];
	sprintf(char_buff, " %d", index);
	string str_buff="accelx";
	str_buff.append(char_buff);
	sprintf(char_buff, " %f", value);
	str_buff.append(char_buff);
	const char *Command = str_buff.c_str();
	sendLine(Command);
	readAck();
	if(readBool()) {
		return true;
	}
	else {
		return false;
	}
}

bool CCommunicator::accely(int index, double value)
{
	// Set the desired accelaration in x axis to the specified value in hovertank mode.
	char char_buff[20];
	sprintf(char_buff, " %d", index);
	string str_buff="accely";
	str_buff.append(char_buff);
	sprintf(char_buff, " %f", value);
	str_buff.append(char_buff);
	const char *Command = str_buff.c_str();
	sendLine(Command);
	readAck();
	if(readBool()) {
		return true;
	}
	else {
		return false;
	}
}

vector<string> CSplitString::split()
{
	MyVector.clear();
	size_t LastLoc = -1;
	size_t CurLoc = MyString.find(" ", 0);
	while (CurLoc != string::npos) {
		string subString = MyString.substr(LastLoc+1, CurLoc-LastLoc-1);
		if(0!= subString.compare("")
				&& 0!=subString.compare(" "))
		{
		    MyVector.push_back(MyString.substr(LastLoc+1, CurLoc-LastLoc-1));
		}
		LastLoc=CurLoc;
		CurLoc = MyString.find(" ", LastLoc+1);
	}
	MyVector.push_back(MyString.substr(LastLoc+1, MyString.size()-LastLoc));
	return MyVector;
}

CSplitString::CSplitString(string str) {
	MyString=str;
	MyVector.begin();
}
