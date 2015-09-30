/*
 * CAgent.h
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#ifndef CAGENT_H_
#define CAGENT_H_

#include <string>
#include "CBaseObject.h"
using namespace std;

class CAgent : public CBaseObject
{

public:
	CAgent(string callSign);
	virtual ~CAgent();

	void setPosX(double x) { mPosX = x; };
	void setPosY(double y) { mPosY = y; };
	void setOrientation(double orientation) { mOrientation = orientation; };
	void setStatus(string status) { mStatus = status; };
    void setColor(string color) { mColor = color; };
    void setFlag(string flag) { mFlag = flag; };

    virtual void print();

	double getPosX(void) { return mPosX; };
	double getPosY(void) { return mPosY; };
	double getOrientation(void) { return mOrientation; };

	string getCallSign(void) { return mCallSign; };
	string getStatus(void) { return mStatus; };
    string getColor(void) { return mColor; };
	string getFlag(void) { return mFlag; };


protected:

	string mCallSign;
	string mStatus;

	double mPosX;
	double mPosY;
	double mOrientation;
	string mColor;
	string mFlag;

};

#endif /* CAGENT_H_ */
