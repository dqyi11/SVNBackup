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


enum AgentMotionStatus
{
	POS_CHANGE       = 0,
	POS_UNCHANGE     = 1,
	ON_THE_UPPER     = 2,
	ON_THE_BOTTOM    = 3,
	ON_THE_LEFTMOST  = 4,
	ON_THE_RIGHTMOST = 5,
};

class CAgent : public CBaseObject
{

public:
	CAgent(string callSign);
	virtual ~CAgent();

	void setPosX(double x) { mLastPosX = mPosX; mPosX = x; };
	void setPosY(double y) { mLastPosY = mPosY; mPosY = y; };
	void setOrientation(double orientation) { mLastOrientation = mOrientation; mOrientation = orientation; };
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

	AgentMotionStatus getAgentMotionStatus(void);

	void setEstimatedPosX(double posX) { mLastEstimatedPosX = mEstimatedPosX; mEstimatedPosX = posX; };
	void setEstimatedPosY(double posY) { mLastEstimatedPosY = mEstimatedPosY; mEstimatedPosY = posY; };
	double getLastEstimatedPosX() { return mLastEstimatedPosX; };
	double getLastEstimatedPosY() { return mLastEstimatedPosY; };
	double getEstimatedPosX() { return mEstimatedPosX; };
	double getEstimatedPosY() { return mEstimatedPosY; };
	void setEstimatedSpeed(double speed) { mEstimatedSpeed = speed; };
	double getEstimatedSpeed(void) { return mEstimatedSpeed; };

	double getLastPosX(void) { return mLastPosX; };
	double getLastPosY(void) { return mLastPosY; };


protected:

	string mCallSign;
	string mStatus;

	double mPosX;
	double mPosY;
	double mOrientation;
	string mColor;
	string mFlag;

	double mLastPosX;
	double mLastPosY;
	double mLastOrientation;

	double mEstimatedPosX;
	double mEstimatedPosY;

	double mLastEstimatedPosX;
	double mLastEstimatedPosY;
	double mEstimatedSpeed;

};

#endif /* CAGENT_H_ */