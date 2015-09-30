/*
 * CMyAgent.h
 *
 *  Created on: Sep 28, 2012
 *      Author: walter
 */

#ifndef CMYAGENT_H_
#define CMYAGENT_H_

#include "CAgent.h"
#include "CPDController.h"

class CMyAgent: public CAgent {
public:
	CMyAgent(int index, string callSign);
	virtual ~CMyAgent();

	void setShotsAvailable(int shot) { mShotsAvailable = shot; };
	void setTimeToReload(int time) { mTimeToReload = time; };
    void setVelocityX(double vx) { mVelocityX = vx; };
    void setVelocityY(double vy) { mVelocityY = vy; };
    void setAngleVelocity(double va) { mAngleVelocity = va; };

    virtual void print();

	int getIndex(void) { return mIndex; };
	int getShotsAvailable(void) { return mShotsAvailable; };
	int getTimeToReload(void) { return mTimeToReload; };
	double getVelocityX(void) { return mVelocityX; };
	double getVelocityY(void) { return mVelocityY; };
	double getAngleVelocity(void) { return mAngleVelocity; };

	/* CONTROL */
	void setCtrlTimeStepLength(double T);
	void setCtrlParamP(double P);
	void setCtrlParamD(double D);

    double getCtrlTimeStepLength(void);
    double getCtrlParamP(void);
    double getCtrlParamD(void);

    void setTargetPosition(double X, double Y) { mTargetX = X; mTargetY = Y; };
    void setRandomWalkMode(bool enable) { mEnableRandomWalk = enable; };

    void getTargetPosition(double & X, double & Y) { X = mTargetX; Y = mTargetY; };
    bool getRandomWalkMode(void) { return mEnableRandomWalk; };

    ControlParams getControlParams();

private:
	CPDController * mpController;

	int mIndex;
	double mVelocityX;
	double mVelocityY;
	double mAngleVelocity;
	int mShotsAvailable;
	int mTimeToReload;

	bool mEnableRandomWalk;
	double mTargetX;
	double mTargetY;
};

#endif /* CMYAGENT_H_ */
