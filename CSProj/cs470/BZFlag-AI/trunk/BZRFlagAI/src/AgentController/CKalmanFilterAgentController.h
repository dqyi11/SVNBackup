/*
 * CKalmanFilterAgentController.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CKALMANFILTERAGENTCONTROLLER_H_
#define CKALMANFILTERAGENTCONTROLLER_H_

#include "CAgentController.h"
#include "../KalmanFilter/CKalmanFilterAgent.h"
#include <string>

using namespace std;

enum shootType
{
	STATIC = 0,
	CONFORMING = 1,
	UNCONFORMING = 2,
};

class CKalmanFilterAgentController: public CAgentController {
public:
	CKalmanFilterAgentController(string enemyColor, int shootType);
	virtual ~CKalmanFilterAgentController();

	void initKalmanFilter(shootType type);
	void updateKalmanFilter();

	void setEnemyColor(string color) { mEnemyColor = color; };
	string getEnemyColor() { return mEnemyColor; };

	void setShootType(int type) { mShootType = (shootType)type; };
	shootType getShootType() { return mShootType; };

	double getShootWaitingTime(CMyAgent * myAgent, CAgent * enemyAgent);
	double getShootWaitingTime(double myXPos, double myYPos, double myOrientation, double myVel,
			                   double enXPos, double enYPos, double enOrientation, double enEstVel) ;

	virtual void init();
	void deinit();

	virtual void run();

	void shoot();

	void estimateEnemyAgentSpeed();

private:
	CMyAgent * mpMyAgent;
	CAgent * mpEnemyAgent;

	CKalmanFilterAgent * mpKalmanFilterAgent;

	string mEnemyColor;
	shootType mShootType;
};

#endif /* CKALMANFILTERAGENTCONTROLLER_H_ */
