/*
 * CKalmanFilterAgentController.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */
#include <sys/time.h>
#include <unistd.h>

#include "CKalmanFilterAgentController.h"

#include "../Base/CWorldController.h"
#include "../ControlParams.h"
#include "../Base/CVector.h"

#include <iostream>

using namespace std;

CKalmanFilterAgentController::CKalmanFilterAgentController(string enemyColor, int shootType)
: CAgentController(1)
{
	setEnemyColor(enemyColor);
	setShootType(shootType);
	// TODO Auto-generated constructor stub
	mpMyAgent = (CMyAgent*)(GET_WORLD()->mpMyTeam->mAgentList[0]);

	CTeam * enemyTeam = GET_WORLD()->findTeam(enemyColor);
	if(enemyTeam)
	{
		if(enemyTeam->mAgentList.size()>0)
		{
			mpEnemyAgent = enemyTeam->mAgentList[0];
		}
		else
		{
			cout << " enemy team is empty " << endl;
		}
	}
	else
	{
		cout << "cannot find enemy team " << endl;
	}

	mpKalmanFilterAgent = NULL;

}

CKalmanFilterAgentController::~CKalmanFilterAgentController() {
	// TODO Auto-generated destructor stub
	mpKalmanFilterAgent = NULL;
}

void CKalmanFilterAgentController::init()
{
	mpKalmanFilterAgent = new CKalmanFilterAgent(mpEnemyAgent);
	initKalmanFilter(mShootType);
}

void CKalmanFilterAgentController::deinit()
{
	if(mpKalmanFilterAgent)
	{
		delete mpKalmanFilterAgent;
		mpKalmanFilterAgent = NULL;
	}
}

void CKalmanFilterAgentController::run()
{
	double targetX = 0, targetY = 0;
	double lastX,lastY;
	ControlParams ctrlParams;
	CVector ctrlVector(0,0);
	double currentX = 0, currentY = 0;

	CCommunicator * pCommunicator = GET_COMMUNICATOR();
	running = true;

	mpMyAgent->setCtrlTimeStepLength(CTRL_STEP_TIME);
	mpMyAgent->setCtrlParamP(PD_PROPORTIONAL);
	mpMyAgent->setCtrlParamD(PD_DERIVATIVE);

	targetX = mpEnemyAgent->getPosX();
	targetY = mpEnemyAgent->getPosY();
	mpMyAgent->setTargetPosition(targetX,targetY);

	while(running){
		struct timeval startTime;
		gettimeofday(&startTime, NULL);

		// Update the world state
		GET_WORLDCTRL()->updateWorld();

        mpKalmanFilterAgent->updateFilter(mpEnemyAgent);

		// TODO: add agent behavior here

		if(getShootType() == STATIC){
			mat avState(2,1);
			mpKalmanFilterAgent->mpKalmanFilter->getAverageHistState(avState);
			mpMyAgent->setTargetPosition(avState(0,0), avState(1,0));

			cout << " static " << avState(0,0) << " " << avState(1,0) << endl;

			ctrlParams = mpMyAgent->getControlParams();
			pCommunicator->angvel(mpMyAgent->getIndex(), ctrlParams.angleVelocity);

		}else if(getShootType() == CONFORMING){

			//estimateEnemyAgentSpeed();

			double time = getShootWaitingTime(mpMyAgent, mpEnemyAgent);

			cout << " calc time is " << time << endl;

			if(time < 0)
			{
				double tempTargetX = mpEnemyAgent->getEstimatedPosX() + 10 * cos(mpEnemyAgent->getOrientation());
				double tempTargetY = mpEnemyAgent->getEstimatedPosY() + 10 * sin(mpEnemyAgent->getOrientation());

				//cout << " target is " << tempTargetX << " " << tempTargetY << endl;
				mpMyAgent->setTargetPosition(tempTargetX, tempTargetY);

				ctrlParams = mpMyAgent->getControlParams();
				pCommunicator->angvel(mpMyAgent->getIndex(), ctrlParams.angleVelocity);
			}
			else
			if(time < CTRL_STEP_TIME)
			{
				pCommunicator->angvel(mpMyAgent->getIndex(), 0);
				usleep(time*1000000);
			}
			else if(time > 4 * CTRL_STEP_TIME)
			{
				double tempTargetX = mpEnemyAgent->getEstimatedPosX() - 5 * cos(mpEnemyAgent->getOrientation());
				double tempTargetY = mpEnemyAgent->getEstimatedPosY() - 5 * sin(mpEnemyAgent->getOrientation());

				mpMyAgent->setTargetPosition(tempTargetX, tempTargetY);

				ctrlParams = mpMyAgent->getControlParams();
				pCommunicator->angvel(mpMyAgent->getIndex(), ctrlParams.angleVelocity);
			}


			/*
			ctrlParams = mpMyAgent->getControlParams();
			pCommunicator->angvel(mpMyAgent->getIndex(), ctrlParams.angleVelocity);
			*/

		}
		pCommunicator->shoot(mpMyAgent->getIndex());

		struct timeval endTime;
		gettimeofday(&endTime, NULL);

		double deltaTime = (endTime.tv_sec-startTime.tv_sec) * 1000000
				          + (endTime.tv_usec-startTime.tv_usec);
		double leftTime = CTRL_STEP_TIME * 1000000 - deltaTime;

		if(leftTime > 0)
		{
			usleep(leftTime);
		}

	}
}

void CKalmanFilterAgentController::initKalmanFilter(shootType type)
{


	switch(type)
	{
	case STATIC:
		//transitionValue =
		//MATRIX transition(6,6, &transitionValue);
		//mpKalmanFilter->setTransition();
		break;
	case CONFORMING:
		break;
	case UNCONFORMING:
		break;
	}

}

void CKalmanFilterAgentController::updateKalmanFilter()
{

}

void CKalmanFilterAgentController::estimateEnemyAgentSpeed()
{
	double X1 = mpEnemyAgent->getLastEstimatedPosX();
	double X2 = mpEnemyAgent->getEstimatedPosX();
	double Y1 = mpEnemyAgent->getLastEstimatedPosY();
	double Y2 = mpEnemyAgent->getEstimatedPosY();

	double dist = sqrt(pow(X2-X1,2)+pow(Y2-Y1,2));
	mpEnemyAgent->setEstimatedSpeed(dist/CTRL_STEP_TIME);

}

double CKalmanFilterAgentController::getShootWaitingTime(CMyAgent * myAgent, CAgent * enemyAgent)
{
	if(myAgent==NULL || enemyAgent==NULL)
	{
		cout << " agent handle is null " << endl;
		return -100;
	}

	double myXPos = myAgent->getPosX();
	double myYPos = myAgent->getPosY();
	double myOrientation = myAgent->getOrientation();
	double myVel = GET_WORLD()->shotspeed;

	double enXPos = enemyAgent->getEstimatedPosX();
	double enYPos = enemyAgent->getEstimatedPosY();
	double enOrientation = enemyAgent->getOrientation();
	double enEstVel = enemyAgent->getEstimatedSpeed();

	//cout << "enemy " << enemyAgent->getColor() << " " << enemyAgent->getCallSign() <<  " pos " << enXPos << " and " << enYPos << endl;

	return getShootWaitingTime(myXPos, myYPos, myOrientation, myVel,
			                   enXPos, enYPos, enOrientation, enEstVel);
}

double CKalmanFilterAgentController::getShootWaitingTime(double myXPos, double myYPos, double myOrientation, double myVel,
		                   double enXPos, double enYPos, double enOrientation, double enEstVel)
{
	double distX = myXPos - enXPos;
	double distY = myYPos - enYPos;

	double distMyEn = sqrt(pow(distX,2)+pow(distY,2));
	double orientationFromEnToMy = atan2(myXPos-enXPos, myYPos-enYPos);
	double deltaAngleMyEn = orientationFromEnToMy - enOrientation;

	double newMyXPos = 0;
	double newMyYPos = distMyEn * sin(deltaAngleMyEn);
	double newEnXPos = -distMyEn * cos(deltaAngleMyEn);
	double newEnYPos = 0;

	double crossPosX = newMyYPos / tan(enOrientation-myOrientation);
	double crossPosY = 0;

	double enemyTime = (crossPosX - newEnXPos) / enEstVel;
	double distMyToCross = sqrt(pow(crossPosX-newMyXPos, 2)+pow(crossPosY-newMyYPos,2));
    double myTime = distMyToCross / myVel;

	if(enEstVel<0.001)
	{
		return myTime;
	}

	return enemyTime - myTime;
}
