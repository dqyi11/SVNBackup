/*
 * CPotentialFieldAgentController.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include <math.h>
#include <sys/time.h>
#include <unistd.h>

#include "../ControlParams.h"
#include "../Base/CVector.h"
#include "../Base/CPDController.h"
#include "../Base/CWorldController.h"
#include "../PotentialField/CPotentialFieldManager.h"
#include "CPotentialFieldAgentController.h"


CPotentialFieldAgentController::CPotentialFieldAgentController(int num)
    : CAgentController(num), CPDController(PD_PROPORTIONAL, PD_DERIVATIVE)
{
	// TODO Auto-generated constructor stub
	mpPFMgr = new CPotentialFieldManager();
	mLastPosX = 0.0;
	mLastPosY = 0.0;

}

CPotentialFieldAgentController::~CPotentialFieldAgentController() {
	// TODO Auto-generated destructor stub
}

void CPotentialFieldAgentController::run()
{
	running = true;
	CMyAgent * pAgent = NULL;
	double x,y = 0.0;
	CVector ctrlVector(0,0);
	vector<CMyAgent *>::iterator it;
	vector<CFlagBaseAttractivePotentialField>::iterator itBaseAttr;
	ControlParams ctrlParams;
	CCommunicator * pCommunicator = GET_COMMUNICATOR();
    CFlag * targetFlag = NULL;
    CBase * targetBase = NULL;
	double goalX,goalY = 0.0;
	CWorld* pWorld = GET_WORLD();
	bool goToEnemyBase = true;

	mpPFMgr->drawPotentialMap();


	if (mpPFMgr)
	{
		itBaseAttr = mpPFMgr->mBasesAttractive.begin();
		targetFlag = (*itBaseAttr).getFlag();
		targetBase = (*itBaseAttr).getBase();

		goalX = targetFlag->x; //mpPFMgr->mBasesAttractive[0].mGoalX;
		goalY = targetFlag->y; //mpPFMgr->mBasesAttractive[0].mGoalY;

	}

	double stuck = 0.0;
	while(running)
	{

		if(goToEnemyBase)
		{
			if(pWorld->flagOwnedByMyTeam(targetFlag->color))
			{
				/* we are going to switch our strategy */
				/* TODO */
				cout<<"Sir! I caught the flag!"<<endl;
				cout<<"we are going back out our own base"<<endl;

				mpPFMgr->clearAttractiveFlagBase();
				cout<<"This was done"<<endl;
				targetFlag = GET_WORLD()->getMyTeamFlag();
				targetBase = GET_WORLD()->getMyTeamBase();

				mpPFMgr->addAttractiveFlagBase(targetFlag, targetBase);
			    goalX = targetFlag->x;
			    goalY = targetFlag->y;
			    vector<CObstacleTangentialPotentialField>::iterator itPt;
			    for(itPt=mpPFMgr->mObstaclesTangential.begin();itPt!=mpPFMgr->mObstaclesTangential.end();itPt++)
			    {
			    	(*itPt).changeDirection();
			    }
			    goToEnemyBase = false;
			}
		}

		if(mpPFMgr)
		{
			struct timeval startTime;
			gettimeofday(&startTime, NULL);

			CWorldController * pCtrl = GET_WORLDCTRL();
			pCtrl->updateWorld();



			for(it = agents.begin(); it != agents.end(); it++)
			{
				x = (*it)->getPosX();
				y = (*it)->getPosY();
				ctrlVector = mpPFMgr->getVector(x, y);

				if(x==mLastPosX && y==mLastPosY)
				{
					stuck = rand() - (RAND_MAX / 2);
					cout << "SHOOOOTING!!"<<endl;
					pCommunicator->shoot((*it)->getIndex());

					if(stuck > 0){
						cout<<"Turning this way"<<endl;
						pCommunicator->angvel((*it)->getIndex(), -50);
						pCommunicator->speed((*it)->getIndex(), 10);


					}
					else
					{
						cout<<"Turning that way"<<endl;
						pCommunicator->angvel((*it)->getIndex(), 50);
						pCommunicator->speed((*it)->getIndex(), 10);
					}
				}
				else
				{

				double deltaOrientation = ctrlVector.orientation() - (*it)->getOrientation();

				ctrlParams = calcControlParams((*it), goalX, goalY, ctrlVector);
				// ctrlParams = CPDController::calcControlParams(ctrlVector.mX, ctrlVector.mY, deltaOrientation, CTRL_STEP_TIME);

				// cout << "angle:" << ctrlParams.angleVelocity << " speed:" << ctrlParams.speed << endl;
				pCommunicator->angvel((*it)->getIndex(), ctrlParams.angleVelocity);
				pCommunicator->speed((*it)->getIndex(), ctrlParams.speed);

				}

				mLastPosX = x;
				mLastPosY = y;


			}

			struct timeval endTime;
			gettimeofday(&endTime, NULL);

			double deltaTime = (endTime.tv_sec-startTime.tv_sec) * 1000000
					          + (endTime.tv_usec-startTime.tv_usec);
			double leftTime = CTRL_STEP_TIME * 1000000 - deltaTime;
			if(leftTime > 0)
			{
				usleep(leftTime);
				// cout<<"sleep "<<leftTime<<endl;
			}
		}

	}


}

ControlParams CPotentialFieldAgentController::calcControlParams(CMyAgent * agent, double goalX, double goalY, CVector potentialVector)
{
	ControlParams ctrlParams;

    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaOrientation = 0.0;

    if (agent)
    {
    	deltaX = goalX - agent->getPosX();
    	deltaY = goalY - agent->getPosY();
    	//cout << "ctrlV or:" << potentialVector.orientation() << " agent or: " << agent->getOrientation();
    	deltaOrientation = potentialVector.orientation() - agent->getOrientation();
    }

    ctrlParams = CPDController::calcControlParams(deltaX, deltaY, deltaOrientation, CTRL_STEP_TIME);

    return ctrlParams;
}
