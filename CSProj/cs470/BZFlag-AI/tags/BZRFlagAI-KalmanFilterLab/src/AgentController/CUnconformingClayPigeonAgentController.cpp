/*
 * CUnconformingClayPigeonAgentController.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "CUnconformingClayPigeonAgentController.h"
#include "../Base/CWorldController.h"

CUnconformingClayPigeonAgentController::CUnconformingClayPigeonAgentController() {
	// TODO Auto-generated constructor stub

}

CUnconformingClayPigeonAgentController::~CUnconformingClayPigeonAgentController() {
	// TODO Auto-generated destructor stub
}

void CUnconformingClayPigeonAgentController::moveAgent()
{
	double speed = 5;

	double angvel = rand() % 10 + 1;
	angvel = angvel - 5;

	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	pCommunicator->angvel(mpAgent->getIndex(), angvel);
	pCommunicator->speed(mpAgent->getIndex(), speed);

}
