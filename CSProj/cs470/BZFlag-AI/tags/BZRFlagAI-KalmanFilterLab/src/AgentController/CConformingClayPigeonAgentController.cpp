/*
 * CConformingClayPigeonAgentController.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "CConformingClayPigeonAgentController.h"
#include "../Base/CWorldController.h"

CConformingClayPigeonAgentController::CConformingClayPigeonAgentController() {
	// TODO Auto-generated constructor stub

}

CConformingClayPigeonAgentController::~CConformingClayPigeonAgentController() {
	// TODO Auto-generated destructor stub
}

void CConformingClayPigeonAgentController::moveAgent()
{
	double speed = 0.1;

	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	pCommunicator->angvel(mpAgent->getIndex(), 0);
	pCommunicator->speed(mpAgent->getIndex(), speed);

}
