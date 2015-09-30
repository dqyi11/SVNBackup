/*
 * CSittingDuckAgentController.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "CSittingDuckAgentController.h"
#include "../Base/CWorldController.h"

CSittingDuckAgentController::CSittingDuckAgentController() {
	// TODO Auto-generated constructor stub

}

CSittingDuckAgentController::~CSittingDuckAgentController() {
	// TODO Auto-generated destructor stub
}

void CSittingDuckAgentController::moveAgent()
{
	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	pCommunicator->angvel(mpAgent->getIndex(), 0);
	pCommunicator->speed(mpAgent->getIndex(), 0);

}

