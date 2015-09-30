/*
 * CRandomWalkGridFilterAgentController.cpp
 *
 *  Created on: Nov 6, 2012
 *      Author: walter
 */

#include "CRandomWalkGridFilterAgentController.h"
#include "../Base/CWorldController.h"

CRandomWalkGridFilterAgentController::CRandomWalkGridFilterAgentController(int num)
    : CGridFilterAgentController(num)
{
	// TODO Auto-generated constructor stub

}

CRandomWalkGridFilterAgentController::~CRandomWalkGridFilterAgentController() {
	// TODO Auto-generated destructor stub
}

void CRandomWalkGridFilterAgentController::moveAgent(int index)
{
	ControlParams ctrlParams;
	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	CMyAgent * pAgent = getAgent(index);
	if(NULL==pAgent)
	{
		return;
	}

	pAgent->setRandomWalkMode(true);
	ctrlParams = pAgent->getControlParams();
	/*
	ctrlParams.speed = 0.5;
	ctrlParams.angleVelocity = .09;
	*/
	pCommunicator->angvel(index, ctrlParams.angleVelocity);
	pCommunicator->speed(index, ctrlParams.speed);
}
