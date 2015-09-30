/*
 * CDumbAgent.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: fausto
 */

#include <iostream>
#include "CDumbAgentController.h"
#include "../Base/CWorldController.h"
#include "../Base/CController.h"

CDumbAgentController::CDumbAgentController(int num)
    : CAgentController(num)
{

}

void CDumbAgentController::run(){

	CCommunicator * pCommunicator = GET_COMMUNICATOR();
	ControlParams ctrlParams;
	vector<CMyAgent *>::iterator it;
	running = true;



	while(running){

		for(it = agents.begin(); it != agents.end(); it++)
		{
			pCommunicator->shoot((*it)->getIndex());
		}
		for(it = agents.begin(); it != agents.end(); it++)
		{
			ctrlParams.speed = 0.5;
			ctrlParams.angleVelocity = 0;
			pCommunicator->angvel((*it)->getIndex(), ctrlParams.angleVelocity);
			pCommunicator->speed((*it)->getIndex(), ctrlParams.speed);
		}
		sleep(1.5);

		for(it = agents.begin(); it != agents.end(); it++)
		{
			pCommunicator->shoot((*it)->getIndex());
		}
		sleep(1.5);
		for(it = agents.begin(); it != agents.end(); it++)
		{
			pCommunicator->shoot((*it)->getIndex());
		}
		sleep(1.5);
		for(it = agents.begin(); it != agents.end(); it++)
		{
			ctrlParams.speed = 0;
			ctrlParams.angleVelocity = 2;
			pCommunicator->speed((*it)->getIndex(), ctrlParams.speed);
			pCommunicator->angvel((*it)->getIndex(), ctrlParams.angleVelocity);
		}

		sleep(2);

		cout<< "running" << endl;
		//move
	}

}


CDumbAgentController::~CDumbAgentController() {
	// TODO Auto-generated destructor stub
}
