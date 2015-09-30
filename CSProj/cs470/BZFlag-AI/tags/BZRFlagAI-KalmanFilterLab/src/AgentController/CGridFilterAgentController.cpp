/*
 * CGridFilterAgentController.cpp
 *
 *  Created on: Nov 3, 2012
 *      Author: walter
 */

#include <sys/time.h>
#include <unistd.h>

#include "../ControlParams.h"
#include "../AgentController/CGridFilterAgentController.h"
#include "../Base/CWorldController.h"

CGridFilterAgentController::CGridFilterAgentController(int num)
    : CAgentController(num)
{
	// TODO Auto-generated constructor stub
	mpGridMap = NULL;

	vector<CMyAgent*>::iterator it;
	for(it=agents.begin();it!=agents.end();it++)
	{
		int * pCounter = new int();
		*pCounter = 0;
		mAgentInnerCounter.push_back(*pCounter);
	}

}

CGridFilterAgentController::~CGridFilterAgentController()
{
	// TODO Auto-generated destructor stub
	mAgentInnerCounter.clear();

	if(mpGridMap)
	{
		delete mpGridMap;
		mpGridMap = NULL;
	}

	if(mpFilter)
	{
		delete mpFilter;
		mpFilter = NULL;
	}
}

void CGridFilterAgentController::init()
{
	//deinit();
	mpGridMap = new CVisualizedGridMap(GET_WORLD()->mSize, GET_WORLD()->mSize);
	mpFilter = new CGridFilter(GET_WORLD()->truepositive, GET_WORLD()->truenegative, mpGridMap);
}

void CGridFilterAgentController::deinit()
{
	if(mpGridMap)
	{
		delete mpGridMap;
		mpGridMap = NULL;
	}

	if(mpFilter)
	{
		delete mpFilter;
		mpFilter = NULL;
	}
}

void CGridFilterAgentController::showMap()
{
	if(mpGridMap)
	{
		mpGridMap->drawMap();
	}

}

void CGridFilterAgentController::run()
{
	CCommunicator * pCommunicator = GET_COMMUNICATOR();

	vector<CMyAgent *>::iterator it;
	COccGridState state;
	running = true;
	ControlParams ctrlParams;
	int agentNum = agents.size();

	// cout << "Running"<<endl;

	while(running){

		struct timeval startTime;
		gettimeofday(&startTime, NULL);

		// Update the world state
		GET_WORLDCTRL()->updateWorld();

		for(it = agents.begin(); it != agents.end(); it++)
		{
			// move the robot
			moveAgent((*it)->getIndex());
			/*
			ctrlParams.speed = 0.5;
			ctrlParams.angleVelocity = .09;
			pCommunicator->angvel((*it)->getIndex(), ctrlParams.angleVelocity);
			pCommunicator->speed((*it)->getIndex(), ctrlParams.speed);
            */

			pCommunicator->occgrid((*it)->getIndex(), state);

			if(mpFilter)
			{
				mpFilter->filter(&state);
			}

			// visualize the update
			showMap();

		}

		struct timeval endTime;
		gettimeofday(&endTime, NULL);

		double deltaTime = (endTime.tv_sec-startTime.tv_sec) * 1000000
				          + (endTime.tv_usec-startTime.tv_usec);
		double leftTime = CTRL_STEP_TIME * 1000000 - deltaTime;
		// cout<<"sleep "<<leftTime<< " delta " << deltaTime << endl;
		if(leftTime > 0)
		{
			usleep(leftTime);
			 // cout<<"sleep "<<leftTime<<endl;
		}

	}
}

