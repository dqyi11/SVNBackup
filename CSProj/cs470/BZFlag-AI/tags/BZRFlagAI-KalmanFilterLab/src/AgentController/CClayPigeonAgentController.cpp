/*
 * CClayPigeonAgentController.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include <sys/time.h>
#include <unistd.h>

#include "CClayPigeonAgentController.h"
#include "../Base/CWorldController.h"
#include "../ControlParams.h"

CClayPigeonAgentController::CClayPigeonAgentController()
: CAgentController(1)
{
	// TODO Auto-generated constructor stub

	mpAgent = (CMyAgent*)(GET_WORLD()->mpMyTeam->mAgentList[0]);

}

CClayPigeonAgentController::~CClayPigeonAgentController() {
	// TODO Auto-generated destructor stub
	mpAgent = NULL;
}

void CClayPigeonAgentController::run()
{
	running = true;

	while(running){

		struct timeval startTime;
		gettimeofday(&startTime, NULL);

		// Update the world state
		GET_WORLDCTRL()->updateWorld();

		moveAgent();

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
