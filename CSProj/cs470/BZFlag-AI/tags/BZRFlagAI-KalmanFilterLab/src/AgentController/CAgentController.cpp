/*
 * CAgentController.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include "../Base/CWorldController.h"
#include "../AgentController/CAgentController.h"

CAgentController::CAgentController(int num)
{
	// TODO Auto-generated constructor stub
	running = false;

	CAgent * agent = NULL;
	CMyAgent * myagent = NULL;
	CTeam * pMyTeam = GET_WORLD()->mpMyTeam;

	if (pMyTeam)
	{
		if (num > (int)pMyTeam->mAgentList.size())
		{
			return;
		}

		for(int i = 0; i < num; i++)
		{
			 agent = pMyTeam->mAgentList[i];
			 myagent = (CMyAgent *)agent;
			 agents.push_back(myagent);
		}

	}

}

CAgentController::~CAgentController()
{
	// TODO Auto-generated destructor stub
}

void CAgentController::stop(){
	running = false;
}

CMyAgent * CAgentController::getAgent(int index)
{
	CMyAgent * pAgent = NULL;
	if ((index >= agents.size())
		|| (index<0))
	{
		return pAgent;
	}

	return agents[index];
}

void CAgentController::init()
{
}


