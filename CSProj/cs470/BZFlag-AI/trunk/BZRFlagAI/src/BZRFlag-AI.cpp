//============================================================================
// Name        : BZRFlag-AI.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iostream>
#include "stdlib.h"

#include "Base/CWorldController.h"
#include "AgentController/CAgentController.h"

//#define USE_SITTING_DUCK
//#define USE_CONFORMING_CLAY_PIGEON
//#define USE_UNCONFORMING_CLAY_PIGEON
#define USE_KALMAN_FILTER_AGENT

#ifdef USE_SITTING_DUCK
#include "AgentController/CSittingDuckAgentController.h"
#endif
#ifdef USE_CONFORMING_CLAY_PIGEON
#include "AgentController/CConformingClayPigeonAgentController.h"
#endif
#ifdef USE_UNCONFORMING_CLAY_PIGEON
#include "AgentController/CUnconformingClayPigeonAgentController.h"
#endif
#ifdef USE_KALMAN_FILTER_AGENT
#include "AgentController/CKalmanFilterAgentController.h"
#endif

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 59722;

int main(int argc, char * argv[])
{
	int port = kDefaultServerPort;
	CCommunicator * pCommunicator = NULL;
	CWorldController * pWorldCtrl = NULL;
	CAgentController * pAgentController = NULL;

	int agentNum = 10;

	for(int i = 0; i < argc; ++i)
	{
		//cout << "index " << i << " : " << argv[i] <<endl;
	}

	if (argc == 2)
	{
		port = atoi(argv[1]);
	}
	else
	{
		cout << "unknown parameter" << endl;
	}

	pCommunicator = new CCommunicator(kDefaultServerName, port);
	pWorldCtrl = CWorldController::getInstance();

	if(pCommunicator->GetStatus())
	{
		pWorldCtrl->init(pCommunicator);

	}

#ifdef USE_KALMAN_FILTER_AGENT
	string enemyColor = "";
	CTeam * pTeam = NULL;
	bool incorrectInput = true;
	while(incorrectInput)
	{
		cout << "Please select an enemy color: " << endl;
		cin >> enemyColor;

		if(0!=GET_WORLD()->mpMyTeam->getColor().compare(enemyColor))
		{
			pTeam = GET_WORLD()->findTeam(enemyColor);
			if(NULL!=pTeam)
			{
				incorrectInput = false;
			}
		}
		if(incorrectInput)
		{
			cout << " wrong input, try again " << endl;
			cin >> enemyColor;
		}

	}

	int enemyShootType = 0;
	incorrectInput = true;
	while(incorrectInput)
	{
		cout << "Please input shoot type: " << endl;
		cin >> enemyShootType;

		if(enemyShootType>=0 && enemyShootType<=2)
		{
			incorrectInput = false;
		}

		if(incorrectInput)
		{
			cout << "wrong input, try again " << endl;
		}
 	}
#endif


#ifdef USE_SITTING_DUCK
	pAgentController = new CSittingDuckAgentController();
#endif
#ifdef USE_CONFORMING_CLAY_PIGEON
	pAgentController = new CConformingClayPigeonAgentController();
#endif
#ifdef USE_UNCONFORMING_CLAY_PIGEON
	pAgentController = new CUnconformingClayPigeonAgentController();
#endif
#ifdef USE_KALMAN_FILTER_AGENT
	pAgentController = new CKalmanFilterAgentController(enemyColor, enemyShootType);
#endif


	if(pAgentController)
	{
		pAgentController->init();
		pAgentController->run();
	}

	//cout << "Hello World" << endl; // prints Hello World
	return 0;

}
