//============================================================================
// Name        : BZRFlag-AI.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iostream>
#include "stdlib.h"

//#define USE_RAND_WALK
//#define USE_RAND_TARGET
//#define USE_CELL_GREEDY
#define USE_CELL_WALK

#include "Base/CCommunicator.h"
#include "Base/CWorldController.h"

#ifdef USE_RAND_WALK
#include "AgentController/CRandomWalkGridFilterAgentController.h"
#endif
#ifdef USE_RAND_TARGET
#include "AgentController/CRandomTargetGridFilterAgentController.h"
#endif
#ifdef USE_CELL_GREEDY
#include "AgentController/CCellGreedyGridFilterAgentController.h"
#endif
#ifdef USE_CELL_WALK
#include "AgentController/CCellWalkGridFilterAgentController.h"
#endif

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 59722;

int main(int argc, char * argv[])
{
	int port = kDefaultServerPort;
	CCommunicator * pCommunicator = NULL;
	CWorldController * pWorldCtrl = NULL;
	CGridFilterAgentController * pAgentController = NULL;

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

#ifdef USE_RAND_WALK
	pAgentController = new CRandomWalkGridFilterAgentController(agentNum);
#endif

#ifdef USE_RAND_TARGET
	pAgentController = new CRandomTargetGridFilterAgentController(agentNum);
#endif

#ifdef USE_CELL_GREEDY
	pAgentController= new CCellGreedyGridFilterAgentController(agentNum, 40);
#endif

#ifdef USE_CELL_WALK
	pAgentController = new CCellWalkGridFilterAgentController(agentNum, 40);
#endif

	if(pAgentController)
	{
		pAgentController->init();
		pAgentController->run();
	}


	//cout << "Hello World" << endl; // prints Hello World
	return 0;

}
