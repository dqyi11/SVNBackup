//============================================================================
// Name        : BZRFlag-AI.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#define USE_PF_AGENT


#include <iostream>
#include "stdlib.h"

#include "CCommunicator.h"
#include "CWorldController.h"

#ifdef USE_DUMB_AGENT
#include "CDumbAgentController.h"
#endif

#ifdef USE_PF_AGENT
#include "CPotentialFieldAgentController.h"
#endif

using namespace std;

const char *kDefaultServerName = "localhost";
const int kDefaultServerPort = 59722;

int main(int argc, char * argv[])
{
	int port = kDefaultServerPort;
	CCommunicator * pCommunicator = NULL;
	CWorldController * pWorldCtrl = NULL;
#ifdef USE_DUMB_AGENT
	CDumbAgentController * pAgentController = NULL;
#endif

#ifdef USE_PF_AGENT
	CPotentialFieldAgentController * pAgentController = NULL;
#endif

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

#ifdef USE_DUMB_AGENT
	pAgentController = new CDumbAgentController(2);

	if(pAgentController)
	{
		pAgentController->run();
	}
#endif

#ifdef USE_PF_AGENT
	string enemy_color = "";
	CTeam * pTeam = NULL;
	bool incorrect_input = true;
	while(incorrect_input)
	{
	    cout << "Please select an enemy color: " << endl;
	    cin >> enemy_color;
	    if(0!=GET_WORLD()->mpMyTeam->getColor().compare(enemy_color))
	    {
	    	pTeam = GET_WORLD()->findTeam(enemy_color);
	    	if(NULL != pTeam)
	    	{
	    	    incorrect_input = false;
	    	}
	    }
	    if(incorrect_input)
	    {
	    	cout<<"wrong input, try again"<<endl;
	    	cin >> enemy_color;
	    }
	}

	pAgentController = new CPotentialFieldAgentController(1);
	if(pAgentController)
	{
		CFlag * pFlag = GET_WORLD()->findFlag(enemy_color);
		CBase * pBase = GET_WORLD()->findTeam(enemy_color)->base;
		pAgentController->getPotentialFieldManager()->addAttractiveFlagBase(pFlag, pBase);
		list<CObstacle *>::iterator itObst;
		CObstacle * pObstacle = NULL;

		for(itObst=GET_WORLD()->obstacles.begin(); itObst!=GET_WORLD()->obstacles.end(); itObst++)
		{
			pObstacle = (*itObst);
			pAgentController->getPotentialFieldManager()->addRepulsiveObstacle(pObstacle);
			pAgentController->getPotentialFieldManager()->addTangentialObstacle(pObstacle);
		}

	    pAgentController->run();
	}
#endif

	//cout << "Hello World" << endl; // prints Hello World
	return 0;

}
