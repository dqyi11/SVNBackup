//============================================================================
// Name        : WingmanPathPlanning.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
using namespace std;

#include "WingmanPathPlanning.h"

int main(int argc, char * argv[]) {

	/* CASE 1
	AgentMotion seq[7] = {EAST, EAST, EAST, EAST, NORTHEAST, NORTHEAST, NORTHEAST};
	int seqNum = 7;
    */

	/* CASE 2
	AgentMotion seq[7] = {EAST, EAST, EAST, EAST, EAST, EAST, EAST};
	int seqNum = 7;
	*/

	AgentMotion seq[15] = {SOUTHWEST, SOUTHWEST, EAST, EAST, EAST,
			EAST, EAST, NORTHWEST, NORTHEAST, NORTHWEST,
			NORTHEAST, WEST, WEST, WEST, WEST,
			/*
			WEST, WEST, NORTHEAST, NORTHWEST, NORTHEAST,
			NORTHWEST, NORTHEAST, EAST, EAST, EAST
			*/

	};
	int seqNum = 15;

    int planStep = 1;
    int wingmanRadius = 2;


	if(argc == 1)
	{
		return 0;
	}

	for(int i = 0; i < argc; ++i)
	{
		//cout << "index " << i << " : " << argv[i] <<endl;
	}

	if(argc==2)
	{
		planStep = atoi(argv[1]);
	}
	if(argc==3)
	{
		wingmanRadius = atoi(argv[2]);
		planStep = atoi(argv[1]);
	}


	cout << "PLANNIGN LENGTH IS " << planStep << endl;
	cout << "WINGMAN RADIUS IS " << wingmanRadius << endl;

	CVisualHexagonDiscretizedMap * pMap = new CVisualHexagonDiscretizedMap(16, 16, 1);
	CMapGraphConvertor * pConvertor = new CMapGraphConvertor();
	CPathPlanningGraphCreator * pPathPlanningCreator = new CPathPlanningGraphCreator();


	pMap->init();

	pConvertor->setMap(pMap);
	pConvertor->init();
	pConvertor->visualizeGraph();

	CHuman * pHuman = new CHuman();
	pHuman->setPosX(5);
	pHuman->setPosY(5);
	pHuman->setWingmanRadius(wingmanRadius);


	CHexaGridGraph * pGraph = pConvertor->getGraph();
	CHexaVertex * pStartVertex = pGraph->findVertex(5,5);
	if(pStartVertex)
	{
		cout << " find start vertex " << pStartVertex->mName << " pos " << pStartVertex->mPosX << " and " << pStartVertex->mPosY << endl;
	}

	pHuman->loadMotion(seq,seqNum);

	CRobot * pRobot = new CRobot();

	pMap->addAgent(pHuman);
	//pMap->addAgent(pRobot);

	pMap->run(seqNum);

	pPathPlanningCreator->setGridGraph(pConvertor->getGraph());
	pPathPlanningCreator->setHuman(pHuman);
	pPathPlanningCreator->setMap(pMap);
	pPathPlanningCreator->init();

    pPathPlanningCreator->visualizeGraph("pathPlanningGraph.png");

	CPathPlanningGraph * pPathPlanningGraph = pPathPlanningCreator->getPlanningGraph();

	if(pPathPlanningGraph)
	{
		cout << " pPathPlanning is OK " << endl;
	}

    CPathPlanningGraphPruner * pPathPlanningPruner =
    		new CPathPlanningGraphPruner(pPathPlanningGraph);
    pPathPlanningPruner->setStartVertex(pStartVertex->mName);
    pPathPlanningPruner->setDiscretizedMap(pMap);

    cout << " pruning " << endl;
    pPathPlanningPruner->pruneGraph();
    //pPathPlanningPruner->getGraph()->printLevelVertex();
    pPathPlanningPruner->visualizeGraph("prunedPathPlanningGraph.png");

    cout << " plotting " << endl;

	pMap->plotMap();

	CExhaustiveSearchPathPlanner * pathPlanner
	    = new CExhaustiveSearchPathPlanner(pPathPlanningGraph,pRobot);
	cout << " init map " << endl;
	pathPlanner->init(pMap);

	cout << "generating all possible path " << endl;
	//pathPlanner->generateAllPossiblePath();

	// pathPlanner->printAllPath();

	cout << " Scoring " << endl;
	pathPlanner->scoreAllPossiblePath();

	pathPlanner->plotPathScoreDist();

	CGreedySearchPathPlanner * greedyPathPlanner
	    = new CGreedySearchPathPlanner(pPathPlanningGraph, pRobot);
	greedyPathPlanner->init(pMap);

    greedyPathPlanner->doGreedySearch();


    CBatchSearchPathPlanner * batchPathPlanner
        = new CBatchSearchPathPlanner(pPathPlanningGraph, pRobot);
    batchPathPlanner->setBatchCnt(planStep);
    batchPathPlanner->init(pMap);

    batchPathPlanner->doBatchSearch();

    CPath pathBatchMax = batchPathPlanner->getSearchedPath();
    batchPathPlanner->initObservedCells(pathBatchMax);
    cout << "BATCH MAX: " << batchPathPlanner->scorePath(pathBatchMax) << endl;

    CRecedingHorizonPathPlanner * recedingPlanner
        = new CRecedingHorizonPathPlanner(pPathPlanningGraph, pRobot);
    recedingPlanner->setRecedingHorizonLength(planStep);
    recedingPlanner->init(pMap);

    recedingPlanner->doRecedingHorizonSearch();

    CPath pathRHMax = recedingPlanner->getSearchedPath();
    recedingPlanner->initObservedCells(pathRHMax);
    cout << "RECEDING MAX: " << recedingPlanner->scorePath(pathRHMax) << endl;

	cout << "EXIT" << endl;

	return 0;
}
