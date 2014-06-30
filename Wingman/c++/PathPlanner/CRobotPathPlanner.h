/*
 * CRobotPathPlanner.h
 *
 *  Created on: Dec 28, 2012
 *      Author: walter
 */

#ifndef CROBOTPATHPLANNER_H_
#define CROBOTPATHPLANNER_H_

#include <list>
#include "../Base/CGrid.h"

#include "../Base/CPathPlanningGraph.h"
#include "../Visualizer/CVisualHexagonDiscretizedMap.h"
#include "../Visualizer/CDataArrayVisualizer.h"

using namespace std;

class CCellObserved {
public:
	CCellObserved();
	~CCellObserved();

	int mCellIndex;
	int mObsTimes;
	bool mIsVisitedCell;
	double mOriginProb;
	double mDeltaProb;
	double mReward;
};

class CPath {
public:
	CPath();
	virtual ~CPath();

	void addVertex(CLevelVertex vertex);

	void addCellObserved(int index);
	double getCellReward(int index);

	int getCellObservedTimes(int index);
	void calcTotalReward();
	bool hasCellObserved(int index);
	double getTotalReward() { return mTotalReward; };

	void clearCellObsHist() { mCellObsHist.clear(); };

	bool isVisitedCell(int index);

	void printPath();
	void printCellObsHist();

	void clear();


	list<CLevelVertex> mVertexSeq;
	list<CCellObserved> mCellObsHist;

	double mTotalReward;
};

class CStepBatch
{
public:
	CStepBatch();
	virtual ~CStepBatch();

	CLevelVertex * getFirstStep();
	CLevelVertex * getLastStep();
	int size() { return mBatch.size(); };
	void addVertex(CLevelVertex * vertex) { mBatch.push_back(vertex); };
	void print();

	list<CLevelVertex*> mBatch;

	double mReward;
};

class CRobotPathPlanner {
public:
	CRobotPathPlanner(CPathPlanningGraph * graph, CAgent * agent);
	virtual ~CRobotPathPlanner();

	void setPlanStep(int step) { mPlanStep = step; };

	void addPath(CPath path);

	CPath findMaxPath();

	bool initObservedCells(CPath & path);

	double scorePath(CPath & path);

    bool init(CVisualHexagonDiscretizedMap * map);

    void printAllPath();

    void plotPathScoreDist();

protected:
	int mPlanStep;
	list<CPath> mAllPathList;
	CDataArrayVisualizer mDataVisualizer;

	CPathPlanningGraph * mpGraph;
	CVisualHexagonDiscretizedMap * mpMap;

	CAgent * mpAgent;


};

#endif /* CROBOTPATHPLANNER_H_ */
