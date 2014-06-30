/*
 * CRecedingHorizonPathPlanner.h
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#ifndef CRECEDINGHORIZONPATHPLANNER_H_
#define CRECEDINGHORIZONPATHPLANNER_H_

#include "CRobotPathPlanner.h"
#include "../Utilities/CBenchmarker.h"

class CRecedingHorizonPathPlanner: public CRobotPathPlanner {
public:
	CRecedingHorizonPathPlanner(CPathPlanningGraph * graph, CAgent * agent);
	virtual ~CRecedingHorizonPathPlanner();

	void setRecedingHorizonLength(int length) { mRecedingHorizonLength = length; };
	int getRecedingHorizonLength() { return mRecedingHorizonLength; };

	bool doRecedingHorizonSearch();
	bool doRecedingHorizonSearch(CLevelVertex * startVertex);

	CStepBatch findNextMaxStepBatch(CLevelVertex * vertex);

	void depthFirstLimitedSearch(CLevelVertex * start, CStepBatch batch, int level);

	void scoreAllPossibleBatches();

	double scoreBatch(CStepBatch & batch);

	CStepBatch getMaxStepBatch();

	CPath getSearchedPath() { return mGeneratedPath; };

private:
	CPath mGeneratedPath;
	int mRecedingHorizonLength;
	CBenchmarker mBenchmark;

	list<CStepBatch> mAllPossibleBatches;
};

#endif /* CRECEDINGHORIZONPATHPLANNER_H_ */
