/*
 * CBatchSearchPathPlanner.h
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#ifndef CBATCHSEARCHPATHPLANNER_H_
#define CBATCHSEARCHPATHPLANNER_H_

#include "CRobotPathPlanner.h"
#include "../Utilities/CBenchmarker.h"


class CBatchSearchPathPlanner: public CRobotPathPlanner {
public:
	CBatchSearchPathPlanner(CPathPlanningGraph * graph, CAgent * agent);
	virtual ~CBatchSearchPathPlanner();

	void setBatchCnt(int batchCnt) { mBatchCnt = batchCnt; };
	int getBatchCnt() { return mBatchCnt; };

	bool doBatchSearch();
	bool doBatchSearch(CLevelVertex * startVertex);

	CStepBatch findNextMaxStepBatch(CLevelVertex * vertex);

	void depthFirstLimitedSearch(CLevelVertex * start, CStepBatch batch, int level);

	void scoreAllPossibleBatches();

	double scoreBatch(CStepBatch & batch);

	CStepBatch getMaxStepBatch();

	CPath getSearchedPath() { return mGeneratedPath; };

private:
	CPath mGeneratedPath;
	int mBatchCnt;
	CBenchmarker mBenchmark;

	list<CStepBatch> mAllPossibleBatches;
};

#endif /* CBATCHSEARCHPATHPLANNER_H_ */
