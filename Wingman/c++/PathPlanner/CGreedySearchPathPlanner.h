/*
 * CGreedySearchPathPlanner.h
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#ifndef CGREEDYSEARCHPATHPLANNER_H_
#define CGREEDYSEARCHPATHPLANNER_H_

#include "CRobotPathPlanner.h"

class CGreedySearchPathPlanner: public CRobotPathPlanner {
public:
	CGreedySearchPathPlanner(CPathPlanningGraph * graph, CAgent * agent);
	virtual ~CGreedySearchPathPlanner();

	bool doGreedySearch(CLevelVertex * startVertex);

	bool doGreedySearch();

	CLevelVertex * findNextMaxStep(CLevelVertex * vertex);

	CPath getSearchedPath() { return mGeneratedPath; };

	double scoreVertex(CLevelVertex * vertex);

private:
	CPath mGeneratedPath;
};

#endif /* CGREEDYSEARCHPATHPLANNER_H_ */
