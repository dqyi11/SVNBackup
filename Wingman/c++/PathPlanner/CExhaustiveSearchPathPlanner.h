/*
 * CExhaustiveSearchPathPlanner.h
 *
 *  Created on: Jan 20, 2013
 *      Author: walter
 */

#ifndef CEXHAUSTIVESEARCHPATHPLANNER_H_
#define CEXHAUSTIVESEARCHPATHPLANNER_H_

#include "CRobotPathPlanner.h"

class CExhaustiveSearchPathPlanner : public CRobotPathPlanner {
public:
	CExhaustiveSearchPathPlanner(CPathPlanningGraph * graph, CAgent * agent);
	virtual ~CExhaustiveSearchPathPlanner();

    void generateAllPossiblePath();

    void scoreAllPossiblePath();

    void depthFirstSearch(CLevelVertex * start, CPath path, int level);

};

#endif /* CEXHAUSTIVESEARCHPATHPLANNER_H_ */
