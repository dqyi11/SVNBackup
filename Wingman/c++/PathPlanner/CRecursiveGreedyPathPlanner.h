/*
 * CRecursiveGreedyPathPlanner.h
 *
 *  Created on: Jan 24, 2013
 *      Author: walter
 */

#ifndef CRECURSIVEGREEDYPATHPLANNER_H_
#define CRECURSIVEGREEDYPATHPLANNER_H_

#include "CRobotPathPlanner.h"

class CRecursiveGreedyPathPlanner: public CRobotPathPlanner {
public:
	CRecursiveGreedyPathPlanner(CPathPlanningGraph * graph, CAgent * agent);
	virtual ~CRecursiveGreedyPathPlanner();
};

#endif /* CRECURSIVEGREEDYPATHPLANNER_H_ */
