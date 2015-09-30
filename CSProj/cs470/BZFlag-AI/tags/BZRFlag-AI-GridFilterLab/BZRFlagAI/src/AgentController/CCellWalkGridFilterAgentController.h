/*
 * CCellWalkGridFilterAgentController.h
 *
 *  Created on: Nov 7, 2012
 *      Author: walter
 */

#ifndef CCELLWALKGRIDFILTERAGENTCONTROLLER_H_
#define CCELLWALKGRIDFILTERAGENTCONTROLLER_H_

#include "CCellBasedGridFilterAgentController.h"

// INT FOR STATUS
// 0 == UNVISITED
// 1 == VISITED

class CCellWalkGridFilterAgentController : public CCellBasedGridFilterAgentController {
public:
	CCellWalkGridFilterAgentController(int num, int grid_size);
	virtual ~CCellWalkGridFilterAgentController();

	virtual void moveAgent(int index);
	void init();

	int findUnvisitedCellInRegion(int x, int y, int w, int h);

	int getNextGridIndex(int index);


};

#endif /* CCELLWALKGRIDFILTERAGENTCONTROLLER_H_ */
