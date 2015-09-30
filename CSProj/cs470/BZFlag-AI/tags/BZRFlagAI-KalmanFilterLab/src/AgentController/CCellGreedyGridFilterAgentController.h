/*
 * CCellGreedyGridFilterAgentController.h
 *
 *  Created on: Nov 10, 2012
 *      Author: walter
 */

#ifndef CCELLGREEDYGRIDFILTERAGENTCONTROLLER_H_
#define CCELLGREEDYGRIDFILTERAGENTCONTROLLER_H_

#include "CCellBasedGridFilterAgentController.h"

class CCellGreedyGridFilterAgentController: public CCellBasedGridFilterAgentController {
public:
	CCellGreedyGridFilterAgentController(int num, int grid_size);
	virtual ~CCellGreedyGridFilterAgentController();

	virtual void moveAgent(int index);
	virtual void init() {};

	int getNextGridIndex(int index);



private:
	bool * mpInTargetRegion;
};

#endif /* CCELLGREEDYGRIDFILTERAGENTCONTROLLER_H_ */
