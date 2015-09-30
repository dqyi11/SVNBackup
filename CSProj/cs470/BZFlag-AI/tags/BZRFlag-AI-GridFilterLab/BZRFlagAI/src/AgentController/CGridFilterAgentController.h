/*
 * CGridFilterAgentController.h
 *
 *  Created on: Nov 3, 2012
 *      Author: walter
 */

#ifndef CGRIDFILTERAGENT_CONTROLLER_H_
#define CGRIDFILTERAGENT_CONTROLLER_H_

#include "../AgentController/CAgentController.h"
#include "../GridFilter/CVisualizedGridMap.h"
#include "../GridFilter/CGridFilter.h"

class CGridFilterAgentController : public CAgentController
{
public:
	CGridFilterAgentController(int num);
	virtual ~CGridFilterAgentController();

	virtual void init();
	void deinit();
	virtual void run();
	virtual void moveAgent(int index)=0;

	void showMap();

protected:
	CVisualizedGridMap * mpGridMap;
	CGridFilter * mpFilter;
	vector<int> mAgentInnerCounter;

};

#endif /* CGRIDFILTERAGENT_CONTROLLER_H_ */
