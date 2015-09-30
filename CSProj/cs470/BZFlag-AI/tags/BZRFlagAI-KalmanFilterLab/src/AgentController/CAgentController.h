/*
 * CAgentController.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CAGENTCONTROLLER_H_
#define CAGENTCONTROLLER_H_

#include <vector>
#include "../Base/CMyAgent.h"

class CAgentController {
public:
	CAgentController(int num);
	virtual ~CAgentController();

	virtual void init();
	virtual void run() = 0;
	void stop();
	bool addAgent(int index);

	CMyAgent * getAgent(int index);

protected:
	vector <CMyAgent*> agents;
	bool running;
};

#endif /* CAGENTCONTROLLER_H_ */
