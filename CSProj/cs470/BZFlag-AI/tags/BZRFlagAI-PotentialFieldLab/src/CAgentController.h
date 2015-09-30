/*
 * CAgentController.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CAGENTCONTROLLER_H_
#define CAGENTCONTROLLER_H_

#include "CMyAgent.h"
#include <vector>

class CAgentController {
public:
	CAgentController(int num);
	virtual ~CAgentController();

	virtual void run() = 0;
	void stop();
	bool addAgent(int index);

protected:
	vector <CMyAgent*> agents;
	bool running;
};

#endif /* CAGENTCONTROLLER_H_ */
