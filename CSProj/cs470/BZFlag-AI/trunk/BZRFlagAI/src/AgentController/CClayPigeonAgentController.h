/*
 * CClayPigeonAgentController.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CCLAYPIGEONAGENTCONTROLLER_H_
#define CCLAYPIGEONAGENTCONTROLLER_H_

#include "CAgentController.h"

class CClayPigeonAgentController: public CAgentController {
public:
	CClayPigeonAgentController();
	virtual ~CClayPigeonAgentController();

	virtual void run();
	virtual void moveAgent()=0;
protected:

	CMyAgent * mpAgent;
};

#endif /* CCLAYPIGEONAGENTCONTROLLER_H_ */
