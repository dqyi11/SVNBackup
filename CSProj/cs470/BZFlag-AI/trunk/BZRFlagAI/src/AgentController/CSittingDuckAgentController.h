/*
 * CSittingDuckAgentController.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CSITTINGDUCKAGENTCONTROLLER_H_
#define CSITTINGDUCKAGENTCONTROLLER_H_

#include "CClayPigeonAgentController.h"

class CSittingDuckAgentController: public CClayPigeonAgentController {
public:
	CSittingDuckAgentController();
	virtual ~CSittingDuckAgentController();

	virtual void moveAgent();
};

#endif /* CSITTINGDUCKAGENTCONTROLLER_H_ */
