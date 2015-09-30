/*
 * CConformingClayPigeonAgentController.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CCONFORMINGCLAYPIGEONAGENTCONTROLLER_H_
#define CCONFORMINGCLAYPIGEONAGENTCONTROLLER_H_

#include "CClayPigeonAgentController.h"

class CConformingClayPigeonAgentController: public CClayPigeonAgentController {
public:
	CConformingClayPigeonAgentController();
	virtual ~CConformingClayPigeonAgentController();

	virtual void moveAgent();
};

#endif /* CCONFORMINGCLAYPIGEONAGENTCONTROLLER_H_ */
