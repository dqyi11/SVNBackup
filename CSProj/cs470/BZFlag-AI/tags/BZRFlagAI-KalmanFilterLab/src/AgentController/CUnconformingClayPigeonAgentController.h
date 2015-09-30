/*
 * CUnconformingClayPigeonAgentController.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CUNCONFORMINGCLAYPIGEONAGENTCONTROLLER_H_
#define CUNCONFORMINGCLAYPIGEONAGENTCONTROLLER_H_

#include "CClayPigeonAgentController.h"

class CUnconformingClayPigeonAgentController: public CClayPigeonAgentController {
public:
	CUnconformingClayPigeonAgentController();
	virtual ~CUnconformingClayPigeonAgentController();

	virtual void moveAgent();
};

#endif /* CUNCONFORMINGCLAYPIGEONAGENTCONTROLLER_H_ */
