/*
 * CDumbAgentController.h
 *
 *  Created on: Sep 29, 2012
 *      Author: fausto
 */

#ifndef CDUMBAGENTCONTROLLER_H_
#define CDUMBAGENTCONTROLLER_H_

#include "CAgentController.h"

class CDumbAgentController : public CAgentController
{
public:
	CDumbAgentController(int num);
	virtual ~CDumbAgentController();

	virtual void run();

};

#endif /* CDUMBAGENT_H_ */
