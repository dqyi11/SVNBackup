/*
 * CDumbAgent.h
 *
 *  Created on: Sep 29, 2012
 *      Author: fausto
 */

#ifndef CDUMBAGENT_H_
#define CDUMBAGENT_H_

#include "CAgentController.h"

class CDumbAgentController : public CAgentController
{
public:
	CDumbAgentController(int num);
	virtual ~CDumbAgentController();

	virtual void run();

};

#endif /* CDUMBAGENT_H_ */
