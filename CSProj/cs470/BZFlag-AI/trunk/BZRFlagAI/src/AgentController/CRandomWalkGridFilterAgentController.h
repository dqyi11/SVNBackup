/*
 * CRandomWalkGridFilterAgentController.h
 *
 *  Created on: Nov 6, 2012
 *      Author: walter
 */

#ifndef CRANDOMWALKGRIDFILTERAGENTCONTROLLER_H_
#define CRANDOMWALKGRIDFILTERAGENTCONTROLLER_H_

#include "CGridFilterAgentController.h"

class CRandomWalkGridFilterAgentController: public CGridFilterAgentController {
public:
	CRandomWalkGridFilterAgentController(int num);
	virtual ~CRandomWalkGridFilterAgentController();

	virtual void moveAgent(int index);
};

#endif /* CRANDOMWALKGRIDFILTERAGENTCONTROLLER_H_ */
