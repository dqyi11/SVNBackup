/*
 * CRandomTargetGridFilterAgentController.h
 *
 *  Created on: Nov 9, 2012
 *      Author: walter
 */

#ifndef CRANDOMTARGETGRIDFILTERAGENTCONTROLLER_H_
#define CRANDOMTARGETGRIDFILTERAGENTCONTROLLER_H_

#include "CGridFilterAgentController.h"

class CRandomTargetGridFilterAgentController: public CGridFilterAgentController {
public:
	CRandomTargetGridFilterAgentController(int num);
	virtual ~CRandomTargetGridFilterAgentController();

	virtual void moveAgent(int index);

private:
	double mNearTargetThreshold;
	bool * mpRegenerateTarget;
};

#endif /* CRANDOMTARGETGRIDFILTERAGENTCONTROLLER_H_ */
