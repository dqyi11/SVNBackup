/*
 * CPotentialFieldAgentController.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CPOTENTIALFIELDAGENTCONTROLLER_H_
#define CPOTENTIALFIELDAGENTCONTROLLER_H_

#include "CAgentController.h"
#include "CController.h"
#include "CPDController.h"
#include "CPotentialFieldManager.h"
#include "CObstacleTangentialPotentialField.h"

class CVector;
//class CPotentialFieldManager;

class CPotentialFieldAgentController : public CAgentController, public CPDController
{
public:
	CPotentialFieldAgentController(int num);
	virtual ~CPotentialFieldAgentController();

	virtual void run();

	CPotentialFieldManager * getPotentialFieldManager() { return mpPFMgr; };
protected:
	ControlParams calcControlParams(CMyAgent * agent, double goalX, double goalY, CVector potentialVector);

private:
	double mCtrlStepTime;
	CPotentialFieldManager * mpPFMgr;

	double mLastPosX;
	double mLastPosY;
};

#endif /* CPOTENTIALFIELDAGENTCONTROLLER_H_ */
