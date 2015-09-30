/*
 * CPotentialFieldAgentController.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CPOTENTIALFIELDAGENTCONTROLLER_H_
#define CPOTENTIALFIELDAGENTCONTROLLER_H_

#include "../AgentController/CAgentController.h"
#include "../Base/CController.h"
#include "../Base/CPDController.h"
#include "../PotentialField/CPotentialFieldManager.h"
#include "../PotentialField/CObstacleTangentialPotentialField.h"

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
