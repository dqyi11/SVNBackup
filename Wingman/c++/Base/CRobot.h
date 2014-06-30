/*
 * CRobot.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CROBOT_H_
#define CROBOT_H_

#include "CAgent.h"

class CRobot : public CAgent{
public:
	CRobot();
	virtual ~CRobot();

	virtual void updateByObservation(CGridSet grids);
};

#endif /* CROBOT_H_ */
