/*
 * CObstacleTangentialPotentialField.h
 *
 *  Created on: Oct 1, 2012
 *      Author: walter
 */

#ifndef COBSTACLETANGENTIALPOTENTIALFIELD_H_
#define COBSTACLETANGENTIALPOTENTIALFIELD_H_

#include "../Base/CObstacle.h"
#include "../ControlParams.h"
#include "CTangentialPotentialField.h"

class CObstacleTangentialPotentialField  : public CTangentialPotentialField{
public:
	CObstacleTangentialPotentialField(CObstacle * obstaclePtr);
	virtual ~CObstacleTangentialPotentialField();
	void init();

	CObstacle * obstacle;
};

#endif /* COBSTACLETANGENTIALPOTENTIALFIELD_H_ */
