/*
 * ObstacleRepulsivePotentialField.h
 *
 *  Created on: Sep 30, 2012
 *      Author: fausto
 */

#ifndef OBSTACLEREPULSIVEPOTENTIALFIELD_H_
#define OBSTACLEREPULSIVEPOTENTIALFIELD_H_

#include "CRepulsivePotentialField.h"
#include "CObstacle.h"
#include "ControlParams.h"

class CObstacleRepulsivePotentialField : public CRepulsivePotentialField{
public:
	CObstacleRepulsivePotentialField(CObstacle * obstaclePtr);
	virtual ~CObstacleRepulsivePotentialField();
	void init();

	CObstacle * obstacle;
};

#endif /* OBSTACLEREPULSIVEPOTENTIALFIELD_H_ */
