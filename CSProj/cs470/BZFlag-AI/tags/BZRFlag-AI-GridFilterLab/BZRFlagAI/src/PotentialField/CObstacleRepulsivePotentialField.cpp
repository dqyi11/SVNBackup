/*
 * ObstacleRepulsivePotentialField.cpp
 *
 *  Created on: Sep 30, 2012
 *      Author: fausto
 */

#include "CObstacleRepulsivePotentialField.h"

CObstacleRepulsivePotentialField::CObstacleRepulsivePotentialField(CObstacle * obstaclePtr)
		: CRepulsivePotentialField(0.0,0.0,0.0,0.0,0.0) {
	obstacle = obstaclePtr;

	init();

}

CObstacleRepulsivePotentialField::~CObstacleRepulsivePotentialField() {
	// TODO Auto-generated destructor stub
}

void CObstacleRepulsivePotentialField::init()
{

	obsX = obstacle->getMidPointX();
	obsY = obstacle->getMidPointY();
	obsRadius = obstacle->getRadius();
	// obsRadius = 50;

	/*
	for(int i = 0; i < obstacle->xvals.size();i++)
	{
		cout<<"we have:" <<obstacle->xvals[i] << " " << obstacle->yvals[i] <<endl;
	}
	cout << "mid is:" << obstacle->getMidPointX() << " " << obstacle->getMidPointY() << " " << obstacle->getRadius() << endl;

	//cout <<"init obs:" << obsX << " " << obsY << " " << obsRadius << endl;
    */
	oBeta = REPULSIVE_BETA;


	// double totalDistance = sqrt(pow((agentX-obsX), 2)+pow((agentY-obsY),2));
	oSpreadingRange = OBSTACLE_REP_SPREADING_RANGE; //totalDistance - obsRadius;


}

