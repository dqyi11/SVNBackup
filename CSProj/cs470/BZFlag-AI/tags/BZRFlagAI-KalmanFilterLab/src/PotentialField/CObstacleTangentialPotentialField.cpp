/*
 * CObstacleTangentialPotentialField.cpp
 *
 *  Created on: Oct 1, 2012
 *      Author: walter
 */

#include "CObstacleTangentialPotentialField.h"

CObstacleTangentialPotentialField::CObstacleTangentialPotentialField(CObstacle * obstaclePtr)
    : CTangentialPotentialField(0.0,0.0,0.0,0.0,0.0)
{
	// TODO Auto-generated constructor stub
	obstacle = obstaclePtr;

	init();

}

CObstacleTangentialPotentialField::~CObstacleTangentialPotentialField() {
	// TODO Auto-generated destructor stub
}

void CObstacleTangentialPotentialField::init()
{

	tanX = obstacle->getMidPointX();
	tanY = obstacle->getMidPointY();
	tanRadius = obstacle->getRadius();

	/*
	for(int i = 0; i < obstacle->xvals.size();i++)
	{
		cout<<"we have:" <<obstacle->xvals[i] << " " << obstacle->yvals[i] <<endl;
	}
	cout << "mid is:" << obstacle->getMidPointX() << " " << obstacle->getMidPointY() << " " << obstacle->getRadius() << endl;

	//cout <<"init obs:" << obsX << " " << obsY << " " << obsRadius << endl;
    */
	tanBeta = TANGENTIAL_GAMMA;


	// double totalDistance = sqrt(pow((agentX-obsX), 2)+pow((agentY-obsY),2));
	tanSpreadingRange = OBSTACLE_TAN_SPREADING_RANGE; //totalDistance - obsRadius;


}
