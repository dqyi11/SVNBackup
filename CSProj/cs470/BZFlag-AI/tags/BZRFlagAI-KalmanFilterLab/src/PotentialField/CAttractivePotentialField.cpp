/*
 * CAttractivePotentialField.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include <math.h>
#include <iostream>
#include "CAttractivePotentialField.h"

using namespace std;

CAttractivePotentialField::CAttractivePotentialField(double goalX, double goalY, double radius, double spreadingRange, double alpha)
{
	// TODO Auto-generated constructor stub
	mGoalX = goalX;
	mGoalY = goalY;
	mGoalRadius = radius;
	mSpreadingRange = spreadingRange;
	mAlpha = alpha;

}

CAttractivePotentialField::~CAttractivePotentialField()
{
	// TODO Auto-generated destructor stub
}

CVector CAttractivePotentialField::getVector(double x, double y)
{
	CVector v(0,0);
	double distance = 0.0;

	double theta = 0.0;

	// cout << "GX:"<<mGoalX<<"GY:"<<mGoalY<<"X"<<x<<"Y"<<y<<endl;
	// cout << "vector:" << distance << " " << theta << endl;

	if((mGoalX==x) && (mGoalY==y))
	{
		return v;
	}

	distance = getDistance(mGoalX, mGoalY, x, y);
	theta = getTheta(mGoalX, mGoalY, x, y);

	if (distance <= mGoalRadius)
	{
		return v;
	}
	else if((distance > mGoalRadius)
			&& (distance < mGoalRadius + mSpreadingRange))
	{
		v.mX = mAlpha * (distance - mGoalRadius) * cos(theta);
		v.mY = mAlpha * (distance - mGoalRadius) * sin(theta);
	}
	else if(distance >= mGoalRadius + mSpreadingRange)
	{
		v.mX = mAlpha * mSpreadingRange * cos(theta);
		v.mY = mAlpha * mSpreadingRange * sin(theta);
	}

	// cout <<"get v:"<<v.mX<<"+"<<v.mY<<endl;

	return v;

}

