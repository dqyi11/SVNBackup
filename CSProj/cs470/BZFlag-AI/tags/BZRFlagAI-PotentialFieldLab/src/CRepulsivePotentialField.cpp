/*
 * CRepulsivePotentialField.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include "CRepulsivePotentialField.h"
#include <math.h>
#include "ControlParams.h"
#include <limits>

CRepulsivePotentialField::CRepulsivePotentialField(double oX, double oY, double radius, double spreadingRange, double beta) {
	obsX = oX;
	obsY = oY;
	obsRadius = radius;
	oSpreadingRange = spreadingRange;
	oBeta = beta;
}

CRepulsivePotentialField::~CRepulsivePotentialField() {
	// TODO Auto-generated destructor stub
}

CVector CRepulsivePotentialField::getVector(double x, double y)
{
	CVector v(0,0);
	double distance = getDistance(obsX, obsY, x, y);
	double theta = getTheta(obsX, obsY, x, y);

	if (distance < obsRadius)
	{
		v.mX = -sign(cos(theta)) * LIMIT_MAX; // numeric_limits<double>::max() / 2;
		//double.MAX;
		v.mY = -sign(sin(theta)) * LIMIT_MAX; // numeric_limits<double>::max() / 2;
		return v;
	}
	else if((distance >= obsRadius)
			&& (distance <= obsRadius + oSpreadingRange))
	{
		v.mX = -oBeta * (oSpreadingRange + obsRadius - distance) * cos(theta);
		v.mY = -oBeta * (oSpreadingRange + obsRadius - distance) * sin(theta);
		return v;
	}
	else if(distance > oSpreadingRange + obsRadius)
	{
		v.mX = 0;
		v.mY = 0;
	}

	return v;


}

double CRepulsivePotentialField::sign(double x){
		if(x < 0){
			return -1;
		}

		if(x > 0){
			return 1;
		}

		return 0;
}

