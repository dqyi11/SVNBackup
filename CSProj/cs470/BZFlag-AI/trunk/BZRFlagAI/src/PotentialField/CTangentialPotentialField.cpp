/*
 * CTangentialPotentialField.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include <math.h>
#include <limits>
#include "CTangentialPotentialField.h"
#include "../ControlParams.h"

//#define PI     3.14159265358979323846

using namespace std;

CTangentialPotentialField::CTangentialPotentialField(double tX, double tY, double radius, double spreadingRange, double beta, bool dir)
{
	tanX = tX;
	tanY = tY;
	tanRadius = radius;
	tanSpreadingRange = spreadingRange;
	tanBeta = beta;
	direction = dir;

}

CTangentialPotentialField::~CTangentialPotentialField() {
	// TODO Auto-generated destructor stub
}

CVector CTangentialPotentialField::getVector(double x, double y){
	CVector v(0,0);
	double distance = getDistance(tanX, tanY, x, y);
	double theta = getTheta(tanX, tanY, x, y);

	if (distance < tanRadius)
	{
		v.mX = -sign(cos(theta)) * LIMIT_MAX; // numeric_limits<double>::max();
		v.mY = -sign(sin(theta)) * LIMIT_MAX; // numeric_limits<double>::max();

		if(direction){
			v.rotate(PI/2);
		}else{
			v.rotate(-PI/2);
		}
		return v;
	}
	else if((distance >= tanRadius)
			&& (distance <= tanRadius + tanSpreadingRange))
	{
		v.mX = -tanBeta * (tanSpreadingRange + tanRadius - distance) * cos(theta);
		v.mY = -tanBeta * (tanSpreadingRange + tanRadius - distance) * sin(theta);

		if(direction){
			v.rotate(PI/2);
		}else{
			v.rotate(-PI/2);
		}
		return v;
	}
	else if(distance > tanSpreadingRange + tanRadius)
	{
		v.mX = 0;
		v.mY = 0;
	}

	return v;

}

double CTangentialPotentialField::sign(double x){
		if(x < 0){
			return -1;
		}

		if(x > 0){
			return 1;
		}

		return 0;
}


