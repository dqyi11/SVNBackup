/*
 * CPotentialField.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include <math.h>
#include "CPotentialField.h"
#include "../ControlParams.h"

CPotentialField::CPotentialField() {
	// TODO Auto-generated constructor stub

}

CPotentialField::~CPotentialField() {
	// TODO Auto-generated destructor stub
}

double CPotentialField::getDistance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow((x1 - x2), 2)+ pow((y1 - y2), 2));
}

double CPotentialField::getTheta(double x1, double y1, double x2, double y2)
{
	if (x1==x2)
	{
		if (y1>y2)
		{
		    return PI/2;
		}
		else
		{
			return -PI/2;
		}
	}


	if (y1==y2)
	{
		if (x1>x2)
		{
			return 0;
		}
		else
		{
			return PI;
		}
	}
    return atan2(y1-y2, x1-x2);
}

