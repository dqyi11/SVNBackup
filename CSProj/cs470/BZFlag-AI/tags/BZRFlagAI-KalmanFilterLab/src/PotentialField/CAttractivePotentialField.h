/*
 * CAttractivePotentialField.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CATTRACTIVEPOTENTIALFIELD_H_
#define CATTRACTIVEPOTENTIALFIELD_H_

#include "CPotentialField.h"

const double ALPHA_CONSTANT = 1;

class CAttractivePotentialField : public CPotentialField
{
public:
	CAttractivePotentialField(double goalX, double goalY, double radius, double spreadingRange, double alpha = ALPHA_CONSTANT);
	virtual ~CAttractivePotentialField();

	virtual CVector getVector(double x, double y);

	double mGoalX;
	double mGoalY;
	double mGoalRadius;
	double mSpreadingRange;
	double mAlpha;
};

#endif /* CATTRACTIVEPOTENTIALFIELD_H_ */
