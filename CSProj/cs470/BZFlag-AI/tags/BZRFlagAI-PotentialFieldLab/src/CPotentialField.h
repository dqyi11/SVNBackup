/*
 * CPotentialField.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CPOTENTIALFIELD_H_
#define CPOTENTIALFIELD_H_

#include "CVector.h"

class CPotentialField {
public:
	CPotentialField();
	virtual ~CPotentialField();

	virtual CVector getVector(double x, double y) = 0;

	double getDistance(double x1, double y1, double x2, double y2);
	double getTheta(double x1, double y1, double x2, double y2);
};

#endif /* CPOTENTIALFIELD_H_ */
