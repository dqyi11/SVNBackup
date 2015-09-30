/*
 * CRepulsivePotentialField.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CREPULSIVEPOTENTIALFIELD_H_
#define CREPULSIVEPOTENTIALFIELD_H_

#include "CPotentialField.h"

using namespace std;

class CRepulsivePotentialField: public CPotentialField {
public:
	CRepulsivePotentialField(double oX, double oY, double radius, double spreadingRange, double beta);
	virtual ~CRepulsivePotentialField();
	virtual CVector getVector(double x, double y);
	double sign(double x);

	double obsX;
	double obsY;
	double obsRadius;
	double oSpreadingRange;
	double oBeta;
};

#endif /* CREPULSIVEPOTENTIALFIELD_H_ */
