/*
 * CTangentialPotentialField.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CTANGENTIALPOTENTIALFIELD_H_
#define CTANGENTIALPOTENTIALFIELD_H_

#include "CPotentialField.h"

class CTangentialPotentialField: public CPotentialField {
public:
	CTangentialPotentialField(double tX, double tY, double radius, double spreadingRange, double beta, bool dir = false);
	virtual ~CTangentialPotentialField();
	double sign(double x);

	virtual CVector getVector(double x, double y);
	void changeDirection(){ direction ? direction = false: direction = true;};
	bool getDirection(){return direction;};

protected:
	double tanX;
	double tanY;
	double tanRadius;
	double tanSpreadingRange;
	double tanBeta;

	bool direction;
};

#endif /* CTANGENTIALPOTENTIALFIELD_H_ */
