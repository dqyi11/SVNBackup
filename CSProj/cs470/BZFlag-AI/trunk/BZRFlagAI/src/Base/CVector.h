/*
 * CVector.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CVECTOR_H_
#define CVECTOR_H_

class CVector {
public:
	CVector(double x, double y);
	virtual ~CVector();

    const CVector operator+(const CVector &v1);
    CVector& operator+=(const CVector &v1);
    double length();
    void normalize();
    void rotate(double rotateAngle);
    double orientation();
    void scale(double k) { mX = mX/k; mY = mY/k; };

	double mX;
	double mY;
};

#endif /* CVECTOR_H_ */
