/*
 * CVector.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include "CVector.h"
#include <math.h>

CVector::CVector(double x, double y)
{
	// TODO Auto-generated constructor stub
	mX = x;
	mY = y;

}

CVector::~CVector() {
	// TODO Auto-generated destructor stub
}

const CVector CVector::operator+(const CVector &v1)
{
	CVector v0(this->mX, this->mY);
	v0 += v1;
	return v0;
}

CVector& CVector::operator+=(const CVector &v1)
{
	CVector v0(0,0);
	v0.mX = v0.mX + v1.mX;
	v0.mY = v0.mY + v1.mY;

	return v0;
}

double CVector::length()
{
	return sqrt(pow(mX, 2)+pow(mY,2));
}

void CVector::normalize()
{
	double vectorLength = length();
	if (length() > 0)
	{
	    mX = mX / vectorLength;
	    mY = mY / vectorLength;
	}

}

void CVector::rotate(double rotateAngle)
{
	double vectorLength = length();
	double vectorOrientation = orientation();

	mX = vectorLength * cos(vectorOrientation + rotateAngle);
	mY = vectorLength * sin(vectorOrientation + rotateAngle);

}

double CVector::orientation()
{
	return atan2(mY, mX);

}
