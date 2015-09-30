/*
 * CShot.h
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#ifndef CSHOT_H_
#define CSHOT_H_

class CShot {
public:
	CShot(double x, double y, double vx, double vy);
	virtual ~CShot();

    double mX;
    double mY;
    double mVX;
    double mVY;

};

#endif /* CSHOT_H_ */
