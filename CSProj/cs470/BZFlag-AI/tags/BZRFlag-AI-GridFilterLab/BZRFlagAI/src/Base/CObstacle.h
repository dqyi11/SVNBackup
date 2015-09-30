/*
 * CObstacle.h
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#ifndef COBSTACLE_H_
#define COBSTACLE_H_

#include <stdlib.h>
#include <vector>
#include <iostream>
#include "CBaseObject.h"
#include <math.h>
#include <assert.h>

using namespace std;


class CObstacle : public CBaseObject
{
public:
	CObstacle();
	virtual ~CObstacle();

	virtual void print();

	double getMidPointX();
	double getMidPointY();
	double getRadius();

	vector <double> xvals;
	vector <double> yvals;



};

#endif /* COBSTACLE_H_ */
