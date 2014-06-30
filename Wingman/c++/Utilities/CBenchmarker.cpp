/*
 * CBenchmarker.cpp
 *
 *  Created on: Jan 29, 2013
 *      Author: walter
 */

#include "CBenchmarker.h"
#include <unistd.h>

CBenchmarker::CBenchmarker() {
	// TODO Auto-generated constructor stub
	mUnitType = USEC;
}

CBenchmarker::~CBenchmarker() {
	// TODO Auto-generated destructor stub
}

void CBenchmarker::setStartTime()
{
	gettimeofday(&mStartTime, NULL);
}

void CBenchmarker::setEndTime()
{
	gettimeofday(&mEndTime, NULL);
}

double CBenchmarker::getTimeElasped()
{
	double deltaTime = (mEndTime.tv_sec-mStartTime.tv_sec) * 1000000
				  + (mEndTime.tv_usec-mStartTime.tv_usec);
	return deltaTime / getUnitCount();
}

double CBenchmarker::getUnitCount()
{
	double count = 0;
	switch(mUnitType)
	{
	case SEC:
		count = 1000000;
		break;
	case MSEC:
		count = 1000;
		break;
	case USEC:
	default:
		count = 1;
		break;
	}

	return count;
}
