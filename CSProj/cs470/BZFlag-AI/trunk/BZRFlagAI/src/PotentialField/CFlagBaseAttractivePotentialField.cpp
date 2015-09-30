/*
 * CBaseAttractivePotentialField.cpp
 *
 *  Created on: Sep 29, 2012
 *      Author: walter
 */

#include <math.h>
#include "CFlagBaseAttractivePotentialField.h"
#include "../Base/CWorldController.h"
#include "../ControlParams.h"

CFlagBaseAttractivePotentialField::CFlagBaseAttractivePotentialField(CFlag * flag, CBase * base)
    : CAttractivePotentialField(0.0, 0.0, 0.0, 0.0, 0.0)
{
	// TODO Auto-generated constructor stub
	mpBase = base;
	mpFlag = flag;

	init();
}

CFlagBaseAttractivePotentialField::~CFlagBaseAttractivePotentialField() {
	// TODO Auto-generated destructor stub
}

void CFlagBaseAttractivePotentialField::init()
{
	mGoalX = mpFlag->x;
	mGoalY = mpFlag->y;
	mGoalRadius = GET_WORLD()->flagradius;
	// mSpreadingRange = SPREADING_RANGE;
	mAlpha = ATTRACTIVE_ALPHA;

	double totalDistance = 0;
	for(int i=0; i < mpBase->xvals.size(); i++)
	{
		totalDistance = totalDistance +
		    sqrt(pow((mpBase->xvals[i]-mGoalX), 2)+pow((mpBase->yvals[i]-mGoalY),2));

	}

	mSpreadingRange = totalDistance/mpBase->xvals.size();
}

