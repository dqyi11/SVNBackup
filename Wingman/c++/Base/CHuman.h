/*
 * CHuman.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CHUMAN_H_
#define CHUMAN_H_

#include "CAgent.h"

class CHuman : public CAgent {
public:
	CHuman();
	virtual ~CHuman();

	virtual void updateByObservation(CGridSet grids);

	void setWingmanRadius(int radius) { mWingmanRadius = radius; };
	int getWingmanRadius() { return mWingmanRadius; };

	int mWingmanRadius;
};

#endif /* CHUMAN_H_ */
