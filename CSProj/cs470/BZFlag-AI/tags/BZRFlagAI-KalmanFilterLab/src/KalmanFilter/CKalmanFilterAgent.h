/*
 * CKalmanFilterAgent.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CKALMANFILTERAGENT_H_
#define CKALMANFILTERAGENT_H_

#include "../Base/CAgent.h"
#include "CVisualizedKalmanFilter.h"

class CKalmanFilterAgent {
public:
	CKalmanFilterAgent(CAgent * agent);
	virtual ~CKalmanFilterAgent();

	void setUpdatePeriodLength(double length) { mUpdatePeriodLength = length; };
	void initFilter();
	void updateFilter(CAgent * agent);

	CAgent * mpAgent;
	CVisualizedKalmanFilter * mpKalmanFilter;
	double mUpdatePeriodLength;
};

#endif /* CKALMANFILTERAGENT_H_ */
