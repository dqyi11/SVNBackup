/*
 * CBenchmarker.h
 *
 *  Created on: Jan 29, 2013
 *      Author: walter
 */

#ifndef CBENCHMARKER_H_
#define CBENCHMARKER_H_

#include <sys/time.h>

enum UnitType { SEC, MSEC, USEC };

class CBenchmarker {
public:
	CBenchmarker();
	virtual ~CBenchmarker();

	void setType(UnitType unit) { mUnitType = unit; };
	UnitType getType() { return mUnitType; };

	void setStartTime();
	void setEndTime();

	double getTimeElasped();

protected:

	double getUnitCount();

private:
	UnitType mUnitType;

	struct timeval mStartTime;
	struct timeval mEndTime;
};

#endif /* CBENCHMARKER_H_ */
