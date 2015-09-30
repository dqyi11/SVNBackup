/*
 * CGridFilter.h
 *
 */

#ifndef CGRIDFILTER_H_
#define CGRIDFILTER_H_

#include "COccGridState.h"
#include "CGridMap.h"

class CGridFilter {
public:
	CGridFilter(double tHit, double fDetection, CGridMap * cmap);
	virtual ~CGridFilter();

	void filter(COccGridState * state);

private:

	double trueHit;
	double falseAlarm;
	double missedDetection; // 1-TrueHit
	double trueMiss;	// 1- FalseAlarm
	CGridMap * map;
};

#endif /* CGRIDFILTER_H_ */
