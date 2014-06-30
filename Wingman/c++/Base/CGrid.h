/*
 * CGrid.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CGRID_H_
#define CGRID_H_

#include <list>

using namespace std;

enum GridType { EMPTY, OBSTACLE };

class CGrid {
public:
	CGrid();
	virtual ~CGrid();

	int mX;
	int mY;

	int mIndexId;

	double mProbabilityValue;

	GridType mType;
};

class CGridSet {
public:
	CGridSet();
	virtual ~CGridSet();

	bool hasGrid(CGrid * pGrid);
	void addGrid(CGrid * pGrid);

	void removeGrid(CGrid * pGrid);

	void append(CGridSet otherSet);
	void clear() { mSet.clear(); };

	void print();

    list<CGrid*> mSet;
};

#endif /* CGRID_H_ */
