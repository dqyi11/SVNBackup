/*
 * CGridMap.h
 *
 *  Created on: Oct 30, 2012
 *      Author: walter
 */

#ifndef CGRIDMAP_H_
#define CGRIDMAP_H_

class CGridMap {
public:
	CGridMap(int width, int height);
	virtual ~CGridMap();

	double getProbability(int x, int y);
	bool  setProbability(int x, int y, double prob);
	void printMap();

protected:
	bool convertPosToIndex(int x, int y, int & index);
	bool convertIndexToPos(int index, int & x, int & y);

	int mGridWidth;
	int mGridHeight;
	int mGridNum;

	double ** mpProbablisitcMap;
};

#endif /* CGRIDMAP_H_ */
