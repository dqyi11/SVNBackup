/*
 * CDataArrayVisualizer.h
 *
 *  Created on: Jan 23, 2013
 *      Author: walter
 */

#ifndef CDATAARRAYVISUALIZER_H_
#define CDATAARRAYVISUALIZER_H_

#include <list>

using namespace std;

class CDataArrayVisualizer {
public:
	CDataArrayVisualizer();
	virtual ~CDataArrayVisualizer();

	void addData(double value) { mDataValue.push_back(value); };
	void clearData() { mDataValue.clear(); };

	void plot();

	void sortData();

	list<double> mDataValue;
};

#endif /* CDATAARRAYVISUALIZER_H_ */
