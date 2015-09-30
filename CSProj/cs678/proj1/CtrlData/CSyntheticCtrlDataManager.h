/*
 * CSyntheticCtrlDataManager.h
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#ifndef CSYNTHETICCTRLDATAMANAGER_H_
#define CSYNTHETICCTRLDATAMANAGER_H_

#include "CCtrlData.h"
#include <vector>

using namespace std;

typedef struct ctrlData {
	double mX;
	double mResult[6];
};

typedef struct ctrlEpoch {
	vector<ctrlData> mSet;
};

class CSyntheticCtrlDataManager {
public:
	CSyntheticCtrlDataManager();
	virtual ~CSyntheticCtrlDataManager();

	void prepareTrainingData(int size, int k);
	void prepareTestingData(int size, int k);

	void printTrainingData();
	void printTestingData();

	int getTrainingDataSize() { return mTrainingData.size(); };
	ctrlEpoch * getTrainingData(int i);

	int getTestingDataSize() { return mTestingData.size(); };
	ctrlEpoch * getTestingData(int i);

	vector<ctrlEpoch> * getTrainingData();
	vector<ctrlEpoch> * getTestingData();

	vector<ctrlEpoch> mTrainingData;
	vector<ctrlEpoch> mTestingData;

	CCtrlData mSource;
};

#endif /* CSYNTHETICCTRLDATAMANAGER_H_ */
