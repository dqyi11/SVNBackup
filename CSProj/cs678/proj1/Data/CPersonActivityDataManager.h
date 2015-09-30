/*
 * CPersonActivityDataManager.h
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#ifndef CPERSONACTIVITYDATAMANAGER_H_
#define CPERSONACTIVITYDATAMANAGER_H_

#include "CPersonActivityDataArray.h"
#include <list>

using namespace std;

typedef struct inputData {
	double mX;
	double mY;
	double mZ;
	double mType;
	double mResult[11];
};

typedef struct inputEpoch {
	vector<inputData> mSet;
};

typedef struct inputData2 {
	double mX1;
	double mY1;
	double mZ1;
	double mX2;
	double mY2;
	double mZ2;
	double mX3;
	double mY3;
	double mZ3;
	double mX4;
	double mY4;
	double mZ4;
	double mResult;
};

class CPersonActivityDataManager {
public:
	CPersonActivityDataManager();
	virtual ~CPersonActivityDataManager();

	CPersonActivityDataArray * getArray(string seqNum, TagType tag);
	CPersonActivityDataArray * createArray(string seqNum, TagType tag);

	void addSeqNum(string seqNum);

	void print();
	void dumpArrayDataToFile();
	void dumpSeqNumDataToFile();

	void selfcheck();

	void prepareTrainingData(int size, int k);
	void prepareTestingData(int size, int k);

	int getADAListSize() { return mArrayList.size(); };
	CPersonActivityDataArray* getADAList(int i);

	inputData convert(CMetaData data, CPersonActivityDataArray* array);

	void printTrainingData();
	void printTestingData();

	int getTrainingDataSize() { return mTrainingData.size(); };
	inputEpoch * getTrainingData(int i);

	int getTestingDataSize() { return mTestingData.size(); };
	inputEpoch * getTestingData(int i);

	vector<inputEpoch> * getTrainingData();
	vector<inputEpoch> * getTestingData();

	vector<CPersonActivityDataArray*> mArrayList;
	vector<string> mSeqNumList;

	vector<inputEpoch> mTrainingData;
	vector<inputEpoch> mTestingData;
};

#endif /* CPERSONACTIVITYDATAMANAGER_H_ */
