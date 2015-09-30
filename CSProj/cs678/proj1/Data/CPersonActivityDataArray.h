/*
 * CPersonActivityDataArray.h
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#ifndef CPERSONACTIVITYDATAARRAY_H_
#define CPERSONACTIVITYDATAARRAY_H_

#include <string>
#include <vector>
#include "CPersonActivityData.h"

using namespace std;

typedef struct CMetaData
{
	double mX;
	double mY;
	double mZ;
	ActivityType mActivity;
	long long int mTimeStamp;
};

class CPersonActivityDataArray {
public:
	CPersonActivityDataArray(string seqNum, TagType tag);
	virtual ~CPersonActivityDataArray();

	void addMetaData(CMetaData & data);

	string getSeqNum() { return mSeqNum; };
	TagType getTag() { return mTag; };

	vector<CMetaData> mPosInfoSeq;

	void print();

	bool checkSequence();

	void dumpToFile();
private:
	string mSeqNum;
	TagType mTag;
};

#endif /* CPERSONACTIVITYDATAARRAY_H_ */
