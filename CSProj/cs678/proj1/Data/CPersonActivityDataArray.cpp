/*
 * CPersonActivityDataArray.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#include "CPersonActivityDataArray.h"
#include <iostream>
#include <fstream>
#include <string.h>

using namespace std;

CPersonActivityDataArray::CPersonActivityDataArray(string seqNum, TagType tag)
{
	// TODO Auto-generated constructor stub
	mSeqNum = seqNum;
	mTag = tag;
}

CPersonActivityDataArray::~CPersonActivityDataArray() {
	// TODO Auto-generated destructor stub
	mPosInfoSeq.clear();
}

void CPersonActivityDataArray::addMetaData(CMetaData & data)
{
	mPosInfoSeq.push_back(data);
}

void CPersonActivityDataArray::print()
{
	cout << "SEQ:" << mSeqNum <<",Tag:" << mTag << endl;
	vector<CMetaData>::iterator it;
	for(it=mPosInfoSeq.begin();it!=mPosInfoSeq.end();it++)
	{
		cout << "Time:" <<(*it).mTimeStamp
				<< ",X:" << (*it).mX
				<< ",Y:" << (*it).mY
				<< ",Z:" << (*it).mZ << endl;
	}

}

bool CPersonActivityDataArray::checkSequence()
{
	int seqSize = mPosInfoSeq.size();
	for(int i=1;i<seqSize;i++)
	{
		if(mPosInfoSeq[i].mTimeStamp < mPosInfoSeq[i-1].mTimeStamp)
		{
			return false;
		}
	}

	return true;
}

void CPersonActivityDataArray::dumpToFile()
{
	ofstream fileWriter;
	char fileName[512];
	memset(fileName,0x0,512);
	sprintf(fileName,"hmB_%s_%d.dat",mSeqNum.c_str(),mTag);
	fileWriter.open(fileName);
	vector<CMetaData>::iterator it;
	for(it=mPosInfoSeq.begin();it!=mPosInfoSeq.end();it++)
	{
		fileWriter <<(*it).mTimeStamp
				<< " " << (*it).mX
				<< " " << (*it).mY
				<< " " << (*it).mZ
				<< " " << (*it).mActivity << endl;
	}
	fileWriter.close();
}
