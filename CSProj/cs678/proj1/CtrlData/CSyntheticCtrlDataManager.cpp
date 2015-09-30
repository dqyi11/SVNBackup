/*
 * CSyntheticCtrlDataManager.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#include "CSyntheticCtrlDataManager.h"
#include "../Utility/CRandomNumGenerator.h"

#include <iostream>

using namespace std;

CSyntheticCtrlDataManager::CSyntheticCtrlDataManager() {
	// TODO Auto-generated constructor stub

}

CSyntheticCtrlDataManager::~CSyntheticCtrlDataManager() {
	// TODO Auto-generated destructor stub
}

void CSyntheticCtrlDataManager::prepareTrainingData(int size, int k)
{
	mTrainingData.clear();

	CRandomNumGenerator rndGen;
	rndGen.setMin(0);
	rndGen.setMax(5);
	int rowStartIndex = 0;
	int rowEndIndex = 99;
	int colStartIndex = 0;
	int colEndIndex = 59;
	CRandomNumGenerator rndGenRw;
	rndGenRw.setMin(rowStartIndex);
	rndGenRw.setMax(rowEndIndex);

	CRandomNumGenerator rndGenCl;
	rndGenCl.setMin(colStartIndex);
	rndGenCl.setMax(colEndIndex-k);

	for(int i=0;i<size;i++)
	{
		int typeIdx = rndGen.getRndNum();

		int rwIdx = rndGenRw.getRndNum();
		int clIdx = rndGenCl.getRndNum();

		ctrlEpoch epoch;
		for(int j=clIdx;j<clIdx+k;j++)
		{

			double value = mSource.getData(typeIdx,rwIdx,j);
			//cout << "prepare data " << typeIdx << "+" << value << endl;
			ctrlData data;
			data.mX = value;
			for(int k=0;k<6;k++)
			{
				data.mResult[k] = 0;
			}
			data.mResult[typeIdx] = 1;

			epoch.mSet.push_back(data);
		}

		mTrainingData.push_back(epoch);

	}

}

void CSyntheticCtrlDataManager::prepareTestingData(int size, int k)
{
	CRandomNumGenerator rndGen;
	rndGen.setMin(0);
	rndGen.setMax(5);
	int rowStartIndex = 0;
	int rowEndIndex = 99;
	int colStartIndex = 0;
	int colEndIndex = 59;
	CRandomNumGenerator rndGenRw;
	rndGenRw.setMin(rowStartIndex);
	rndGenRw.setMax(rowEndIndex);

	CRandomNumGenerator rndGenCl;
	rndGenCl.setMin(colStartIndex);
	rndGenCl.setMax(colEndIndex-k);

	for(int i=0;i<size;i++)
	{
		int typeIdx = rndGen.getRndNum();

		int rwIdx = rndGenRw.getRndNum();
		int clIdx = rndGenCl.getRndNum();

		ctrlEpoch epoch;
		for(int j=clIdx;j<clIdx+k;j++)
		{
			double value = mSource.getData(typeIdx,rwIdx,j);
			ctrlData data;
			data.mX = value;
			for(int k=0;k<5;k++)
			{
				data.mResult[k] = 0;
			}
			data.mResult[typeIdx] = 1;

			epoch.mSet.push_back(data);
		}

		mTestingData.push_back(epoch);

	}
}

vector<ctrlEpoch> * CSyntheticCtrlDataManager::getTrainingData()
{
	return &mTrainingData;
}

vector<ctrlEpoch> * CSyntheticCtrlDataManager::getTestingData()
{
	return &mTestingData;
}

void CSyntheticCtrlDataManager::printTrainingData()
{
    /*
	vector<ctrlEpoch>::iterator it;
	vector<ctrlEpoch>::iterator itD;
	cout << "TRAINING DATA, SIZE " << mTrainingData.size() << endl;
	int i = 0;
	for(it=mTrainingData.begin();it!=mTrainingData.end();it++)
	{
		cout << " INDEX " << i << ", SIZE " << (*it).mSet.size() << endl;
		for(itD=(*it).mSet.begin();itD!=(*it).mSet.end();itD++)
		{
			cout << "DATA " << (*itD).mX <<"%" << (*itD).mY << "%" << (*itD).mZ;
			cout << " " << (*itD).mType;
			cout << " = " << (*itD).mResult[0] << "+" << (*itD).mResult[1];
			cout << "+" << (*itD).mResult[2] << "+" << (*itD).mResult[3] << "+" << (*itD).mResult[4];
			cout << "+" << (*itD).mResult[5] << "+" << (*itD).mResult[6] << "+" << (*itD).mResult[7];
			cout << "+" << (*itD).mResult[8] << "+" << (*itD).mResult[9] << "+" << (*itD).mResult[10] <<endl;
		}
	}
	*/
}

void CSyntheticCtrlDataManager::printTestingData()
{
	/*
	vector<inputEpoch>::iterator it;
	vector<inputData>::iterator itD;
	cout << "TESTING DATA, SIZE " << mTestingData.size() << endl;
	int i = 0;
	for(it=mTestingData.begin();it!=mTestingData.end();it++)
	{
		cout << " INDEX " << i << ", SIZE " << (*it).mSet.size() << endl;
		for(itD=(*it).mSet.begin();itD!=(*it).mSet.end();itD++)
		{
			cout << "DATA " << (*itD).mX <<"%" << (*itD).mY << "%" << (*itD).mZ;
			cout << " " << (*itD).mType;
			cout << " = " << (*itD).mResult[0] << "+" << (*itD).mResult[1];
			cout << "+" << (*itD).mResult[2] << "+" << (*itD).mResult[3] << "+" << (*itD).mResult[4];
			cout << "+" << (*itD).mResult[5] << "+" << (*itD).mResult[6] << "+" << (*itD).mResult[7];
			cout << "+" << (*itD).mResult[8] << "+" << (*itD).mResult[9] << "+" << (*itD).mResult[10] <<endl;
		}
	}
	*/
}

ctrlEpoch * CSyntheticCtrlDataManager::getTrainingData(int i)
{
	if(i>=0 && i<getTrainingDataSize())
	{
		return &(mTrainingData[i]);
	}

	return NULL;
}

ctrlEpoch * CSyntheticCtrlDataManager::getTestingData(int i)
{
	if(i>=0 && i<getTestingDataSize())
	{
		return &(mTestingData[i]);
	}

	return NULL;
}
