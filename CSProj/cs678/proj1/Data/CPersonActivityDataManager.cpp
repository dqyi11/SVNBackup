/*
 * CPersonActivityDataManager.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#include "CPersonActivityDataManager.h"
#include "../Utility/CRandomNumGenerator.h"

#include <iostream>

using namespace std;

CPersonActivityDataManager::CPersonActivityDataManager() {
	// TODO Auto-generated constructor stub

}

CPersonActivityDataManager::~CPersonActivityDataManager() {
	// TODO Auto-generated destructor stub
}

CPersonActivityDataArray * CPersonActivityDataManager::getArray(string seqNum, TagType tag)
{
	vector<CPersonActivityDataArray*>::iterator it;
	CPersonActivityDataArray * pArray = NULL;
	for(it=mArrayList.begin();it!=mArrayList.end();it++)
	{
	    pArray = (*it);
		if(0==pArray->getSeqNum().compare(seqNum)
				&& pArray->getTag()==tag)
		{
			return pArray;
		}
	}

	pArray = createArray(seqNum, tag);
	mArrayList.push_back(pArray);

	return pArray;
}

CPersonActivityDataArray * CPersonActivityDataManager::createArray(string seqNum, TagType tag)
{
	CPersonActivityDataArray * pArray = new CPersonActivityDataArray(seqNum, tag);

	return pArray;
}

void CPersonActivityDataManager::print()
{
	vector<CPersonActivityDataArray*>::iterator it;
	for(it=mArrayList.begin();it!=mArrayList.end();it++)
	{
		(*it)->print();
	}
}

void CPersonActivityDataManager::dumpArrayDataToFile()
{
	vector<CPersonActivityDataArray*>::iterator it;
	for(it=mArrayList.begin();it!=mArrayList.end();it++)
	{
		(*it)->dumpToFile();
	}
}

void CPersonActivityDataManager::selfcheck()
{
	vector<CPersonActivityDataArray*>::iterator it;
	for(it=mArrayList.begin();it!=mArrayList.end();it++)
	{
		cout << "SEQ:" << (*it)->getSeqNum() << " ";
		cout << "TAG:" << (*it)->getTag() << " ";
		cout << "SIZE:" << (*it)->mPosInfoSeq.size() << " ";
		cout << " ORDERED? ";
		cout << (*it)->checkSequence();
		cout << endl;
	}
}

void CPersonActivityDataManager::dumpSeqNumDataToFile()
{

}

void CPersonActivityDataManager::prepareTrainingData(int size, int k)
{
	mTrainingData.clear();

	int setSize = getADAListSize();
	CRandomNumGenerator rndGen;
	rndGen.setMin(0);
	rndGen.setMax(setSize-1);
	for(int i=0;i<size;i++)
	{
		int selectSet = rndGen.getRndNum();
		CPersonActivityDataArray * pArray = getADAList(selectSet);

		int startIndex = 0;
		int endIndex = pArray->mPosInfoSeq.size();
		CRandomNumGenerator rndGen2;
		rndGen2.setMin(startIndex);
		rndGen2.setMax(endIndex-k);

		int epochIndex = rndGen2.getRndNum();

		inputEpoch epoch;
		for(int j=epochIndex;j<epochIndex+k;j++)
		{
			CMetaData  metaData = pArray->mPosInfoSeq[j];
			inputData data = convert(metaData, pArray);
			epoch.mSet.push_back(data);
		}

		mTrainingData.push_back(epoch);

	}

}

void CPersonActivityDataManager::prepareTestingData(int size, int k)
{
	int setSize = getADAListSize();
	CRandomNumGenerator rndGen;
	rndGen.setMin(0);
	rndGen.setMax(setSize-1);
	for(int i=0;i<size;i++)
	{
		int selectSet = rndGen.getRndNum();
		CPersonActivityDataArray * pArray = getADAList(selectSet);

		int startIndex = 0;
		int endIndex = pArray->mPosInfoSeq.size();
		CRandomNumGenerator rndGen2;
		rndGen2.setMin(startIndex);
		rndGen2.setMax(endIndex-k);

		int epochIndex = rndGen2.getRndNum();

		inputEpoch epoch;
		for(int j=epochIndex;j<epochIndex+k;j++)
		{
			CMetaData  metaData = pArray->mPosInfoSeq[j];
			inputData data = convert(metaData, pArray);
			epoch.mSet.push_back(data);
		}

		mTestingData.push_back(epoch);

	}
}

vector<inputEpoch> * CPersonActivityDataManager::getTrainingData()
{
	return &mTrainingData;
}

vector<inputEpoch> * CPersonActivityDataManager::getTestingData()
{
	return &mTestingData;
}

CPersonActivityDataArray* CPersonActivityDataManager::getADAList(int i)
{
	if(i>=0 && i<getADAListSize())
	{
		return mArrayList[i];
	}

	return NULL;
}

inputData CPersonActivityDataManager::convert(CMetaData data, CPersonActivityDataArray* array)
{
	inputData iData;
	iData.mX = data.mX;
	iData.mY = data.mY;
	iData.mZ = data.mZ;
	iData.mType = (double)array->getTag()+1;
	for(int i=0;i<10;i++)
	{
		iData.mResult[i]=0;
	}

	switch(data.mActivity)
	{
	case WALKING:
		iData.mResult[0] = 1;
		break;
	case FALLING:
		iData.mResult[1] = 1;
		break;
	case LYING_DOWN:
		iData.mResult[2] = 1;
		break;
	case LYING:
		iData.mResult[3] = 1;
		break;
	case SITTING_DOWN:
		iData.mResult[4] = 1;
		break;
	case SITTING:
		iData.mResult[5] = 1;
		break;
	case STANDING_UP_FROM_LYING:
		iData.mResult[6] = 1;
		break;
	case ON_ALL_FOURS:
		iData.mResult[7] = 1;
		break;
	case SITTING_ON_THE_GROUND:
		iData.mResult[8] = 1;
		break;
	case STANDING_UP_FROM_SITTING:
		iData.mResult[9] = 1;
		break;
	case STANDING_UP_FROM_SITTING_ON_THE_GROUND:
		iData.mResult[10] = 1;
		break;
	}

	return iData;
}

void CPersonActivityDataManager::printTrainingData()
{
	vector<inputEpoch>::iterator it;
	vector<inputData>::iterator itD;
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
}

void CPersonActivityDataManager::printTestingData()
{
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
}

inputEpoch * CPersonActivityDataManager::getTrainingData(int i)
{
	if(i>=0 && i<getTrainingDataSize())
	{
		return &(mTrainingData[i]);
	}

	return NULL;
}

inputEpoch * CPersonActivityDataManager::getTestingData(int i)
{
	if(i>=0 && i<getTestingDataSize())
	{
		return &(mTestingData[i]);
	}

	return NULL;
}
