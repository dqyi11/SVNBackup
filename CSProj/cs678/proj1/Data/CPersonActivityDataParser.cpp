/*
 * CPersonActivityDataParser.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#include "CPersonActivityDataParser.h"
#include <iostream>

using namespace std;

CPersonActivityDataParser::CPersonActivityDataParser(const char* fileName)
    : CParser(fileName)
{
	// TODO Auto-generated constructor stub

}

CPersonActivityDataParser::~CPersonActivityDataParser() {
	// TODO Auto-generated destructor stub
}

bool CPersonActivityDataParser::initData(vector<string> elementSet)
{
	//cout << " CPersonActivityDataParser::initData " << elementSet.size() << endl;
	if(elementSet.size()!=8)
	{
		return false;
	}

	string seqNum = elementSet[0];
	CPersonActivitySequence * pSeq = getSequence(seqNum);
	if(NULL==pSeq)
	{
		pSeq = new CPersonActivitySequence(seqNum);
		mSeqArray.push_back(pSeq);
	}

	CPersonActivityData data;
	data.setSeqNum(seqNum);
	data.setTag(elementSet[1]);
	data.setTimeStamp(elementSet[2]);
	data.setDateTime(elementSet[3]);
	data.setX(elementSet[4]);
	data.setY(elementSet[5]);
	data.setZ(elementSet[6]);
	data.setActivity(elementSet[7]);

	pSeq->addData(data);

}

CPersonActivitySequence * CPersonActivityDataParser::getSequence(string seqNum)
{
	vector<CPersonActivitySequence*>::iterator it;
	CPersonActivitySequence * pSeq = NULL;
	for(it=mSeqArray.begin();it!=mSeqArray.end();it++)
	{
		if(0==(*it)->mSeqNum.compare(seqNum))
		{
			pSeq = (*it);
		}
	}

	return pSeq;
}

void CPersonActivityDataParser::print()
{
	vector<CPersonActivitySequence*>::iterator it;
	for(it=mSeqArray.begin();it!=mSeqArray.end();it++)
	{
		cout << "SeqNum " << (*it)->mSeqNum << " size " << (*it)->getSize() << endl;
		(*it)->print();
	}
}

bool CPersonActivityDataParser::initDataManager(CPersonActivityDataManager * pManager)
{
	if(NULL==pManager)
	{
		return false;
	}

	vector<CPersonActivitySequence*>::iterator it;
	list<CPersonActivityData>::iterator itD;
	for(it=mSeqArray.begin();it!=mSeqArray.end();it++)
	{
		CPersonActivitySequence * pSeq = (*it);
		pSeq->sort();
		long long int miniTimeStamp = pSeq->getMiniTimeStamp();
		for(itD=pSeq->mSequence.begin();itD!=pSeq->mSequence.end();itD++)
		{
			CPersonActivityData data = (*itD);
			CPersonActivityDataArray * pArray = pManager->getArray(data.getSeqNum(),
					data.getTag());

			CMetaData posData;
			posData.mX = data.getX();
			posData.mY = data.getY();
			posData.mZ = data.getZ();
			posData.mTimeStamp = data.getTimeStamp();
			posData.mActivity = data.getActivity();

			pArray->addMetaData(posData);
		}

	}

	return true;

}
