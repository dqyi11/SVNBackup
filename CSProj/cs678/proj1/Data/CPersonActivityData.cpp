/*
 * CPersonActivityData.cpp
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#include "CPersonActivityData.h"
#include <iostream>
#include <stdlib.h>

using namespace std;

const int ActivityTypeNum = 11;
const string ActivityTypeString[] = { "walking", "falling", "lying down",
		"lying", "sitting down", "sitting", "standing up from lying",
		"on all fours", "sitting on the ground", "standing up from sitting",
		"standing up from sitting on the ground"
};

const int TagTypeNum = 4;
const string TagTypeString[] = {"010-000-024-033", "010-000-030-096",
		"020-000-033-111", "020-000-032-221"
};

bool comp(CPersonActivityData &lhs, CPersonActivityData &rhs)
{
	return lhs.getTimeStamp() < rhs.getTimeStamp();
}

CPersonActivityData::CPersonActivityData() {
	// TODO Auto-generated constructor stub
	mSeqNum = "";
	mTag = EMPTY;
	mX = 0;
	mY = 0;
	mZ = 0;
	mActivity = UNKNOWN;
	mDateTime = "";
	mTimeStamp = 0;
}

CPersonActivityData::~CPersonActivityData() {
	// TODO Auto-generated destructor stub
}

void CPersonActivityData::print()
{
	cout << "Person Activity: ";
	cout << mSeqNum << " " << mTag << " X:" << mX
			<< " Y:" << mY << " Z:" << mZ << " "
			<< mActivity << " D:" << mDateTime
			<< " T:" << mTimeStamp << endl;

}

void CPersonActivityData::setX(string x)
{
	double doubleX = atof(x.c_str());
	setX(doubleX);
}

void CPersonActivityData::setY(string y)
{
	double doubleY = atof(y.c_str());
	setY(doubleY);
}

void CPersonActivityData::setZ(string z)
{
	double doubleZ = atof(z.c_str());
	setZ(doubleZ);
}

void CPersonActivityData::setTimeStamp(string timeStamp)
{
	long long int timeStampInt = atoll(timeStamp.c_str());
	setTimeStamp(timeStampInt);
}

void CPersonActivityData::setTag(string tag)
{
	for(int i=0;i<TagTypeNum;i++)
	{
		if(0==TagTypeString[i].compare(tag))
		{
			setTag(i);
			return;
		}
	}
}

void CPersonActivityData::setTag(int i)
{
	switch(i)
	{
	case 0:
		mTag = ANKLE_LEFT;
		break;
	case 1:
		mTag = ANKLE_RIGHT;
		break;
	case 2:
		mTag = BELT;
		break;
	case 3:
		mTag = CHEST;
		break;
	}

}

void CPersonActivityData::setActivity(string activity)
{
	for(int i=0;i<ActivityTypeNum;i++)
	{
		if(0==ActivityTypeString[i].compare(activity))
		{
			setActivity(i);
			return;
		}
	}
}

void CPersonActivityData::setActivity(int i)
{
	switch(i)
	{
	case 0:
		mActivity = WALKING;
		break;
	case 1:
		mActivity = FALLING;
		break;
	case 2:
		mActivity = LYING_DOWN;
		break;
	case 3:
		mActivity = LYING;
		break;
	case 4:
		mActivity = SITTING_DOWN;
		break;
	case 5:
		mActivity = SITTING;
		break;
	case 6:
		mActivity = STANDING_UP_FROM_LYING;
		break;
	case 7:
		mActivity = ON_ALL_FOURS;
		break;
	case 8:
		mActivity = SITTING_ON_THE_GROUND;
		break;
	case 9:
		mActivity = STANDING_UP_FROM_SITTING;
		break;
	case 10:
		mActivity = STANDING_UP_FROM_SITTING_ON_THE_GROUND;
		break;
	}
}

CPersonActivitySequence::CPersonActivitySequence(string seqNum)
{
	mSeqNum = seqNum;
	mSequence.clear();
}

CPersonActivitySequence::~CPersonActivitySequence()
{
	mSequence.clear();
}

void CPersonActivitySequence::addData(CPersonActivityData & data)
{
	mSequence.push_back(data);
}

void CPersonActivitySequence::print()
{
	list<CPersonActivityData>::iterator it;
	for(it=mSequence.begin();it!=mSequence.end();it++)
	{
		(*it).print();
	}

}

void CPersonActivitySequence::sort()
{
	mSequence.sort(comp);
}

long long int CPersonActivitySequence::getMiniTimeStamp()
{
	if(mSequence.size()==0)
	{
		return 0;
	}

	long long int miniTimeStamp = mSequence.front().getTimeStamp();
	list<CPersonActivityData>::iterator it;
	for(it=mSequence.begin();it!=mSequence.end();it++)
	{
		if((*it).getTimeStamp()<miniTimeStamp)
		{
			miniTimeStamp = (*it).getTimeStamp();
		}
	}

	return miniTimeStamp;
}
