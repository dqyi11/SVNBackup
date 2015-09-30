/*
 * CPersonActivityData.h
 *
 *  Created on: Jan 30, 2013
 *      Author: walter
 */

#ifndef CPERSONACTIVITYDATA_H_
#define CPERSONACTIVITYDATA_H_

#include <string>
#include <list>

using namespace std;

enum TagType { ANKLE_LEFT, ANKLE_RIGHT, CHEST, BELT, EMPTY };

enum ActivityType { WALKING, FALLING, LYING_DOWN, LYING,
	SITTING_DOWN, SITTING, STANDING_UP_FROM_LYING, ON_ALL_FOURS,
	SITTING_ON_THE_GROUND, STANDING_UP_FROM_SITTING, STANDING_UP_FROM_SITTING_ON_THE_GROUND,
	UNKNOWN,
};

class CPersonActivityData {
public:
	CPersonActivityData();
	virtual ~CPersonActivityData();

	void print();

	void setSeqNum(string seqNum) { mSeqNum = seqNum; };
	string getSeqNum() { return mSeqNum; };

	void setTag(string tag);
	void setTag(TagType tag);
	TagType getTag() { return mTag; };

	void setX(string x);
	void setX(double x) { mX = x; };
	double getX() { return mX; };

	void setY(string y);
	void setY(double y) { mY = y; };
	double getY() { return mY; };

	void setZ(string z);
	void setZ(double z) { mZ = z; };
	double getZ() { return mZ; };

	void setActivity(string activity);
	void setActivity(ActivityType activity) { mActivity = activity; };
	ActivityType getActivity() { return mActivity; };

	void setDateTime(string dateTime) { mDateTime = dateTime; };
	string getDateTime() { return mDateTime; };

	void setTimeStamp(string timeStamp);
	void setTimeStamp(long long int timeStamp) { mTimeStamp = timeStamp; };
	long long int getTimeStamp() { return mTimeStamp; };

private:

	void setTag(int i);
	void setActivity(int i);
	string mSeqNum;
	TagType mTag;
	double mX;
	double mY;
	double mZ;
	ActivityType mActivity;

	string mDateTime;
	long long int mTimeStamp;
};

class CPersonActivitySequence
{
public:
	CPersonActivitySequence(string seqNum);
	virtual ~CPersonActivitySequence();

	void addData(CPersonActivityData & data);
	void print();

	void sort();

	int getSize() { return mSequence.size(); };

	long long int getMiniTimeStamp();

	string mSeqNum;
    list<CPersonActivityData> mSequence;
};

#endif /* CPERSONACTIVITYDATA_H_ */
