/*
 * Tuple.cpp
 *
 *  Created on: Feb 19, 2013
 *      Author: walter
 */

#include "Tuple.h"
#include <sstream>

using namespace std;

Tuple::Tuple() {
	// TODO Auto-generated constructor stub

}

Tuple::~Tuple() {
	// TODO Auto-generated destructor stub
}

string Tuple::toString() {
	stringstream rep;
	rep << "(";
	for(map<string, string>::iterator iter=mTuples.begin(); iter != mTuples.end(); iter++){
		rep << iter->first << ": " << iter->second << " ";
	}
	rep << ")";
	return rep.str();
}

bool Tuple::compare(Tuple & tuple)
{
	int mySize = mTuples.size();
	int otherSize = tuple.mTuples.size();

	if(mySize!=otherSize)
	{
		return false;
	}

	map<string, string>::iterator itMy = mTuples.begin();
	map<string, string>::iterator itOther = tuple.mTuples.begin();

	while(itMy!=mTuples.end()
			&& itOther!=tuple.mTuples.end())
	{
		if((*itMy).first.compare((*itOther).first)!=0)
		{
			return false;
		}

		if((*itMy).second.compare((*itOther).second)!=0)
		{
			return false;
		}

		itMy++;
		itOther++;
	}

	return true;
}

int Tuple::compareTo(Tuple & tuple)
{
	int mySize = mTuples.size();
	int otherSize = tuple.mTuples.size();

	if(mySize!=otherSize)
	{
		return mySize-otherSize;
	}

	map<string, string>::iterator itMy = mTuples.begin();
	map<string, string>::iterator itOther = tuple.mTuples.begin();

	while(itMy!=mTuples.end()
			&& itOther!=tuple.mTuples.end())
	{
		int ret = 0;
		ret = (*itMy).first.compare((*itOther).first);
		if(ret!=0)
		{
			return ret;
		}

		ret = (*itMy).second.compare((*itOther).second);
		if(ret!=0)
		{
			return ret;
		}

		itMy++;
		itOther++;
	}

	return 0;
}
