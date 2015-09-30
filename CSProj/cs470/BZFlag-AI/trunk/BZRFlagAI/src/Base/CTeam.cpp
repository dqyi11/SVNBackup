/*
 * CTeam.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#include "CTeam.h"
#include <iostream>

CTeam::CTeam(string teamColor)
{
	// TODO Auto-generated constructor stub
	base = new CBase(teamColor);
	mColor = teamColor;
	mCount = 0;

}

CTeam::~CTeam()
{
	// TODO Auto-generated destructor stub
	clearAgent();
}


bool CTeam::addAgent(CAgent * agent)
{

	if (hasAgent(agent->getCallSign()))
	{
		return false;

	}
	mAgentList.push_back(agent);

	return true;
}


void CTeam::clearAgent(void)
{
	while(false == mAgentList.empty())
	{
	    mAgentList.pop_back();
	}

}

bool CTeam::hasAgent(string callSign)
{
	vector<CAgent *>::iterator it;
	for (it = mAgentList.begin(); it != mAgentList.end(); it++)
	{
		if (0 == (*it)->getCallSign().compare(callSign))
		{
			return true;
		}

	}

	return false;
}

CAgent * CTeam::findAgent(string callSign)
{
	CAgent * agent = NULL;
	vector<CAgent *>::iterator it;
	for (it = mAgentList.begin(); it != mAgentList.end(); it++)
	{
		if (0 == (*it)->getCallSign().compare(callSign))
		{
			agent = (*it);
		}

	}

	return agent;


}

void CTeam::print()
{
	vector<CAgent *>::iterator it;
	cout << "The team state is : "<<"[COLOR]" << mColor << "[COUNT]" << mCount << endl;
	cout << "Its base is : " << endl;
	if (base)
	{
		base->print();
	}
	cout << "Its agents are : "<<endl;
	for (it = mAgentList.begin(); it != mAgentList.end(); it++)
	{
		(*it)->print();
    }
}
