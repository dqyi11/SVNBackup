/*
 * CWorld.cpp
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#include "CWorld.h"

CWorld::CWorld()
{
	// TODO Auto-generated constructor stub
	mpMyTeam = 0;

	mSize = 0; // world size
	mMyColor = "";
	tankangvel = 0.0;
	tanklength = 0.0;
	tankradius = 0.0;
	tankspeed = 0.0;
	tankalive = "";
	tankdead = "";
	linearaccel = 0.0;
	angularaccel = 0.0;
	tankwidth = 0.0;
	shotradius = 0.0;
	shotrange = 0;
	shotspeed = 0;
	flagradius = 0.0;
	explodetime = 0;
	truepositive = 0;
	truenegative = 0;

}

CWorld::~CWorld()
{
	// TODO Auto-generated destructor stub
}

CTeam * CWorld::findTeam(string color)
{
	list<CTeam *>::iterator it;
	for (it = otherTeam.begin(); it != otherTeam.end(); it++)
	{
		if(0 == (*it)->getColor().compare(color))
		{
		    return (*it);
		}
	}

	return NULL;
}

CFlag * CWorld::findFlag(string color)
{
	list<CFlag *>::iterator it;
	for (it = flags.begin(); it != flags.end(); it++)
	{
		if(0 == (*it)->color.compare(color))
		{
		    return (*it);
		}
	}

	return NULL;
}

void CWorld::print()
{

	list<CTeam *>::iterator itOtherTeam;
	list <CObstacle*>::iterator itObstacle;
	list<CFlag*>::iterator itFlag;

	cout << "The world state is : ";
	cout << "[SIZE]" << mSize;
	cout << "[TANK ANGEL VEL]"<< tankangvel;
	cout << "[TANK LENGTH]" << tanklength;
	cout << "[TANK RADIUS]" << tankradius;
	cout << "[TANK SPEED]" << tankspeed;
	cout << "[TANK ALIVE]" << tankalive;
	cout << "[TANK DEAD]" << tankdead;
	cout << "[LINEAR ACCEL]" << linearaccel;
	cout << "[ANGULAR ACCEL]" << angularaccel;
	cout << "[TANK WIDTH]" << tankwidth;
	cout << "[SHOT RADIUS]" << shotradius;
	cout << "[SHOT RANGE]" << shotrange;
	cout << "[SHOT SPEED]" << shotspeed;
	cout << "[FLAG RADIUS]" << flagradius;
	cout << "[EXPLODE TIME]" << explodetime;
	cout << "[TRUE POSITIVE]" << truepositive;
	cout << "[TRUE NEGATIVE]" << truenegative << endl;

	cout << "MY team State is : " << endl;
	if (mpMyTeam)
	{
		mpMyTeam->print();

	}

	cout << endl;
	cout << "OTher team states are : "<<endl;
	for (itOtherTeam = otherTeam.begin(); itOtherTeam != otherTeam.end(); itOtherTeam++)
	{
		(*itOtherTeam)->print();
	}

	cout<<endl;
	cout << "Obstacle states are : " << endl;
	for (itObstacle = obstacles.begin(); itObstacle != obstacles.end(); itObstacle++)
	{
		(*itObstacle)->print();
	}

	cout<<endl;
	cout << "Flag states are : " << endl;
	for (itFlag = flags.begin(); itFlag != flags.end(); itFlag++)
	{
		(*itFlag)->print();
	}

}

CMyAgent * CWorld::findMyAgent(int index)
{
	CMyAgent * pAgent = NULL;
	vector<CAgent*>::iterator it;
	for(it=mpMyTeam->mAgentList.begin();it!=mpMyTeam->mAgentList.end();it++)
	{
		pAgent = (CMyAgent*)(*it);
		if(pAgent->getIndex()==index)
		{
			return pAgent;
		}
	}

	return pAgent;

}

CMyAgent * CWorld::findMyAgent(string callSign)
{
	CMyAgent * pAgent = NULL;
	vector<CAgent*>::iterator it;
	for(it=mpMyTeam->mAgentList.begin();it!=mpMyTeam->mAgentList.end();it++)
	{
		pAgent = (CMyAgent*)(*it);
		if(0==pAgent->getCallSign().compare(callSign))
		{
			return pAgent;
		}
	}

	return pAgent;
}

bool CWorld::flagOwnedByMyTeam(string flagColor)
{
	list<CFlag*>::iterator it;
	for(it=flags.begin();it!=flags.end();it++)
	{
		if(0==(*it)->color.compare(flagColor))
		{
			if(0==(*it)->PTcolor.compare(mpMyTeam->getColor()))
			{
				return true;
			}
		}

	}

	return false;
}

CBase * CWorld::getMyTeamBase()
{
	return mpMyTeam->base;
}

CFlag * CWorld::getMyTeamFlag()
{
	string color = mpMyTeam->getColor();

	return findFlag(color);
}
