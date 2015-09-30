/*
 * CWorld.h
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#ifndef CWORLD_H_
#define CWORLD_H_

#include <list>
#include "CBaseObject.h"
#include "CTeam.h"
#include <string>
#include "CObstacle.h"
#include "CFlag.h"
#include "CMyAgent.h"

using namespace std;

class CWorld : public CBaseObject
{
public:
	CWorld();
	virtual ~CWorld();

	CTeam * findTeam(string color);
	CFlag * findFlag(string color);
	CMyAgent * findMyAgent(int index);
	CMyAgent * findMyAgent(string callSign);

	bool flagOwnedByMyTeam(string flagColor);

	CBase * getMyTeamBase();
	CFlag * getMyTeamFlag();

	virtual void print();


	list<CTeam *> otherTeam;
	list <CObstacle*> obstacles;
	list<CFlag*> flags;
	CTeam * mpMyTeam;

	int mSize; // world size
	string mMyColor;
	double tankangvel;
	double tanklength;
	double tankradius;
	int tankspeed;
	string tankalive;
	string tankdead;
	double linearaccel;
	double angularaccel;
	double tankwidth;
	double shotradius;
	int shotrange;
	int shotspeed;
	double flagradius;
	int explodetime;
	double truepositive;
	double truenegative;

};

#endif /* CWORLD_H_ */
