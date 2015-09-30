/*
 * CTeam.h
 *
 *  Created on: Sep 26, 2012
 *      Author: walter
 */

#ifndef CTEAM_H_
#define CTEAM_H_

#include "CAgent.h"
#include <vector>
#include <string>
#include "CBase.h"
#include "CBaseObject.h"

using namespace std;

class CWorldController;


class CTeam : public CBaseObject
{
	friend class CWorldController;
public:
	CTeam(string teamColor);
	virtual ~CTeam();

	void clearAgent(void);
	bool hasAgent(string callSign);
    bool addAgent(CAgent * agent);
    CAgent * findAgent(string callSign);

    virtual void print();

    void setNumCount(int count) { mCount = count; };
    int getNumCount(void) { return mCount; };
    string getColor(void) { return mColor; };

	vector<CAgent *> mAgentList;

	string mColor;
	int mCount;
	CBase * base;

//	float mBaseCorner[4][2];
};

#endif /* CTEAM_H_ */
