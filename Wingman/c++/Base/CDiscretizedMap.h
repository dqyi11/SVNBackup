/*
 * CDiscretizedMap.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CDISCRETIZEDMAP_H_
#define CDISCRETIZEDMAP_H_

#include "CGrid.h"

#include <vector>
#include "CAgent.h"

using namespace std;

class CDiscretizedMap {
public:
	CDiscretizedMap(int w, int h);
	virtual ~CDiscretizedMap();

	void init();

	virtual void plotMap() = 0;
	virtual void updatePosByMotion(AgentMotion motion, int & x, int & y) = 0;

	void run(int T);
	void update(int t);

	void addAgent(CAgent * agent) { mAgents.push_back(agent); };
	void moveAgent(CAgent * agent, int t);

	bool setGridValue(int x, int y, double value);

	virtual CGridSet getGridSet(int posX, int posY, int radius, bool includeCenter=true) = 0;

	virtual CGrid * getGrid(int x, int y) = 0;

	int getWidth() { return mWidth; };
	int getHeight() { return mHeight; };

protected:
	CGrid ** mpGridLattice;
	int mWidth;
	int mHeight;

	vector<CAgent*> mAgents;
};

#endif /* CDISCRETIZEDMAP_H_ */
