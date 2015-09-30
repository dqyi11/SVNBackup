/*
 * CGridGreedyGridFilterAgentController.h
 *
 *  Created on: Nov 10, 2012
 *      Author: walter
 */

#ifndef CCELLBASEDGRIDFILTERAGENTCONTROLLER_H_
#define CCELLBASEDGRIDFILTERAGENTCONTROLLER_H_

#include "CGridFilterAgentController.h"
#include <deque>

class CPathCell
{
public:
	CPathCell(int x, int y) { mCellX = x; mCellY = y; };
	~CPathCell() {};
	int mCellX;
	int mCellY;
};

class CCellSequence
{
public:
	CCellSequence(int index) { mIndex = index; };
	~CCellSequence() { mSequence.clear(); };

	deque<CPathCell*> mSequence;
	int mIndex;
};

class CCellBasedGridFilterAgentController: public CGridFilterAgentController {
public:
	CCellBasedGridFilterAgentController(int num, int grid_size);
	virtual ~CCellBasedGridFilterAgentController();

	bool getGridCenterPos(int gridIndex, double & posX, double & posY);
	bool getGridIndexByPos(double posX, double posY, int & gridIndex);
	bool getGridIndexByGridPos(int gridX, int gridY, int & gridIndex);
	bool convertGridIndexToGridPos(int gridIndex, int & gridX, int & gridY);

	bool hasGridBeenVisited(int index);
	bool hasGridBeenVisited(int gridX, int gridY);
	void markGridVisited(int index);

	bool insideTargetGrid(double targetX, double targetY, double currentX, double currentY);

	virtual void moveAgent(int index) = 0;

	int getNearestUnvisited(int index);
protected:

	int * mpGridStatusMap;
	int mGridSize;
	int mWidthNum;
	int mHeightNum;
	int mGridNum;

	vector<CCellSequence*> mCellSequences;
};

#endif /* CCELLBASEDGRIDFILTERAGENTCONTROLLER_H_ */
