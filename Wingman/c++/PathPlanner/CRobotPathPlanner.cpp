/*
 * CRobotPathPlanner.cpp
 *
 *  Created on: Dec 28, 2012
 *      Author: walter
 */

#include "CRobotPathPlanner.h"

#include <iostream>

using namespace std;

CCellObserved::CCellObserved()
{
	mCellIndex = -1;
	mDeltaProb = 0;
	mReward = 0;
}

CCellObserved::~CCellObserved()
{
	mCellIndex = -1;
	mDeltaProb = 0;
	mReward = 0;
}

CPath::CPath()
{
	mVertexSeq.clear();
	mCellObsHist.clear();
	mTotalReward = 0;
};

CPath::~CPath()
{
	mVertexSeq.clear();
	mCellObsHist.clear();
	mTotalReward = 0;
};

void CPath::addVertex(CLevelVertex vertex)
{
	mVertexSeq.push_back(vertex);
};

bool CPath::hasCellObserved(int index)
{
	list<CCellObserved>::iterator it;

	for(it=mCellObsHist.begin();it!=mCellObsHist.end();it++)
	{
		if((*it).mCellIndex==index)
		{
			return true;
		}
	}
	return false;
}

void CPath::addCellObserved(int index)
{
	list<CCellObserved>::iterator it;

	if(hasCellObserved(index))
	{
		for(it=mCellObsHist.begin();it!=mCellObsHist.end();it++)
		{
			if((*it).mCellIndex==index && (*it).mIsVisitedCell==false)
			{
				(*it).mObsTimes += 1;
			}
		}
	}
	else
	{
		CCellObserved cellObserved;
		cellObserved.mCellIndex = index;
		cellObserved.mObsTimes = 1;
		cellObserved.mIsVisitedCell = isVisitedCell(index);
		mCellObsHist.push_back(cellObserved);
	}

}

double CPath::getCellReward(int index)
{
	list<CCellObserved>::iterator it;
	for(it=mCellObsHist.begin();it!=mCellObsHist.end();it++)
	{
		if((*it).mCellIndex==index)
		{
			return (*it).mReward;
		}
	}

	return 0;
}

void CPath::calcTotalReward()
{
	mTotalReward = 0;

	list<CCellObserved>::iterator it;
	for(it=mCellObsHist.begin();it!=mCellObsHist.end();it++)
	{
		mTotalReward += (*it).mReward;
	}
}

bool CPath::isVisitedCell(int index)
{
	list<CLevelVertex>::iterator it;
	for(it=mVertexSeq.begin();it!=mVertexSeq.end();it++)
	{
		if((*it).mpHexaVertex->mIndex==index)
		{
			return true;
		}
	}

	return false;
}

int CPath::getCellObservedTimes(int index)
{
	list<CCellObserved>::iterator it;
	for(it=mCellObsHist.begin();it!=mCellObsHist.end();it++)
	{
		if((*it).mCellIndex==index)
		{
			return (*it).mObsTimes;
		}
	}

	return 0;
}

void CPath::clear()
{
	mVertexSeq.clear();
	mCellObsHist.clear();
	mTotalReward = 0;
}

void CPath::printPath()
{
	list<CLevelVertex>::iterator it;
	cout << " PATH : ";
	for(it=mVertexSeq.begin();it!=mVertexSeq.end();it++)
	{
		cout << " " << (*it).mId;
	}
	cout << endl;
}

void CPath::printCellObsHist()
{
	list<CCellObserved>::iterator it;
	cout << " OBS HIST : ";
	for(it=mCellObsHist.begin();it!=mCellObsHist.end();it++)
	{
		cout << " " << (*it).mCellIndex;
	}
	cout << endl;
}

CStepBatch::CStepBatch()
{
	mBatch.clear();
	mReward = 0;
}

CStepBatch::~CStepBatch()
{
	mBatch.clear();
	mReward = 0;
}

CLevelVertex * CStepBatch::getFirstStep()
{
	if(mBatch.size()==0)
	{
		return NULL;
	}

	return mBatch.front();
}

CLevelVertex * CStepBatch::getLastStep()
{
	if(mBatch.size()==0)
	{
		return NULL;
	}

	return mBatch.back();
}

void CStepBatch::print()
{
	cout << "step batch: ";
	list<CLevelVertex*>::iterator it;
	for(it=mBatch.begin();it!=mBatch.end();it++)
	{
		cout << (*it)->mId << " ";
	}
	cout << endl;
}

CRobotPathPlanner::CRobotPathPlanner(CPathPlanningGraph * graph, CAgent * agent)
{
	// TODO Auto-generated constructor stub

	mpGraph = graph;
	mpMap = NULL;
	mpAgent = agent;

	if(mpGraph)
	{
		mPlanStep = mpGraph->getPlanningLength();
	}

}

CRobotPathPlanner::~CRobotPathPlanner()
{
	// TODO Auto-generated destructor stub
	mpGraph = NULL;
	mpAgent = NULL;
	if(mpMap)
	{
		delete mpMap;
		mpMap = NULL;
	}
}

void CRobotPathPlanner::addPath(CPath path)
{
	mAllPathList.push_back(path);
}

CPath CRobotPathPlanner::findMaxPath()
{
	CPath path;
	path.mVertexSeq.clear();

	if(0==mAllPathList.size())
	{
		return path;
	}

	double maxScore = -1;

	list<CPath>::iterator it;
	for(it=mAllPathList.begin();it!=mAllPathList.end();it++)
	{
		if((*it).getTotalReward() > maxScore)
		{
			path = (*it);
		}
	}

	return path;
}

bool CRobotPathPlanner::init(CVisualHexagonDiscretizedMap * map)
{
	if(NULL==map)
	{
		return false;
	}

	if(mpMap)
	{
		delete mpMap;
		mpMap = NULL;
	}

	mpMap = map; //new CVisualHexagonDiscretizedMap(*map);

	return true;
}

double CRobotPathPlanner::scorePath(CPath & path)
{
	list<CCellObserved>::iterator it;
	double likelihood = mpAgent->getObservationLikelihood();
	for(it=path.mCellObsHist.begin();it!=path.mCellObsHist.end();it++)
	{
		CVisualHexagon * hexagon = mpMap->getHexagon((*it).mCellIndex);
		int xPos = hexagon->getGrid()->mX;
		int yPos = hexagon->getGrid()->mY;
		double currentProbability = mpMap->getGrid(xPos,yPos)->mProbabilityValue;
		(*it).mOriginProb = currentProbability;

		if((*it).mIsVisitedCell==true)
		{
			(*it).mDeltaProb = currentProbability;
		}
		else
		{
			for(int i=0;i<(*it).mObsTimes;i++)
			{
				(*it).mDeltaProb = (*it).mDeltaProb
						+ likelihood * (currentProbability-(*it).mDeltaProb);
			}
		}
	}

	/* calc entropy after prob updated */
	path.mTotalReward = 0;
	for(it=path.mCellObsHist.begin();it!=path.mCellObsHist.end();it++)
	{
		(*it).mReward = mpMap->getEntropy((*it).mOriginProb)
				- mpMap->getEntropy((*it).mOriginProb-(*it).mDeltaProb);
		path.mTotalReward += (*it).mReward;
	}

	// try to unit observed cells after calculation
	path.clearCellObsHist();

	return path.mTotalReward;
}

bool CRobotPathPlanner::initObservedCells(CPath & path)
{
	if(NULL==mpMap)
	{
		return false;
	}

	if(NULL==mpAgent)
	{
		return false;
	}

	path.clearCellObsHist();

	// init visited cells
	list<CLevelVertex>::iterator it;
	for(it=path.mVertexSeq.begin();it!=path.mVertexSeq.end();it++)
	{
		//cout << " OPERATE " << (*it).mpHexaVertex->mIndex << endl;
		path.addCellObserved((*it).mpHexaVertex->mIndex);
	}

	// init neighboring cells
	for(it=path.mVertexSeq.begin();it!=path.mVertexSeq.end();it++)
	{
		// get neighbor cell set;
		int posX = (*it).mpHexaVertex->mPosX;
	    int posY = (*it).mpHexaVertex->mPosY;
	    CGridSet gridSet = mpMap->getGridSet(posX, posY,
	    		mpAgent->getObservationRange(), false);

	    list<CGrid*>::iterator itG;
		for(itG=gridSet.mSet.begin();itG!=gridSet.mSet.end();itG++)
		{
			CVisualHexagon * hexagon = &(mpMap->mpVisualHexagons[(*itG)->mX][(*itG)->mY]);
			if(hexagon->getIndex()!=-1)
			{
				path.addCellObserved(hexagon->getIndex());
			}
		}

	}

	return true;
}

void CRobotPathPlanner::printAllPath()
{
	list<CPath>::iterator it;
	for(it=mAllPathList.begin();it!=mAllPathList.end();it++)
	{
		(*it).printPath();
	}

}

void CRobotPathPlanner::plotPathScoreDist()
{
	mDataVisualizer.clearData();
	list<CPath>::iterator it;
	for(it=mAllPathList.begin();it!=mAllPathList.end();it++)
	{
		mDataVisualizer.addData((*it).mTotalReward);
	}

	mDataVisualizer.sortData();
	mDataVisualizer.plot();

}
