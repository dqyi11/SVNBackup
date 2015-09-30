/*
 * CPotentialFieldManager.cpp
 *
 *  Created on: Sep 30, 2012
 *      Author: walter
 */

#include "CPotentialFieldManager.h"
#include <string.h>
#include "ControlParams.h"
#include "GNUPlot/gnuplot_i.hpp"

const int MAX_LEN = 500;

CPotentialFieldManager::CPotentialFieldManager() {
	// TODO Auto-generated constructor stub

}

CPotentialFieldManager::~CPotentialFieldManager() {
	// TODO Auto-generated destructor stub
}

bool CPotentialFieldManager::addAttractiveFlagBase(CFlag * pFlag, CBase * pBase)
{
	if(NULL == pBase)
	{
		//cout<<"Flag is NULL"<<endl;
		return false;
	}

	//cout<<"In Here"<<endl;
	CFlagBaseAttractivePotentialField * pFlagBaseAttractive
	    = new CFlagBaseAttractivePotentialField(pFlag, pBase);
	mBasesAttractive.push_back(*pFlagBaseAttractive);

	//cout << "Field Added"<<endl;

	return true;
}

void CPotentialFieldManager::clearAttractiveFlagBase(){
	mBasesAttractive.clear();
}

bool CPotentialFieldManager::removeAttractiveFlagBase(CFlag * pFlag, CBase * pBase)
{
	vector<CFlagBaseAttractivePotentialField>::iterator it;

	int count = 0;
	for(it=mBasesAttractive.begin(); it!=mBasesAttractive.end(); it++)
	{
		if(((*it).getBase()==pBase) &&((*it).getFlag()==pFlag))
		{
			//cout<<"Field Removed" << endl;
			mBasesAttractive.erase(it);
			//cout<<"Field Removed 2" << endl;

		}

		count++;
	}

    return true;
	// cout<< mBasesAttractive.size() << endl;

}

bool CPotentialFieldManager::addRepulsiveObstacle(CObstacle * pObstacle)
{
	if(NULL == pObstacle)
	{
		return false;
	}

	CObstacleRepulsivePotentialField * pObstacleRepulsive
	    = new CObstacleRepulsivePotentialField(pObstacle);
	mObstaclesRepulsive.push_back(*pObstacleRepulsive);

	return true;
}

bool CPotentialFieldManager::addTangentialObstacle(CObstacle * pObstacle)
{
	if(NULL == pObstacle)
	{
		return false;
	}

	CObstacleTangentialPotentialField * pObstacleTangential
	    = new CObstacleTangentialPotentialField(pObstacle);
	mObstaclesTangential.push_back(*pObstacleTangential);

	return true;
}

void CPotentialFieldManager::drawPotentialMap(void)
{
	Gnuplot mPotentialFieldMap("Potential Field");
	int step = PLOT_STEP;
	CVector v(0,0);
	//vector<double> x, y, rx, ry;
	double fromX, fromY, toX, toY;
	char pArrowPlot[MAX_LEN];

	mPotentialFieldMap << "set term png size 1600, 1200 crop";
	mPotentialFieldMap.set_xrange(-400, 400);
	mPotentialFieldMap.set_yrange(-400, 400);
	mPotentialFieldMap << "set output 'PotentialField.png'";

	mPotentialFieldMap.set_grid();

	// plot obstacle
	mPotentialFieldMap << "unset key";
	mPotentialFieldMap << "set size square";
	mPotentialFieldMap << "unset arrow";

	vector<CObstacleRepulsivePotentialField>::iterator itObst;
	CObstacle * pObstacle = NULL;
	for(itObst=mObstaclesRepulsive.begin(); itObst!=mObstaclesRepulsive.end(); itObst++)
	{
		pObstacle = (*itObst).obstacle;
		int numCnt = pObstacle->xvals.size();
		for(int i=0;i<(numCnt-1);i++)
		{
			memset(pArrowPlot, 0x0, MAX_LEN);
			sprintf(pArrowPlot, "set arrow from %f, %f to %f, %f nohead lt 3\n", pObstacle->xvals[i], pObstacle->yvals[i], pObstacle->xvals[i+1], pObstacle->yvals[i+1]);
			mPotentialFieldMap << pArrowPlot;

		}

		memset(pArrowPlot, 0x0, MAX_LEN);
		sprintf(pArrowPlot, "set arrow from %f, %f to %f, %f nohead lt 3\n", pObstacle->xvals[numCnt-1], pObstacle->yvals[numCnt-1], pObstacle->xvals[0], pObstacle->yvals[0]);
		mPotentialFieldMap << pArrowPlot;

	}

	// plot vectors
	mPotentialFieldMap << "plot '-' with vectors head\n";
	for(int i=-400; i<=400; i+=step)
	{
		for(int j=-400; j<=400; j+=step)
		{
			memset(pArrowPlot, 0x0, MAX_LEN);
			v = getVector(i, j);
			 v.normalize();
			//v.scale(2);
			// cout << "at " << i << " " << j << " is " << v.mX << " " << v.mY << endl;
			fromX = i;
			fromY = j;
			toX = fromX + v.mX;
			toY = fromY + v.mY;

			sprintf(pArrowPlot, "%f %f %f %f\n", fromX, fromY, (v.mX * PLOT_STEP), (v.mY * PLOT_STEP));
			//string arrowPlot;
			//arrowPlot.assign(pArrowPlot);
			// cout<<"plotting " << pArrowPlot << endl;
            mPotentialFieldMap << pArrowPlot;
		}
	}

	mPotentialFieldMap << "e";

	//mPotentialFieldMap << "set output 'potentialField.png'\n";


}

CVector CPotentialFieldManager::getVector(double x, double y)
{
	CVector v(0,0);
	//CVector v1(0,0);
	vector<CFlagBaseAttractivePotentialField>::iterator itAttr;
	vector<CObstacleRepulsivePotentialField>::iterator itObst;
	vector<CObstacleTangentialPotentialField>::iterator itTan;

	for(itAttr=mBasesAttractive.begin(); itAttr!=mBasesAttractive.end(); itAttr++)
	{
		//v += (*itAttr).getVector(x,y);
		CVector AttrVector = (*itAttr).getVector(x,y);
		v.mX = v.mX + AttrVector.mX;
		v.mY = v.mY + AttrVector.mY;
		/*
		v.mX = v.mX + (*itAttr).getVector(x,y).mX;
		v.mY = v.mY + (*itAttr).getVector(x,y).mY;
		*/
		// cout << "from base " << (*itAttr).getVector(x,y).mX << "+" << (*itAttr).getVector(x,y).mY << endl;

	}

	for(itObst=mObstaclesRepulsive.begin(); itObst!=mObstaclesRepulsive.end(); itObst++)
	{
		//v += (*itObst).getVector(x,y);
		v.mX = v.mX + (*itObst).getVector(x,y).mX;
		v.mY = v.mY + (*itObst).getVector(x,y).mY;


	}

	for(itTan=mObstaclesTangential.begin(); itTan!=mObstaclesTangential.end(); itTan++)
	{
		v.mX = v.mX + (*itTan).getVector(x,y).mX;
		v.mY = v.mY + (*itTan).getVector(x,y).mY;
	}
	return v;

}

void CPotentialFieldManager::update()
{

}

void CPotentialFieldManager::init()
{


}
