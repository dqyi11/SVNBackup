/*
 * CGridMap.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: walter
 */

#include "CGridMap.h"
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>

using namespace std;

CGridMap::CGridMap(int width, int height) {
	// TODO Auto-generated constructor stub
	mGridWidth = width;
	mGridHeight = height;
	mGridNum= width * height;

    mpProbablisitcMap = new double*[mGridWidth];
    for(int i = 0; i < mGridWidth; i++){
    	mpProbablisitcMap[i] = new double[mGridHeight];
    }
	for(int x = 0; x < mGridWidth; x++){
		for(int y = 0; y < mGridHeight; y++){
			mpProbablisitcMap[x][y] = .75;
		}
	}

}

CGridMap::~CGridMap() {
	// TODO Auto-generated destructor stub
    if(mpProbablisitcMap)
    {
    	/*
    	delete [] mpProbablisitcMap;
    	mpProbablisitcMap = 0;
    	*/
    	for(int i=0;i<mGridHeight;i++)
    	{
    		delete [] mpProbablisitcMap[i];
    		mpProbablisitcMap[i] = NULL;
    	}

    	delete [] mpProbablisitcMap;
    	mpProbablisitcMap = NULL;
    }
}

bool CGridMap::convertPosToIndex(int x, int y, int & index)
{


	if((x<0)||(y<0)||(x>mGridWidth)||(y>mGridHeight))
	{
		return false;
	}


	index = x+y*mGridWidth;

	return true;
}

bool CGridMap::convertIndexToPos(int index, int & x, int & y)
{
	if(index>mGridNum)
	{
		return false;
	}

	y = (int)floor(index/mGridWidth);
	x = index - y*mGridWidth;

	return true;
}

double CGridMap::getProbability(int x, int y)
{
	int arrX = mGridWidth/2 + x;
	int arrY = mGridHeight/2 + y;
	int index = 0;
	double prob = 0;

	prob = mpProbablisitcMap[arrX][arrY];

	return prob;
}

bool CGridMap::setProbability(int x, int y, double prob)
{
	int arrX = mGridWidth/2 +x;
	int arrY = mGridHeight/2+y;

	mpProbablisitcMap[arrX][arrY] = prob;
	return true;

}

void CGridMap::printMap()
{
	  ofstream myfile;
	  myfile.open ("grid.txt");
	  myfile << "Writing this to a file.\n";

	for(int x = 0; x < mGridWidth; x++){
		for(int y = 0; y < mGridHeight; y++){
			 myfile<<mpProbablisitcMap[x][y]<< " ";
		}
		 myfile<<""<<endl;
	}
	 myfile.close();
}
