/*
 * COccGridState.cpp
 *
 *  Created on: Oct 31, 2012
 *      Author: walter
 */

#include <iostream>
#include <stdlib.h>
//#include <stdio.h>
#include "../CommandConst.h"
#include "COccGridState.h"

COccGridState::COccGridState() {
	// TODO Auto-generated constructor stub
	mPosX = 0;
	mPosY = 0;
    mWidth = 0;
	mHeight = 0;
	mpGridStates = NULL;

	mGridNum = 0;

}

COccGridState::~COccGridState() {
	// TODO Auto-generated destructor stub
	if(mpGridStates)
	{
		for(int i=0;i<mHeight;i++)
		{
			delete [] mpGridStates;
			mpGridStates[i]=NULL;
		}

		delete [] mpGridStates;
		mpGridStates = NULL;
	}

}

int COccGridState::getState(int x, int y)
{
	int arrX,arrY;
	//arrX= x - mPosX;
	//arrY= y -mPosY;

	arrY= x - mPosX;
    arrX= y -mPosY;

	/*
	arrX = x - mPosX;
	arrY = y - mPosY;
	*/

	/*
	if(arrX >= mWidth){
		arrX = mWidth-1;
	}

	if(arrY >= mHeight){
		arrY = mHeight-1;
	}
	*/


	/*
	int oneCount = 0;
	int zCount = 0;
	if(arrX > 0){
		if(mpGridStates[arrX-1][arrY] == 1){
			oneCount++;
		}else{
			zCount++;
		}
	}
	if(arrY > 0){
		if(mpGridStates[arrX][arrY-1] == 1){
			oneCount++;
		}else{
			zCount++;
		}
	}
	if((arrX > 0) && (arrY > 0)){
		if(mpGridStates[arrX-1][arrY-1] == 1){
			oneCount++;
		}else{
			zCount++;
		}
	}
	if(arrY < mHeight-1){
		if(mpGridStates[arrX][arrY+1] == 1){
			oneCount++;
		}else{
			zCount++;
		}
	}
	if(arrX < mWidth-1){
		if(mpGridStates[arrX+1][arrY] == 1){
			oneCount++;
		}else{
			zCount++;
		}
	}

	if((arrX < mWidth-1)&& (arrY < mHeight-1)){
		if(mpGridStates[arrX+1][arrY+1] == 1){
			oneCount++;
		}else{
			zCount++;
		}
	}

	if(mpGridStates[arrX][arrY] == 1){
		oneCount++;
	}else{
		zCount++;
	}

	if(oneCount > zCount){
		return 1;
	}
	if(zCount > oneCount){
		return 0;
	}
	*/

	return mpGridStates[arrX][arrY];

//	return false;
}

bool COccGridState::convertPosToIndex(int x, int y, int & index)
{

	if((x<mPosX) || (y<mPosY)
			|| (x>mPosX+mWidth)
			|| (y>mPosY+mHeight))
	{
		return false;
	}

	index = x + y * mWidth;

	return true;
}


bool COccGridState::init(string value)
{
	// cout << "COccGridState::init" << endl;
    // cout << value << endl;

    /* >>> format sample <<<
     * at 20,20
	 * size 5x4
	 * 0110
	 * 0111
	 * 0111
	 * 0001
	 * 0100
     */

    size_t subBgn = 0, subEnd = 0, subPt = 0;
    size_t segBgn = 0, segEnd = 0;
    string sub = "";
    int index = 0;

    segEnd = value.find("\n", segBgn);
    if(string::npos==segEnd)
    {
    	return false;
    }

    // get first line
    sub = value.substr(segBgn, segEnd-segBgn);
    subBgn = 0;
    subEnd = sub.size()-1;

    subPt = sub.find(AT, subBgn);
    if(string::npos==subPt)
    {
    	return false;
    }

    // trim "at"
    subBgn = subPt + 2;
    sub = sub.substr(subBgn, subEnd-subBgn);
    subBgn = 0;

    // trim space
    sub = trimSpace(sub);

    subPt = sub.find(COMMA);
    if(string::npos==subPt)
    {
    	return false;
    }

    int pos = sub.find(",");
    sub = trimSpace(sub);
    sub = trimSpace(sub);


    string sub2 = sub.substr(0,pos);
    string sub3 = sub.substr(pos+1,sub.size()-pos);

    mPosX = atoi(trimSpace(sub.substr(subBgn, subPt)).c_str());
    mPosY = atoi(trimSpace(sub.substr(subPt+1, subEnd-subPt)).c_str());

    // get second line
    segBgn = segEnd + 1;

    segEnd = value.find("\n", segBgn);
    if(string::npos==segEnd)
    {
    	return false;
    }

    sub = value.substr(segBgn, segEnd-segBgn);
    subBgn = 0;
    subEnd = sub.size()-1;

    subPt = sub.find(SIZE, subBgn);
    if(string::npos==subPt)
    {
    	return false;
    }

    // trim "at"
    subBgn = subPt + 4;
    sub = sub.substr(subBgn, subEnd-subBgn);
    subBgn = 0;

    // trim space
    sub = trimSpace(sub);

    subPt = sub.find(PLUS);
    if(string::npos==subPt)
    {
    	return false;
    }

    mHeight = atoi(trimSpace(sub.substr(subBgn, subPt)).c_str());
    mWidth = atoi(trimSpace(sub.substr(subPt+1, subEnd-subPt)).c_str());


    mGridNum = mWidth*mHeight;

    if(mpGridStates)
    {
    	delete mpGridStates;
    	mpGridStates = NULL;
    }
    mpGridStates = new int*[mWidth];
    for(int y = 0; y < mWidth; y++){
		mpGridStates[y] = new int[mHeight];
    }


    segBgn = segEnd + 1;

    for(int j=0;j<mHeight;j++)
    {
    	// get next line
        segEnd = value.find("\n", segBgn);
        if(string::npos==segEnd)
        {
        	return false;
        }

        sub = value.substr(segBgn, segEnd-segBgn);

        if(sub.size()<mWidth)
        {
        	return false;
        }
        subBgn = 0;
        subEnd = sub.size()-1;
        subPt = subBgn;


        for(int i=0;i<mWidth;i++)
        {
        		mpGridStates[i][j]=atoi((sub.substr(i,1)).c_str());

        }
        segBgn = segEnd + 1;
    }

	return true;
}

string COccGridState::trimSpace(string input)
{
	int size = input.size();
	int subBgn = 0, subEnd = 0;

	if(0==size)
	{
		return input;
	}

	// trim space at the beginning
	subBgn = 0;
	subEnd = size-1;
	while((subEnd>=subBgn)&&(input[subBgn]==' '))
	{
		subBgn++;
	}

	// trim space at the end
	while((subEnd>=subBgn)&&(input[subEnd]==' '))
	{
		subEnd--;
	}

	return input.substr(subBgn, subEnd-subBgn+1);

}
