/*
 * COccGridState.h
 *
 *  Created on: Oct 31, 2012
 *      Author: walter
 */

#ifndef COCCGRIDSTATE_H_
#define COCCGRIDSTATE_H_

#include <string>

using namespace std;

class COccGridState {
public:
	COccGridState();
	virtual ~COccGridState();

	int  getPosX(void)  { return mPosX; };
	int  getPosY(void)  { return mPosY; };

	int  getWidth(void)   { return mWidth; };
	int  getHeight(void)  { return mHeight; };

	int getState(int x, int y);

	bool init(string value);

protected:

	void setPosX(int x) { mPosX = x; };
	void setPosY(int y) { mPosY = y; };
	void setWidth(int w)  { mWidth = w; };
	void setHeight(int h) { mHeight = h; };

	bool convertPosToIndex(int x, int y, int & index);

	string trimSpace(string input);

	int mPosX;
	int mPosY;
	int mWidth;
	int mHeight;
	int ** mpGridStates;

	int mGridNum;
};

#endif /* COCCGRIDSTATE_H_ */
