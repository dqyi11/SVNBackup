/*
 * CHexagonDiscretizedMap.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CHEXAGONDISCRETIZEDMAP_H_
#define CHEXAGONDISCRETIZEDMAP_H_

#include "CDiscretizedMap.h"

class CHexagonDiscretizedMap: public CDiscretizedMap {
public:

	CHexagonDiscretizedMap(int w, int h, int hexSize);
	virtual ~CHexagonDiscretizedMap();

	void init();
	int getHexagonSize() { return mHexagonSize; };

	virtual void updatePosByMotion(AgentMotion motion, int & x, int & y);

	virtual CGridSet getGridSet(int posX, int posY, int radius, bool includeCenter=true);
	virtual CGrid * getGrid(int x, int y);

	int getWidth() { return mWidth/2; };
	int getHeight() { return mHeight; };

protected:
	CHexagonDiscretizedMap();
	int mHexagonSize;

};

#endif /* CHEXAGONDISCRETIZEDMAP_H_ */
