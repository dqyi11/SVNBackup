/*
 * CVisualHexagonDiscretizedMap.h
 *
 *  Created on: Dec 5, 2012
 *      Author: walter
 */

#ifndef CVISUALHEXAGONDISCRETIZEDMAP_H_
#define CVISUALHEXAGONDISCRETIZEDMAP_H_

#include "../Base/CHexagonDiscretizedMap.h"
#include <string>

using namespace std;

class CVisualHexagon
{
public:
	CVisualHexagon();
	~CVisualHexagon();

	void init(int X, int Y, int d);

	void setIndex(int index) { mIndexId = index; };
	void setValue(double value) { mValue = value; };

	double getXVertex(int i) { return mXVertex[i]; };
	double getYVertex(int i) { return mYVertex[i]; };
	int getIndex() { return mIndexId; };
	double getValue() { return mValue; };

	string getRGBColor();

	double getCenterX() { return mCenterX; };
	double getCenterY() { return mCenterY; };

	void setGrid(CGrid * grid) { mpGrid = grid; };
	CGrid * getGrid() { return mpGrid; };

private:
	CGrid * mpGrid;
	double mCenterX;
	double mCenterY;

	double mXVertex[7];
	double mYVertex[7];

	double mValue;

	int mIndexId;
};

class CVisualHexagonDiscretizedMap: public CHexagonDiscretizedMap {
public:
	//CVisualHexagonDiscretizedMap();
	CVisualHexagonDiscretizedMap(int width, int height, int hexSize);
	CVisualHexagonDiscretizedMap(const CVisualHexagonDiscretizedMap & map);
	virtual ~CVisualHexagonDiscretizedMap();

	void init();
	void syncValue();

	double getEntropy(double probability);

	virtual void plotMap();

	CVisualHexagon * getHexagon(int index);


	CVisualHexagon ** mpVisualHexagons;

};

#endif /* CVISUALHEXAGONDISCRETIZEDMAP_H_ */
