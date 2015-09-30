/*
 * CVisualizedGridMap.cpp
 *
 *  Created on: Oct 30, 2012
 *      Author: walter
 */

#include "CVisualizedGridMap.h"

CVisualizedGridMap::CVisualizedGridMap(int width, int height)
    : CGridMap(width, height)
{
	// TODO Auto-generated constructor stub
	mpPlotter = new C3DSurfacePlotter(width, height);

	if(mpPlotter)
	{
		mpPlotter->init(mpProbablisitcMap);
	}
}

CVisualizedGridMap::~CVisualizedGridMap() {
	// TODO Auto-generated destructor stub
	if(mpPlotter)
	{
		mpPlotter->deinit();
	}

	if(mpPlotter)
	{
		delete mpPlotter;
		mpPlotter = 0;
	}
}

void CVisualizedGridMap::drawMap()
{
	if(mpPlotter)
	{
		mpPlotter->Draw();
	}


}
