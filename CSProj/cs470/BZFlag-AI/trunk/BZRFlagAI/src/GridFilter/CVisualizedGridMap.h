/*
 * CVisualizedGridMap.h
 *
 *  Created on: Oct 30, 2012
 *      Author: walter
 */

#ifndef CVISUALIZEDGRIDMAP_H_
#define CVISUALIZEDGRIDMAP_H_

#include "../3DPlot/C3DSurfacePlotter.h"
#include "CGridMap.h"

class CVisualizedGridMap : public CGridMap
{
public:
	CVisualizedGridMap(int width, int height);
	virtual ~CVisualizedGridMap();

	void drawMap();

private:
	C3DSurfacePlotter * mpPlotter;
};

#endif /* CVISUALIZEDGRIDMAP_H_ */
