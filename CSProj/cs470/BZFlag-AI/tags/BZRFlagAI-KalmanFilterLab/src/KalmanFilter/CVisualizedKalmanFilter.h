/*
 * CVisualizedKalmanFilter.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CVISUALIZEDKALMANFILTER_H_
#define CVISUALIZEDKALMANFILTER_H_

#include "CKalmanFilter.h"
#include "C2DGaussianDistributionGenerator.h"
#include "../3DPlot/C3DSurfacePlotter.h"

class CVisualizedKalmanFilter: public CKalmanFilter {
public:
	CVisualizedKalmanFilter(int width, int height);
	virtual ~CVisualizedKalmanFilter();

	bool update();

	bool updateSurface();
	void draw();
	C2DGaussianDistributionGenerator mCalculator;

private:
	int mWidth;
	int mHeight;

	double ** mpSurface;


	C3DSurfacePlotter * mpPlotter;
};

#endif /* CVISUALIZEDKALMANFILTER_H_ */
