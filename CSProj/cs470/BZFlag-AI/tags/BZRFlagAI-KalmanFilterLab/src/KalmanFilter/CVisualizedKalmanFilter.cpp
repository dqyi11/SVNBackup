/*
 * CVisualizedKalmanFilter.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "CVisualizedKalmanFilter.h"

#include <armadillo>

using namespace arma;

CVisualizedKalmanFilter::CVisualizedKalmanFilter(int width, int height)
: CKalmanFilter(6,2)
{
	// TODO Auto-generated constructor stub
	mWidth = width;
	mHeight = height;
	mpSurface = new double*[mWidth];
	for(int i = 0; i < mWidth; i++){
		mpSurface[i] = new double[mHeight];
	}

	mpPlotter = new C3DSurfacePlotter(width, height);

	if(mpPlotter)
	{
		mpPlotter->init(mpSurface);
	}

}

CVisualizedKalmanFilter::~CVisualizedKalmanFilter() {
	// TODO Auto-generated destructor stub
	if(mpPlotter)
	{
		mpPlotter->deinit();
	}

	if(mpPlotter)
	{
		delete mpPlotter;
		mpPlotter = NULL;
	}

	if(mpSurface)
	{
    	for(int i=0;i<mHeight;i++)
    	{
    		delete [] mpSurface[i];
    		mpSurface[i] = NULL;
    	}

    	delete [] mpSurface;
    	mpSurface = NULL;
	}
}

bool CVisualizedKalmanFilter::updateSurface()
{
	if(mpSurface)
	{
		for(int i=0;i<mHeight;i++)
		{
			for(int j=0;j<mWidth;j++)
			{
				mpSurface[j][i]=mCalculator.getValue(j-mWidth/2, i-mHeight/2);
			}
		}

		return true;
	}

	return false;
}

void CVisualizedKalmanFilter::draw()
{
	if(mpPlotter)
	{
		mpPlotter->Draw();
	}
}

bool CVisualizedKalmanFilter::update()
{
	if(!CKalmanFilter::update())
	{
		return false;
	}

	mat mean(2,1);
	mat covariance(2,2);

    //cout << " mState " << mState;
    //cout << " value " << mState(0,0) << " " << mState(3,0) << endl;
	mean(0,0) = mState(0,0);
	mean(1,0) = mState(3,0);

	//cout << " est error cov " << mEstErrorCov << endl;

    covariance(0,0) = mEstErrorCov(0,0);
    covariance(0,1) = mEstErrorCov(0,3);
    covariance(1,0) = mEstErrorCov(3,0);
    covariance(1,1) = mEstErrorCov(3,3);


	mCalculator.setCovaraince(covariance);
	mCalculator.setMean(mean);

	//cout << " mean " << mCalculator.mMean << endl;
	//cout << " covariance " << mCalculator.mCovariance << endl;

	updateSurface();
	draw();
}
