/*
 * CGaussianDistributionGenerator.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "C2DGaussianDistributionGenerator.h"

C2DGaussianDistributionGenerator::C2DGaussianDistributionGenerator() {
	// TODO Auto-generated constructor stub
    mMean = mat(2,1);
    mMean.zeros();

    //mMean(0,0) = -154.075 ;
    //mMean(1,0) = 72.5832;

    mCovariance = mat(2,2);
    mCovariance.eye();

    //mCovariance(0,0) = 10;
    //mCovariance(1,1) = 10;

    mParamB = 1;

}

C2DGaussianDistributionGenerator::~C2DGaussianDistributionGenerator() {
	// TODO Auto-generated destructor stub
}

bool C2DGaussianDistributionGenerator::setMean(mat mean)
{
	mMean = mean;
	return true;
}

bool C2DGaussianDistributionGenerator::setCovaraince(mat covariance)
{
	mCovariance = covariance;

	mParamB = sqrt(4 * 3.14 * 3.14 * det(mCovariance));
	return true;
}

double C2DGaussianDistributionGenerator::getValue(int x, int y)
{

	double dX = (double)x;
	double dY = (double)y;
	double ret = 0.0;
	mat st(2,1);
	st(0,0) = dX;
	st(1,0) = dY;

	mat err = st - mMean;

	//cout << " err " << err << endl;
	//cout << " mMean " << mMean << endl;
	//cout << " mCovariance " << mCovariance << endl;
	mat a = trans(err) * inv(mCovariance) * err;

	//cout << " a " << a << endl;

	double paramA = -0.5 * det(a);

	//cout << " after param A " << paramA << " " << mParamB << endl;

	ret = exp(paramA); // / mParamB;


	return ret;

}
