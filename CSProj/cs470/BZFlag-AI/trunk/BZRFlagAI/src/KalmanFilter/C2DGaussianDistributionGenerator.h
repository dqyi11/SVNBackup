/*
 * CGaussianDistributionGenerator.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef C2DGAUSSIANDISTRIBUTIONGENERATOR_H_
#define C2DGAUSSIANDISTRIBUTIONGENERATOR_H_

#include <armadillo>

using namespace arma;

class C2DGaussianDistributionGenerator {
public:
	C2DGaussianDistributionGenerator();
	virtual ~C2DGaussianDistributionGenerator();

	bool setMean(mat mean);
	bool setCovaraince(mat covariance);

	double getValue(int x, int y);

	mat mMean;
	mat mCovariance;

	double mParamB;


};

#endif /* C2DGAUSSIANDISTRIBUTIONGENERATOR_H_ */
