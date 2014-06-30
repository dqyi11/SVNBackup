/*
 * CBayesianFilter.h
 *
 *  Created on: Jan 11, 2013
 *      Author: walter
 */

#ifndef CBAYESIANFILTER_H_
#define CBAYESIANFILTER_H_

class CBayesianFilter {
public:
	CBayesianFilter();
	virtual ~CBayesianFilter();

	void setLikelihood(double likelihood) { mLikelihood = likelihood; };
	double getLikelihood() { return mLikelihood; };




private:
	double mLikelihood;
};

#endif /* CBAYESIANFILTER_H_ */
