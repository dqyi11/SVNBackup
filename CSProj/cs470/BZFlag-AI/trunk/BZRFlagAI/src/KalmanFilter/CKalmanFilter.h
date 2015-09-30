
/*
 * CKalmanFilter.h
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#ifndef CKALMANFILTER_H_
#define CKALMANFILTER_H_

//#include "../MatrixCalc/MATRIX.h"
#include <vector>
#include <armadillo>

using namespace arma;
using namespace std;

class CKalmanFilter {
public:
	CKalmanFilter(int stateDimension, int observeDimension);
	virtual ~CKalmanFilter();

	bool init();
	bool deinit();

	bool update();

	bool setState(mat state);
	bool setObserve(mat observe);
	bool setTransition(mat transition);
	bool setMeasurement(mat measurement);
	bool setTransitionNoiseCov(mat fNoiseCov);
	bool setMeasurementNoiseCov(mat hNoiseCov);

	bool getCurrentState(mat &state);
	bool getCurrentObserve(mat &observe);
	bool getTransition(mat &transition);
	bool getMeasurement(mat &measurement);
	bool getTransitionNoiseCov(mat &fNoiseCov);
	bool getMeasurementNoiseCov(mat &hNoiseCov);
	bool getState(int timeStep, mat &state);
	bool getObeserve(int timeStep, mat &observe);
	bool getEstErrorCov(int timeStep, mat &estErrorCov);
	int getTimeState();

	bool getAverageHistState(mat & avState);

protected:
	int mStateDimension;
	int mObserveDimension;

	int mCurrentTimeStep;
	mat mState;
	mat mObserve;
	mat mTransition;
	mat mMeasurement;

	mat mTransitionNoiseCov;
	mat mMeasurementNoiseCov;

	mat mKalmanGain;
	mat mEstErrorCov;

	vector<mat> mStateHist;
	vector<mat> mObserveHist;
	vector<mat> mEstErrorCovHist;
};

#endif /* CKALMANFILTER_H_ */
