/*
 * CKalmanFilter.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "CKalmanFilter.h"
#include <iostream>

using namespace std;

CKalmanFilter::CKalmanFilter(int stateDimension, int observeDimension) {

	mStateDimension = stateDimension;
	mObserveDimension = observeDimension;

	mState = mat(mStateDimension, 1);
	mState.eye();
	mObserve = mat(mObserveDimension, 1);
	mObserve.eye();

	mTransition = mat(mStateDimension, mStateDimension);
	mTransition.eye();
	mMeasurement = mat(mObserveDimension, mStateDimension);
	mMeasurement.eye();

	mTransitionNoiseCov = mat(mStateDimension, mStateDimension);
	mTransitionNoiseCov.eye();
	mMeasurementNoiseCov = mat(mObserveDimension, mObserveDimension);
	mMeasurementNoiseCov.eye();

	mEstErrorCov = mat(mStateDimension, mStateDimension);
	mEstErrorCov.eye();
	mKalmanGain = mat(mStateDimension, mStateDimension);
	mKalmanGain.zeros();
}

CKalmanFilter::~CKalmanFilter() {
	// TODO Auto-generated destructor stub
	deinit();

	/*
	if(mpState)
	{
		delete mpState;
		mpState = NULL;
	}

	if(mpObserve)
	{
		delete mpObserve;
		mpObserve = NULL;
	}

	if(mpTransition)
	{
		delete mpTransition;
		mpTransition = NULL;
	}

	if(mpMeasurement)
	{
		delete mpMeasurement;
		mpMeasurement = NULL;
	}

	if(mpTransitionNoiseCov)
	{
		delete mpTransitionNoiseCov;
		mpTransitionNoiseCov = NULL;
	}

	if(mpMeasurementNoiseCov)
	{
		delete mpMeasurementNoiseCov;
		mpMeasurementNoiseCov = NULL;
	}

	if(mpEstErrorCov)
	{
		delete mpEstErrorCov;
		mpEstErrorCov = NULL;
	}

	if(mpKalmanGain)
	{
		delete mpKalmanGain;
		mpKalmanGain = NULL;
	}
	*/
}

bool CKalmanFilter::init() {
	mCurrentTimeStep = 0;

	mStateHist.clear();
	mObserveHist.clear();
	mEstErrorCovHist.clear();

	mStateHist.push_back(mState);
	mObserveHist.push_back(mObserve);
	mEstErrorCovHist.push_back(mEstErrorCov);

	// TODO: validate if the matrix has been correctly initialized

	return true;
}


bool CKalmanFilter::deinit()
{
	mCurrentTimeStep = 0;

	mStateHist.clear();
	mObserveHist.clear();
	mEstErrorCovHist.clear();

	return true;
}

bool CKalmanFilter::setState(mat state)
{
	mState = state;
	return true;
}

bool CKalmanFilter::setObserve(mat observe)
{
	mObserve = observe;

	//cout << " mObserve " << mObserve;
	return true;
}

bool CKalmanFilter::setTransition(mat transition)
{
	mTransition = transition;
	return true;
}

bool CKalmanFilter::setMeasurement(mat measurement)
{
	mMeasurement = measurement;
	return true;
}

bool CKalmanFilter::setTransitionNoiseCov(mat fNoiseCov)
{
	mTransitionNoiseCov = fNoiseCov;
	return true;
}

bool CKalmanFilter::setMeasurementNoiseCov(mat hNoiseCov)
{
	mMeasurementNoiseCov = hNoiseCov;
	return true;
}

bool CKalmanFilter::getCurrentState(mat &state)
{
    state = mState;
	return true;
}

bool CKalmanFilter::getCurrentObserve(mat &observe)
{
	observe = mObserve;
	return true;
}

bool CKalmanFilter::getTransition(mat &transition)
{
	transition = mTransition;
	return true;
}

bool CKalmanFilter::getMeasurement(mat &measurement)
{
	measurement = mMeasurement;
	return true;
}

bool CKalmanFilter::getTransitionNoiseCov(mat &fNoiseCov)
{
	fNoiseCov = mTransitionNoiseCov;
	return true;
}

bool CKalmanFilter::getMeasurementNoiseCov(mat &hNoiseCov)
{
	hNoiseCov = mMeasurementNoiseCov;
	return true;
}

bool CKalmanFilter::getState(int timeStep, mat &state)
{
	if(timeStep < (int)mStateHist.size())
	{
		state = mStateHist[timeStep];
		return true;
	}
	return false;
}

bool CKalmanFilter::getObeserve(int timeStep, mat &observe)
{
	if(timeStep < (int)mObserveHist.size())
	{
		observe = mObserveHist[timeStep];
		return true;
	}
	return false;
}

bool CKalmanFilter::getEstErrorCov(int timeStep, mat &estErrorCov)
{
	if(timeStep < (int)mEstErrorCovHist.size())
	{
		estErrorCov = mEstErrorCovHist[timeStep];
		return true;
	}
	return false;
}

int CKalmanFilter::getTimeState(){
	return mCurrentTimeStep;
}

bool CKalmanFilter::update()
{
	mCurrentTimeStep++;

	//cout << " update kalman filter " << mCurrentTimeStep << endl;

	//cout << " mState " << mState << endl;
	//cout << " mObserve " << mObserve << endl;
	//cout << " mTransition " << mTransition << endl;
	//cout << " mTransNoisCove " << mTransitionNoiseCov << endl;
	//cout << " mMeasurementNoiseCov " << mMeasurementNoiseCov << endl;
	//cout << " mMeasurement " << mMeasurement << endl;
	/* calc kalman gain matrix */
	//cout << " mTransition " << mTransition << endl;
	//cout << " est error cov " <<  mEstErrorCovHist[mCurrentTimeStep-1] << endl;
	//cout << " trans mTransition " << trans(mTransition) << endl;
	mat predictErrorCov = mTransition * mEstErrorCovHist[mCurrentTimeStep-1] * trans(mTransition);

	//cout << " A " << predictErrorCov << endl;

	predictErrorCov = predictErrorCov + mTransitionNoiseCov;

	//cout << " B " << predictErrorCov << endl;

	mat kalGainDenom = mMeasurement * predictErrorCov * trans(mMeasurement);

	kalGainDenom = kalGainDenom + mMeasurementNoiseCov;

	mKalmanGain = predictErrorCov * trans(mMeasurement) * inv(kalGainDenom);

	//out << " calc kalman gain finish " << endl;

	mat predictState = mTransition * mStateHist[mCurrentTimeStep-1];

	//cout << " mObserve " << mObserve << endl;
	//cout << " mMeasure predict " << mMeasurement * predictState << endl;
	mat estimatedState = predictState + mKalmanGain * (mObserve - mMeasurement * predictState);

	mState = estimatedState;

	//cout << " finish estimate state " << endl;
	mat I(6,6);
	I.eye();

	//cout << " mMeasure " << mMeasurement << endl;
	//cout << " predictErrorCov " << predictErrorCov << endl;
	//cout << " mKalmainGAin " << mKalmanGain << endl;

	mEstErrorCov = (I - mKalmanGain * mMeasurement) * predictErrorCov;

	/*
	mat error(2,2);
	error = mObserve - mMeasurement * mState;

	cout << " error " << error << endl;
    */

	mStateHist.push_back(mState);
	mObserveHist.push_back(mObserve);
	mEstErrorCovHist.push_back(mEstErrorCov);

	//cout << " finish update " << endl;

	//MATRIX state = (*mpState);

	return true;
}

bool CKalmanFilter::getAverageHistState(mat & avState)
{
	mat average(mStateDimension, 1);
	average.zeros();
	int size = mStateHist.size();

	if(size==0)
	{
		return false;
	}

	vector<mat>::iterator it;
	for(it=mStateHist.begin();it!=mStateHist.end();it++)
	{
		average += (*it);
	}

	average = average / size;

	avState = average;

    return true;
}
