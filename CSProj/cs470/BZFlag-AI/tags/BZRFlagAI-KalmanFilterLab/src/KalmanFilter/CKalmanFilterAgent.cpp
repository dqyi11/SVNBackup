/*
 * CKalmanFilterAgent.cpp
 *
 *  Created on: Nov 26, 2012
 *      Author: walter
 */

#include "CKalmanFilterAgent.h"
#include "../ControlParams.h"
#include "KalmanFilterConst.h"
#include "../Base/CWorldController.h"

using namespace arma;

CKalmanFilterAgent::CKalmanFilterAgent(CAgent * agent)
{

	// TODO Auto-generated constructor stub
	mpAgent = agent;
	mpKalmanFilter = new CVisualizedKalmanFilter(GET_WORLD()->mSize, GET_WORLD()->mSize);
	initFilter();
	mpKalmanFilter->init();
	//mpKalmanFilter->update();

}

void CKalmanFilterAgent::initFilter(){

	cout << " init Filter " << endl;
	mat state(6,1);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<1;j++)
		{
			//cout << " adding " << initialState[i*1+j] << endl;
			state(i,j) = initialState[i*1+j];
		}
		//state << endr;
	}

	//cout << " state " << state << endl;
	mpKalmanFilter->setState(state);



	mat observe(2,1);
	for(int i=0;i<2;i++)
	{
		for(int j=0;j<1;j++)
		{
			observe(i,j) = 0;
		}
		//observe << endr;
	}
	mpKalmanFilter->setObserve(observe);

	mat transition(6,6);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
			transition(i,j) = TransitionModel[i*6+j];
		}
		//transition << endr;
	}

	// cout << " transition " << transition << endl;
	mpKalmanFilter->setTransition(transition);

	mat transitionCov(6,6);
	for(int i=0;i<6;i++)
	{
		for(int j=0;j<6;j++)
		{
			transitionCov(i,j) = TransNoiseCov[i*6+j];
		}
		//transitionCov << endr;
	}

	// cout << " transition noise cov " << transitionCov << endl;
	mpKalmanFilter->setTransitionNoiseCov(transitionCov);

	mat obsMod(2,6);
	for(int i=0;i<2;i++)
	{
		for(int j=0;j<6;j++)
		{
			obsMod(i,j) = ObsModel[i*6+j];
		}
		//obsMod << endr;
	}
	mpKalmanFilter->setMeasurement(obsMod);

	mat hNoiseCov(2,2);
	for(int i=0;i<2;i++)
	{
		for(int j=0;j<2;j++)
		{
			hNoiseCov(i,j) = ObsNoiseCov[i*2+j];
		}
		//hNoiseCov << endr;
	}
	mpKalmanFilter->setMeasurementNoiseCov(hNoiseCov);

	//cout << " finish init kalman filter " <<endl;
}

CKalmanFilterAgent::~CKalmanFilterAgent() {
	// TODO Auto-generated destructor stub
	if(mpKalmanFilter)
	{
		delete mpKalmanFilter;
		mpKalmanFilter = NULL;
	}
}


void CKalmanFilterAgent::updateFilter(CAgent * agent)
{
	double xpos = agent->getPosX();
	double ypos = agent->getPosY();

	//cout << " observe x " << xpos << " " << ypos << endl;
 	mat newObserve(2,1);
	newObserve(0,0) = xpos;
	newObserve(1,0) = ypos;

	mpKalmanFilter->setObserve(newObserve);
	mpKalmanFilter->update();

	mat estimatedState(6,1);
	mpKalmanFilter->getCurrentState(estimatedState);

	agent->setEstimatedPosX(estimatedState(0,0));
	agent->setEstimatedPosY(estimatedState(3,0));

	double estimatedVel = sqrt(pow(estimatedState(1,0),2)+pow(estimatedState(4,0),2));

	agent->setEstimatedSpeed(estimatedVel);


}
