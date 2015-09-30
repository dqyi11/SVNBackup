/*
 * CNeuron.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: walter
 */

#include "CNeuron.h"
#include <cmath>
#include <iostream>

using namespace std;

CNeuronLink::CNeuronLink(CNeuron * from, CNeuron * to, double weight)
{
	mpFromNeuron = from;
	mpToNeuron = to;
	mWeight = weight;
}

CNeuronLink::~CNeuronLink()
{
	mpFromNeuron = NULL;
	mpToNeuron = NULL;
}

CNeuron::CNeuron(string name, NeuronType type)
{
	// TODO Auto-generated constructor stub
	mName = name;
	mType = type;
	mFuncType = LOGISTIC;
	mInputs.clear();
	mOutputs.clear();
	mInputSum = 0;
	mActivation = 0;
	mFuncParamA = 1;
	mFuncParamB = 0;

	mK = 0;
	mpLinkNeuron = NULL;

}

CNeuron::~CNeuron()
{
	// TODO Auto-generated destructor stub
	mInputs.clear();
	mOutputs.clear();
	mpLinkNeuron = NULL;
}

CNeuronLink * CNeuron::getInputLink(CNeuron * pNeuron)
{
	vector<CNeuronLink*>::iterator it;
	for(it=mInputs.begin();it!=mInputs.end();it++)
	{
		if((*it)->mpFromNeuron == pNeuron)
		{
			return (*it);
		}
	}

    return NULL;
}

CNeuronLink * CNeuron::getOutputLink(CNeuron * pNeuron)
{
	vector<CNeuronLink*>::iterator it;
	for(it=mOutputs.begin();it!=mOutputs.end();it++)
	{
		if((*it)->mpToNeuron == pNeuron)
		{
			return (*it);
		}
	}

    return NULL;
}

bool CNeuron::hasInputLink(CNeuronLink * pLink)
{
	vector<CNeuronLink*>::iterator it;
	for(it=mInputs.begin();it!=mInputs.end();it++)
	{
		if((*it) == pLink)
		{
			return true;
		}
	}

	return false;
}

bool CNeuron::hasOutputLink(CNeuronLink * pLink)
{
	vector<CNeuronLink*>::iterator it;
	for(it=mOutputs.begin();it!=mOutputs.end();it++)
	{
		if((*it) == pLink)
		{
			return true;
		}
	}

	return false;
}

void CNeuron::addInputLink(CNeuronLink * pLink)
{
	if(!hasInputLink(pLink))
	{
		mInputs.push_back(pLink);
	}

}

void CNeuron::addOutputLink(CNeuronLink * pLink)
{
	if(!hasOutputLink(pLink))
	{
		mOutputs.push_back(pLink);
	}
}

double CNeuron::activationFunction(double input)
{
	double output = 0.5;
	switch(mFuncType)
	{
	case LOGISTIC:
	default:
		output = 1/(1+ exp(- mFuncParamA * input));
		break;
	case HYPERTANGENT:
        output = (1 - exp(- mFuncParamA * input))/(1+exp(- mFuncParamA * input));
		break;
	}

	return output;
}

double CNeuron::activationFuncDerivate(double input)
{
	double output = 0.5;
	switch(mFuncType)
	{
	case LOGISTIC:
	default:
		output = mFuncParamA * exp(-mFuncParamA * input)/pow((1+ exp(- mFuncParamA * input)),2);
		break;
	case HYPERTANGENT:
        output = (1 - exp(- mFuncParamA * input))/(1+exp(- mFuncParamA * input));
		break;
	}

	return output;
}

void CNeuron::update()
{
	mInputSum = 0;
	vector<CNeuronLink*>::iterator it;

	//cout << "update neuron " << mName << endl;
	for(it=mInputs.begin();it!=mInputs.end();it++)
	{
		/*
		cout << "calc input from " << (*it)->mpFromNeuron->getName();
		cout << " with weight " << (*it)->mWeight << " and input ";
		cout << (*it)->mpFromNeuron->getActivation() << endl;
		*/
		mInputSum += (*it)->mWeight * (*it)->mpFromNeuron->getActivation();
	}

	//cout << "input sum is " << mInputSum << endl;

	mActivation = activationFunction(mInputSum);

	//cout << " activation is " << mActivation << endl;
}

void CNeuron::print()
{
	cout << " NEURON: " << mName << " , INPUT: " << mInputs.size();
	cout << " , OUTPUT: " << mOutputs.size() << endl;
}
