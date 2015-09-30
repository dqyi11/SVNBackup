/*
 * CNetworkHistManager.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#include "CNetworkHistManager.h"
#include <iostream>
#include <fstream>
#include <string.h>

using namespace std;

CNeuronHist::CNeuronHist(CNeuron * neuron)
{
	mpNeuron = neuron;
	mActivation.clear();
}

CNeuronHist::~CNeuronHist()
{
	mpNeuron = NULL;
	mActivation.clear();
}

CLinkHist::CLinkHist(CNeuronLink * link)
{
	mpLink = link;
	mWeight.clear();
}

CLinkHist::~CLinkHist()
{
	mpLink = NULL;
	mWeight.clear();
}

void CNeuronHist::dumpToFile()
{

}

void CLinkHist::dumpToFile()
{
	ofstream fileWriter;
	char fileName[512];
	memset(fileName,0x0,512);
	sprintf(fileName,"W_%s_%s.dat", mpLink->mpFromNeuron->getName().c_str(),
			mpLink->mpToNeuron->getName().c_str());
	fileWriter.open(fileName);

	vector<double>::iterator it;
	for(it=mWeight.begin();it!=mWeight.end();it++)
	{

	fileWriter << (*it) << endl;
	}
	fileWriter.close();
}

CNetworkHistManager::CNetworkHistManager(CNeuralNetwork * network)
{
	// TODO Auto-generated constructor stub
	mpNetwork = network;
	mNeuronHists.clear();
	mLinkHists.clear();
}

CNetworkHistManager::~CNetworkHistManager()
{
	// TODO Auto-generated destructor stub
	mNeuronHists.clear();
	mLinkHists.clear();
	mpNetwork = NULL;
}

void CNetworkHistManager::init()
{
	if(mpNetwork)
	{
		for(int i=0;i<mpNetwork->getLayerNum();i++)
		{
			CLayer * pLayer = mpNetwork->getLayer(i);
			for(int j=0;j<pLayer->getNeuronNum();j++)
			{
				CNeuron * pNeuron = pLayer->getNeuron(j);
				CNeuronHist neuronHist(pNeuron);
				mNeuronHists.push_back(neuronHist);
			}
		}

		for(int i=0;i<mpNetwork->getLinkNum();i++)
		{
			CNeuronLink * pLink = mpNetwork->getLink(i);
			CLinkHist linkHist(pLink);
			mLinkHists.push_back(linkHist);
		}

	}
}

void CNetworkHistManager::recordHist()
{
	vector<CNeuronHist>::iterator itN;
	for(itN=mNeuronHists.begin();itN!=mNeuronHists.end();itN++)
	{
		double activation = (*itN).mpNeuron->getActivation();
		(*itN).mActivation.push_back(activation);
	}

	vector<CLinkHist>::iterator itL;
	for(itL=mLinkHists.begin();itL!=mLinkHists.end();itL++)
	{
		double weight = (*itL).mpLink->mWeight;
		(*itL).mWeight.push_back(weight);

	}
}

double CNetworkHistManager::getActivation(CNeuron * neuron, int k)
{
	if(k<0)
	{
		return 0;
	}

	if(NULL==neuron)
	{
		return 0;
	}

	vector<CNeuronHist>::iterator itN;
	for(itN=mNeuronHists.begin();itN!=mNeuronHists.end();itN++)
	{
		if((*itN).mpNeuron == neuron)
		{
			if(k < (*itN).mActivation.size())
			{
				return (*itN).mActivation[k];
			}
		}
	}

	return 0;
}

double CNetworkHistManager::getWeight(CNeuronLink * link, int k)
{
	if(k<0)
	{
		return 0;
	}

	if(NULL==link)
	{
		return 0;
	}

	vector<CLinkHist>::iterator itL;
	for(itL=mLinkHists.begin();itL!=mLinkHists.end();itL++)
	{
		if((*itL).mpLink == link)
		{
			if(k < (*itL).mWeight.size())
			{
				return (*itL).mWeight[k];
			}
		}
	}

	return 0;
}

void CNetworkHistManager::dumpLinkHistToFile()
{

	vector<CLinkHist>::iterator it;
	for(it=mLinkHists.begin();it!=mLinkHists.end();it++)
	{
		(*it).dumpToFile();
	}



}

void CNetworkHistManager::dumpNeuronHistToFile()
{

}
