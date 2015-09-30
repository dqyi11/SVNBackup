/*
 * CNeuralNetwork.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: walter
 */

#include "CNeuralNetwork.h"
#include <iostream>

CNeuralNetwork::CNeuralNetwork() {
	// TODO Auto-generated constructor stub
	mLayers.clear();
	mLinks.clear();
}

CNeuralNetwork::~CNeuralNetwork() {
	// TODO Auto-generated destructor stub
	vector<CNeuronLink*>::iterator itL;
	CNeuronLink * pLink = NULL;
	for(itL=mLinks.begin();itL!=mLinks.end();itL++)
	{
		pLink = (*itL);
		delete pLink;
		pLink = NULL;
	}
	mLinks.clear();

	vector<CLayer*>::iterator it;
	CLayer * pLayer = NULL;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		pLayer = (*it);
		delete pLayer;
		pLayer = NULL;
	}
	mLayers.clear();
}

void CNeuralNetwork::addLayer(CLayer * pLayer)
{
	mLayers.push_back(pLayer);
}

bool CNeuralNetwork::hasLayer(string name)
{
	vector<CLayer*>::iterator it;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		if(0==(*it)->getName().compare(name))
		{
			return true;
		}
	}

	return false;
}

CLayer * CNeuralNetwork::createLayer(string name, NeuronType type)
{
	CLayer * pLayer = new CLayer(name, type);

	return pLayer;
}

void CNeuralNetwork::addLayer(string name, NeuronType type)
{
	if(hasLayer(name))
	{
		return;
	}

	CLayer * pNeuron = createLayer(name, type);

	addLayer(pNeuron);
}

CLayer * CNeuralNetwork::getLayer(string name)
{
	vector<CLayer*>::iterator it;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		if(0==(*it)->getName().compare(name))
		{
			return (*it);
		}
	}

	return NULL;
}

bool CNeuralNetwork::addNeuron(string layer, string neuron, NeuronType type)
{
	CLayer * pLayer = getLayer(layer);
	if(NULL==pLayer)
	{
		return false;
	}

    return pLayer->addNeuron(neuron, type);
}

CNeuronLink * CNeuralNetwork::connect(string from, string to, double weight)
{
	CNeuronLink * pLink = getConnect(from, to);

	if(NULL==pLink)
	{
		pLink = createLink(from, to, weight);
		mLinks.push_back(pLink);
	}

	return pLink;
}

CNeuronLink * CNeuralNetwork::getConnect(string from, string to)
{
	CNeuron * pFrom = getNeuron(from);
	CNeuron * pTo = getNeuron(to);

    return getConnect(pFrom, pTo);

}

CNeuronLink * CNeuralNetwork::getConnect(CNeuron * pFrom, CNeuron * pTo)
{
	vector<CNeuronLink*>::iterator it;
	for(it=mLinks.begin();it!=mLinks.end();it++)
	{
		if((*it)->mpFromNeuron==pFrom
				&&
			(*it)->mpToNeuron==pTo)
		{
			return (*it);
		}

	}

	return NULL;
}

CNeuron * CNeuralNetwork::getNeuron(string name)
{
	vector<CLayer*>::iterator it;
	CNeuron * pNeuron = NULL;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		pNeuron = (*it)->getNeuron(name);
		if(pNeuron)
		{
			return pNeuron;
		}
	}

	return pNeuron;
}

CNeuronLink * CNeuralNetwork::createLink(string from, string to, double weight)
{
	CNeuronLink * pLink = NULL;
	CNeuron * pFrom = getNeuron(from);
	if(NULL == pFrom)
	{
		return pLink;
	}
	CNeuron * pTo = getNeuron(to);
	if(NULL == pTo)
	{
		return pLink;
	}

	pLink = new CNeuronLink(pFrom, pTo, weight);
	pFrom->addOutputLink(pLink);
	pTo->addInputLink(pLink);

	return pLink;
}

CLayer * CNeuralNetwork::getLayer(int index)
{
	if(index < 0 || index>=getLayerNum())
	{
		return NULL;
	}

	return mLayers[index];
}

CNeuronLink * CNeuralNetwork::getLink(int index)
{
	if(index < 0 || index>=getLinkNum())
	{
		return NULL;
	}

	return mLinks[index];
}

bool CNeuralNetwork::addNeuron(string layer, string neuron)
{
	CLayer * pLayer = getLayer(layer);
	if(NULL==pLayer)
	{
		return false;
	}

    return pLayer->addNeuron(neuron, pLayer->getType());
}

void CNeuralNetwork::fullyConnect(string fromLayer, string toLayer, double weight)
{
	CLayer * pFromLayer = getLayer(fromLayer);
	CLayer * pToLayer = getLayer(toLayer);

	fullyConnect(pFromLayer, pToLayer, weight);

}

void CNeuralNetwork::fullyConnect(CLayer * pFrom, CLayer * pTo, double weight)
{
	if(NULL==pFrom || NULL==pTo)
	{
		return;
	}

	for(int i=0;i<pFrom->getNeuronNum();i++)
	{
		for(int j=0;j<pTo->getNeuronNum();j++)
		{
			connect(pFrom->getNeuron(i)->getName(), pTo->getNeuron(j)->getName(), weight);
		}
	}
}

CLayer * CNeuralNetwork::getInputLayer()
{
	vector<CLayer*>::iterator it;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		if((*it)->getType()==INPUT)
		{
			return (*it);
		}
	}
	return NULL;
}

CLayer * CNeuralNetwork::getOutputLayer()
{
	vector<CLayer*>::iterator it;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		if((*it)->getType()==OUTPUT)
		{
			return (*it);
		}
	}
	return NULL;
}

CLayer * CNeuralNetwork::getHiddenLayer()
{
	vector<CLayer*>::iterator it;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		if((*it)->getType()==HIDDEN)
		{
			return (*it);
		}
	}
	return NULL;
}

void CNeuralNetwork::calculate()
{

	// 1. find input layer
	// 2. calculate from input layer
	if(mLayers.size()==3)
	{
		cout << " here " << endl;
		// two layer network
		CLayer * outputLayer = getOutputLayer();
		for(int i=0;i<outputLayer->getNeuronNum();i++)
		{
			CNeuron * pNeuron = outputLayer->getNeuron(i);
			pNeuron->update();
		}

	}
	else
	{
		CLayer * hiddenLayer = getHiddenLayer();
		for(int i=0;i<hiddenLayer->getNeuronNum();i++)
		{
			CNeuron * pNeuron = hiddenLayer->getNeuron(i);
			pNeuron->update();
		}

		CLayer * outputLayer = getOutputLayer();
		for(int i=0;i<outputLayer->getNeuronNum();i++)
		{
			CNeuron * pNeuron = outputLayer->getNeuron(i);
			pNeuron->update();
		}
	}

}

void CNeuralNetwork::print()
{
	vector<CLayer*>::iterator it;
	for(it=mLayers.begin();it!=mLayers.end();it++)
	{
		(*it)->print();
	}
}
