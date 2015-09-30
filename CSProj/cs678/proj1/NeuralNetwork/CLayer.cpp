/*
 * CLayer.cpp
 *
 *  Created on: Feb 3, 2013
 *      Author: walter
 */

#include "CLayer.h"
#include <iostream>

CLayer::CLayer(string name, NeuronType type)
{
	// TODO Auto-generated constructor stub
	mName = name;
	mType = type;
	mNeurons.clear();
}

CLayer::~CLayer() {
	// TODO Auto-generated destructor stub
	vector<CNeuron*>::iterator it;
	CNeuron * pNeuron = NULL;
	for(it=mNeurons.begin();it!=mNeurons.end();it++)
	{
		pNeuron = (*it);
		delete pNeuron;
		pNeuron = NULL;
	}

	mNeurons.clear();
}

bool CLayer::addNeuron(CNeuron * pNeuron)
{
	if(NULL==pNeuron)
	{
		return false;
	}

	if(pNeuron->getType()==getType())
	{
	    mNeurons.push_back(pNeuron);
	    return true;
	}

	return false;
}

bool CLayer::hasNeuron(string name)
{
	vector<CNeuron*>::iterator it;
	for(it=mNeurons.begin();it!=mNeurons.end();it++)
	{
		if(0==(*it)->getName().compare(name))
		{
			return true;
		}
	}

	return false;
}

CNeuron * CLayer::createNeuron(string name, NeuronType type)
{
	CNeuron * pNeuron = new CNeuron(name, type);

	return pNeuron;
}

bool CLayer::addNeuron(string name, NeuronType type)
{
	if(hasNeuron(name))
	{
		return true;
	}

	CNeuron * pNeuron = createNeuron(name, type);

	if(!addNeuron(pNeuron))
	{
		delete pNeuron;
		pNeuron = NULL;
		return false;
	}

	return true;
}

CNeuron * CLayer::getNeuron(string name)
{
	vector<CNeuron*>::iterator it;
	for(it=mNeurons.begin();it!=mNeurons.end();it++)
	{
		if(0==(*it)->getName().compare(name))
		{
			return (*it);
		}
	}

	return NULL;
}

CNeuron * CLayer::getNeuron(int index)
{
	if(index<0 || index>=getNeuronNum())
	{
		return NULL;
	}

	return mNeurons[index];
}

void CLayer::printState()
{
	vector<CNeuron*>::iterator it;
	for(it=mNeurons.begin();it!=mNeurons.end();it++)
	{
		cout << "Neuron: " << (*it)->getName() << " V=" << (*it)->getActivation() << endl;
	}
}

void CLayer::print()
{
	vector<CNeuron*>::iterator it;
	cout << " LAYER: " << mName << endl;
	for(it=mNeurons.begin();it!=mNeurons.end();it++)
	{
		(*it)->print();
	}
}

int CLayer::getIndex(CNeuron * pNeuron)
{
	vector<CNeuron*>::iterator it;
	int index = 0;
	for(it=mNeurons.begin();it!=mNeurons.end();it++)
	{
		if((*it)==pNeuron)
		{
			return index;
		}

		index++;
	}

	return -1;
}
