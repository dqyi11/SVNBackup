/*
 * CNeuralNetwork.h
 *
 *  Created on: Feb 1, 2013
 *      Author: walter
 */

#ifndef CNEURALNETWORK_H_
#define CNEURALNETWORK_H_

#include "CLayer.h"
#include <vector>

using namespace std;

class CNeuralNetwork {
	friend class CNetworkHistManager;
public:
	CNeuralNetwork();
	virtual ~CNeuralNetwork();

    void addLayer(string name, NeuronType type);
	bool hasLayer(string name);

	bool addNeuron(string layer, string neuron, NeuronType type);

	bool addNeuron(string layer, string neuron);

	CLayer * getLayer(string name);

	CNeuronLink * getConnect(string from, string to);
	CNeuronLink * connect(string from, string to, double weight = 0.1);

	int getLayerNum() { return mLayers.size(); };
	CLayer * getLayer(int index);

	int getLinkNum() { return mLinks.size(); };
	CNeuronLink * getLink(int index);

	void fullyConnect(string fromLayer, string toLayer, double weight);
	void fullyConnect(CLayer * pFrom, CLayer * pTo, double weight);

	void calculate();

	CLayer * getInputLayer();
	CLayer * getOutputLayer();
	CLayer * getHiddenLayer();

	CNeuron * getNeuron(string name);

	void print();

protected:

	CLayer * createLayer(string name, NeuronType type);
	void addLayer(CLayer * pLayer);

	CNeuronLink * createLink(string from, string to, double weight);
	CNeuronLink * getConnect(CNeuron * pFrom, CNeuron * pTo);

private:
	vector<CLayer*> mLayers;
	vector<CNeuronLink*> mLinks;

};

#endif /* CNEURALNETWORK_H_ */
