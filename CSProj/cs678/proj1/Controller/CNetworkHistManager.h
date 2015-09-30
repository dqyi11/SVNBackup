/*
 * CNetworkHistManager.h
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#ifndef CNETWORKHISTMANAGER_H_
#define CNETWORKHISTMANAGER_H_

#include "../NeuralNetwork/CNeuralNetwork.h"

class CNeuronHist
{
public:
	CNeuronHist(CNeuron * neuron);
	virtual ~CNeuronHist();

	void dumpToFile();

	CNeuron * mpNeuron;
	vector<double> mActivation;
};

class CLinkHist
{
public:
	CLinkHist(CNeuronLink * link);
	virtual ~CLinkHist();

	void dumpToFile();

    CNeuronLink * mpLink;
	vector<double> mWeight;
};

class CNetworkHistManager {
public:
	CNetworkHistManager(CNeuralNetwork * network);
	virtual ~CNetworkHistManager();

	double getActivation(CNeuron * neuron, int k);
	double getWeight(CNeuronLink * link, int k);

	void recordHist();
	void init();

	void dumpLinkHistToFile();
	void dumpNeuronHistToFile();

	void clearNeuronHist() { mNeuronHists.clear(); };
	void clearLinkHist() { mLinkHists.clear(); };


private:
	CNeuralNetwork * mpNetwork;
	vector<CNeuronHist> mNeuronHists;
	vector<CLinkHist> mLinkHists;
};

#endif /* CNETWORKHISTMANAGER_H_ */
