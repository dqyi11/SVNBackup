/*
 * CLayer.h
 *
 *  Created on: Feb 3, 2013
 *      Author: walter
 */

#ifndef CLAYER_H_
#define CLAYER_H_

#include "CNeuron.h"
#include <vector>
#include <string>

using namespace std;

class CLayer {
public:
	CLayer(string name, NeuronType type);
	virtual ~CLayer();

    bool addNeuron(string name, NeuronType type);
	bool hasNeuron(string name);

	string getName() { return mName; };
	NeuronType getType() { return mType; };
	CNeuron * getNeuron(string name);

	int getNeuronNum() { return mNeurons.size(); };
	CNeuron * getNeuron(int index);

	void print();
	void printState();

	int getIndex(CNeuron * pNeuron);
protected:

	CNeuron * createNeuron(string name, NeuronType type);
	bool addNeuron(CNeuron * pNeuron);

private:
	string mName;
	vector<CNeuron*> mNeurons;
	NeuronType mType;
};

#endif /* CLAYER_H_ */
