/*
 * CNeuron.h
 *
 *  Created on: Feb 1, 2013
 *      Author: walter
 */

#ifndef CNEURON_H_
#define CNEURON_H_

#include <vector>
#include <string>

using namespace std;


enum NeuronType
{
	INPUT, OUTPUT, HIDDEN,
};

enum ActivationFuncType
{
	LOGISTIC, HYPERTANGENT,
};

typedef struct learningTempVar
{
	double mOutputError;
	double mInputSum;
	double mInputSumDerivate;
	double mLocalGradient;
};

class CNeuron;

class CNeuronLink
{
public:
	CNeuronLink(CNeuron * from, CNeuron * to, double weight);
	virtual ~CNeuronLink();

	double mWeight;
	CNeuron * mpFromNeuron;
	CNeuron * mpToNeuron;
};

class CNeuron {
public:
	CNeuron(string name, NeuronType type);
	virtual ~CNeuron();

	string getName() { return mName; };
	NeuronType getType() { return mType; };

	CNeuronLink * getInputLink(CNeuron * pNeuron);
	CNeuronLink * getOutputLink(CNeuron * pNeuron);

	void addInputLink(CNeuronLink * pLink);
	void addOutputLink(CNeuronLink * pLink);

	bool hasInputLink(CNeuronLink * pLink);
	bool hasOutputLink(CNeuronLink * pLink);

	double getActivation() { return mActivation; };
	double getThreshold() { return mThreshold; };

	ActivationFuncType getFuncType() { return mFuncType; };
	void setFuncType(ActivationFuncType type) { mFuncType = type; };

	double getFuncParamA() { return mFuncParamA; };
	void setFuncParamA(double a) { mFuncParamA = a; };
	double getFuncParamB() { return mFuncParamB; };
	void setFuncParamB(double b) { mFuncParamB = b; };

	double activationFunction(double input);
	double activationFuncDerivate(double input);

	void update();

	void setActivation(double activation) { mActivation = activation; };
	void setThreshold(double threshold) { mThreshold = threshold; };

	void setK(int k) { mK = k; };
	int getK() { return mK; };

	void setLinKNeuron(CNeuron * neuron) { mpLinkNeuron = neuron; };
	CNeuron * getLinkNeuron() { return mpLinkNeuron; };

	void print();

	vector<CNeuronLink*> mInputs;
	vector<CNeuronLink*> mOutputs;
private:
	string mName;
	NeuronType mType;

	int mK;
	CNeuron * mpLinkNeuron;

	ActivationFuncType mFuncType;
	double mFuncParamA;
	double mFuncParamB;

	double mActivation;
	double mThreshold;

public:
	double mInputSum;

};

#endif /* CNEURON_H_ */
