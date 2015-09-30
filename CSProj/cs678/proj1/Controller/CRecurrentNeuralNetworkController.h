/*
 * CRecurrentNeuralNetworkController.h
 *
 *  Created on: Feb 4, 2013
 *      Author: walter
 */

#ifndef CRECURRENTNEURALNETWORKCONTROLLER_H_
#define CRECURRENTNEURALNETWORKCONTROLLER_H_

#include "../NeuralNetwork/CNeuralNetwork.h"
#include "../Visualizer/CNeuralNetworkVisualizer.h"
#include "CNetworkHistManager.h"
#include "../Data/CPersonActivityDataManager.h"


class CRecurrentNeuralNetworkController {
public:
	CRecurrentNeuralNetworkController();
	virtual ~CRecurrentNeuralNetworkController();

	void setLearningRate(double rate) { mLearningRate = rate; };
	double getLearningRate() { return mLearningRate; };

	void setK(int k) { mK = k; };
	int getK() { return mK; };

	void setInputNum(int num) { mInputNum = num; };
	int getInputNum() { return mInputNum; };

	void setHiddenNum(int num) { mHiddenNum = num; };
	int getHiddenNum() { return mHiddenNum; };

	void setOutputNum(int num) { mOutputNum = num; };
	int getOutputNum() { return mOutputNum; };

	bool init();

	void update(int t, int base);

	CNeuralNetwork * getNetwork() { return mpNetwork; };
	void draw(char * filename);

	void setDataManager(CPersonActivityDataManager * dataMgr) { mpDataMgr = dataMgr; };
	CPersonActivityDataManager * getDataManager() { return mpDataMgr; };

	void setTrainingSize(int size) { mTrainingSize = size; };
	int getTrainingSize() { return mTrainingSize; };

	void setTestingSize(int size) { mTestingSize = size; };
	int getTestingSize() { return mTestingSize; };

	void training();
	void testing();

	void saveHist();

private:
	CNeuralNetwork * mpNetwork;
	CNeuralNetworkVisualizer * mpVisualizer;
	CNetworkHistManager * mpHistMgr;
	CPersonActivityDataManager * mpDataMgr;
	int mK;
	int mInputNum;
	int mHiddenNum;
	int mOutputNum;

	int mTrainingSize;
	int mTestingSize;

	double mLearningRate;

};

#endif /* CRECURRENTNEURALNETWORKCONTROLLER_H_ */
