/*
 * CSimpleRNNController.h
 *
 *  Created on: Feb 6, 2013
 *      Author: walter
 */

#ifndef CSIMPLERNNCONTROLLER_H_
#define CSIMPLERNNCONTROLLER_H_

#include "../NeuralNetwork/CNeuralNetwork.h"
#include "../Visualizer/CNeuralNetworkVisualizer.h"
#include "CNetworkHistManager.h"
#include "../CtrlData/CSyntheticCtrlDataManager.h"


class CSimpleRNNController {
public:
	CSimpleRNNController();
	virtual ~CSimpleRNNController();

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

	void setDataManager(CSyntheticCtrlDataManager * dataMgr) { mpDataMgr = dataMgr; };
	CSyntheticCtrlDataManager * getDataManager() { return mpDataMgr; };

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
	CSyntheticCtrlDataManager * mpDataMgr;
	int mK;
	int mInputNum;
	int mHiddenNum;
	int mOutputNum;

	int mTrainingSize;
	int mTestingSize;

	double mLearningRate;

};

#endif /* CSIMPLERNNCONTROLLER_H_ */
