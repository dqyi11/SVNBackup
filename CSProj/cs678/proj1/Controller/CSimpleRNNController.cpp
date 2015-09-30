/*
 * CSimpleRNNController.cpp
 *
 *  Created on: Feb 6, 2013
 *      Author: walter
 */

#include "CSimpleRNNController.h"

#include "CSimpleRNNController.h"
#include <sstream>
#include <iostream>
#include <cmath>

using namespace std;

CSimpleRNNController::CSimpleRNNController() {
	// TODO Auto-generated constructor stub
	mpNetwork = new CNeuralNetwork();
	mpVisualizer = new CNeuralNetworkVisualizer(mpNetwork);
	mpHistMgr = new CNetworkHistManager(mpNetwork);
	mpDataMgr = NULL;

	mK = -1;
	mInputNum = -1;
	mHiddenNum = -1;
	mOutputNum = -1;
	mTrainingSize = 1;
}

CSimpleRNNController::~CSimpleRNNController() {
	// TODO Auto-generated destructor stub
	if(mpHistMgr)
	{
		delete mpHistMgr;
		mpHistMgr = NULL;
	}
	if(mpVisualizer)
	{
		delete mpVisualizer;
		mpVisualizer = NULL;
	}
	if(mpNetwork)
	{
		delete mpNetwork;
		mpNetwork = NULL;
	}


}

bool CSimpleRNNController::init()
{
	stringstream strStream;
	string nodeName = "";

	if(mK<=0 || mInputNum<=0 || mOutputNum<=0)
	{
		return false;
	}

	if(mHiddenNum<=0)
	{
		mpNetwork->addLayer("I", INPUT);
		//init input
		for(int i=1;i<=mInputNum;i++)
		{
			strStream.clear();
			strStream << "I-" << i;
			strStream >> nodeName;
			mpNetwork->addNeuron("I", nodeName);

		}

		mpNetwork->addLayer("O", OUTPUT);
		//init hidden
		for(int i=1;i<=mOutputNum;i++)
		{
			strStream.clear();
			strStream << "O-" << i;
			strStream >> nodeName;
			mpNetwork->addNeuron("O", nodeName);
		}

		mpNetwork->addLayer("C", INPUT);
		CLayer * outputLayer = mpNetwork->getOutputLayer();
		//init recurrent context
		for(int j=1;j<=mK;j++)
		{
			for(int i=1;i<=mOutputNum;i++)
			{
				strStream.clear();
				strStream << "O-" << i << "(" << j << ")";
				strStream >> nodeName;
				mpNetwork->addNeuron("C", nodeName);
				CNeuron * cNeuron = mpNetwork->getNeuron(nodeName);

				cNeuron->setK(j);
				cNeuron->setLinKNeuron(outputLayer->getNeuron(i));
			}
		}
		mpNetwork->fullyConnect("I","O",0.1);
		mpNetwork->fullyConnect("C","O",0.1);

		mpHistMgr->init();
	}
	else
	{

		if(mK<=0 || mInputNum<=0 || mHiddenNum<=0 || mOutputNum<=0)
		{
			return false;
		}

		mpNetwork->addLayer("I", INPUT);
		//init input
		for(int i=1;i<=mInputNum;i++)
		{
			strStream.clear();
			strStream << "I-" << i;
			strStream >> nodeName;
			mpNetwork->addNeuron("I", nodeName);

		}

		mpNetwork->addLayer("H", HIDDEN);
		//init hidden
		for(int i=1;i<=mHiddenNum;i++)
		{
			strStream.clear();
			strStream << "H-" << i;
			strStream >> nodeName;
			mpNetwork->addNeuron("H", nodeName);
		}

		mpNetwork->addLayer("O", OUTPUT);
		//init hidden
		for(int i=1;i<=mOutputNum;i++)
		{
			strStream.clear();
			strStream << "O-" << i;
			strStream >> nodeName;
			mpNetwork->addNeuron("O", nodeName);
		}

		CLayer * hiddenLayer = mpNetwork->getLayer("H");
		mpNetwork->addLayer("C", INPUT);
		//init recurrent context
		for(int j=1;j<=mK;j++)
		{
			for(int i=1;i<=mHiddenNum;i++)
			{
				strStream.clear();
				strStream << "H-" << i << "(" << j << ")";
				strStream >> nodeName;
				mpNetwork->addNeuron("C", nodeName);

				CNeuron * cNeuron = mpNetwork->getNeuron(nodeName);

				cNeuron->setK(j);
				cNeuron->setLinKNeuron(hiddenLayer->getNeuron(i));
			}
		}

		mpNetwork->fullyConnect("I","H",0.1);
		mpNetwork->fullyConnect("C","H",0.1);
		mpNetwork->fullyConnect("H","O",0.2);

		mpHistMgr->init();

	}

	// mpNetwork->print();

	return true;
}

void CSimpleRNNController::draw(char * filename)
{
	mpVisualizer->draw(filename);
}

void CSimpleRNNController::update(int t, int base)
{
	ctrlEpoch * pEpoch = mpDataMgr->getTrainingData(t);
	if(NULL==pEpoch)
	{
		return;
	}
	if(mpNetwork)
	{
		CLayer * inputLayer = mpNetwork->getLayer("I");
	    CLayer * contextLayer = mpNetwork->getLayer("C");
	    CLayer * outputLayer = mpNetwork->getOutputLayer();
	    CLayer * hiddenLayer = mpNetwork->getLayer("H");
	    int outputNum = outputLayer->getNeuronNum();
		learningTempVar outputVar[outputNum][mK];
		int hiddenNum = hiddenLayer->getNeuronNum();
		learningTempVar hiddenVar[hiddenNum][mK];

		for(int j=0;j<mK;j++)
		{
			cout << j << " in epoch :";
			ctrlData iData = pEpoch->mSet[j];
			cout << iData.mX << ":";
			cout << iData.mResult[0] << iData.mResult[1] << iData.mResult[2]
			      << iData.mResult[3] << iData.mResult[4] << iData.mResult[5] <<endl;


			for(int i=0;i<inputLayer->getNeuronNum();i++)
			{
				CNeuron * iNeuron = inputLayer->getNeuron(i);
				double value;
				switch(i)
				{
				case 0:
				default:
					value = iData.mX;
					break;
				}
				iNeuron->setActivation(value);
			}

			for(int i=0;i<contextLayer->getNeuronNum();i++)
			{
				CNeuron * cNeuron = contextLayer->getNeuron(i);
				for(int x=0;x<mK;x++)
				{
					CNeuron * oNeuron = cNeuron->getLinkNeuron();
					int lookBackNum = cNeuron->getK();

					int histIndex = t*mK-lookBackNum+j+base;
					//cout << " current t " << t ;
					//cout << " lookbackNum " << lookBackNum << " histIndex " << histIndex << endl;
					double value = mpHistMgr->getActivation(oNeuron, histIndex);
					cNeuron->setActivation(value);
				}
;			}
			/*
			cout << "INPUT " << endl;
			inputLayer->printState();
			*/

			mpNetwork->calculate();

			mpHistMgr->recordHist();

			/*
			cout << "OUPUT " << endl;
			outputLayer->printState();
			*/

			for(int i=0;i<outputLayer->getNeuronNum();i++)
			{
				CNeuron * oNeuron = outputLayer->getNeuron(i);
				outputVar[i][j].mOutputError = pEpoch->mSet[j].mResult[i]-oNeuron->getActivation();
				outputVar[i][j].mInputSum = oNeuron->mInputSum;
				outputVar[i][j].mInputSumDerivate = oNeuron->activationFuncDerivate(oNeuron->mInputSum);
			}

			for(int i=0;i<hiddenLayer->getNeuronNum();i++)
			{
				CNeuron * hNeuron = hiddenLayer->getNeuron(i);
				hiddenVar[i][j].mOutputError = 0;
				hiddenVar[i][j].mInputSum = hNeuron->mInputSum;
				outputVar[i][j].mInputSumDerivate = hNeuron->activationFuncDerivate(hNeuron->mInputSum);
			}
		}

		//cout << " training .. " << endl;

		// calc local gradient
		for(int j=mK-1;j>=0;j--)
		{
			for(int i=0;i<outputLayer->getNeuronNum();i++)
			{
				CNeuron * oNeuron = outputLayer->getNeuron(i);
				//if(j==mK-1)
				{
					outputVar[i][j].mLocalGradient = outputVar[i][j].mInputSumDerivate
							* outputVar[i][j].mOutputError;
				}

			}

			for(int i=0;i<hiddenLayer->getNeuronNum();i++)
			{
				CNeuron * hNeuron = hiddenLayer->getNeuron(i);

				double propagateFromLastLevel = 0;

				if(j==mK-1)
				{
					vector<CNeuronLink*>::iterator it;
					for(it=hNeuron->mOutputs.begin();it!=hNeuron->mOutputs.end();it++)
					{
						CNeuron * tempToNeuron = (*it)->mpToNeuron;

						int outputLayerIndex = outputLayer->getIndex(tempToNeuron);
					    propagateFromLastLevel += (*it)->mWeight *
					    		outputVar[outputLayerIndex][j].mLocalGradient;
					}
				}
				else
				{
					vector<CNeuronLink*>::iterator it;
					for(it=hNeuron->mOutputs.begin();it!=hNeuron->mOutputs.end();it++)
					{
						CNeuron * tempToNeuron = (*it)->mpToNeuron;

						int outputLayerIndex = outputLayer->getIndex(tempToNeuron);
					    propagateFromLastLevel += (*it)->mWeight *
					    		outputVar[outputLayerIndex][j].mLocalGradient;
					}

					// from last level of hidden
					for(it=hNeuron->mInputs.begin();it!=hNeuron->mInputs.end();it++)
					{
						CNeuron * tempFromNeuron = (*it)->mpFromNeuron;

						CNeuron * linkNeuron = tempFromNeuron->getLinkNeuron();

						int linkHiddenLink = hiddenLayer->getIndex(linkNeuron);

						//cout << "linkHiddenLink " << linkHiddenLink << endl;

						if(linkHiddenLink >= 0)
						{

							propagateFromLastLevel += (*it)->mWeight * hiddenVar[linkHiddenLink][j+1].mLocalGradient;
						}


					}

				}

				hiddenVar[i][j].mLocalGradient = hiddenVar[i][j].mInputSumDerivate
											* propagateFromLastLevel;

			}
		}

		//cout << " LOCAL GRADIENT DONE " << endl;

		// use local gradient to update weight
		for(int i=0;i<outputLayer->getNeuronNum();i++)
		{
			CNeuron * oNeuron = outputLayer->getNeuron(i);

			vector<CNeuronLink*>::iterator it;
			for(it=oNeuron->mInputs.begin();it!=oNeuron->mInputs.end();it++)
			{
				CNeuron * tempFromNeuron = (*it)->mpFromNeuron;
				double temp = 0;
				for(int k=0;k<mK;k++)
				{
					int histIndex = t*mK-mK+k+base;
					temp += mpHistMgr->getActivation(tempFromNeuron, histIndex)
							* outputVar[i][k].mLocalGradient;
				}

				(*it)->mWeight -= mLearningRate * temp;

			}

		}

		//cout << "output layer weights updated "  << endl;

		for(int i=0;i<hiddenLayer->getNeuronNum();i++)
		{
			CNeuron * hNeuron = hiddenLayer->getNeuron(i);

			vector<CNeuronLink*>::iterator it;
			for(it=hNeuron->mInputs.begin();it!=hNeuron->mInputs.end();it++)
			{
				CNeuron * tempFromNeuron = (*it)->mpFromNeuron;
				double temp = 0;
				for(int k=0;k<mK;k++)
				{
					int histIndex = t*mK-mK+k+base;
					temp += outputVar[i][k].mLocalGradient
							* mpHistMgr->getActivation(tempFromNeuron, histIndex);

				}

				(*it)->mWeight -= mLearningRate * temp;

			}

		}

	}

}

void CSimpleRNNController::saveHist()
{
	if(mpHistMgr)
	{
		mpHistMgr->dumpLinkHistToFile();
	}
}

void CSimpleRNNController::testing()
{
	mpDataMgr->prepareTestingData(mTestingSize, mK+1);

	cout << " preparing test data " << mTestingSize << endl;
	double totalError = 0;
	double oneTimeError = 0;
	double epochError = 0;
	for(int t=0;t<mTestingSize;t++)
	{
		ctrlEpoch * pEpoch = mpDataMgr->getTestingData(t);

		epochError = 0;

		if(mpNetwork)
		{

			cout << "tesing round " << t << endl;

			CLayer * inputLayer = mpNetwork->getLayer("I");
			CLayer * contextLayer = mpNetwork->getLayer("C");
			CLayer * outputLayer = mpNetwork->getOutputLayer();
			CLayer * hiddenLayer = mpNetwork->getLayer("H");
			int outputNum = outputLayer->getNeuronNum();
			learningTempVar outputVar[outputNum][mK];
			int hiddenNum = hiddenLayer->getNeuronNum();
			learningTempVar hiddenVar[hiddenNum][mK];

			for(int j=0;j<mK;j++)
			{
				cout << "mK is " << j << endl;
				cout << "epoch size " << pEpoch->mSet.size() << endl;
				ctrlData iData = pEpoch->mSet[j];

				cout << "input layer size " << inputLayer->getNeuronNum() << endl;

				for(int i=0;i<inputLayer->getNeuronNum();i++)
				{
					CNeuron * iNeuron = inputLayer->getNeuron(i);
					double value = iData.mX;
					iNeuron->setActivation(value);
				}


				cout << " Check point " << endl;
				for(int i=0;i<contextLayer->getNeuronNum();i++)
				{
					CNeuron * cNeuron = contextLayer->getNeuron(i);
					for(int x=0;x<mK;x++)
					{
						CNeuron * hNeuron = cNeuron->getLinkNeuron();
						//int lookBackNum = cNeuron->getK();

						int histIndex = t*mK-mK+j;
						//int histIndex = t*mK-lookBackNum+j;
						//cout << " current t " << t ;
						//cout << " lookbackNum " << lookBackNum << " histIndex " << histIndex << endl;
						double value = mpHistMgr->getActivation(hNeuron, histIndex);
						cNeuron->setActivation(value);
					}
				}

				//cout << " calculating " << endl;

				mpNetwork->calculate();

				double oneTimeError = 0;
				for(int i=0;i<outputLayer->getNeuronNum();i++)
				{
					CNeuron * oNeuron= outputLayer->getNeuron(i);
					cout << " ouput activation is " << oNeuron->getActivation()<< endl;
					cout << " epoch is " << pEpoch->mSet[j].mResult[i] << endl;
					oneTimeError += pow(oNeuron->getActivation()-pEpoch->mSet[j].mResult[i],2);

					// cout << "DODODO " << oneTimeError << endl;
				}

				oneTimeError = sqrt(oneTimeError/6);

				epochError += oneTimeError;

				// cout << "KAKAKA " << epochError << endl;

			}

		}

		cout << " now total error is " << totalError << endl;
		totalError += epochError/mK;
	}

	totalError = totalError/ mTestingSize;


	cout << "Total Error: " << totalError << endl;

}


void CSimpleRNNController::training()
{
	int loopSize = 10000;
	int loopTime = mTrainingSize/loopSize;

	if(mTrainingSize < loopSize)
	{
		if(mpDataMgr)
		{
			cout << "mpDataMgr is not NULL " << endl;
			mpDataMgr->prepareTrainingData(mTrainingSize, mK+1);
			//mpDataMgr->printTrainingData();
		}

		for(int i=0;i<mTrainingSize;i++)
		{
			cout << " DOING " << i << endl;
			update(i, 0);
		}
	}
	else
	{
		for(int j=0;j<loopTime;j++)
		{
			if(mpDataMgr)
			{
				cout << "mpDataMgr is not NULL " << endl;
				mpDataMgr->prepareTrainingData(loopSize, mK+1);
				//mpDataMgr->printTrainingData();
			}

			for(int i=0;i<loopSize;i++)
			{
				cout << " DOING " << i << endl;
				update(i, loopTime*loopSize);
			}
		}
	}
}


