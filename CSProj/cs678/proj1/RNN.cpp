//============================================================================
// Name        : CS678-Project1.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "RNN.h"
#include <iostream>
using namespace std;

int main(int argc, char* argv[]) {

	cout << "Hello World" << endl; // prints Hello World

	/*
	CPersonActivityDataParser parser(argv[1]);
	parser.doParsing();

	CPersonActivityDataManager manager;

	parser.initDataManager(&manager);
	*/

	CSyntheticCtrlDataManager manager;

	//manager.dumpArrayDataToFile();

	int trainingSize = 10;

	/*
	CRecurrentNeuralNetworkController controller;
	controller.setK(1);
	controller.setInputNum(4);
	controller.setHiddenNum(12);
	controller.setOutputNum(11);
	*/

	CSimpleRNNController controller;
	controller.setK(1);
	controller.setInputNum(1);
	controller.setHiddenNum(4);
	controller.setOutputNum(6);
	controller.setLearningRate(10);
	controller.setDataManager(&manager);
	controller.setTrainingSize(trainingSize);

	controller.init();
	//controller.draw("NeuralNet.png");


	controller.training();

	/*
	cout << " SAVING ... " << endl;
	controller.saveHist();
	*/

	/*
	controller.setTestingSize(10);
	controller.testing();
	*/

	/*
	CNeuralNetwork neuralNet;
	CNeuralNetworkVisualizer netVisualizer(&neuralNet);
	neuralNet.addLayer("X", INPUT);
	neuralNet.addLayer("Y", HIDDEN);
	neuralNet.addLayer("Z", OUTPUT);

	neuralNet.addNeuron("X","X1");
	neuralNet.addNeuron("X","X2");

	neuralNet.addNeuron("Y","Y1");
	neuralNet.addNeuron("Y","Y2");
	neuralNet.addNeuron("Y","Y3");

	neuralNet.addNeuron("Z","Z1");

	neuralNet.connect("X1","Y1");
	neuralNet.connect("X1","Y2");
	neuralNet.connect("X1","Y3");
	neuralNet.connect("X2","Y1");
	neuralNet.connect("X2","Y2");
	neuralNet.connect("X2","Y3");
	neuralNet.connect("Y1","Z1");
	neuralNet.connect("Y2","Z1");
	neuralNet.connect("Y3","Z1");

	netVisualizer.draw("NeuralNet.png");
	*/

	return 0;
}
