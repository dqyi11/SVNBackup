using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace NeuralNetwork
{
    class XORTrainer
    {
        MLPNetwork _network;

        bool _normalizeInput;
        bool _normalizeOuput;

        bool _randomSelect = false;

        const int _testSetSize = 40;
        const int _trainSetSize = 100;

        double _successRate = 0;
        const double _acceptableSuccessRate = 0.95;

        double[][] _inputPattern =
            {
                new double[2] { 0.0, 0.0 },
                new double[2] { 1.0, 0.0 },
                new double[2] { 0.0, 1.0 },
                new double[2] { 1.0, 1.0 } };
        double[][] _targetOutputPattern = 
             {
                 new double[1] { 0.0 },
                 new double[1] { 1.0 },
                 new double[1] { 1.0 },
                 new double[1] { 0.0 } };


        public XORTrainer(bool nInput = true, bool nOutput = false)
        {
            _normalizeInput = nInput;
            _normalizeOuput = nOutput;
        }

        public void InitMLP(int nrHidden, double learnRate)
        {
            int nrInput = 2;
            int nrOutput = 1;
            _network = new MLPNetwork(nrInput, nrHidden, nrOutput, learnRate);
        }

        public void OnlineTrain()
        {
            int trainCnt = 0;
            Random rnd = new Random();

            double[] input;
            double[] label;

            while (!stopCriteria())
            {
                for (int i = 0; i < _trainSetSize; i++)
                {
                    int index = rnd.Next(4);
                    input = _inputPattern[index];
                    label = _targetOutputPattern[index];

                    _network.OnlineTrain(input, label);
                }

                Test();
                trainCnt++;

                Console.WriteLine("TRAINING TIME " + trainCnt + " with " + _successRate);
            }            
        }

        public void BatchTrain()
        {
            int trainCnt = 0;
            Random rnd = new Random();

            double [][] inputSet = new double[_trainSetSize][];
            double[][] labelSet = new double[_trainSetSize][];

            while (!stopCriteria())
            {
                for (int i = 0; i < _trainSetSize; i++)
                {
                    int index = rnd.Next(4);

                    inputSet[i] = _inputPattern[index];
                    labelSet[i] = _targetOutputPattern[index];
                    
                }
                _network.BatchTrain(inputSet, labelSet);

                Test();
                trainCnt++;

                Console.WriteLine("TRAINING TIME " + trainCnt + " with " + _successRate);
            }      
        }

        void Test()
        {
            int successTime = 0;

            if (!_randomSelect)
            {
                for (int i = 0; i < 4; i++)
                {
                    if (!_normalizeOuput)
                    {
                        double[] labelData = _targetOutputPattern[i];
                        double[] inputData = _inputPattern[i];
                        _network.SetInput(inputData);
                        _network.Feedfowrad();

                        double [] outputData = _network.GetIntOutput();
                       
                        if (labelData.Length == outputData.Length)
                        {
                            bool success=true;
                            for (int k = 0; k < outputData.Length; k++)
                            {
                                if (labelData[k] != outputData[k])
                                {
                                    success = false;
                                }
                            }
                            if (success)
                            {
                                successTime++;
                            }
                        }
                         
                    }

                }

            }
            
            _successRate = (double)successTime / (double)_testSetSize;
        }

        bool stopCriteria()
        {
            if (_successRate >= _acceptableSuccessRate)
            {
                return true;
            }

            return false;
        }
    }
}
