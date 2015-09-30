using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;
using NeuralNetwork;

namespace NeuralNetwork
{
    class MnistMLPTrainer
    {
        MnistDataMgr _dataMgr;
        MLP _network;

        bool _normalizeInput;
        bool _normalizeOuput;

        bool _randomSelect = false;

        const int _testSetSize = 10000;
        const int _trainSetSize = 30000;

        double _successRate = 0;
        double _mse = 0;
        const double _acceptableSuccessRate = 0.995;

        int[] _layerStruct;

        public MnistMLPTrainer(MnistDataMgr mgr, bool nInput = true, bool nOutput = false)
        {
            _dataMgr = mgr;
            _normalizeInput = nInput;
            _normalizeOuput = nOutput;
        }

        public MLP GetNetwork()
        {
            return _network;
        }

        public void InitMLP(int nrHidden, double learnRate)
        {
            int nrInput = _dataMgr.inputNum;
            int nrOutput;
            if (_normalizeOuput)
            {
                nrOutput = 1;
            }
            else
            {
                nrOutput = 10;
            }

            _layerStruct = new int[] { nrInput, nrHidden, nrOutput };

            _network = new MLP(_layerStruct, learnRate);
        }

        public void InitMLP(int[] layerStruct, double learnRate)
        {
            _normalizeOuput = false;
            _normalizeInput = true;
            _layerStruct = layerStruct;
            _network = new MLP(_layerStruct, learnRate);
        }

        public void OnlineTrain()
        {
            int trainCnt = 0;
            Random rnd = new Random();

            while (!stopCriteria())
            {
                Console.Write("\n");
                for (int i = 0; i < _trainSetSize; i++)
                {
                    int index = rnd.Next(_dataMgr.count - 1);

                    if (!_normalizeOuput)
                    {
                        double[] input;

                        if (_normalizeInput)
                        {
                            input = _dataMgr.GetNormalizedInputData(index);
                        }
                        else{
                            input = _dataMgr.GetInputData(index);
                        }
                        double[] label = _dataMgr.GetIntLabelData(index);

                        _network.OnlineTrain(input, label);
                    }
                }
 
                Test();
                trainCnt++;

                if (trainCnt % 5==0)
                {
                    _network.DumpToFile();
                
                }

                Console.WriteLine("TRAINING TIME " + trainCnt + " with successrate " + _successRate + " and mse " + _mse);
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
                    int index = rnd.Next(_dataMgr.count - 1);

                    inputSet[i] = _dataMgr.GetInputData(index);

                    if (!_normalizeOuput)
                    {
                        labelSet[i] = _dataMgr.GetIntLabelData(index);
                    }
                }

                _network.BatchTrain(inputSet, labelSet);

                Test();
                trainCnt++;

                //Console.WriteLine("TRAINING TIME " + trainCnt + " with successrate " + _successRate + " and mse " + _mse);
                Console.WriteLine(trainCnt + " ， " + _successRate + " ， " + _mse);
            }      
        }

        void Test()
        {
            int successTime = 0;
            double mse = 0;

            if (!_randomSelect)
            {
                Random rnd = new Random();
                int testStartIndex = rnd.Next(_dataMgr.count - _testSetSize);

                for (int i = testStartIndex; i < testStartIndex + _testSetSize; i++)
                {
                    if (!_normalizeOuput)
                    {
                        double labelData = _dataMgr.GetDoubleLabelData(i);
                        double[] labelData2 = _dataMgr.GetIntLabelData(i);
                        double[] inputData;
                        if (_normalizeInput)
                        {
                            inputData = _dataMgr.GetNormalizedInputData(i);
                        }
                        else
                        {
                            inputData = _dataMgr.GetInputData(i);
                        }

                        _network.SetInput(inputData);
                        _network.Feedfowrad();

                        double maxIndex = _network.GetMaxOutputIndex();

                        if (labelData == maxIndex)
                        {
                            successTime++;
                        }

                        double [] outputData = _network.GetOutput();

                        mse += GetMSE(outputData, labelData2);                 
                    }
                }

            }

            _mse = mse / (double)_testSetSize;
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

        double GetMSE(double[] a, double[] b)
        {
            double mse = 0;

            for (int i = 0; i < a.Length; i++)
            {
                mse += Math.Pow(a[i] - b[i], 2);
            }

            mse = Math.Sqrt(mse / a.Length);

            return mse;
        }
    }
}
