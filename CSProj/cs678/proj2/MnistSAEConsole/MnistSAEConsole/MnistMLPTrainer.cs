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

        const int _testSetSize = 20000;
        const int _trainSetSize = 60000;

        double _successRate = 0;
        double _mse = 0;
        const double _acceptableSuccessRate = 0.995;

        int[] _layerStruct;

        public MnistMLPTrainer(MnistDataMgr mgr, MLP mlp, bool nInput = true, bool nOutput = false)
        {
            _dataMgr = mgr;
            _normalizeInput = nInput;
            _normalizeOuput = nOutput;
            _network = mlp;
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

            if (null == _network)
            {
                _network = new MLP(_layerStruct, learnRate);
            }
        }

        public void OnlineTrain()
        {
            int trainCnt = 0;
            Random rnd = new Random();

            while (!stopCriteria(trainCnt))
            {
                Console.Write("\n");
                for (int i = 0; i < _trainSetSize; i++)
                {
                    int index = rnd.Next(_dataMgr.count - 1);

                    //Console.WriteLine(i);
                    //Console.Write("-");

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
                //Console.Write("\n");
                Test();
                trainCnt++;

                Console.WriteLine("TRAINING TIME " + trainCnt + " with successrate " + _successRate + " and mse " + _mse);
            }            
        }

        public void BatchTrain()
        {
            int trainCnt = 0;
            Random rnd = new Random();

            double [][] inputSet = new double[_trainSetSize][];
            double[][] labelSet = new double[_trainSetSize][];

            while (!stopCriteria(trainCnt))
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

                Console.WriteLine("TRAINING TIME " + trainCnt + " with successrate " + _successRate + " and mse " + _mse);
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

                        //Console.Write(maxIndex);
                        //Console.Write("/"+labelData + " ");

                        //Console.Write("compare " + labelData + " and " + maxIndex + "    ");
                        if (labelData == maxIndex)
                        {
                            successTime++;
                        }

                        
                        double [] outputData = _network.GetOutput();

                        mse += GetMSE(outputData, labelData2);                 
                    }

                }

            }

            //Console.Write("\n");
            _mse = mse / (double)_testSetSize;
            _successRate = (double)successTime / (double)_testSetSize;
        }

        bool stopCriteria(int trainCnt)
        {
            if (trainCnt > 10)
            {
                return true;
            }

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
