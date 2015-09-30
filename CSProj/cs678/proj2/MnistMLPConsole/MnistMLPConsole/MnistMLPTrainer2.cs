using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;
using NeuralNetwork;

namespace NeuralNetwork
{
    class MnistMLPTrainer2
    {
        MnistDataMgr _dataMgr;
        MLPNetworkWithBias _network;

        bool _normalizeInput;
        bool _normalizeOuput;

        bool _randomSelect = false;

        const int _testSetSize = 200;
        const int _trainSetSize = 500;

        double _successRate = 0;
        double _mse = 0;
        const double _acceptableSuccessRate = 1;


        public MnistMLPTrainer2(MnistDataMgr mgr, bool nInput = true, bool nOutput = true)
        {
            _dataMgr = mgr;
            _normalizeInput = nInput;
            _normalizeOuput = nOutput;
        }

        public void InitMLP(int nrHidden, double learnRate)
        {
            int nrInput = _dataMgr.count;
            int nrOutput;
            if (_normalizeOuput)
            {
                nrOutput = 1;
            }
            else
            {
                nrOutput = 10;
            }

            _network = new MLPNetworkWithBias(nrInput, nrHidden, nrOutput, learnRate);
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

                    //Console.WriteLine(i);
                    Console.Write("-");
          
                    double[] input;

                    if (_normalizeInput)
                    {
                        input = _dataMgr.GetNormalizedInputData(index);
                    }
                    else{
                        input = _dataMgr.GetInputData(index);
                    }
                    double [] label = new double[]{ _dataMgr.GetDoubleLabelData(index)/10};

                    _network.OnlineTrain(input, label);
                    
                }
                Console.Write("\n");
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
                    if (_normalizeOuput)
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

                        double[] output = _network.GetOutput();

                        Console.Write(output[0]);
                        Console.Write("/"+labelData/10 + " ");

                        //Console.Write("compare " + labelData + " and " + maxIndex + "    ");
                        if (labelData/10 == output[0])
                        {
                            successTime++;
                        }

                        
                        //double [] outputData = _network.GetIntOutput();


                        mse += GetMSE(output[0], labelData/10);                 
                    }

                }

            }

            Console.Write("\n");
            _mse = mse / (double)_testSetSize;
            _successRate = (double)successTime / (double)_testSetSize;
        }

        bool stopCriteria()
        {
            /*
            if (_successRate >= _acceptableSuccessRate)
            {
                return true;
            }
             */

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

        double GetMSE(double a, double b)
        {
            double mse = 0;
            mse = Math.Pow(a - b, 2);

            mse = Math.Sqrt(mse);

            return mse;
        }
    }
}
