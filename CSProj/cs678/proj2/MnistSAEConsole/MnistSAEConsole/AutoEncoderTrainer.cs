using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;

namespace NeuralNetwork
{
    class AutoEncoderTrainer
    {
        MnistDataMgr _dataMgr;
        AutoEncoder _autoEncoder;

        const int _testSetSize = 20000;
        const int _trainSetSize = 60000;

        bool _randomSelect = false;
        double _mse = 0;

        public AutoEncoderTrainer(AutoEncoder encoder, MnistDataMgr mgr)
        {
            _autoEncoder = encoder;
            _dataMgr = mgr;
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

                    double[] input = _dataMgr.GetNormalizedInputData(index);
                      
                    //double[] label = _dataMgr.GetIntLabelData(index);

                    _autoEncoder.GetMLP().OnlineTrain(input, input);
                    
                }
                //Console.Write("\n");
                Test();
                trainCnt++;

                Console.WriteLine("TRAINING TIME " + trainCnt +  " mse " + _mse);
            }
        }

        void Test()
        {
            double mse = 0;

            if (!_randomSelect)
            {
                Random rnd = new Random();
                int testStartIndex = rnd.Next(_dataMgr.count - _testSetSize);

                for (int i = testStartIndex; i < testStartIndex + _testSetSize; i++)
                {
                    double[] inputData = _dataMgr.GetNormalizedInputData(i);

                    _autoEncoder.GetMLP().SetInput(inputData);
                    _autoEncoder.GetMLP().Feedfowrad();
                    double[] outputData = _autoEncoder.GetMLP().GetOutput();

                    mse += GetMSE(outputData, inputData);
                    

                }

            }

            //Console.Write("\n");
            _mse = mse / (double)_testSetSize;
        }

        bool stopCriteria(int trainCnt)
        {
            
            if (trainCnt > 500)
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
