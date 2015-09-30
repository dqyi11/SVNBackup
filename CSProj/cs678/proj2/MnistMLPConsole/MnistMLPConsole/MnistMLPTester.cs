using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;

namespace NeuralNetwork
{
    class MnistMLPTester
    {
        MLP _mlp;
        MnistDataMgr _dataMgr;

        int _testSetSize;

        double _mse;
        double _successRate;

        public MnistMLPTester(MLP mlp, MnistDataMgr dataMgr)
        {
            _mlp = mlp;
            _dataMgr = dataMgr;

            _testSetSize = _dataMgr.count;

            _mse = 0;
            _successRate = 0;
        }

        public void Test()
        {
            int successTime = 0;
            double mse = 0;

            for (int i = 0; i < _testSetSize; i++)
            {
                double labelData = _dataMgr.GetDoubleLabelData(i);
                double [] inputData = _dataMgr.GetNormalizedInputData(i);
                double[] labelData2 = _dataMgr.GetIntLabelData(i);

                _mlp.SetInput(inputData);
                _mlp.Feedfowrad();

                double maxIndex = _mlp.GetMaxOutputIndex();

                if (labelData == maxIndex)
                {
                    successTime++;
                }

                double[] outputData = _mlp.GetIntOutput();

                mse += GetMSE(outputData, labelData2);     
            }

            _mse = mse / (double)_testSetSize;
            _successRate = (double)successTime / (double)_testSetSize;

            Console.WriteLine("Test set - success rate : " + _successRate + " - mse : " + _mse);
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
