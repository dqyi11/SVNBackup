using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using HouseData;

namespace KernelMachine
{
    class KernelRegressionTester
    {
        HouseDataMgr _dataMgr;
        double[][] _input;
        double[] _output;
        KernelRegression _regressionMachine;

        Random _rnd;

        public KernelRegressionTester(HouseDataMgr dataMgr, KernelRegression machine = null)
        {
            _dataMgr = dataMgr;

            _input = new double[_dataMgr.count][];
            _output = new double[_dataMgr.count];

            for (int i = 0; i < _dataMgr.count; i++)
            {
                _input[i] = _dataMgr.GetInputData(i);
                _output[i] = _dataMgr.GetLabelData(i);
            }

            if (null == machine)
            {
                _regressionMachine = new KernelRegression();
            }
            else
            {
                _regressionMachine = machine;
            }

            _rnd = new Random();
        }

        public void Train()
        {
            _regressionMachine._lamda = 0.8;

            _regressionMachine.Train(_input, _output);

        }

        public void Test()
        {
            int testSize = 10;
            for (int i = 0; i < testSize; i++)
            {
                int index = _rnd.Next(_dataMgr.count-1);
                double[] testInput = _dataMgr.GetInputData(index);
                double testOutput = _dataMgr.GetLabelData(index);

                double predictOuput = _regressionMachine.Predict(testInput);

                Console.WriteLine("Test Index:" + index + ", Expected:" + testOutput + " Predict:" + predictOuput);
            }

            double mse = 0.0;
            for (int i = 0; i < _output.Length; i++)
            {
                double[] testInput = _dataMgr.GetInputData(i);
                double testOutput = _dataMgr.GetLabelData(i);
                double predictOuput = _regressionMachine.Predict(testInput);

                mse += (testOutput - predictOuput) * (testOutput - predictOuput);
            }

            mse = Math.Sqrt(mse) / _output.Length;

            Console.WriteLine("MSE:" + mse);
        }

    }
}
