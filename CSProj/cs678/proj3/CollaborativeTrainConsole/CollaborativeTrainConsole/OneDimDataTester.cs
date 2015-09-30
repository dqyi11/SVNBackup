using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class OneDimDataTester
    {
        OneDimDataGenerator _dataGnr;

        int _trainDataSize = 100;
        int _testDataSize = 1000;

        double[][] _trainDataX;
        double[] _trainDataY;

        Random _rnd;

        KernelRegression _regressionMachine;

        public OneDimDataTester(KernelRegression machine = null)
        {
            _dataGnr = new OneDimDataGenerator();
            _rnd = new Random();

            _trainDataX = new double[_trainDataSize][];
            _trainDataY = new double[_trainDataSize];

            for (int i = 0; i < _trainDataSize; i++)
            {
                _trainDataX[i] = new double[1];
                _trainDataX[i][0] = 2 * _rnd.NextDouble() - 1.0;
            }

            _trainDataY = _dataGnr.Calc(_trainDataX);

            if (machine == null)
            {
                _regressionMachine = new KernelRegression();
            }
            else
            {
                _regressionMachine = machine;
            }
        }

        public void Train()
        {
            _regressionMachine._lamda = 0.8;

            _regressionMachine.Train(_trainDataX, _trainDataY);

        }

        public void Test()
        {
            double[][] testDataX = new double[_testDataSize][];
            double[] testDataY = new double[_testDataSize];
            double[] testDataRefY = new double[_testDataSize];

            for (int i = 0; i < _testDataSize; i++)
            {
                testDataX[i] = new double[1];
                testDataX[i][0] = - 1.0 + (double)((2 * i)) / (double)(_testDataSize);
            }

            testDataY = _dataGnr.Calc(testDataX);
            testDataRefY = _regressionMachine.Predict(testDataX);

            DataAnalyzer analyzer = new DataAnalyzer();
            double[] err = analyzer.GetDifference(testDataY, testDataRefY);

            double mean = analyzer.Mean(err);
            double variance = analyzer.Variance(err);

            Console.WriteLine("testDataX:");
            for (int i = 0; i < testDataX.Length; i++)
            {
                Console.Write(testDataX[i][0] + ",");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("testDataY:");
            for (int i = 0; i < testDataY.Length; i++)
            {
                Console.Write(testDataY[i] + ",");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("testDataRefY:");
            for (int i = 0; i < testDataRefY.Length; i++)
            {
                Console.Write(testDataRefY[i] + ",");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("testDataError:");
            for (int i = 0; i < err.Length; i++)
            {
                Console.Write(err[i] + ",");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("Mean: " + mean);
            Console.WriteLine("Variance: " + variance);

        }


    }
}
