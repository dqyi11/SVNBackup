using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class TwoDimDataTester
    {
        TwoDimDataGenerator _dataGnr;
        KernelRegression _regressionMachine;

        int _trainDataSize = 100;
        int _testDataSize = 1000;

        double[][] _trainDataX;
        double[] _trainDataY;

        Random _rnd;

        public TwoDimDataTester(KernelRegression machine = null)
        {
            _dataGnr = new TwoDimDataGenerator();

            _trainDataX = new double[_trainDataSize][];
            _trainDataY = new double[_trainDataSize];

            _rnd = new Random();

            for (int i = 0; i < _trainDataSize; i++)
            {
                _trainDataX[i] = new double[2];
                _trainDataX[i][0] = 2 * _rnd.NextDouble() - 1.0;
                _trainDataX[i][1] = 2 * _rnd.NextDouble() - 1.0;
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
            double[][] testDataX = new double[_testDataSize*_testDataSize][];
            double[] testDataY = new double[_testDataSize*_testDataSize];
            double[] testDataRefY = new double[_testDataSize*_testDataSize];

            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _testDataSize; j++)
                {
                    testDataX[j + i * _testDataSize] = new double[2];
                    testDataX[j + i * _testDataSize][0] = -1.0 + (double)((2 * i)) / (double)(_testDataSize);
                    testDataX[j + i * _testDataSize][1] = -1.0 + (double)((2 * j)) / (double)(_testDataSize);

                }
            }

            testDataY = _dataGnr.Calc(testDataX);
            testDataRefY = _regressionMachine.Predict(testDataX);

            DataAnalyzer analyzer = new DataAnalyzer();
            double[] err = analyzer.GetDifference(testDataY, testDataRefY);

            double mean = analyzer.Mean(err);
            double variance = analyzer.Variance(err);

            Console.WriteLine("testDataX1:");
            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _testDataSize; j++)
                {
                    Console.Write(testDataX[i * _testDataSize + j][0] + ",");
                }
                Console.Write("\n");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("testDataX2:");
            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _testDataSize; j++)
                {
                    Console.Write(testDataX[i * _testDataSize + j][1] + ",");
                }
                Console.Write("\n");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("testDataY:");
            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _testDataSize; j++)
                {
                    Console.Write(testDataY[i * _testDataSize + j] + ",");
                }
                Console.Write("\n");
            }
            Console.Write("\n");

            Console.WriteLine("testDataRefY:");
            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _testDataSize; j++)
                {
                    Console.Write(testDataRefY[i * _testDataSize + j] + ",");
                }
                Console.Write("\n");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("testDataError:");
            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _testDataSize; j++)
                {
                    Console.Write(err[i * _testDataSize + j] + ",");
                }
                Console.Write("\n");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("Mean: " + mean);
            Console.WriteLine("Variance: " + variance);


        }
    }
}
