using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class DistributedOneDimDataTrainer
    {
        int _agentNum;
        int[,] _adjacency;

        List<KernelAgent> _agents;

        int _trainDataSize = 100;
        int _testDataSize = 1000;

        int _iterationTime = 100;

        double[][] _trainDataX;
        double[] _trainDataY;

        double _commDist = 0.5; //0.2;

        OneDimDataGenerator _dataGnr;
        Random _rnd;

        KernelRegression _centerMachine;

        public DistributedOneDimDataTrainer()
        {
            _agentNum = _trainDataSize;

            _adjacency = new int[_agentNum, _agentNum];
            _dataGnr = new OneDimDataGenerator();
            _rnd = new Random();

            _trainDataX = new double[_trainDataSize][];
            _trainDataY = new double[_trainDataSize];

            _agents = new List<KernelAgent>();

            _centerMachine = new KernelRegression();

            for (int i = 0; i < _agentNum; i++)
            {
                _trainDataX[i] = new double[1];
                _trainDataX[i][0] = 2 * _rnd.NextDouble() - 1.0;

                _agents.Add(new KernelAgent(_trainDataX[i]));
            }

            _trainDataY = _dataGnr.Calc(_trainDataX);

            Init();
        }

        void Connect(int a, int b)
        {
            _adjacency[a, b] = 1;
            _adjacency[b, a] = 1;
        }

        void Init()
        {
            for (int i = 0; i < _agentNum; i++)
            {
                for (int j = 0; j < _agentNum; j++)
                {
                    if (_agents[i].GetDistance(_agents[j].GetPos()) <= _commDist)
                    {
                        Connect(i, j);
                    }
                }
            }

        }

        public void PrintAdjacency()
        {
            Console.WriteLine("Adjacent Matrix:");
            for (int i=0; i < _agentNum; i++)
            {
                for (int j=0; j < _agentNum; j++)
                {
                    Console.Write(_adjacency[i,j] + " ");
                }
                Console.Write("\n");
            }
            Console.Write("\n");
        }

        public void TrainCenterMachine()
        {
            _centerMachine._lamda = 0.8;

            //_centerMachine.Train(_trainDataX, _trainDataY);
            _centerMachine.IterativeTrain(_trainDataX, _trainDataY);

        }

        public void TrainDistributedMachines()
        {
            // set parameters
            for (int i = 0; i < _agents.Count; i++)
            {
                _agents[i].GetMachine()._lamda = 0.8;
            }

            for (int i = 0; i < _agentNum; i++)
            {
                for (int j = 0; j < _agentNum; j++)
                {
                    if (_adjacency[i, j] == 1)
                    {
                        _agents[i].AddNeighbor(_agents[j]);                       
                    }
                }
            }

            for (int t = 0; t < _iterationTime; t++)
            {
                for (int i = 0; i < _agentNum; i++)
                {
                    _agents[i].SetValue(_dataGnr.Calc(_agents[i].GetPos()));
                }

                for (int i = 0; i < _agentNum; i++)
                {
                    _agents[i].Train();
                }                
            }
        }

        int GetNeighborSize(int i)
        {
            int neighborSize = 0;
            for (int j = 0; j < _agentNum; j++)
            {
                if (_adjacency[i, j] == 1)
                {
                    neighborSize++;
                }
            }
            return neighborSize;
        }

        public void TestCenterMachine()
        {
            Console.Write("\n");
            Console.Write("\n");
            Console.WriteLine("TestCenterMachine");

            double[][] testDataX = new double[_testDataSize][];
            double[] testDataY = new double[_testDataSize];
            double[] testDataRefY = new double[_testDataSize];

            for (int i = 0; i < _testDataSize; i++)
            {
                testDataX[i] = new double[1];
                testDataX[i][0] = -1.0 + (double)((2 * i)) / (double)(_testDataSize);
            }

            testDataY = _dataGnr.Calc(testDataX);
            testDataRefY = _centerMachine.Predict(testDataX);

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

        public void TestDistributedMachines()
        {
            Console.Write("\n");
            Console.Write("\n");
            Console.WriteLine("TestDistributedMachines");

            double[][] testDataX = new double[_testDataSize][];
            double[] testDataY = new double[_testDataSize];
            double[] testDataRefY = new double[_testDataSize];

            for (int i = 0; i < _testDataSize; i++)
            {
                testDataX[i] = new double[1];
                testDataX[i][0] = -1.0 + (double)((2 * i)) / (double)(_testDataSize);
            }

            testDataY = _dataGnr.Calc(testDataX);

            int totalConnectivity = 0;
            for (int j = 0; j < _agentNum; j++)
            {
                totalConnectivity += _agents[j].connectivity;
            }

            for (int j = 0; j < _agentNum; j++)
            {
                testDataRefY = _agents[j].GetMachine().Predict(testDataX);
                DataAnalyzer analyzer2 = new DataAnalyzer();
                double[] err2 = analyzer2.GetDifference(testDataY, testDataRefY);

                double mean2 = analyzer2.Mean(err2);
                double variance2 = analyzer2.Variance(err2);

                Console.WriteLine("Agent: " + j);
                Console.WriteLine("Mean: " + mean2);
                Console.WriteLine("Variance: " + variance2);
            }

            return;
            
            for (int i = 0; i < _testDataSize; i++)
            {
                for (int j = 0; j < _agentNum; j++)
                {
                    testDataRefY[i] += _agents[j].GetMachine().Predict(testDataX[i]) 
                        * _agents[j].connectivity;
                }

                testDataRefY[i] = testDataRefY[i] / totalConnectivity;
            }

            DataAnalyzer analyzer = new DataAnalyzer();
            double[] err = analyzer.GetDifference(testDataY, testDataRefY);

            double mean = analyzer.Mean(err);
            double variance = analyzer.Variance(err);

            Console.WriteLine("testDataRefY:");
            for (int i = 0; i < testDataRefY.Length; i++)
            {
                Console.Write(testDataRefY[i] + ",");
            }
            Console.Write("\n");
            Console.Write("\n");

            Console.WriteLine("Mean: " + mean);
            Console.WriteLine("Variance: " + variance);
        }
    }
}
