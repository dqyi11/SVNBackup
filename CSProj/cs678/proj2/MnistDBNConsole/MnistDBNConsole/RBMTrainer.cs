using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;


namespace NeuralNetwork
{
    class RBMTrainer 
    {
        MnistDataMgr _dataMgr;
        RBM _rbm;

        const int _trainSetSize = 100;
        const int _batchSize = 100;
        const int _batchTime = 60000;
        const int _epochTime = 10;
        Random _rnd;

        public RBMTrainer(MnistDataMgr dataMgr, RBM rbm)
        {
            _dataMgr = dataMgr;
            _rbm = rbm;

            _rnd = new Random();
        }

        public void OnlineTrain()
        {
            for (int i = 0; i < _trainSetSize; i++)
            {
                int index = _rnd.Next(_dataMgr.count - 1);

                double[] input = _dataMgr.GetNormalizedInputData(index);

                _rbm.OnlineTrain(input);

                Console.WriteLine("Index:" + index
                    + " Error:" + _rbm.GetMSE() + " Energy:" + _rbm.GetFreeEnergy());

            }
        }

        public void BatchTrain()
        {
            int runTime = _dataMgr.count / _batchSize;
            for (int i = 0; i < _epochTime; i++)
            {
                
                for (int j = 0; j < runTime; j++)
                {
                    double[][] inputs = new double[_trainSetSize][];
                    for (int k = 0; k < _batchSize; k++)
                    {
                        int index = j * _batchSize + k;
                        //int index = _rnd.Next(_dataMgr.count - 1);
                        double[] input = _dataMgr.GetNormalizedInputData(index);
                        inputs[k] = new double[_dataMgr.inputNum];
                        inputs[k] = input;
                    }
                    _rbm.BatchTrain(inputs);

                    Console.WriteLine(_rbm.GetFreeEnergy());
                    //Console.WriteLine("batch:" + j + " Error:" + _rbm.GetMSE());
                    /*
                    Console.WriteLine("batch:" + j
                    + " Error:" + _rbm.GetMSE() + " Energy:" + _rbm.GetFreeEnergy());
                     */
                }
            }
        }
    }
}
