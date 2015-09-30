using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;

namespace NeuralNetwork
{
    class LRBMTrainer
    {
        MnistDataMgr _dataMgr;
        LRBM _lrbm;

        const int _trainSetSize = 100;
        const int _batchSize = 1000;
        const int _batchTime = 60000;
        const int _epochTime = 100;
        Random _rnd;

        public LRBMTrainer(MnistDataMgr dataMgr, LRBM lrbm)
        {
            _dataMgr = dataMgr;
            _lrbm = lrbm;

            _rnd = new Random();
        }

        public void OnlineTrain()
        {
            int layerNum = _lrbm.layerCnt;
            for (int l = 0; l < layerNum; l++)
            {
                for (int i = 0; i < _trainSetSize; i++)
                {
                    int index = _rnd.Next(_dataMgr.count - 1);

                    double[] input = _dataMgr.GetNormalizedInputData(index);

                    if (l == 0)
                    {
                        _lrbm._rbms[l].OnlineTrain(input);
                    }
                    else
                    {
                        input = _lrbm.Feedforward(l - 1, input);
                        _lrbm._rbms[l].OnlineTrain(input);
                    }

                    Console.WriteLine("Layer: " + l + " Index:" + index
                        + " Error:" + _lrbm._rbms[l].GetMSE() + " Energy:" + _lrbm._rbms[l].GetFreeEnergy());

                }
            }
        }

        public void BatchTrain()
        {
            int layerNum = _lrbm.layerCnt;

            int runTime = 10;
            //int runTime = _dataMgr.count / _batchSize;
            for (int l = 0; l < layerNum; l++)
            {
                for (int i = 0; i < _epochTime; i++)
                {

                    for (int j = 0; j < runTime; j++)
                    {
                        double[][] inputs = new double[_batchSize][];
                        for (int k = 0; k < _batchSize; k++)
                        {
                            int index = j * _batchSize + k;
                            //int index = _rnd.Next(_dataMgr.count - 1);
                            double[] input = _dataMgr.GetNormalizedInputData(index);
                            if (l == 0)
                            {
                                inputs[k] = new double[_dataMgr.inputNum];
                                inputs[k] = input;
                            }
                            else
                            {
                                inputs[k] = new double[_lrbm.GetHiddenLayerSize(l - 1)];
                                inputs[k] = _lrbm.Feedforward(l - 1, input);
                            }
                        }

                        _lrbm._rbms[l].BatchTrain(inputs);

                        Console.WriteLine("Layer: " + l + " batch:" + i
                            + " Error:" + _lrbm._rbms[l].GetMSE() + " Energy:" + _lrbm._rbms[l].GetFreeEnergy());
                    }
                }
            }
        }
    }
}
