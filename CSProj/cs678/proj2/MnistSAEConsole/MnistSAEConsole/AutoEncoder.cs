using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace NeuralNetwork
{
    class AutoEncoder
    {
        double _learningRate;
        int _visibleNbr;
        int _hiddenNbr;

        MLP _mlp;

        public AutoEncoder(double learningRate, int visibleNbr, int hiddenNbr)
        {
            _learningRate = learningRate;
            _visibleNbr = visibleNbr;
            _hiddenNbr = hiddenNbr;
            Init();
        }

        void Init()
        {
            int [] netStruct = new int[] {_visibleNbr, _hiddenNbr, _visibleNbr};
            _mlp = new MLP(netStruct, _learningRate);
        }

        public MLP GetMLP()
        {
            return _mlp;
        }




    }
}
