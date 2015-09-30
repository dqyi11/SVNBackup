using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace NeuralNetwork
{
    class MLP
    {
        public enum TrainType{ TRAIN_ONLY_OUTPUT_LAYER, NORMAL_BP, };
        double _learningRate;
        int[] _layerNum;
        List<double[]> _layers;
        List<double[,]> _weights;
        List<double[]> _biases;

        List<double[,]> _deltaWeights;
        List<double[]> _deltaBiases;

        Random _rnd;
        int _layerCnt;

        double[][] _layerNet;

        double[] _outputTarget;

        double [][] _layerError;
        double [][] _layerTemp;

        bool _trainOnlyOutputLayer;

        public TrainType _trainType
        {
            set
            {
                if (value == TrainType.NORMAL_BP)
                {
                    _trainOnlyOutputLayer = false;
                }
                else if (value == TrainType.TRAIN_ONLY_OUTPUT_LAYER)
                {
                    _trainOnlyOutputLayer = true;
                }
            }

            get
            {
                if (_trainOnlyOutputLayer)
                {
                    return TrainType.TRAIN_ONLY_OUTPUT_LAYER;
                }
                else
                {
                    return TrainType.NORMAL_BP;
                }
            }

        }

        public MLP(int[] layerNum, double learningRate)
        {
            _trainOnlyOutputLayer = false;
            _layers = new List<double[]>();
            _weights = new List<double[,]>();
            _deltaWeights = new List<double[,]>();
            _biases = new List<double[]>();
            _deltaBiases = new List<double[]>();
            _rnd = new Random();

            _learningRate = learningRate;
            _layerNum = layerNum;
            _layerCnt = layerNum.Length;

            _layerNet = new double[_layerCnt][];

            _layerError = new double[_layerCnt][];
            _layerTemp = new double[_layerCnt][];

            for (int i = 0; i < _layerNum.Length; i++)
            {
                _layers.Add(new double[_layerNum[i]]);
                _layerNet[i] = new double[_layerNum[i]];
                _layerError[i] = new double[_layerNum[i]];
                _layerTemp[i] = new double[_layerNum[i]];
            }

            for (int i = 0; i < _layerNum.Length - 1; i++)
            {
                _weights.Add(new double[_layerNum[i], _layerNum[i + 1]]);
                _deltaWeights.Add(new double[_layerNum[i], _layerNum[i + 1]]);
                _biases.Add(new double[_layerNum[i + 1]]);
                _deltaBiases.Add(new double[_layerNum[i + 1]]);
                
            }

            for (int i = 0; i < _layerNum.Length - 1; i++)
            {
                for (int j = 0; j < _layerNum[i+1]; j++)
                {
                    for (int k = 0; k < _layerNum[i]; k++)
                    {
                        _weights[i][k, j] = _rnd.NextDouble() - 0.5;
                    }

                    _biases[i][j] = 0.0;
                }
            }

            _outputTarget = new double[_layerNum[_layerCnt-1]];
        }

        public MLP(List<double[]> layers, List<double[,]> weights, List<double[]> biases, double learningRate)
        {
            _trainOnlyOutputLayer = false;
            _layers = new List<double[]>();
            _weights = new List<double[,]>();
            _deltaWeights = new List<double[,]>();
            _biases = new List<double[]>();
            _deltaBiases = new List<double[]>();
            _rnd = new Random();

            _learningRate = learningRate ;
            _layerCnt = layers.Count;
            int[] _layerNum = new int[layers.Count];

            _layerNet = new double[_layerCnt][];
            _layerError = new double[_layerCnt][];
            _layerTemp = new double[_layerCnt][];

            for (int i = 0; i < layers.Count; i++)
            {
                _layerNum[i] = layers[i].Length;                
                _layers.Add((double [])layers[i].Clone());
                _layerNet[i] = new double[_layerNum[i]];
                _layerError[i] = new double[_layerNum[i]];
                _layerTemp[i] = new double[_layerNum[i]];
            }

            for (int i = 0; i < weights.Count; i++)
            {
                //_weights.Add((double [,])weights[i].Clone());
                double [,] weight = new double[_layerNum[i], _layerNum[i + 1]];
                _deltaWeights.Add(new double[_layerNum[i], _layerNum[i + 1]]);
                for (int j = 0; j < _layerNum[i]; j++)
                {
                    for (int k = 0; k < _layerNum[i + 1]; k++)
                    {
                        weight[j,k] = weights[i][j,k];
                    }
                }
                _weights.Add(weight);
            }

            for (int i = 0; i < biases.Count; i++)
            {
                //_biases.Add((double[])biases[i].Clone());
                double [] bias = new double[_layerNum[i+1]];
                _deltaBiases.Add(new double[_layerNum[i + 1]]);
                for(int j=0;j<_layerNum[i+1];j++)
                {
                    bias[j] = biases[i][j];
                }
                _biases.Add(bias);

            }

            _outputTarget = new double[_layerNum[_layerCnt - 1]];
        }

        public void SetInput(params double[] input)
        {
            for (int i = 0; i < input.Length; i++)
            {
                _layers[0][i] = input[i];
            }
        }

        public void SetTargetOutput(params double[] target)
        {
            for (int i = 0; i < target.Length; i++)
            {
                _outputTarget[i] = target[i];
            }
        }

        public double[] GetOutput()
        {
            return (double[])_layers[_layerCnt-1].Clone();
        }

        public double GetMaxOutputIndex()
        {
            int maxIndex = 0;
            double cmpVal = 0;
            for (int i = 0; i < _layers[_layerCnt - 1].Length; i++)
            {

                if (_layers[_layerCnt - 1][i] > cmpVal)
                {
                    maxIndex = i;
                    cmpVal = _layers[_layerCnt - 1][i];
                }

            }

            return (double)maxIndex;
        }

        public double[] GetIntOutput()
        {
            double[] output = new double[_layers[_layerCnt - 1].Length];

            int maxIndex = 0;
            double cmpVal = 0;
            for (int i = 0; i < _layers[_layerCnt - 1].Length; i++)
            {
                output[i] = 0;
                if (_layers[_layerCnt - 1][i] > cmpVal)
                {
                    maxIndex = i;
                }
            }

            output[maxIndex] = 1;

            return (double[])output.Clone();
        }

        public void Feedfowrad()
        {

            for (int l = 1; l < _layerCnt; l++)
            {
                for (int i = 0; i < _layerNet[l].Length; i++)
                {
                    _layerNet[l][i] = 0.0;
                    for (int j = 0; j < _layers[l - 1].Length; j++)
                    {
                        _layerNet[l][i] += _weights[l - 1][j, i] * _layers[l - 1][j];
                    }
                    _layerNet[l][i] += _biases[l-1][i];

                    _layers[l][i] = 1.0 / (1.0 + Math.Exp(-_layerNet[l][i]));
                }
            }
        }

        public void BackpropagateError()
        {
            // error output
            for (int i = 0; i < _layers[_layerCnt-1].Length; i++)
            {
                _layerError[_layerCnt-1][i] = _outputTarget[i] - _layers[_layerCnt - 1][i];
                _layerTemp[_layerCnt-1][i] = _layers[_layerCnt - 1][i] * (1 - _layers[_layerCnt - 1][i]) * _layerError[_layerCnt - 1][i];
            }

            for (int j = 0; j < _layers[_layerCnt - 1].Length; j++)
            {
                for (int i = 0; i < _layers[_layerCnt - 2].Length; i++)
                {
                    _deltaWeights[_layerCnt - 2][i, j] += _layerTemp[_layerCnt - 1][j] * _layers[_layerCnt - 2][i];
                }
                _deltaBiases[_layerCnt - 2][j] += _layerTemp[_layerCnt - 1][j];
            }

            if(_trainOnlyOutputLayer)
            {
                return;
            }

            // error prop
            for (int l = _layerCnt - 2; l >= 1; l--)
            {
                for (int i = 0; i < _layers[l].Length; i++)
                {
                    double sumError = 0;
                    for (int j = 0; j < _layers[l+1].Length; j++)
                    {
                        sumError += _weights[l][i,j] * _layerTemp[l+1][j];
                    }

                    _layerTemp[l][i] = _layers[l][i] * (1.0 - _layers[l][i]) * sumError;
                }


                for (int j = 0; j < _layers[l].Length; j++)
                {
                    for (int i = 0; i < _layers[l - 1].Length; i++)
                    {
                        _deltaWeights[l-1][i, j] += _layerTemp[l][j] * _layers[l-1][i];
                    }
                    _deltaBiases[l-1][j] += _layerTemp[l][j];
                }
            }
           
        }

        public void Learn()
        {
            for (int l = 0; l < _layerCnt - 2; l++)
            {
                for (int i = 0; i < _layers[l].Length; i++)
                {
                    for (int j = 0; j < _layers[l + 1].Length; j++)
                    {
                        _weights[l][i, j] += _deltaWeights[l][i, j];
                    }
                }

                for (int i = 0; i < _layers[l + 1].Length; i++)
                {
                    _biases[l][i] += _deltaBiases[l][i];
                }
            }
        }

        public void ClearDeltaWeight()
        {
            for (int l = 0; l < _layerCnt - 2; l++)
            {
                for (int i = 0; i < _layers[l].Length; i++)
                {
                    for (int j = 0; j < _layers[l + 1].Length; j++)
                    {
                        _deltaWeights[l][i, j] = 0.0;
                    }
                }

                for (int i = 0; i < _layers[l + 1].Length; i++)
                {
                    _deltaBiases[l][i] = 0.0;
                }
            }
        }

        public void BatchTrain(double[][] inputPattern, double[][] targetOutputPattern)
        {
            if (inputPattern == null || targetOutputPattern == null)
            {
                throw new InvalidOperationException("Input set is null");
            }

            ClearDeltaWeight();

            for (int i = 0; i < inputPattern.Length; i++)
            {
                SetInput(inputPattern[i]);
                SetTargetOutput(targetOutputPattern[i]);
                Feedfowrad();
                BackpropagateError();
            }

            Learn();

        }

        public void OnlineTrain(double[] inputPattern, double[] expOutputPattern)
        {
            ClearDeltaWeight();

            SetInput(inputPattern);
            SetTargetOutput(expOutputPattern);
            Feedfowrad();
            BackpropagateError();

            Learn();
        }

        public double[] Run(params double[] input)
        {
            SetInput(input);
            Feedfowrad();

            return GetOutput();
        }

        public void DumpToFile()
        {
            string filepath = "MLP-PARAM-";
            filepath += DateTime.Now.Ticks + ".txt";
            StreamWriter writer = new StreamWriter(filepath);
            writer.AutoFlush = true;

            for (int l = 0; l < _layerCnt - 1; l++)
            {
                writer.WriteLine("Layer Num: " + l);
                writer.WriteLine("WEIGHTS " + _layers[l].Length + " * " + _layers[l+1].Length + " : ");

                for (int i = 0; i < _layers[l].Length; i++)
                {
                    for (int j = 0; j < _layers[l+1].Length; j++)
                    {
                        writer.Write(_weights[l][i, j] + ", ");
                    }
                    writer.Write("\n");
                }
                writer.Write("\n");

                writer.WriteLine("BIAS " + _layers[l+1].Length + " : ");
                for (int j = 0; j < _layers[l+1].Length; j++)
                {
                    writer.Write(_biases[l][j] + ", ");
                }
                writer.Write("\n");
                writer.Write("\n");
            }

            writer.Flush();
            writer.Close();

        }

    }
}
