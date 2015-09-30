using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace NeuralNetwork
{
    [Serializable()]
    class RBM : ISerializable
    {
        public double[,] _visible;
        public double[,] _hidden;
        public double[,] _weightVisibleHidden;

        public double[] _visibleBias;
        public double[] _hiddenBias;

        public double[,] _visibleProb;
        public double[,] _hiddenProb;

        public double[] _visibleBiasDelta;
        public double[] _hiddenBiasDelta;
        public double[,] _weightVisibleHiddenDelta;

        public double[] _visibleError;

        int _visibleNbr;
        int _hiddenNbr;

        int _relaxStep;
        double _learningRate;

        Random _rnd;

        double _mse;

        public int _index;

        //Serialization function.
        public void GetObjectData(SerializationInfo info, StreamingContext ctxt)
        {
            //You can use any custom name for your name-value pair. But make sure you
            // read the values with the same name. For ex:- If you write EmpId as "EmployeeId"
            // then you should read the same with "EmployeeId"
            info.AddValue("VisibleNbr", _visibleNbr);
            info.AddValue("HiddenNbr", _hiddenNbr);
            info.AddValue("RelaxStep", _relaxStep);
            info.AddValue("LearningRate", _learningRate);

            info.AddValue("Visible", _visible);
            info.AddValue("Hidden", _hidden);
            info.AddValue("WeightVisibleHidden", _weightVisibleHidden);
            info.AddValue("VisibleBias", _visibleBias);
            info.AddValue("HiddenBias", _hiddenBias);

            info.AddValue("VisibleProb", _visibleProb);
            info.AddValue("HiddenProb", _hiddenProb);

            info.AddValue("Index", _index);
           

        }

        public void DumpParamsToFile(string filename)
        {
            Stream stream = File.Open(filename, FileMode.Create);
            BinaryFormatter bformatter = new BinaryFormatter();

            Console.WriteLine("Writing RBM Params Information");
            bformatter.Serialize(stream, this);
            stream.Close();
        }

        public int visibleNbr
        {
            get
            {
                return _visibleNbr;
            }
        }

        public int hiddenNbr
        {
            get
            {
                return _hiddenNbr;
            }
        }

        public RBM(SerializationInfo info, StreamingContext ctxt)
        {
            _relaxStep = (int)info.GetValue("RelaxStep", typeof(int));
            _learningRate = (double)info.GetValue("LearningRate", typeof(double));
            _index = (int)info.GetValue("Index", typeof(int));

            _hiddenNbr = (int)info.GetValue("HiddenNbr", typeof(int));
            _visibleNbr = (int)info.GetValue("VisibleNbr", typeof(int));

            _visible = (double[,])info.GetValue("Visible", typeof(double[,]));
            _hidden = (double[,])info.GetValue("Hidden", typeof(double[,]));
            _weightVisibleHidden = (double[,])info.GetValue("WeightVisibleHidden", typeof(double[,]));

            _visibleBias = (double[])info.GetValue("VisibleBias", typeof(double[]));
            _hiddenBias = (double[])info.GetValue("HiddenBias", typeof(double[]));

            _visibleError = new double[visibleNbr];

            _visibleProb = new double[visibleNbr, _relaxStep + 1];
            _hiddenProb = new double[hiddenNbr, _relaxStep + 1];

            _visibleBiasDelta = new double[visibleNbr];
            _hiddenBiasDelta = new double[hiddenNbr];

            _rnd = new Random();

        }
        
        public RBM(double learningRate, int visibleNbr, int hiddenNbr, int relaxStep=1)
        {
            _relaxStep = relaxStep;
            _learningRate = learningRate;

            _visibleNbr = visibleNbr;
            _hiddenNbr = hiddenNbr;

            _visible = new double[visibleNbr, relaxStep+1];
            _hidden = new double[hiddenNbr, relaxStep+1];

            _visibleBias = new double[visibleNbr];
            _hiddenBias = new double[hiddenNbr];

            _visibleError = new double[visibleNbr];

            _visibleProb = new double[visibleNbr, relaxStep+1];
            _hiddenProb = new double[hiddenNbr, relaxStep+1];

            _visibleBiasDelta = new double[visibleNbr];
            _hiddenBiasDelta = new double[hiddenNbr];

            _weightVisibleHidden = new double[visibleNbr, hiddenNbr];

            _weightVisibleHiddenDelta = new double[visibleNbr, hiddenNbr];

            _rnd = new Random();

            _index = 0;

            Init();

        }

        public double[,] GetWeightVisibleHidden()
        {
            return (double[,])_weightVisibleHidden.Clone();
        }

        public double[] GetHiddenBias()
        {
            return (double[])_hiddenBias.Clone();
        }

        public double[] GetVisibleBias()
        {
            return (double[])_visibleBias.Clone();
        }

        void clearWeightDelat()
        {
            for (int i = 0; i < _visibleNbr; i++)
            {
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    _weightVisibleHiddenDelta[i, j] = 0;
                }
                _visibleBiasDelta[i] = 0;
            }
            for (int j = 0; j < _hiddenNbr; j++)
            {
                _hiddenBiasDelta[j] = 0;
            }
        }

        public RBM(RBM rmb)
        {
            _visible = rmb._visible;
            _hidden = rmb._hidden;
            _weightVisibleHidden = rmb._weightVisibleHidden;
            _visibleBias = rmb._visibleBias;
            _hiddenBias = rmb._hiddenBias;
        }

        public void Init()
        {
            
            for (int i = 0; i < _visibleNbr; i++)
            {
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    _weightVisibleHidden[i, j] = (_rnd.NextDouble() - 0.5);
                }

                _visibleBias[i] = 0;
            }

            for (int j = 0; j < _hiddenNbr; j++)
            {
                _hiddenBias[j] = 0;
            }
        }

        public void SetInput(double[] input)
        {
            for (int i = 0; i < _visibleNbr; i++)
            {
                _visible[i,0] = input[i];
            }
        }

        public double[] GetVisible()
        {
            double [] visibleOut = new double[_visibleNbr];
            for(int i=0;i<_visibleNbr;i++)
            {
                visibleOut[i] = _visible[i,_relaxStep];
            }
            return (double [])visibleOut.Clone() ;
        }

        public double[] GetVisibleProb()
        {
            double[] visibleProbOut = new double[_visibleNbr];
            for (int i = 0; i < _visibleNbr; i++)
            {
                visibleProbOut[i] = _visibleProb[i, _relaxStep];
            }
            return (double[])visibleProbOut.Clone();
        }

        public double[] Feedforward(double[] input)
        {
            double[] output = new double[_hiddenNbr];

            for (int i = 0; i < _hiddenNbr; i++)
            {
                double activation = 0;
                for (int j = 0; j < _visibleNbr; j++)
                {
                    activation += _weightVisibleHidden[j, i] * input[j];
                }
                activation += _hiddenBias[i];
                output[i] = 1.0 / (1 + Math.Exp(-activation));
            }

            return (double [])output.Clone();
        }

        public void UpdateHidden(int k, bool withSampling = true)
        {
            for (int i = 0; i < _hiddenNbr; i++)
            {
                double hiddenNet = 0.0;
                for (int j = 0; j < _visibleNbr; j++)
                {
                    hiddenNet += _weightVisibleHidden[j, i] * _visible[j,k];                 
                }
                hiddenNet += _hiddenBias[i];

                _hiddenProb[i,k] = 1/(1+Math.Exp(-hiddenNet));

                if (withSampling)
                {
                    _hidden[i, k] = GenerateRndByProbability(_hiddenProb[i, k]);
                }
                else
                {
                    _hidden[i, k] = _hiddenProb[i, k];
                }

                //Console.WriteLine("hidden: "+ _hidden[i,k] + " with prob " + _hiddenProb[i,k]);
            }
        }

        public void UpdateInput(int k, bool withSampling = true)
        {
            for (int i = 0; i < _visibleNbr; i++)
            {
                double inputNet = 0.0;
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    inputNet += _weightVisibleHidden[i,j] * _hidden[j,k-1];
                }
                inputNet += _visibleBias[i];

                _visibleProb[i,k] = 1 / (1 + Math.Exp(-inputNet));

                if (withSampling)
                {
                    _visible[i, k] = GenerateRndByProbability(_visibleProb[i, k]);
                }
                else
                {
                    _visible[i, k] = _visibleProb[i, k];
                }
            }
        }

        public double GenerateRndByProbability(double prob)
        {
            if (_rnd.NextDouble() > prob)
            {
                return 0.0;
            }
            return 1.0;
        }

        public void BatchTrain(double[][] inputs)
        {
            int batchSize = inputs.Length;

            clearWeightDelat();

            _mse = 0.0;

            for (int b = 0; b < batchSize; b++)
            {
                double[] input = inputs[b];
                SetInput(input);
                int k = 0;
                while(k<_relaxStep)
                {
                    UpdateHidden(k);
                    k++;
                    UpdateInput(k);
                }
                UpdateHidden(k);

                for (int i = 0; i < _visibleNbr; i++)
                {
                    for (int j = 0; j < _hiddenNbr; j++)
                    {
                        _weightVisibleHiddenDelta[i, j] += _learningRate * (_hidden[j, 0] * _visible[i, 0] - _hiddenProb[j, _relaxStep] * _visible[i, _relaxStep]);
                    }

                    _visibleError[i] = _visible[i, 0] - _visible[i, _relaxStep];
                    _visibleBiasDelta[i] += _learningRate * _visibleError[i];
                }

                for (int j = 0; j < _hiddenNbr; j++)
                {
                    _hiddenBiasDelta[j] += _learningRate * (_hidden[j, 0] - _hiddenProb[j, _relaxStep]);
                }

                double error = 0.0; 
                for (int i = 0; i < _visibleNbr; i++)
                {
                    error += _visibleError[i] * _visibleError[i];
                }
                error = Math.Sqrt(error / _visibleError.Length);
                _mse += error;
            }

            _mse = _mse / batchSize;

            // update weights
            for (int i = 0; i < _visibleNbr; i++)
            {
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    _weightVisibleHidden[i, j] += _weightVisibleHiddenDelta[i, j]/batchSize;
                }
                _visibleBias[i] += _visibleBiasDelta[i]/batchSize;
            }

            for (int j = 0; j < _hiddenNbr; j++)
            {
                _hiddenBias[j] += _hiddenBiasDelta[j]/batchSize;
            }
        }

        public void Calc(double[] input)
        {
            SetInput(input);
            /*
            int k = 0;
            while (k < _relaxStep)
            {
                UpdateHidden(k, false);
                k++;
                UpdateInput(k, false);
            }
            UpdateHidden(k, false);
             */
            //UpdateHidden(0, false);
            int k = 0;
            while (k < _relaxStep)
            {
                UpdateHidden(k, false);
                k++;
                UpdateInput(k, false);
            }
        }

        public void OnlineTrain(double[] input)
        {
            SetInput(input);
            int k = 0;
            while(k<_relaxStep)
            {
                UpdateHidden(k);
                k++;
                UpdateInput(k);
            }
            UpdateHidden(k);

            for(int i = 0;i < _visibleNbr; i++)
            {
                for(int j=0;j < _hiddenNbr; j++)
                {
                    _weightVisibleHiddenDelta[i,j] = _learningRate * (_hidden[j,0]*_visible[i,0]-_hiddenProb[j,_relaxStep]*_visible[i,_relaxStep]);
                }

                _visibleError[i] = _visible[i, 0] - _visible[i, _relaxStep];
                _visibleBiasDelta[i] = _learningRate * _visibleError[i];

            }

            for (int j = 0; j < _hiddenNbr; j++)
            {
                _hiddenBiasDelta[j] = _learningRate * (_hidden[j, 0] - _hiddenProb[j, _relaxStep]);
            }

            // update weights
            for (int i = 0; i < _visibleNbr; i++)
            {
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    _weightVisibleHidden[i, j] += _weightVisibleHiddenDelta[i, j];
                }
                _visibleBias[i] += _visibleBiasDelta[i];
            }

            for (int j = 0; j < _hiddenNbr; j++)
            {
                _hiddenBias[j] += _hiddenBiasDelta[j];
            }

            _mse = 0.0;
            for (int i = 0; i < _visibleNbr; i++)
            {
                _mse += _visibleError[i] * _visibleError[i];
            }
            _mse = Math.Sqrt(_mse / _visibleError.Length);

        }

        public double GetMSE()
        {
            return _mse;
        }

        public double GetFreeEnergy()
        {
            double energy = 0;
            for (int i = 0; i < _visibleNbr; i++)
            {
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    energy += _weightVisibleHidden[i, j] * _visibleBias[i] * _hiddenBias[j];
                }
            }
            return -energy;
        }

        public void DumpToFile()
        {
            string filepath = "RBM-" + _index + "-";
            filepath += DateTime.Now.Ticks + ".txt";
            StreamWriter writer = new StreamWriter(filepath);
            writer.AutoFlush = true;

            writer.WriteLine("WEIGHTS " + _visibleNbr + " * " + _hiddenNbr + " :");
            for (int i = 0; i < _visibleNbr; i++)
            {
                for (int j = 0; j < _hiddenNbr; j++)
                {
                    writer.Write(_weightVisibleHidden[i, j] + ",");
                }
                writer.Write("\n");
            }

            writer.Write("\n");
            writer.WriteLine("VISIBLE BIAS " + _visibleNbr + " : ");
            for (int i = 0; i < _visibleNbr; i++)
            {
                writer.Write(_visibleBias[i] + ", ");
            }

            writer.Write("\n");
            writer.WriteLine("HIDDEN BIAS " + _hiddenNbr + " : ");
            for (int j = 0; j < _hiddenNbr; j++)
            {
                writer.Write(_hiddenBias[j] + ", ");
            }

            writer.Close();
        }
    }
}
