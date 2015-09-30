using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace NeuralNetwork
{
    class MLPNetworkWithBias
    {
        double _learnRate;

        public double [] _input;
        public double [] _hidden;
        
        public double [] _output;
        public double [,] _weightInHidden;
        public double [,] _weightHiddenOut;

        public double [,] _deltaWeightInHidden;
        public double [,] _deltaWeightHiddenOut;

        public double[] _deltaHiddenBias;
        public double[] _deltaOutputBias;

        public double [] _hiddenNet;
        public double [] _outputNet;

        public double [] _outputTarget;

        public double [] _outputError;
        public double [] _hiddenError;

        public double [] _outputTemp;
        public double [] _hiddenTemp;

        StringBuilder _sb;
        StreamWriter _outfile;

        public double [] _hiddenBias;
        public double [] _outputBias;

        int _nrInput;
        int _nrHidden;
        int _nrOutput;

        bool _log;

        public MLPNetworkWithBias(int nrInput, int nrHidden, int nrOutput, double learnRate)
        {
            _learnRate = learnRate;
            _nrInput = nrInput;
            _nrHidden = nrHidden;
            _nrOutput = nrOutput;

            _input = new double[nrInput];
            _hidden = new double[nrHidden];
            _output = new double[nrOutput];

            _hiddenNet = new double[nrHidden];
            _outputNet = new double[nrOutput];

            _hiddenBias = new double[nrHidden];
            _outputBias = new double[nrOutput];

            _outputTarget = new double[nrOutput];
            _outputError = new double[nrOutput];
            _hiddenError = new double[nrHidden];

            _outputTemp = new double[nrOutput];
            _hiddenTemp = new double[nrHidden];

            _weightInHidden = new double[nrInput, nrHidden];
            _weightHiddenOut = new double[nrHidden, nrOutput];

            _deltaWeightInHidden = new double[nrInput, nrHidden];
            _deltaWeightHiddenOut = new double[nrHidden, nrOutput];

            _deltaHiddenBias = new double[nrHidden];
            _deltaOutputBias = new double[nrOutput];

            _log = false;

            Init();
        }

        public void Init()
        {
            Random rnd = new Random();
            for (int j = 0; j < _hidden.Length; j++)
            {
                for (int i = 0; i < _input.Length; i++)
                {
                    _weightInHidden[i, j] = (rnd.NextDouble() - 0.5);
                }

                _hiddenBias[j] = 0; // (rnd.NextDouble() - 0.5); 

            }
            for (int j = 0; j < _output.Length; j++)
            {
                for (int i = 0; i < _hidden.Length; i++)
                {
                    _weightHiddenOut[i, j] = (rnd.NextDouble() - 0.5);
                }
                _outputBias[j] = 0; // (rnd.NextDouble() - 0.5);
            }
            ClearDeltaWeight();
        }

        public void SetInput(params double [] input)
        {
            for (int i = 0; i < input.Length; i++)
            {
                _input[i] = input[i];
            }
        }

        public void SetTargetOutput(params double [] target)
        {
            for (int i = 0; i < target.Length; i++)
            {
                _outputTarget[i] = target[i];
            }
        }

        public double [] GetOutput()
        {
            return (double[])_output.Clone();
        }

        public double GetMaxOutputIndex()
        {
            int maxIndex = 0;
            double cmpVal = 0;
            for (int i = 0; i < _output.Length; i++)
            {

                if (_output[i] > cmpVal)
                {
                    maxIndex = i;
                    cmpVal = _output[i];
                }
           
            }

            return (double)maxIndex;
        }

        public double[] GetIntOutput()
        {
            double[] output = new double[_output.Length];

            int maxIndex = 0;
            double cmpVal = 0;
            for (int i = 0; i < _output.Length; i++)
            {
                output[i] = 0;
                if (_output[i] > cmpVal)
                {
                    maxIndex = i;
                }
            }

            output[maxIndex] = 1;

            return output;
        }

        public void Feedfowrad()
        {
            for (int i = 0; i < _hiddenNet.Length; i++)
            {
                _hiddenNet[i]=0;
                for (int j = 0; j < _input.Length; j++)
                {
                    _hiddenNet[i] += _input[j] * _weightInHidden[j,i];
                }

                _hiddenNet[i] += _hiddenBias[i];

                _hidden[i] = 1.0 / (1.0 + Math.Exp(-_hiddenNet[i])); 
            }

            for (int i = 0; i < _outputNet.Length; i++)
            {
                _outputNet[i] = 0;
                for (int j = 0; j < _hidden.Length; j++)
                {
                    _outputNet[i] += _hidden[j] * _weightHiddenOut[j,i];
                }

                _outputNet[i] += _outputBias[i];

                _output[i] = 1.0 / (1.0 + Math.Exp(-_outputNet[i]));
            }

        }

        public void BackpropagateError()
        {
            for (int i = 0; i < _outputNet.Length; i++)
            {
                _outputError[i] = _outputTarget[i] - _output[i];
                _outputTemp[i] = _output[i] * (1 - _output[i]) * _outputError[i];  
            }

            for (int j = 0; j < _outputTemp.Length; j++)
            {
                for (int i = 0; i < _hidden.Length; i++)
                {               
                    _deltaWeightHiddenOut[i, j] += _outputTemp[j] * _hidden[i];
                }

                _deltaOutputBias[j] += _outputTemp[j];
            }

            for (int i = 0; i < _hiddenTemp.Length; i++)
            {
                double sumError = 0;
                for (int j = 0; j < _outputTemp.Length; j++)
                {
                    sumError += _weightHiddenOut[i,j] * _outputTemp[j];
                }

                _hiddenTemp[i] = _hidden[i] * (1.0 - _hidden[i]) * sumError;
            }

            for (int j = 0; j < _hiddenTemp.Length; j++)
            {
                for (int i = 0; i < _input.Length; i++)
                {
                    _deltaWeightInHidden[i, j] += _hiddenTemp[j] * _input[i];
                }

                _deltaHiddenBias[j] += _hiddenTemp[j];
            }
        }

        public void Learn()
        {
            if (_log)
            {
                _sb.AppendLine("WEIGHT HIDDEN=>OUTPUT");
            }
            for (int j = 0; j < _outputTemp.Length; j++)
            {
                for (int i = 0; i < _hidden.Length; i++)
                {      
                    _weightHiddenOut[i, j] += _deltaWeightHiddenOut[i, j] * _learnRate;
                    if (_log)
                    {
                        _sb.Append(_weightHiddenOut[i, j]);
                        _sb.Append(" ");
                    }
                }
                _outputBias[j] += _deltaOutputBias[j] * _learnRate;


                if (_log)
                {
                    _sb.Append("\n");
                }
            }
            if (_log)
            {
                _sb.AppendLine("");
                _sb.AppendLine("WEIGHT INPUT=>HIDDEN");
            }
            for (int j = 0; j < _hiddenTemp.Length; j++)
            {
                for (int i = 0; i < _input.Length; i++)
                {
                    _weightInHidden[i, j] += _deltaWeightInHidden[i, j] * _learnRate;
                    if (_log)
                    {
                        _sb.Append(_weightInHidden[i, j]);
                        _sb.Append(" ");
                    }
                }

                _hiddenBias[j] += _deltaHiddenBias[j] * _learnRate ;

                if (_log)
                {
                    _sb.Append("\n");
                }
            }
        }

        public void ClearDeltaWeight()
        {
            for (int j = 0; j < _hidden.Length; j++)
            {
                for (int i = 0; i < _input.Length; i++)
                {
                    _deltaWeightInHidden[i, j] = 0;
                }

                _deltaHiddenBias[j] = 0;
            }

            for (int j = 0; j < _output.Length; j++)
            {
                for (int i = 0; i < _hidden.Length; i++)
                {                
                    _deltaWeightHiddenOut[i,j] = 0;
                }

                _deltaOutputBias[j] = 0;
            }
        }

        public void BatchTrain(double[][] inputPattern, double[][] targetOutputPattern)
        {
            if (inputPattern == null || targetOutputPattern == null)
            {            
                throw new InvalidOperationException("Input set is null");
            }

            ClearDeltaWeight();
            Console.WriteLine("In Batch ");
            for (int i = 0; i < inputPattern.Length; i++)
            {
                SetInput(inputPattern[i]);
                SetTargetOutput(targetOutputPattern[i]);
                Feedfowrad();
                BackpropagateError();

                // Console.Write(" " + i);
            }
            Console.Write("\n");

            Learn();

            if (_log)
            {
                _outfile.Write(_sb.ToString());
                _sb.Clear();
            }
        }

        public void OnlineTrain(double[] inputPattern, double[] expOutputPattern)
        {
            ClearDeltaWeight();

            SetInput(inputPattern);
            SetTargetOutput(expOutputPattern);
            Feedfowrad();
            BackpropagateError();
            
            Learn();

            if (_log)
            {
                _outfile.Write(_sb.ToString());
                _sb.Clear();
            }
        }

        public double[] Run(params double[] input)
        {
            SetInput(input);
            Feedfowrad();

            return GetOutput();
        }

        public void InitLog(string prefix)
        {
            _sb = new StringBuilder();
            string filename = prefix + "LOG" + DateTime.Now.ToFileTimeUtc() + ".log";
            _outfile = new StreamWriter(filename);

            _log = true;
        }

        public void DumpToFile()
        {
            string filepath = "MLP-PARAM-";
            filepath += DateTime.Now.Ticks + ".txt";
            StreamWriter writer = new StreamWriter(filepath);
            writer.AutoFlush = true;

            writer.WriteLine("INPUT _ HIDDEN WEIGHTS "+_nrInput + " * " + _nrHidden + " : ");

            for (int i = 0; i < _nrInput; i++)
            {
                for (int j = 0; j < _nrHidden; j++)
                {
                    writer.Write(_weightInHidden[i, j] + ", ");
                }
                writer.Write("\n");
            }
            writer.Write("\n");

            writer.WriteLine("HIDDEN BIAS " + _nrHidden + " : ");
            for (int j = 0; j < _nrHidden; j++)
            {
                writer.Write(_hiddenBias[j] + ", ");                
            }
            writer.Write("\n");
            writer.Write("\n");

            writer.WriteLine("HIDDEN _ OUTPUT WEIGHTS " + _nrHidden + " * " + _nrOutput + " : ");

            for (int i = 0; i < _nrHidden; i++)
            {
                for (int j = 0; j < _nrOutput; j++)
                {
                    writer.Write(_weightHiddenOut[i, j] + ", ");
                }
                writer.Write("\n");
            }
            writer.Write("\n");

            writer.WriteLine("OUTPUT BIAS " + _nrOutput + " : ");
            for (int j = 0; j < _nrOutput; j++)
            {
                writer.Write(_outputBias[j] + ", ");
            }
            writer.Write("\n");
            writer.Write("\n");

        }
    }

}
