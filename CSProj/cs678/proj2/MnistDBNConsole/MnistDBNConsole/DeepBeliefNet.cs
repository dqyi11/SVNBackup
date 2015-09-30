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
    class DeepBeliefNet
    {
        double _learningRate;
        //LRBM _lrbm;
        int _outputNbr;

        List<double[]> _layers;
        List<double[,]> _weights;
        List<double[]> _bias;

        Random _rnd;

        MLP _mlp;

        //Serialization function.
        public void GetObjectData(SerializationInfo info, StreamingContext ctxt)
        {
            //You can use any custom name for your name-value pair. But make sure you
            // read the values with the same name. For ex:- If you write EmpId as "EmployeeId"
            // then you should read the same with "EmployeeId"
            //info.AddValue("MLP", _mlp);
            info.AddValue("Layers", _layers);
            info.AddValue("Weights", _weights);
            info.AddValue("Bias", _bias);

            //info.AddValue("Rnd", _rnd);
            info.AddValue("LearningRate", _learningRate);
            info.AddValue("OutputNbr", _outputNbr);
            //info.AddValue("LRBM", _lrbm);
        }

        public DeepBeliefNet(SerializationInfo info, StreamingContext ctxt)
        {
            _layers = (List<double[]>)info.GetValue("Layers", typeof(List<double[]>));
            _weights = (List<double[,]>)info.GetValue("Weights", typeof(List<double[,]>));
            _bias = (List<double[]>)info.GetValue("Bias", typeof(List<double[]>));
            _learningRate = (double)info.GetValue("LearningRate", typeof(double));
            _outputNbr = (int)info.GetValue("OutputNbr", typeof(int));

            _rnd = new Random();
            _mlp = new MLP(_layers, _weights, _bias, _learningRate);
        }

        public void DumpParamsToFile(string filename)
        {
            Stream stream = File.Open(filename, FileMode.Create);
            BinaryFormatter bformatter = new BinaryFormatter();

            Console.WriteLine("Writing RBM Params Information");
            bformatter.Serialize(stream, this);
            stream.Close();
        }

        public DeepBeliefNet(double learningRate, LRBM lrbm, int outputNbr)
        {
            _rnd = new Random();
            _learningRate = learningRate;
            //_lrbm = lrbm;
            _outputNbr = outputNbr;

            _layers = new List<double[]>();
            _weights = new List<double[,]>();
            _bias = new List<double[]>();

            InitLRBM(lrbm);
            InitOutput(lrbm);

            _mlp = new MLP(_layers, _weights, _bias, _learningRate);
        }

        public DeepBeliefNet(double learningRate, LRBM lrbm, int[] outputStruct)
        {
            _rnd = new Random();
            _learningRate = learningRate;
            //_lrbm = lrbm;
            int outputDepth = outputStruct.Length;
            _outputNbr = outputStruct[outputDepth-1];

            _layers = new List<double[]>();
            _weights = new List<double[,]>();
            _bias = new List<double[]>();

            InitLRBM(lrbm);
            InitOutputStruct(outputStruct, lrbm);

            _mlp = new MLP(_layers, _weights, _bias, _learningRate);
        }

        public MLP GetMLP()
        {
            return _mlp;
        }

        void InitLRBM(LRBM lrbm)
        {
            int lrbmLayerCnt = lrbm.layerCnt;

            for (int i = 0; i < lrbmLayerCnt ; i++)
            {
                _layers.Add(new double[lrbm._rbms[i].visibleNbr]);
                _weights.Add(lrbm._rbms[i].GetWeightVisibleHidden());
                _bias.Add(lrbm._rbms[i].GetHiddenBias());
            }

            _layers.Add(new double[lrbm._rbms[lrbmLayerCnt-1].hiddenNbr]);
        }

        void InitOutput(LRBM lrbm)
        {
            int lrbmLayerCnt = lrbm.layerCnt;

            double [,] weight = new double[lrbm._rbms[lrbmLayerCnt-1].hiddenNbr, _outputNbr];
            double[] bias = new double[_outputNbr];

            for (int j = 0; j < _outputNbr; j++)
            {
                for (int k = 0; k < lrbm._rbms[lrbmLayerCnt - 1].hiddenNbr; k++)
                {
                    weight[k, j] = _rnd.NextDouble() - 0.5;
                }

                bias[j] = 0.0;
            }

            _weights.Add(weight);
            _bias.Add(bias);

            _layers.Add(new double[_outputNbr]);
        }

        void InitOutputStruct(int[] outputStruct, LRBM lrbm)
        {
            int currentLevelNbr = lrbm._rbms[lrbm.layerCnt - 1].hiddenNbr;
            int nextLevelNbr = outputStruct[0];

            double[,] weight = new double[currentLevelNbr, nextLevelNbr];
            double[] bias = new double[nextLevelNbr];

            for (int j = 0; j < nextLevelNbr; j++)
            {
                for (int k = 0; k < currentLevelNbr; k++)
                {
                    weight[k, j] = _rnd.NextDouble() - 0.5;
                }

                bias[j] = 0.0;
            }

            _weights.Add(weight);
            _bias.Add(bias);

            _layers.Add(new double[nextLevelNbr]);

            
            for (int i = 1; i < outputStruct.Length ; i++)
            {
                currentLevelNbr = outputStruct[i-1];
                nextLevelNbr = outputStruct[i];
                weight = new double[currentLevelNbr, nextLevelNbr];
                bias = new double[nextLevelNbr];

                for (int j = 0; j < nextLevelNbr; j++)
                {
                    for (int k = 0; k < currentLevelNbr; k++)
                    {
                        weight[k, j] = _rnd.NextDouble() - 0.5;
                    }

                    bias[j] = 0.0;
                }

                _weights.Add(weight);
                _bias.Add(bias);

                _layers.Add(new double[nextLevelNbr]);

            }          
        }


        public void DumpToFile()
        {
            string filepath = "DBN-PARAM-";
            filepath += DateTime.Now.Ticks + ".txt";
            StreamWriter writer = new StreamWriter(filepath);
            writer.AutoFlush = true;

            for (int l = 0; l < _weights.Count; l++)
            {
                writer.WriteLine("WEIGHTS " + _layers[l].Length + " * " + _layers[l + 1].Length + ":");

                for (int i = 0; i < _layers[l].Length; i++)
                {
                    for (int j = 0; j < _layers[l + 1].Length; j++)
                    {
                        writer.Write(_weights[l][i, j] + ", ");
                    }
                    writer.Write("\n");
                }

                writer.Write("\n");
                writer.WriteLine("BIAS " + _bias[l].Length + " : ");
                for (int i = 0; i < _bias[l].Length; i++)
                {
                    writer.Write(_bias[l][i] + ", ");
                }
                writer.Write("\n");
                writer.Write("\n");

            }

            writer.Close();
        }

        public void OnlineTrainOnlyLastLayer(double[] inputPattern, double[] expOutputPattern)
        {
            
        }

        public void BatchTrainOnlyLastLayer()
        {
        }
    }

}
