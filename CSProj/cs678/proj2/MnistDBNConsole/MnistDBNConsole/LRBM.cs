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
    class LRBM
    {
        public List<RBM> _rbms;

        public int layerCnt
        {
            get
            {
                return _rbms.Count;
            }
        }

        //Serialization function.
        public void GetObjectData(SerializationInfo info, StreamingContext ctxt)
        {
            //You can use any custom name for your name-value pair. But make sure you
            // read the values with the same name. For ex:- If you write EmpId as "EmployeeId"
            // then you should read the same with "EmployeeId"
            info.AddValue("RBMS", _rbms);

        }

        public void DumpParamsToFile(string filename)
        {
            Stream stream = File.Open(filename, FileMode.Create);
            BinaryFormatter bformatter = new BinaryFormatter();

            Console.WriteLine("Writing RBM Params Information");
            bformatter.Serialize(stream, this);
            stream.Close();
        }

        public int GetVisibleLayerSize(int layerIdx)
        {
            if (layerIdx < 0 || layerIdx >= _rbms.Count)
            {
                return 0;
            }

            return _rbms[layerIdx].visibleNbr;
        }

        public int GetHiddenLayerSize(int layerIdx)
        {
            if (layerIdx < 0 || layerIdx >= _rbms.Count)
            {
                return 0;
            }

            return _rbms[layerIdx].hiddenNbr;
        }

        public LRBM(double learningRate, int[] layerNbr, int relaxStep = 1)
        {
            _rbms = new List<RBM>();
            for (int i = 0; i < layerNbr.Length - 1; i++)
            {
                RBM rbm = new RBM(learningRate, layerNbr[i], layerNbr[i + 1], relaxStep);
                rbm._index = i;
                _rbms.Add(rbm);
            }
        }

        public double[] Feedforward(int layerIdx, double[] input)
        {
            double[] output = input;
            for (int l = 0; l < layerIdx; l++)
            {
                output = _rbms[l].Feedforward(input);
            }

            return (double [])output.Clone();
        }

        public double[] Feedforward(double[] input)
        {
            return Feedforward(layerCnt, input);
        }

        public void DumpToFile()
        {
            for (int i = 0; i < _rbms.Count; i++)
            {
                _rbms[i].DumpToFile();
            }
        }
    }
}
