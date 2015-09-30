using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class KernelAgent
    {
        KernelRegression _machine;
        double[] _pos;
        double _value;

        List<KernelAgent> _neighbors;

        public int connectivity
        {
            get
            {
                return _neighbors.Count + 1;
            }
        }

        public KernelAgent(double[] pos)
        {
            _machine = new KernelRegression();
            _pos = pos;
            _neighbors = new List<KernelAgent>();
        }

        public KernelRegression GetMachine()
        {
            return _machine;
        }

        public double[] GetPos()
        {
            return _pos;
        }

        public double GetValue()
        {
            return _value;
        }

        public void SetValue(double value)
        {
            _value = value;
        }

        public void ClearNeighbors()
        {
            _neighbors.Clear();
        }

        public void AddNeighbor(KernelAgent agent)
        {
            _neighbors.Add(agent);
        }

        public double RespondQuery()
        {
            return _machine.Predict(_pos);
        }

        public double GetDistance(double[] pos)
        {
            double dist = 0;

            for (int i = 0; i < _pos.Length; i++)
            {
                dist += (pos[i] - _pos[i]) * (pos[i] - _pos[i]);
            }

            dist = Math.Sqrt(dist);

            return dist;
        }

        public void Train()
        {
            double[][] input = new double[connectivity][];
            double[] output = new double[connectivity];

            for (int i = 0; i < connectivity - 1; i++)
            {
                input[i] = _neighbors[i].GetPos();
                output[i] = _neighbors[i].RespondQuery();
            }

            input[connectivity - 1] = _pos;
            output[connectivity - 1] = _value;

            Train(input, output);
        }

        public void Train(double[][] input, double[] output)
        {
            //_machine._lamda = 0.8;
            _machine.IterativeTrain(input, output);
        }
    }
}
