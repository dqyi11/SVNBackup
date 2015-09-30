using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using DotNumerics.LinearAlgebra;

namespace KernelMachine
{
    class KernelRegression
    {
        public enum KernelType { GAUSSIAN, LINEAR };

        public Matrix _C;
        public Matrix _K;
        public Matrix _lastC;

        public double _lamda = 1;

        public int _dataDimension;
        public int _trainSize;

        public double[][] _inputKer;

        public KernelType _kernelType;

        public KernelRegression(KernelType type = KernelType.GAUSSIAN)
        {
            _kernelType = type;
        }

        public bool Train(double[][] input, double[] output)
        {
            _dataDimension = input[0].Length; // data dimension
            _trainSize = input.Count(); // train size

            _inputKer = (double[][])input.Clone();

            if (output.Length != _trainSize)
            {
                return false;
            }

            _K = new Matrix(_trainSize);

            for (int i = 0; i < _trainSize; i++)
            {
                for (int j = 0; j < _trainSize; j++)
                {
                    _K[i, j] = CalcKernel(input[i], input[j]);
                }
            }

            Matrix Y = new Matrix(output.Length, 1);
            for (int i = 0; i < output.Length; i++)
            {
                Y[i, 0] = output[i];
            }

            Matrix lamdaI = new Matrix(_trainSize);

            for (int i = 0; i < _trainSize; i++)
            {
                lamdaI[i, i] = _lamda;
            }

            Matrix temp = _K.Subtract(lamdaI);
            temp = temp.Inverse();
            Matrix temp2 = temp.Multiply(Y);
            _C = (Matrix)temp2.Clone();
            _lastC = (Matrix)temp2.Clone();

            return true;
        }

        public bool IterativeTrain(double[][] input, double[] output)
        {
            _dataDimension = input[0].Length; // data dimension
            _trainSize = input.Count(); // train size

            _inputKer = (double[][])input.Clone();

            if (output.Length != _trainSize)
            {
                return false;
            }

            _K = new Matrix(_trainSize);

            for (int i = 0; i < _trainSize; i++)
            {
                for (int j = 0; j < _trainSize; j++)
                {
                    _K[i, j] = CalcKernel(input[i], input[j]);
                }
            }

            Matrix Y = new Matrix(output.Length, 1);

            for (int i = 0; i < output.Length; i++)
            {
                Y[i, 0] = output[i];
            }

            Matrix lamdaI = new Matrix(_trainSize);

            for (int i = 0; i < _trainSize; i++)
            {
                lamdaI[i, i] = _lamda;
            }

            Matrix temp = _K.Subtract(lamdaI);
            temp = temp.Inverse();

            if (_lastC == null)
            {
                _lastC = new Matrix(_trainSize, 1);
            }

            Matrix temp2 = new Matrix(_trainSize, 1);
            for (int i = 0; i < _trainSize; i++)
            {
                temp2[i, 0] = _lamda * _lastC[i, 0];
            }

            temp2 = Y.Add(temp2);
            temp2 = temp.Multiply(temp2);
            _C = (Matrix)temp2.Clone();
            _lastC = (Matrix)temp2.Clone();

            return true;
        }

        double CalcKernel(double[] a, double[] b)
        {
            double sumSqr = 0;

            for (int i = 0; i < a.Length; i++)
            {
                sumSqr += (a[i] - b[i]) * (a[i] - b[i]);
            }

            return Math.Exp(-sumSqr);
        }

        public double Predict(double[] input)
        {
            double predictVal = 0;

            for (int i = 0; i < _trainSize; i++)
            {
                predictVal += _C[i, 0] * CalcKernel(input, _inputKer[i]);           
            }

            return predictVal;
        }

        public double[] Predict(double[][] input)
        {
            double[] predictVal = new double[input.Length];

            for (int i = 0; i < input.Length; i++)
            {
                predictVal[i] = Predict(input[i]);
            }
            
            return predictVal;
        }
    }
}
