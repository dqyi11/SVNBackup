using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class DataAnalyzer
    {
        public double Mean(double[] input)
        {
            double mean = 0;

            for (int i = 0; i < input.Length; i++)
            {
                mean += input[i];
            }

            mean = mean / input.Length;

            return mean;
        }

        public double Variance(double[] input)
        {
            double variance = 0;

            double mean = Mean(input);

            for (int i = 0; i < input.Length; i++)
            {
                variance += (mean - input[i]) * (mean - input[i]);
            }

            variance = variance / input.Length;

            return variance;
        }

        public double[] GetDifference(double[] A, double[] B)
        {
            double[] diff = new double[A.Length];

            for (int i = 0; i < diff.Length; i++)
            {
                diff[i] = A[i] - B[i];
            }

            return diff;
        }
    }
}
