using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class OneDimDataGenerator
    {
        double extParam = 2;
 
        public double[] Calc(double[][] x)
        {
            double[] y = new double[x.Length];

            for (int i = 0; i < x.Length; i++)
            {
                y[i] = Math.Sin(extParam * x[i][0]);
            }

            return y;
        }

        public double Calc(double[] x)
        {
            return Math.Sin(extParam * x[0]);
        }
    }
}
