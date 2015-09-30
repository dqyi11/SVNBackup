using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class TwoDimDataGenerator
    {
        double[,] bumpCenters = new double[,] { {0.4, -0.3}, {0.8, 0.2}, { -0.1, -0.5}};
        double[] bumpVars = new double[] { 2, 3, 1.5 };

        public double[] Calc(double [][] x)
        {
            double[] y = new double[x.Length];

            int bumpCnt = bumpVars.Length;

            for (int i = 0; i < x.Length; i++)
            {
                double temp = 0;
                for(int j=0; j<bumpCnt; j++)
                {
                    double dist =
                     (x[i][0] - bumpCenters[j, 0]) * (x[i][0] - bumpCenters[j, 0])
                     + (x[i][1] - bumpCenters[j, 1]) * (x[i][1] - bumpCenters[j, 1]);
                    temp += Math.Exp(-dist);
                }

                y[i] = temp;
            }

            return y;
        }
    }
}
