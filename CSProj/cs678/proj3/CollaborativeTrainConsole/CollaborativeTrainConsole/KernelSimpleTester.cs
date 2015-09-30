using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KernelMachine
{
    class KernelSimpleTester
    {
        KernelRegression _regressionMachine;

        double[][] learnInput = new double[4][]{new double[]{1},
                                           new double[]{3},
                                           new double[]{5},
                                           new double[]{7}};

        double[] learnOutput = new double[] { 12, 18.0, 20.0, 17.0 };

        double[][] testInput  = new double[3][]{new double[]{2},
                                           new double[]{4},
                                           new double[]{6}};
        double[] testOutput = new double[] { 14.0, 18.5, 19.0 };


        public KernelSimpleTester(KernelRegression machine=null)
        {
            if (null == machine)
            {
                _regressionMachine = new KernelRegression();
            }
            else
            {
                _regressionMachine = machine;
            }
        }

        public void Train()
        {
            _regressionMachine._lamda = 0;

            _regressionMachine.Train(learnInput, learnOutput);
        }

        public void Test()
        {
            double mse = 0.0;
            for (int i = 0; i < testInput.Length; i++)
            {
                double predictOuput = _regressionMachine.Predict(testInput[i]);

                Console.WriteLine(testOutput[i] - predictOuput);
                mse += (testOutput[i] - predictOuput) * (testOutput[i] - predictOuput);
            }

            mse = Math.Sqrt(mse) / testInput.Length;

            Console.WriteLine("MSE:" + mse);

        }


        public void Test2()
        {
            double mse = 0.0;
            for (int i = 0; i < learnInput.Length; i++)
            {
                double predictOuput = _regressionMachine.Predict(learnInput[i]);

                Console.WriteLine(learnOutput[i] - predictOuput);
                mse += (learnOutput[i] - predictOuput) * (learnOutput[i] - predictOuput);
            }

            mse = Math.Sqrt(mse) / learnInput.Length;

            Console.WriteLine("MSE:" + mse);

        }
    }
}
