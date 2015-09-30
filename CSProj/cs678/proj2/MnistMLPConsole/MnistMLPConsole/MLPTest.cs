using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace NeuralNetwork
{
    class MLPTest
    {
         public void TestXOR()
        {
            double [][] _inputPattern =
            {
                new double[2] { 0.0, 0.0 },
                new double[2] { 1.0, 0.0 },
                new double[2] { 0.0, 1.0 },
                new double[2] { 1.0, 1.0 } };
             double[][] _targetOutputPattern = 
             {
                 new double[1] { 0.0 },
                 new double[1] { 1.0 },
                 new double[1] { 1.0 },
                 new double[1] { 0.0 } };
             
             MLPNetwork network = new MLPNetwork(2, 5, 1, 2.8);

             network.InitLog("XOR-");

             Random rnd = new Random();
             
             for (int i = 0; i < 50000; i++)
             {
                 //network.BatchTrain(_inputPattern, _targetOutputPattern);
                 int index = rnd.Next(0, 4);
                 Console.WriteLine(index);
                 network.OnlineTrain(_inputPattern[index], _targetOutputPattern[index]);
             }
             
             for (int i = 0; i < _inputPattern.Length; i++)
             {
                 double[] outst = network.Run(_inputPattern[i]);
                 Console.WriteLine("XOR({0}, {1}) = {2:0.000}", _inputPattern[i][0], _inputPattern[i][1], outst[0]);
             }

             
         }

         public void TestDigit()
         {
             double[] pattern0 = {
                0, 1, 1, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0
            };
             double[] pattern1 = {
                0, 0, 1, 0, 0,
                0, 1, 1, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 1, 0, 0,
                0, 0, 1, 0, 0
            };
             double[] pattern2 = {
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 1, 1, 1, 0,
                0, 1, 0, 0, 0,
                0, 1, 1, 1, 0
            };
             double[] pattern3 = {
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 1, 1, 1, 0
            };
             double[] pattern4 = {
                0, 1, 0, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 1, 0
            };
             double[] pattern5 = {
                0, 1, 1, 1, 0,
                0, 1, 0, 0, 0,
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 1, 1, 1, 0
            };
             double[] pattern6 = {
                0, 1, 1, 1, 0,
                0, 1, 0, 0, 0,
                0, 1, 1, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0
            };
             double[] pattern7 = {
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 0, 0, 1, 0,
                0, 0, 1, 0, 0,
                0, 1, 0, 0, 0
            };
             double[] pattern8 = {
                0, 1, 1, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0
            };
             double[] pattern9 = {
                0, 1, 1, 1, 0,
                0, 1, 0, 1, 0,
                0, 1, 1, 1, 0,
                0, 0, 0, 1, 0,
                0, 1, 1, 1, 0
            };

             double[][] _inputPattern = { 
                pattern0, pattern1, pattern2, pattern3, pattern4, 
                pattern5, pattern6, pattern7, pattern8, pattern9};
             double[][] _targetOutputPattern = {
                new double[] { 0.0 },
                new double[] { 0.1 },
                new double[] { 0.2 },
                new double[] { 0.3 },
                new double[] { 0.4 },
                new double[] { 0.5 },
                new double[] { 0.6 },
                new double[] { 0.7 },
                new double[] { 0.8 },
                new double[] { 0.9 }
            };

             MLPNetwork network = new MLPNetwork(25, 25, 1, 0.8);

             network.InitLog("Digit-");

             Random rnd = new Random();

             for (int i = 0; i < 500000; i++)
             {
                 //network.BatchTrain(_inputPattern, _targetOutputPattern);
                
                 int index = rnd.Next(0, 10);
                 network.OnlineTrain(_inputPattern[index], _targetOutputPattern[index]);
                  
             }

             for (int i = 0; i < _inputPattern.Length; i++)
             {
                 double[] output = network.Run(_inputPattern[i]);
                 Console.WriteLine("{0} pattern = {1:0.000} which is digit {2}", i,
                     output[0], (int)Math.Round(output[0] * 10));
             }
             


         }
    }
}
