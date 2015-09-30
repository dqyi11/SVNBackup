using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using HouseData;
using KernelMachine;

namespace CollaborativeTrainConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            string filename = "REGRESSION-";
            filename += DateTime.Now.Ticks;
            filename += ".txt";
            StreamWriter sw = new StreamWriter(filename);
            Console.SetOut(sw);
            sw.AutoFlush = true;

            HouseDataMgr dataMgr = new HouseDataMgr(@"house.csv");
            dataMgr.Load();

            Console.WriteLine("Data Loaded");

            DistributedOneDimDataTrainer trainer = new DistributedOneDimDataTrainer();
            trainer.PrintAdjacency();

            trainer.TrainCenterMachine();
            trainer.TrainDistributedMachines();

            trainer.TestCenterMachine();
            trainer.TestDistributedMachines();

            /*
            KernelRegression kernelMachine = new KernelRegression(KernelRegression.KernelType.LINEAR);
            //KernelSimpleTester tester = new KernelSimpleTester(kernelMachine);
            //KernelRegressionTester tester = new KernelRegressionTester(dataMgr);
            //OneDimDataTester tester = new OneDimDataTester();
            TwoDimDataTester tester = new TwoDimDataTester();
            tester.Train();
            tester.Test();
            //tester.Test2();
             */
            

            sw.Flush();
            sw.Close();

            return;
        }
    }
}
