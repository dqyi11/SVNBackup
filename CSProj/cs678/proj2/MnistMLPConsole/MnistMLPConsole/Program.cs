using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;
using NeuralNetwork;
//using CsvMgr.Test;
using System.IO;

namespace MnistMLPConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            //MLPWithBiasTest test = new MLPWithBiasTest();
            //MLPTest test = new MLPTest();
            //test.TestXOR();
            //test.TestDigit();

            //return;
            /*
            CsvMgrTest test = new CsvMgrTest();
            test.LoadTest("test.csv");
             */

            string filename = "MLP-";
            filename += DateTime.Now.Ticks;
            filename += ".txt";
            StreamWriter sw = new StreamWriter(filename);
            Console.SetOut(sw);
            sw.AutoFlush = true;
            
            MnistDataMgr trainDataMgr = new MnistDataMgr("train.csv");
            trainDataMgr.Load();

            Console.WriteLine("Train data loaded");

            MnistDataMgr testDataMgr = new MnistDataMgr("test.csv");
            testDataMgr.Load();

            Console.WriteLine("Test data loaded");

            MnistMLPTrainer trainer = new MnistMLPTrainer(trainDataMgr);
            //int[] layerStruct = { trainDataMgr.inputNum, 100, 50, 20, 10 };
            //int[] layerStruct = { trainDataMgr.inputNum, 150, 50, 10 };
            //int[] layerStruct = { trainDataMgr.inputNum, 100, 50, 10 };
            int[] layerStruct = { trainDataMgr.inputNum, 80, 30, 10 };

            trainer.InitMLP(layerStruct, 0.8);
            //trainer.InitMLP(200, 0.8);
            trainer.OnlineTrain();
            //trainer.BatchTrain();

            //dataMgr.Dump("test2.csv");  

            MnistMLPTester tester = new MnistMLPTester(trainer.GetNetwork(), testDataMgr);
            tester.Test();

            trainer.GetNetwork().DumpToFile();

            sw.Flush();
            sw.Close();

            return;

        }
    }
}
