using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using NeuralNetwork;
using MnistData;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;

namespace MnistDBNConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            string filename = "DBN-";
            filename += DateTime.Now.Ticks;
            filename +=".txt";
            StreamWriter sw = new StreamWriter(filename);
            Console.SetOut(sw);
            sw.AutoFlush = true;
            
            MnistDataMgr trainDataMgr = new MnistDataMgr("train.csv");
            trainDataMgr.Load();

            Console.WriteLine("Train data loaded");

            MnistDataMgr testDataMgr = new MnistDataMgr("test.csv");
            testDataMgr.Load();

            Console.WriteLine("Test data loaded");

            //int [] netStruct = {trainDataMgr.inputNum, 100, 50, 20};
            //int[] netStruct = { trainDataMgr.inputNum, 200 };
            int[] netStruct = { trainDataMgr.inputNum, 200 };
           

            LRBM lrbm = new LRBM(1, netStruct);

            LRBMTrainer lrbmTrainer = new LRBMTrainer(trainDataMgr, lrbm);
            lrbmTrainer.BatchTrain();
            //lrbmTrainer.OnlineTrain();

            lrbm.DumpToFile();

            //DeepBeliefNet dbn = new DeepBeliefNet(0.8, lrbm, 10);
            DeepBeliefNet dbn = new DeepBeliefNet(0.8, lrbm, new int[]{200,10});
            dbn.DumpToFile();
            dbn.GetMLP().DumpToFile();
            //dbn.GetMLP().ClearBias();

            //dbn.GetMLP()._trainType = MLP.TrainType.TRAIN_ONLY_OUTPUT_LAYER;
            dbn.GetMLP()._trainType = MLP.TrainType.NORMAL_BP;
            dbn.GetMLP().learningDepth = 2;

            MnistMLPTrainer trainer = new MnistMLPTrainer(trainDataMgr, dbn.GetMLP());
            trainer.OnlineTrain();

            MnistMLPTester tester = new MnistMLPTester(dbn.GetMLP(), testDataMgr);
            tester.Test();

            sw.Flush();
            sw.Close();

        }
    }
}
