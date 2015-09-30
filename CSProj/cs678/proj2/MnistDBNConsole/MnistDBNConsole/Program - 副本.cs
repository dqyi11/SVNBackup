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


            /*
            Stream stream2 = File.Open("lrbm-param.osl", FileMode.Open);
            BinaryFormatter bformatter2 = new BinaryFormatter();

            Console.WriteLine("Reading Employee Information");
            LRBM lrbm2 = (LRBM)bformatter2.Deserialize(stream2);
            stream2.Close();

            //lrbm2.DumpToFile();



            //dbn2.DumpToFile();
            //dbn2.GetMLP().DumpToFile();

            //dbn2.GetMLP()._trainType = MLP.TrainType.NORMAL_BP;

            //MnistMLPTrainer trainer = new MnistMLPTrainer(trainDataMgr, dbn2.GetMLP());
            //trainer.OnlineTrain();

            //MnistMLPTester tester = new MnistMLPTester(dbn2.GetMLP(), testDataMgr);
            //tester.Test();

            return;
            */
             












            //int [] netStruct = {trainDataMgr.inputNum, 100, 50, 20};
            int[] netStruct = { trainDataMgr.inputNum, 200 };
            //int[] netStruct = { trainDataMgr.inputNum, 100 };

           /*
            RBM rbm = new RBM(0.2, trainDataMgr.inputNum, 100);

            RBMTrainer rbmTrainer = new RBMTrainer(trainDataMgr, rbm);
            //rbmTrainer.OnlineTrain();
            rbmTrainer.BatchTrain();

            rbm.DumpToFile();

            rbm.DumpParamsToFile("rbmParams.osl");

            int testSize = 40;

            for (int t = 0; t < testSize; t++)
            {
                Random rnd1 = new Random();
                int fileIndex = rnd1.Next(trainDataMgr.count-1);
                double[] sample = trainDataMgr.GetNormalizedInputData(fileIndex);

                string wrFilename = "sample-input-" + fileIndex + ".csv";

                StreamWriter sampleWr = new StreamWriter(wrFilename);
                for (int i = 0; i < sample.Length; i++)
                {
                    sampleWr.Write(sample[i] + ", ");
                }

                sampleWr.Flush();
                sampleWr.Close();

                rbm.Calc(sample);
                double[] sampleOut = rbm.GetVisible();
                double[] sampleOutProb = rbm.GetVisibleProb();

                wrFilename = "sample-prob-output-" + fileIndex + ".csv";

                StreamWriter outProbWr = new StreamWriter(wrFilename);
                for (int i = 0; i < sampleOutProb.Length; i++)
                {
                    outProbWr.Write(sampleOutProb[i] + ", ");
                }

                outProbWr.Flush();
                outProbWr.Close();

                wrFilename = "sample-output-" + fileIndex + ".csv";
                StreamWriter outWr = new StreamWriter(wrFilename);
                for (int i = 0; i < sampleOut.Length; i++)
                {
                    outWr.Write(sampleOut[i] + ", ");
                }

                outWr.Flush();
                outWr.Close();
            }

            return;

            */
           

            LRBM lrbm = new LRBM(1, netStruct);

            LRBMTrainer lrbmTrainer = new LRBMTrainer(trainDataMgr, lrbm);
            lrbmTrainer.BatchTrain();
            //lrbmTrainer.OnlineTrain();

            lrbm.DumpToFile();
            lrbm.DumpParamsToFile("lrbm-param.osl");

            return;

            DeepBeliefNet dbn = new DeepBeliefNet(0.8, lrbm, 10);
            //DeepBeliefNet dbn = new DeepBeliefNet(0.8, lrbm, new int[]{50,10});
            dbn.DumpToFile();
            dbn.GetMLP().DumpToFile();

            dbn.DumpParamsToFile("dbn-ser.osl");

            return;




            //dbn.GetMLP().DumpToFile();

            dbn.GetMLP()._trainType = MLP.TrainType.TRAIN_ONLY_OUTPUT_LAYER;
            //dbn.GetMLP()._trainType = MLP.TrainType.NORMAL_BP;

            //MnistMLPTrainer trainer = new MnistMLPTrainer(trainDataMgr, dbn.GetMLP());
            //trainer.OnlineTrain();

            //MnistMLPTester tester = new MnistMLPTester(dbn.GetMLP(), testDataMgr);
            //tester.Test();

            //dbn.GetMLP().DumpToFile();

            sw.Flush();
            sw.Close();

        }
    }
}
