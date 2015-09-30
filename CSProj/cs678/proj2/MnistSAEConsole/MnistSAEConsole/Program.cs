using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using MnistData;
using NeuralNetwork;

namespace MnistSAEConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            string filename = "SAE-";
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

            AutoEncoder autoEncoder = new AutoEncoder(0.8, trainDataMgr.inputNum, 100);
            AutoEncoderTrainer autoEncoderTrainer = new AutoEncoderTrainer(autoEncoder, trainDataMgr);

            autoEncoderTrainer.OnlineTrain();

            sw.Flush();
            sw.Close();
        }
    }
}
