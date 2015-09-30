using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using KernelMachine;
using DataMgr;

namespace SVMConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            IrisDataMgr dataMgr = new IrisDataMgr(@"iris.csv");
            dataMgr.Load();

            SVMTester tester = new SVMTester(dataMgr);
            tester.Train();
            tester.Test2();

            return;
        }
    }
}
