using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using MnistData;

namespace NeuralNetwork
{
    class LRBMTester
    {
        LRBM _lrbm;
        MnistDataMgr _dataMgr;

        int _testSetSize;

        public LRBMTester(LRBM lrbm, MnistDataMgr dataMgr)
        {
            _lrbm = lrbm;
            _dataMgr = dataMgr;

            _testSetSize = _dataMgr.count;
        }

        public void Test()
        {


        }
    }
}
