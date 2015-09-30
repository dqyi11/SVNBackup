using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CsvMgr;

namespace HouseData
{
    class HouseDataMgr
    {
        string _filename;

        public const int _inputNum = 13;
        public const int _labelNum = 1;

        int _recordCnt;

        public double [,] _input;
        public double [,] _label;

        public HouseDataMgr(string filename)
        {
            _filename = filename;
            _recordCnt = 0;
        }

        public int count
        {
            get
            {
                return _recordCnt;
            }
        }

        public int inputNum
        {
            get
            {
                return _inputNum;
            }
        }

        public double[] GetInputData(int index)
        {
            if (index >= _recordCnt)
            {
                return null;
            }

            double[] input = new double[_inputNum];

            for (int i = 0; i < _inputNum; i++)
            {
                input[i] = (double)_input[index, i];
            }
            return (double[])input.Clone();
        }

        public double GetLabelData(int index)
        {
            return (double)_label[index, 0];
        }

        int GetRecordCount()
        {
            int count = 0;
            using (CsvFileReader reader = new CsvFileReader(_filename))
            {
                CsvRow row = new CsvRow();

                while (reader.ReadRow(row))
                {
                    count++;
                }
            }

            return count;
        }

        public void Load()
        {
            _recordCnt = GetRecordCount();

            _input = new double[_recordCnt, _inputNum];
            _label = new double[_recordCnt, _labelNum];

            using (CsvFileReader reader = new CsvFileReader(_filename))
            {
                CsvRow row = new CsvRow();

                int rowNum = 0;
                while (reader.ReadRow(row))
                {
                    int rowCnt = row.Count;

                    for (int i = 0; i < rowCnt; i++)
                    {
                        if (i == rowCnt - 1)
                        {
                            _label[rowNum, 0] = double.Parse(row[i]);
                        }
                        else
                        {
                            _input[rowNum, i] = double.Parse(row[i]);
                        }

                    }

                    rowNum++;

                }
            }
        }

        public void Dump(string dumpFilename)
        {
            using (CsvFileWriter writer = new CsvFileWriter(dumpFilename))
            {
                for (int i = 0; i < _recordCnt; i++)
                {
                    CsvRow row = new CsvRow();
                    for (int j = 0; j < _inputNum + _labelNum; j++)
                    {
                        if (j < _inputNum)
                        {
                            row.Add(_input[i,j].ToString());
                        }
                        else
                        {
                            row.Add(_label[i, j - _inputNum].ToString());
                        }
                    }
                    writer.WriteRow(row);
                }
            }

        }
    }
}
