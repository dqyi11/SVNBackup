using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace CsvMgr.Test
{
    class CsvMgrTest
    {
        public void LoadTest(string filename)
        {
            ReadTest(filename);
        }

        void WriteTest()
        {
            // Write sample data to CSV file
            using (CsvFileWriter writer = new CsvFileWriter("WriteTest.csv"))
            {
                for (int i = 0; i < 100; i++)
                {
                    CsvRow row = new CsvRow();
                    for (int j = 0; j < 5; j++)
                        row.Add(String.Format("Column{0}", j));
                    writer.WriteRow(row);
                }
            }
        }

        void ReadTest(string filename)
        {
            // Read sample data from CSV file
            using (CsvFileReader reader = new CsvFileReader(filename))
            {
                CsvRow row = new CsvRow();
                
                while (reader.ReadRow(row))
                {
                    Console.Write(row.Count);
                    Console.Write(" : ");
                    Console.Write(row[row.Count - 1]);
                    Console.Write("\n");
                    /*
                    foreach (string s in row)
                    {
                        Console.Write(s);
                        Console.Write(" ");
                    }
                    Console.WriteLine();
                     */
                }
            }
        }
    }
}
