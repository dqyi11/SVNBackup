using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GraphAnalysis
{
    class Vertex
    {
        protected string mName;

        public Vertex(string name)
        {
            mName = name;
        }

        public string GetName()
        {
            return mName;
        }

        public void Print()
        {
            Console.WriteLine("VERTEX: " + mName);
        }

    }
}
