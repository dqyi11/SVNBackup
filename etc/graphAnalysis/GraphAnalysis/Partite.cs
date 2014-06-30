using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GraphAnalysis
{
    class Partite
    {
        protected string mName;
        List<Vertex> mVertices;

        public Partite(string name)
        {
            mName = name;
            mVertices = new List<Vertex>();
        }

        public void AddVertex(Vertex vertex)
        {
            mVertices.Add(vertex);
        }

        public string GetName()
        {
            return mName;
        }

        public Vertex this[int index]
        {
            get { return mVertices[index]; }
        }

        public int vertexNum
        {
            get { return mVertices.Count; }
        }

        public void Print()
        {
            Console.WriteLine("PARTITE:" + mName);

            List<Vertex>.Enumerator e = mVertices.GetEnumerator();
            while (e.MoveNext())
            {
                Vertex v = e.Current;
                v.Print();
            }
        }

    }
}
