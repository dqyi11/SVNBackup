using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GraphAnalysis
{
    class Connector
    {
        protected List<Edge> mEdges;

        protected Partite mPartiteA;
        protected Partite mPartiteB;

        public Connector(Partite a, Partite b)
        {
            mPartiteA = a;
            mPartiteB = b;
            mEdges = new List<Edge>();
        }

        public void AddEdge(Edge edge)
        {
            mEdges.Add(edge);
        }

        public void Print()
        {
            Console.WriteLine("CONNECTOR FROM " + mPartiteA.GetName()
                + " TO " + mPartiteB.GetName());

            List<Edge>.Enumerator e = mEdges.GetEnumerator();
            while (e.MoveNext())
            {
                Edge v = e.Current;
                v.Print();
            }
        }

    }
}
