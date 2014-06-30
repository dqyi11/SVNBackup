using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace GraphAnalysis
{
    class Edge
    {
        public Vertex mVertexA;
        public Vertex mVertexB;

        public Edge(Vertex vA, Vertex vB)
        {
            mVertexA = vA;
            mVertexB = vB;
        }

        public void Print()
        {
            Console.WriteLine("EDGE FROM " + mVertexA.GetName()
                + " TO " + mVertexB.GetName());
        }

    }
}
