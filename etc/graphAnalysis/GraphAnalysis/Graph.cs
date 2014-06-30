using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using DotNumerics.LinearAlgebra;

namespace GraphAnalysis
{
    class Graph
    {
        public List<Partite> mPartites;
        public List<Connector> mConnectors;
        public List<Vertex> mVertices;
        public List<Edge> mEdges;

        public Graph()
        {
            mPartites = new List<Partite>();
            mConnectors = new List<Connector>();
            mVertices = new List<Vertex>();
            mEdges = new List<Edge>();          
        }

        public Partite CreatePartite(string name)
        {
            Partite partite = new Partite(name);
            mPartites.Add(partite);
            return partite;
        }

        public Vertex CreateVertex(string name)
        {
            Vertex vertex = new Vertex(name);
            mVertices.Add(vertex);
            return vertex;
        }

        public Connector Connect(Partite a, Partite b)
        {
            Connector connector = new Connector(a, b);
            mConnectors.Add(connector);
            return connector;
        }

        public Edge Connect(Vertex a, Vertex b)
        {
            Edge edge = new Edge(a, b);
            mEdges.Add(edge);
            return edge;
        }

        public Partite GetPartite(string name)
        {
            List<Partite>.Enumerator e = mPartites.GetEnumerator();
            while (e.MoveNext())
            {
                Partite p = e.Current;
                if (p.GetName().CompareTo(name) == 0)
                {
                    return p;
                }
            }

            return null;
        }

        public Vertex GetVertex(string name)
        {
            List<Vertex>.Enumerator e = mVertices.GetEnumerator();
            while (e.MoveNext())
            {
                Vertex v = e.Current;
                if (v.GetName().CompareTo(name) == 0)
                {
                    return v;
                }
            }

            return null;
        }

        public void PrintVertex()
        {
            List<Partite>.Enumerator e = mPartites.GetEnumerator();
            while (e.MoveNext())
            {
                Partite v = e.Current;
                v.Print();
            }
        }

        public void PrintEdge()
        {
            List<Connector>.Enumerator e = mConnectors.GetEnumerator();
            while (e.MoveNext())
            {
                Connector v = e.Current;
                v.Print();
            }
        }

        public bool IsConnected(Vertex a, Vertex b)
        {
            if (0 == a.GetName().CompareTo(b.GetName()))
            {
                return true;
            }

            List<Edge>.Enumerator e = mEdges.GetEnumerator();
            while (e.MoveNext())
            {
                Edge edge = e.Current;
                if ((0 ==edge.mVertexA.GetName().CompareTo(a.GetName())
                    && 0==edge.mVertexB.GetName().CompareTo(b.GetName()))
                    ||
                    (0==edge.mVertexA.GetName().CompareTo(b.GetName())
                    && 0==edge.mVertexB.GetName().CompareTo(a.GetName())))
                {
                    return true;
                }
            }

            return false;
        }

        public int GetDegree(Vertex a)
        {
            int degree = 0;
            List<Edge>.Enumerator e = mEdges.GetEnumerator();
            while (e.MoveNext())
            {
                Edge edge = e.Current;
                if (0 == edge.mVertexA.GetName().CompareTo(a.GetName())
                    ||
                    0 == edge.mVertexB.GetName().CompareTo(a.GetName()))
                {
                    degree++;
                }
            }

            return degree;
        }

        public Matrix GetAdjacentMatrix()
        {
            int vNum = mVertices.Count;
            Matrix adjacentMatrix = new Matrix(vNum, vNum);

            for (int i = 0; i < vNum; i++)
            {
                for (int j = 0; j < vNum; j++)
                {
                    if (IsConnected(mVertices[i], mVertices[j]))
                    {
                        adjacentMatrix[i, j] = 1;
                    }
                    else
                    {
                        adjacentMatrix[i, j] = 0;
                    }

                }
            }

            return adjacentMatrix;
        }

        public Matrix GetDegreeMatrix()
        {
            int vNum = mVertices.Count;
            Matrix degreeMatrix = new Matrix(vNum, vNum);

            for (int i = 0; i < vNum; i++)
            {
                degreeMatrix[i, i] = GetDegree(mVertices[i]);
            }

            return degreeMatrix;
        }
    }
}
